// Out-of-Bacon: 16-entry renamed scheduling window and precise two-wide ROB.
// ROB slots are speculative physical registers; committed values live in ARF.
// RAM loads are ordered, stores/MMIO occur only at retirement, flush restores RAT.
module out_cpu #(parameter CODE_FILE="code.hex", DATA_FILE="data.hex",
    parameter integer CLOCK_HZ=100000000, PIPELINED=0)(
    input wire clk, resetn, output wire uart_tx, output reg halted=0,
    output wire [31:0] debug_pc, debug_insn, debug_wdata,
    output wire debug_commit, debug_commit2, debug_we,
    output wire [4:0] debug_rd,
    input wire [4:0] debug_reg_addr, output wire [31:0] debug_reg_data,
    output wire debug_uart_valid, output wire [7:0] debug_uart_data,
    output reg [31:0] cycles=0,
    output wire debug_timer_valid, output wire [7:0] profile_flags,
    output wire [1:0] profile_issued
);
    (* rom_style="block" *) reg [31:0] imem[0:8191];
    (* ram_style="block" *) reg [31:0] dmem[0:16383];
    reg [31:0] arf[0:31];
    (* ram_style="distributed" *) reg [31:0] arf0[0:31],arf1[0:31];
    reg [31:0] arf_latest=0,arf_known=0;
    function [31:0] committed_value;
        input [4:0] r;
        begin committed_value=r==0 || !arf_known[r] ? 0 : arf_latest[r] ? arf1[r] : arf0[r]; end
    endfunction
    reg [31:0] rat_valid=0;
    reg [3:0] rat[0:31];
    reg [15:0] valid=0, done=0, issued=0, fault=0, ra=0, rb=0;
    reg [31:0] qi[0:15], qp[0:15], qn[0:15], imm[0:15];
    reg [31:0] branch_target[0:15], fallthrough[0:15];
    reg [31:0] va[0:15], vb[0:15], value[0:15], addr[0:15], storeval[0:15], actual[0:15];
    reg [3:0] ta[0:15], tb[0:15];
    reg [4:0] rd[0:15];
    reg [5:0] op[0:15];
    reg [1:0] ot1[0:15], ot2[0:15];
    // kind: 0 ALU, 1 control, 2 load, 3 store, 4 multiply, 5 divide, 6 fault.
    reg [2:0] kind[0:15];
    reg [3:0] head=0, tail=0;
    reg [4:0] count=0;
    integer i;
    initial begin
        for(i=0;i<8192;i=i+1) imem[i]=32'h13;
        for(i=0;i<16384;i=i+1) dmem[i]=0;
        for(i=0;i<32;i=i+1) begin arf[i]=0; arf0[i]=0; arf1[i]=0; rat[i]=0; end
        $readmemh(CODE_FILE,imem); $readmemh(DATA_FILE,dmem);
    end
    reg [63:0] bv=0;
    reg [23:0] btag[0:63];
    reg [31:0] btarget[0:63];
    reg [1:0] bcount[0:63];
    reg [31:0] fpc=32'h8000;
    wire [31:0] fp4=fpc+4, fp8=fpc+8;
    wire pred0=bv[fpc[7:2]] && btag[fpc[7:2]]==fpc[31:8] && bcount[fpc[7:2]][1];
    wire pred1=bv[fp4[7:2]] && btag[fp4[7:2]]==fp4[31:8] && bcount[fp4[7:2]][1];
    wire [31:0] fn0=pred0 ? btarget[fpc[7:2]] : fp4;
    wire [31:0] fn1=pred1 ? btarget[fp4[7:2]] : fp8;
    reg dv=0, dv2=0;
    reg [31:0] di[0:1], dp=32'h8000, dn0=0, dn1=0;
    wire [4:0] ds1[0:1], ds2[0:1], dd[0:1];
    wire [31:0] dimm[0:1];
    wire [5:0] dop[0:1];
    wire [1:0] dt1[0:1],dt2[0:1];
    wire dwe[0:1], dl[0:1], ds[0:1], dbad[0:1];
    wire [4:0] drs1[0:1],drs2[0:1];
    wire source_used_a[0:1],source_used_b[0:1];
    genvar g;
    generate for(g=0;g<2;g=g+1) begin: decoders
        decode dec(di[g],ds1[g],ds2[g],dd[g],dimm[g],dop[g],dt1[g],dt2[g],dwe[g],dl[g],ds[g],dbad[g]);
        assign drs1[g]=PIPELINED!=0 ? di[g][19:15] : ds1[g];
        assign drs2[g]=PIPELINED!=0 ? di[g][24:20] : ds2[g];
        assign source_used_a[g]=PIPELINED==0 || dt1[g]==`OP_TYPE_REG;
        assign source_used_b[g]=PIPELINED==0 || dt2[g]==`OP_TYPE_REG || ds[g];
    end endgenerate
    wire uart_ready;
    wire [3:0] h1=head+1'b1;
    wire hstore=kind[head]==3;
    wire huart=hstore && addr[head]==`UART_ADDR;
    wire retire=resetn && !halted && count!=0 && done[head] && !fault[head] && (!huart || uart_ready);
    wire redirect=retire && kind[head]==1 && actual[head]!=qn[head];
    wire trap=resetn && !halted && count!=0 && done[head] && fault[head];
    wire flush=redirect || trap;
    // Second retirement has no store/control/fault side effect. A younger load
    // may retire here; its read has already completed in ordered memory issue.
    wire retire2=retire && !redirect && kind[head]!=1 && count>1 && done[h1] && !fault[h1] && kind[h1]!=3 && kind[h1]!=1;
    wire [1:0] nr=retire ? (retire2 ? 2 : 1) : 0;
    wire dispatch=resetn && !halted && !flush && dv && count<=14;
    wire [1:0] nd=dispatch ? (dv2 ? 2 : 1) : 0;
    wire front=resetn && !halted && !flush && (!dv || dispatch);
    wire [3:0] t1=tail+1'b1;

    // Wake operands from completed physical slots before the slot can be
    // reused. Dispatch bypasses same-cycle retirement and intra-pair renaming.
    reg [31:0] da[0:1], db[0:1];
    reg dra[0:1], drb[0:1];
    reg [3:0] dta[0:1], dtb[0:1];
    integer d;
    always @* begin
        for(d=0;d<2;d=d+1) begin
            dta[d]=rat[drs1[d]]; dtb[d]=rat[drs2[d]];
            dra[d]=!source_used_a[d] || !rat_valid[drs1[d]] || drs1[d]==0 || done[dta[d]];
            drb[d]=!source_used_b[d] || !rat_valid[drs2[d]] || drs2[d]==0 || done[dtb[d]];
            da[d]=!source_used_a[d] || drs1[d]==0 ? 0 : rat_valid[drs1[d]] ? value[dta[d]] :
                PIPELINED!=0 ? committed_value(drs1[d]) : arf[drs1[d]];
            db[d]=!source_used_b[d] || drs2[d]==0 ? 0 : rat_valid[drs2[d]] ? value[dtb[d]] :
                PIPELINED!=0 ? committed_value(drs2[d]) : arf[drs2[d]];
            // Capture producers finishing on the dispatch edge. Subsequent
            // operand wakeup needs only tagged completion broadcasts.
            if(PIPELINED!=0 && source_used_a[d] && drs1[d]!=0 && rat_valid[drs1[d]] && completed(dta[d])) begin
                da[d]=completion_value(dta[d]); dra[d]=1;
            end
            if(PIPELINED!=0 && source_used_b[d] && drs2[d]!=0 && rat_valid[drs2[d]] && completed(dtb[d])) begin
                db[d]=completion_value(dtb[d]); drb[d]=1;
            end
            if(d==1 && dwe[0] && dd[0]!=0) begin
                if(source_used_a[1] && drs1[1]==dd[0]) begin dra[1]=0; dta[1]=tail; end
                if(source_used_b[1] && drs2[1]==dd[0]) begin drb[1]=0; dtb[1]=tail; end
            end
        end
    end
    // Rotate readiness bits, not sixteen wide instruction payloads. The
    // priority encoder and payload mux now sit in distinct logic cones.
    function [3:0] first_bit;
        input [15:0] bits;
        integer n;
        begin
            first_bit=0;
            for(n=15;n>=0;n=n-1) if(bits[n]) first_bit=n[3:0];
        end
    endfunction
    reg divbusy=0;
    reg [3:0] memory_queue[0:15];
    reg [3:0] memory_head=0,memory_tail=0;
    reg [4:0] memory_count=0;
    wire memory_push0=dispatch && (dl[0] || ds[0]) && !dbad[0];
    wire memory_push1=dispatch && dv2 && (dl[1] || ds[1]) && !dbad[1];
    wire [1:0] memory_pushes={1'b0,memory_push0}+{1'b0,memory_push1};
    wire memory_pop=memory_count!=0 && (directload || (retire && hstore) || debug_timer_valid);
    wire [3:0] memory_head1=memory_head+1'b1;
    wire [3:0] memory_tail1=memory_tail+1'b1;
    // Registered memory ordering deliberately leaves a launch gap. Looking
    // through this cycle's load address/pop joins RAM forwarding, address
    // validation, FIFO selection and issue priority in one routed path.
    wire [3:0] next_memory=memory_queue[memory_head];
    wire next_memory_valid=memory_count!=0;
    reg [1:0] grant_valid=0;
    reg [3:0] grant0=0,grant1=0;
    wire [15:0] source_ready_a,source_ready_b;
    generate for(g=0;g<16;g=g+1) begin: ready_sources
        if(PIPELINED==0) begin: legacy
            assign source_ready_a[g]=ra[g]; assign source_ready_b[g]=rb[g];
        end else begin: anticipated
            // Anticipate ALU completion tags, but wait for a registered load
            // completion before selecting its consumers. This keeps RAM data
            // out of load-address validation and the next issue-grant cone.
            assign source_ready_a[g]=ra[g] ||
                (direct0 && ta[g]==grant0) || (direct1 && ta[g]==grant1) ||
                (lv && ta[g]==lt) || (muldone && ta[g]==multag) ||
                (mullowdone && ta[g]==mullowtag) || (divdone && ta[g]==divtag);
            assign source_ready_b[g]=rb[g] ||
                (direct0 && tb[g]==grant0) || (direct1 && tb[g]==grant1) ||
                (lv && tb[g]==lt) || (muldone && tb[g]==multag) ||
                (mullowdone && tb[g]==mullowtag) || (divdone && tb[g]==divtag);
        end
    end endgenerate
    reg [15:0] pending_memory, ready_bits, simple_bits;
    wire [15:0] rotated_mem=(pending_memory>>head) | (pending_memory<<(5'd16-{1'b0,head}));
    wire [3:0] oldest_mem=head+first_bit(rotated_mem);
    integer s;
    always @* begin
        pending_memory=0; ready_bits=0; simple_bits=0;
        for(s=0;s<16;s=s+1) begin
            pending_memory[s]=valid[s] && (kind[s]==3 || (kind[s]==2 && !issued[s]) || (kind[s]==7 && !done[s]));
            ready_bits[s]=valid[s] && !issued[s] && source_ready_a[s] && source_ready_b[s] &&
                (kind[s]!=5 || !divbusy) && (kind[s]!=7 || s[3:0]==head);
            if((kind[s]==2 || kind[s]==3 || kind[s]==7) && PIPELINED!=0 &&
                (!next_memory_valid || next_memory!=s[3:0])) ready_bits[s]=0;
            if((kind[s]==2 || kind[s]==3 || kind[s]==7) && ev[0] && counter && et[0]!=head) ready_bits[s]=0;
            simple_bits[s]=ready_bits[s] && kind[s]==0;
        end
    end
    wire [15:0] memory_mask=16'b1<<oldest_mem;
    wire [15:0] eligible_bits=PIPELINED!=0 ? ready_bits : ready_bits & (~pending_memory | memory_mask);
    wire [15:0] rotated_ready=(eligible_bits>>head) | (eligible_bits<<(5'd16-{1'b0,head}));
    wire [15:0] special_bits=eligible_bits & ~simple_bits;
    wire [15:0] rotated_special=(special_bits>>head) | (special_bits<<(5'd16-{1'b0,head}));
    wire has_special=PIPELINED!=0 && |special_bits;
    wire next_pick0=PIPELINED!=0 ? |eligible_bits : |rotated_ready;
    wire [3:0] next_sel0=PIPELINED!=0 ? first_bit(has_special ? special_bits : eligible_bits) :
        head+first_bit(rotated_ready);
    wire [15:0] second_bits=simple_bits & ~(16'b1<<next_sel0);
    wire [15:0] rotated_second=(second_bits>>head) | (second_bits<<(5'd16-{1'b0,head}));
    wire next_pick1=PIPELINED!=0 ? |second_bits : |rotated_second;
    wire [3:0] next_sel1=PIPELINED!=0 ? first_bit(second_bits) : head+first_bit(rotated_second);
    wire [3:0] sel0=PIPELINED!=0 ? grant0 : next_sel0;
    wire [3:0] sel1=PIPELINED!=0 ? grant1 : next_sel1;
    wire issue0=resetn && !halted && !flush && (PIPELINED!=0 ? grant_valid[0] : next_pick0);
    wire issue1=resetn && !halted && !flush && (PIPELINED!=0 ? grant_valid[1] : next_pick1);
    wire direct0=issue0 && (kind[sel0]==0 || kind[sel0]==1);
    wire direct1=issue1;
    wire [31:0] current_a[0:1],current_b[0:1];
    wire [31:0] selected_addr=current_a[0]+imm[sel0];
    wire selected_align=((op[sel0]==`ALU_LW || op[sel0]==`ALU_SW) && selected_addr[1:0]!=0) ||
        ((op[sel0]==`ALU_LH || op[sel0]==`ALU_LHU || op[sel0]==`ALU_SH) && selected_addr[1:0]==3);
    wire directload=issue0 && kind[sel0]==2 && selected_addr[31:16]==1 && !selected_align;
    wire directstore=issue0 && kind[sel0]==3;
    wire storefault=selected_align || !(selected_addr[31:16]==1 || selected_addr==`UART_ADDR);
    wire [3:0] selected[0:1];
    assign selected[0]=sel0; assign selected[1]=sel1;
    wire [31:0] direct_result[0:1];
    wire direct_branch[0:1];
    generate for(g=0;g<2;g=g+1) begin: direct_alus
        wire [3:0] q=selected[g];
        assign current_a[g]=va[q];
        assign current_b[g]=vb[q];
        wire [31:0] a=PIPELINED!=0 ? current_a[g] : ot1[q]==`OP_TYPE_REG ? va[q] : ot1[q]==`OP_TYPE_IMM ? imm[q] : 0;
        wire [31:0] b=PIPELINED!=0 ? current_b[g] : ot2[q]==`OP_TYPE_REG ? vb[q] : ot2[q]==`OP_TYPE_IMM ? imm[q] : ot2[q]==`OP_TYPE_PC ? qp[q] : 0;
        alu arithmetic(op[q],a,b,direct_result[g],direct_branch[g]);
    end endgenerate
    // Branch operands bypass the general ALU type/result multiplexers. Direct
    // target addition belongs to dispatch, outside the selection/compare path.
    wire [31:0] branch_a=current_a[0], branch_b=current_b[0];
    wire less_low=branch_a[30:0]<branch_b[30:0];
    wire less_signed=branch_a[31]!=branch_b[31] ? branch_a[31] : less_low;
    wire less_unsigned=branch_a[31]!=branch_b[31] ? branch_b[31] : less_low;
    reg direct_taken;
    always @* case(qi[sel0][14:12])
        0: direct_taken=branch_a==branch_b;
        1: direct_taken=branch_a!=branch_b;
        4: direct_taken=less_signed;
        5: direct_taken=!less_signed;
        6: direct_taken=less_unsigned;
        7: direct_taken=!less_unsigned;
        default: direct_taken=0;
    endcase
    wire [31:0] direct_target=qi[sel0][6:0]==7'h67 ? (selected_addr&32'hfffffffe) :
        (qi[sel0][6:0]!=7'h63 || direct_taken) ? branch_target[sel0] : fallthrough[sel0];
    reg [1:0] ev=0;
    reg [3:0] et[0:1];
    reg [31:0] ea[0:1],eb[0:1],ei[0:1],ep[0:1],eimm[0:1];
    reg [5:0] eop[0:1];
    reg [1:0] eot1[0:1],eot2[0:1];
    reg [2:0] ek[0:1];
    wire [31:0] eres[0:1];
    wire ebr[0:1];
    generate for(g=0;g<2;g=g+1) begin: alus
        wire [31:0] a=eot1[g]==`OP_TYPE_REG ? ea[g] : eot1[g]==`OP_TYPE_IMM ? eimm[g] : 0;
        wire [31:0] b=eot2[g]==`OP_TYPE_REG ? eb[g] : eot2[g]==`OP_TYPE_IMM ? eimm[g] : eot2[g]==`OP_TYPE_PC ? ep[g] : 0;
        alu arith(eop[g],a,b,eres[g],ebr[g]);
    end endgenerate
    wire [31:0] eaddr=ea[0]+eimm[0];
    wire counter=(ek[0]==2 || ek[0]==7) && eaddr==`HARDWARE_COUNTER_ADDR && eop[0]==`ALU_LW;
    wire erange=eaddr[31:16]==1;
    wire euart=ek[0]==3 && eaddr==`UART_ADDR;
    wire ealign=((eop[0]==`ALU_LW || eop[0]==`ALU_SW) && eaddr[1:0]!=0) ||
        ((eop[0]==`ALU_LH || eop[0]==`ALU_LHU || eop[0]==`ALU_SH) && eaddr[1:0]==3);
    wire efault=ealign || !(erange || counter || euart);
    wire [31:0] enext=ei[0][6:0]==7'h67 ? (eaddr & 32'hfffffffe) : ebr[0] ? ep[0]+eimm[0] : ep[0]+4;
    reg lv=0;
    reg [3:0] lt;
    reg [5:0] lop;
    reg [1:0] loff;
    reg [31:0] ldata;
    wire [31:0] shifted=ldata >> {loff,3'b0};
    reg [31:0] loaded;
    always @* case(lop)
        `ALU_LB: loaded={{24{shifted[7]}},shifted[7:0]};
        `ALU_LBU: loaded={24'b0,shifted[7:0]};
        `ALU_LH: loaded={{16{shifted[15]}},shifted[15:0]};
        `ALU_LHU: loaded={16'b0,shifted[15:0]};
        default: loaded=shifted;
    endcase
    wire muldone, mullowdone;
    wire [3:0] mullowtag;
    wire [31:0] mullowresult;
    wire [3:0] multag;
    wire [31:0] mulresult;
    out_multiplier multiplier(clk,resetn && !flush,ev[0] && ek[0]==4 && !flush,et[0],ei[0][14:12],ea[0],eb[0],muldone,multag,mulresult,mullowdone,mullowtag,mullowresult);
    wire divdone;
    wire [31:0] divresult;
    reg [3:0] divtag;
    muldiv divider(clk,resetn && !flush,ev[0] && ek[0]==5 && !flush,PIPELINED!=0 ? {1'b1,ei[0][13:12]} : ei[0][14:12],ea[0],eb[0],divdone,divresult);
    wire [3:0] mask=(op[head]==`ALU_SB ? 4'b1 : op[head]==`ALU_SH ? 4'b11 : 4'b1111)<<addr[head][1:0];
    wire [31:0] storedata=storeval[head] << {addr[head][1:0],3'b0};
    wire ram_read_enable=directload || (ev[0] && ek[0]==2 && erange && !efault && !flush);
    wire [13:0] ram_read_addr=directload ? selected_addr[15:2] : eaddr[15:2];
    always @(posedge clk) begin
        if(retire && hstore && addr[head][31:16]==1) begin
            if(mask[0]) dmem[addr[head][15:2]][7:0]<=storedata[7:0];
            if(mask[1]) dmem[addr[head][15:2]][15:8]<=storedata[15:8];
            if(mask[2]) dmem[addr[head][15:2]][23:16]<=storedata[23:16];
            if(mask[3]) dmem[addr[head][15:2]][31:24]<=storedata[31:24];
        end
        // One syntactic read port: two array-read expressions imply three
        // memory ports with the store port, preventing block RAM inference.
        if(ram_read_enable) ldata<=dmem[ram_read_addr];
    end
    function completed;
        input [3:0] tag;
        begin completed=(direct0 && sel0==tag) || (direct1 && sel1==tag) || (lv && lt==tag) ||
            (muldone && multag==tag) || (mullowdone && mullowtag==tag) || (divdone && divtag==tag) ||
            (ev[0] && et[0]==tag && ek[0]!=4 && ek[0]!=5 && !(ek[0]==2 && erange && !efault) && !(counter && et[0]!=head)); end
    endfunction
    function [31:0] completion_value;
        input [3:0] tag;
        begin completion_value=({32{direct0 && sel0==tag}} & direct_result[0]) |
            ({32{direct1 && sel1==tag}} & direct_result[1]) | ({32{lv && lt==tag}} & loaded) |
            ({32{muldone && multag==tag}} & mulresult) | ({32{mullowdone && mullowtag==tag}} & mullowresult) |
            ({32{divdone && divtag==tag}} & divresult) |
            ({32{ev[0] && et[0]==tag && ek[0]!=4 && ek[0]!=5 && !(ek[0]==2 && erange && !efault) && !(counter && et[0]!=head)}} & (counter ? cycles : eres[0])); end
    endfunction
    generate if(PIPELINED!=0) begin: committed_banks
        // Each bank has one write port; latest-writer bits preserve two-wide
        // WAW ordering. Known bits reset architectural reads without clearing
        // the memories, so Vivado can infer distributed RAM.
        always @(posedge clk) if(retire && rd[head]!=0) arf0[rd[head]]<=value[head];
        always @(posedge clk) if(retire2 && rd[h1]!=0) arf1[rd[h1]]<=value[h1];
        always @(posedge clk) begin
            if(!resetn) arf_known<=0;
            else begin
                if(retire && rd[head]!=0) begin arf_known[rd[head]]<=1; arf_latest[rd[head]]<=0; end
                if(retire2 && rd[h1]!=0) begin arf_known[rd[h1]]<=1; arf_latest[rd[h1]]<=1; end
            end
        end
    end endgenerate
    wire [15:0] writeback_enable;
    wire [31:0] writeback_data[0:15];
    generate for(g=0;g<16;g=g+1) begin: result_ports
        localparam [3:0] INDEX=g;
        wire c0=direct0 && sel0==INDEX;
        wire c1=direct1 && sel1==INDEX;
        wire cl=lv && lt==INDEX;
        wire cm=muldone && multag==INDEX;
        wire cml=mullowdone && mullowtag==INDEX;
        wire cd=divdone && divtag==INDEX;
        wire ce=ev[0] && et[0]==INDEX && ek[0]!=4 && ek[0]!=5 &&
            !(ek[0]==2 && erange && !efault) && !(counter && et[0]!=head);
        assign writeback_enable[g]=c0 || c1 || cl || cm || cml || cd || ce;
        assign writeback_data[g]=({32{c0}} & direct_result[0]) | ({32{c1}} & direct_result[1]) |
            ({32{cl}} & loaded) | ({32{cm}} & mulresult) | ({32{cml}} & mullowresult) |
            ({32{cd}} & divresult) | ({32{ce}} & (counter ? cycles : eres[0]));
`ifdef VERILATOR
        wire [3:0] writers={3'b0,c0}+{3'b0,c1}+{3'b0,cl}+{3'b0,cm}+{3'b0,cml}+{3'b0,cd}+{3'b0,ce};
        always @(posedge clk) if(PIPELINED!=0 && resetn && !halted && !flush && writers>1)
            $fatal(1,"Multiple completions for one live ROB tag");
`endif
    end endgenerate
    reg [31:0] last_pc=32'h8000,last_i=32'h13,fault_pc=0,fault_i=0;
    reg [31:0] last_result=0;
    integer j,k;
    reg [3:0] idx;
    always @(posedge clk) begin
        if(!resetn) begin
            valid<=0; done<=0; issued<=0; fault<=0; ra<=0; rb<=0;
            rat_valid<=0; head<=0; tail<=0; count<=0; dv<=0; dv2<=0;
            fpc<=32'h8000; bv<=0; halted<=0; cycles<=0; ev<=0; lv<=0; divbusy<=0;
            last_pc<=32'h8000; grant_valid<=0; memory_head<=0; memory_tail<=0; memory_count<=0;
            if(PIPELINED==0) for(j=0;j<32;j=j+1) arf[j]<=0;
        end else if(!halted) begin
            cycles<=cycles+1'b1;
            if(PIPELINED!=0) for(j=0;j<16;j=j+1) if(writeback_enable[j]) value[j]<=writeback_data[j];
            if(retire) begin
                last_pc<=qp[head]; last_i<=qi[head]; last_result<=value[head];
                valid[head]<=0;
                if(rd[head]!=0) begin
                    if(PIPELINED==0) arf[rd[head]]<=value[head];
                    if(rat_valid[rd[head]] && rat[rd[head]]==head) rat_valid[rd[head]]<=0;
                end
                if(kind[head]==1) begin
                    bv[qp[head][7:2]]<=1; btag[qp[head][7:2]]<=qp[head][31:8];
                    if(actual[head]!=qp[head]+4) btarget[qp[head][7:2]]<=actual[head];
                    if(!bv[qp[head][7:2]] || btag[qp[head][7:2]]!=qp[head][31:8])
                        bcount[qp[head][7:2]]<=actual[head]!=qp[head]+4 ? 2 : 1;
                    else if(actual[head]!=qp[head]+4 && bcount[qp[head][7:2]]!=3) bcount[qp[head][7:2]]<=bcount[qp[head][7:2]]+1'b1;
                    else if(actual[head]==qp[head]+4 && bcount[qp[head][7:2]]!=0) bcount[qp[head][7:2]]<=bcount[qp[head][7:2]]-1'b1;
                end
            end
            if(retire2) begin
                valid[h1]<=0;
                if(rd[h1]!=0) begin
                    if(PIPELINED==0) arf[rd[h1]]<=value[h1];
                    if(rat_valid[rd[h1]] && rat[rd[h1]]==h1) rat_valid[rd[h1]]<=0;
                end
            end
            if(flush) begin
                valid<=0; done<=0; issued<=0; fault<=0; rat_valid<=0;
                head<=0; tail<=0; count<=0; dv<=0; ev<=0; lv<=0; divbusy<=0;
                fpc<=actual[head]; grant_valid<=0; memory_head<=0; memory_tail<=0; memory_count<=0;
                if(trap) begin halted<=1; fault_pc<=qp[head]; fault_i<=qi[head]; end
            end else begin
                if(PIPELINED!=0) begin
                    memory_count<=memory_count+{3'b0,memory_pushes}-{4'b0,memory_pop};
                    memory_head<=memory_head+{3'b0,memory_pop};
                    memory_tail<=memory_tail+{2'b0,memory_pushes};
                    if(memory_pushes!=0) memory_queue[memory_tail]<=memory_push0 ? tail : t1;
                    if(memory_push0 && memory_push1) memory_queue[memory_tail1]<=t1;
                    grant_valid<={next_pick1,next_pick0}; grant0<=next_sel0; grant1<=next_sel1;
                    if(next_pick0) begin issued[next_sel0]<=1; if(kind[next_sel0]==5) divbusy<=1; end
                    if(next_pick1) issued[next_sel1]<=1;
                end
                count<=count+{3'b0,nd}-{3'b0,nr}; head<=head+{2'b0,nr}; tail<=tail+{2'b0,nd};
                // Physical-result readiness is registered, separating selection
                // and ALU execution from the associative wakeup path.
                if(PIPELINED!=0) begin
                    for(j=0;j<16;j=j+1) begin
                        if(valid[j] && !ra[j] && completed(ta[j])) begin va[j]<=completion_value(ta[j]); ra[j]<=1; end
                        if(valid[j] && !rb[j] && completed(tb[j])) begin vb[j]<=completion_value(tb[j]); rb[j]<=1; end
                    end
                end else begin
                for(j=0;j<16;j=j+1) begin
                    if(valid[j] && !ra[j] && done[ta[j]]) begin va[j]<=value[ta[j]]; ra[j]<=1; end
                    if(valid[j] && !rb[j] && done[tb[j]]) begin vb[j]<=value[tb[j]]; rb[j]<=1; end
                    if(valid[j] && !issued[j]) begin
                        for(k=0;k<2;k=k+1) if(ev[k] && (ek[k]==0 || ek[k]==1 || ek[k]==3 || (k==0 && counter && et[0]==head && !efault))) begin
                            if(!ra[j] && ta[j]==et[k]) begin va[j]<=k==0 && counter ? cycles : eres[k]; ra[j]<=1; end
                            if(!rb[j] && tb[j]==et[k]) begin vb[j]<=k==0 && counter ? cycles : eres[k]; rb[j]<=1; end
                        end
                        for(k=0;k<2;k=k+1) if(k==0 ? direct0 : direct1) begin
                            if(!ra[j] && ta[j]==selected[k]) begin va[j]<=direct_result[k]; ra[j]<=1; end
                            if(!rb[j] && tb[j]==selected[k]) begin vb[j]<=direct_result[k]; rb[j]<=1; end
                        end
                        if(lv) begin
                            if(!ra[j] && ta[j]==lt) begin va[j]<=loaded; ra[j]<=1; end
                            if(!rb[j] && tb[j]==lt) begin vb[j]<=loaded; rb[j]<=1; end
                        end
                        if(muldone) begin
                            if(!ra[j] && ta[j]==multag) begin va[j]<=mulresult; ra[j]<=1; end
                            if(!rb[j] && tb[j]==multag) begin vb[j]<=mulresult; rb[j]<=1; end
                        end
                        if(mullowdone) begin
                            if(!ra[j] && ta[j]==mullowtag) begin va[j]<=mullowresult; ra[j]<=1; end
                            if(!rb[j] && tb[j]==mullowtag) begin vb[j]<=mullowresult; rb[j]<=1; end
                        end
                        if(divdone) begin
                            if(!ra[j] && ta[j]==divtag) begin va[j]<=divresult; ra[j]<=1; end
                            if(!rb[j] && tb[j]==divtag) begin vb[j]<=divresult; rb[j]<=1; end
                        end
                    end
                end
                end
                if(front) begin
                    di[0]<=imem[fpc[14:2]]; di[1]<=imem[fp4[14:2]];
                    dp<=fpc; dn0<=fn0; dn1<=fn1; dv<=1; dv2<=!pred0 && fpc<32'hfffc;
                    fpc<=pred0 || fpc>=32'hfffc ? fn0 : fn1;
                end
                if(dispatch) for(k=0;k<2;k=k+1) if(k==0 || dv2) begin
                    idx=tail+k[3:0];
                    valid[idx]<=1; issued[idx]<=0; done[idx]<=0; fault[idx]<=0;
                    qi[idx]<=di[k]; qp[idx]<=dp+(k*4); qn[idx]<=k==0 ? dn0 : dn1;
                    branch_target[idx]<=dp+(k*4)+dimm[k]; fallthrough[idx]<=dp+(k*4)+4;
                    imm[idx]<=dimm[k]; op[idx]<=dop[k]; ot1[idx]<=dt1[k]; ot2[idx]<=dt2[k];
                    rd[idx]<=dwe[k] ? dd[k] : 0;
                    va[idx]<=PIPELINED!=0 ? (dt1[k]==`OP_TYPE_REG ? da[k] : dt1[k]==`OP_TYPE_IMM ? dimm[k] : 0) : da[k];
                    vb[idx]<=PIPELINED!=0 && !dl[k] && !ds[k] && !(di[k][6:0]==7'h33 && di[k][31:25]==1) ?
                        (dt2[k]==`OP_TYPE_REG ? db[k] : dt2[k]==`OP_TYPE_IMM ? dimm[k] : dt2[k]==`OP_TYPE_PC ? dp+(k*4) : 0) : db[k];
                    ra[idx]<=dra[k]; rb[idx]<=drb[k];
                    ta[idx]<=dta[k]; tb[idx]<=dtb[k];
                    if(dp+(k*4)<32'h8000 || dp+(k*4)>=32'h10000 || dp[1:0]!=0) kind[idx]<=6;
                    else if(di[k][6:0]==7'h33 && di[k][31:25]==1) begin
                        kind[idx]<=di[k][14] ? 5 : 4;
                    end else if(di[k][6:0]==7'h0f && di[k][14:12]==0) kind[idx]<=0;
                    else if(dbad[k]) kind[idx]<=6;
                    else if(dl[k]) kind[idx]<=2;
                    else if(ds[k]) kind[idx]<=3;
                    else if(di[k][6:0]==7'h63 || di[k][6:0]==7'h6f || di[k][6:0]==7'h67) kind[idx]<=1;
                    else kind[idx]<=0;
                    if(dwe[k] && dd[k]!=0) begin rat_valid[dd[k]]<=1; rat[dd[k]]<=idx; end
                end
                ev<={1'b0,issue0 && !direct0 && !directload && !directstore};
                for(k=0;k<2;k=k+1) if(k==0 ? issue0 : issue1) begin
                    idx=k==0 ? sel0 : sel1;
                    issued[idx]<=1;
                    et[k]<=idx; ea[k]<=current_a[k]; eb[k]<=current_b[k]; ei[k]<=qi[idx]; ep[k]<=qp[idx];
                    eimm[k]<=imm[idx]; eop[k]<=op[idx]; eot1[k]<=ot1[idx]; eot2[k]<=ot2[idx]; ek[k]<=kind[idx];
                    if(kind[idx]==5) divbusy<=1;
                end
                lv<=directload || (ev[0] && ek[0]==2 && erange && !efault);
                if(directload) begin lt<=sel0; lop<=op[sel0]; loff<=selected_addr[1:0]; end
                if(ev[0]) begin
                    if(ek[0]==2 && erange && !efault) begin lt<=et[0]; lop<=eop[0]; loff<=eaddr[1:0]; end
                    else if(counter && et[0]!=head) begin
                        kind[et[0]]<=7; issued[et[0]]<=0;
                    end else if(ek[0]!=4 && ek[0]!=5) begin
                        done[et[0]]<=1; if(PIPELINED==0) value[et[0]]<=counter ? cycles : eres[0];
                        addr[et[0]]<=eaddr; storeval[et[0]]<=eb[0]; actual[et[0]]<=enext;
                        fault[et[0]]<=ek[0]==6 || ((ek[0]==2 || ek[0]==3 || ek[0]==7) && efault);
                    end
                    if(ek[0]==5) divtag<=et[0];
                end
                if(ev[1]) begin done[et[1]]<=1; if(PIPELINED==0) value[et[1]]<=eres[1]; end
                if(direct0) begin
                    done[sel0]<=1; if(PIPELINED==0) value[sel0]<=direct_result[0]; actual[sel0]<=direct_target;
                    fault[sel0]<=kind[sel0]==1 && direct_target[1:0]!=0;
                end
                if(direct1) begin done[sel1]<=1; if(PIPELINED==0) value[sel1]<=direct_result[1]; end
                if(directstore) begin
                    done[sel0]<=1; fault[sel0]<=storefault;
                    addr[sel0]<=selected_addr; storeval[sel0]<=current_b[0];
                end
                if(lv) begin done[lt]<=1; if(PIPELINED==0) value[lt]<=loaded; end
                if(muldone) begin done[multag]<=1; if(PIPELINED==0) value[multag]<=mulresult; end
                if(mullowdone) begin done[mullowtag]<=1; if(PIPELINED==0) value[mullowtag]<=mullowresult; end
                if(divdone) begin done[divtag]<=1; if(PIPELINED==0) value[divtag]<=divresult; divbusy<=0; end
            end
        end
    end
    uart_tx #(.CLOCK_HZ(CLOCK_HZ)) serial(clk,resetn,retire && huart,storeval[head][7:0],uart_ready,uart_tx);
    assign debug_pc=halted ? fault_pc : retire ? qp[head] : last_pc;
    assign debug_insn=halted ? fault_i : retire ? qi[head] : last_i;
    assign debug_wdata=retire ? value[head] : last_result;
    assign debug_commit=retire;
    assign debug_commit2=retire2;
    assign debug_we=retire && rd[head]!=0;
    assign debug_rd=rd[head];
    assign debug_reg_data=PIPELINED!=0 ? committed_value(debug_reg_addr) : debug_reg_addr==0 ? 0 : arf[debug_reg_addr];
    assign debug_uart_valid=retire && huart;
    assign debug_uart_data=storeval[head][7:0];
    assign debug_timer_valid=resetn && !halted && !flush && ev[0] && counter && et[0]==head;
    // blocked head on load/mul/div; redirect; empty window; UART; window full;
    // any source-dependent unissued work. Predicates may overlap.
    wire [15:0] waiting=valid & ~issued & ~(ra & rb);
    assign profile_flags={|waiting,count>=15,retire==0 && huart && count!=0,
        count==0,redirect,count!=0 && !done[head] && kind[head]==5,
        count!=0 && !done[head] && kind[head]==4,count!=0 && !done[head] && kind[head]==2};
    assign profile_issued={1'b0,issue0}+{1'b0,issue1};
endmodule

// Fully pipelined four-product multiply: accepts one operation every cycle.
// Tags/valids follow products; signed-high corrections are captured per op.
module out_multiplier(input wire clk,resetn,start,input wire [3:0] tag,
    input wire [2:0] op,input wire [31:0] a,b,
    output wire done,output wire [3:0] result_tag,output wire [31:0] result,
    output wire low_done,output wire [3:0] low_tag,output wire [31:0] low_result);
    reg [3:0] v=0;
    reg [3:0] tags[0:3];
    reg [2:0] ops[0:3];
    reg [31:0] ca[0:3],cb[0:3];
    (* use_dsp="yes" *) reg [31:0] p00,p01,p10,p11;
    reg [32:0] cross_sum;
    reg [31:0] low,high;
    reg [63:0] product;
    reg [31:0] final_result, low_result_reg;
    wire [15:0] low_high=p00[31:16]+p01[15:0]+p10[15:0];
    integer n;
    always @(posedge clk) begin
        if(!resetn) v<=0;
        else v<={v[2:0],start};
        tags[0]<=tag; ops[0]<=op;
        ca[0]<=(op==1 || op==2) && a[31] ? b : 0;
        cb[0]<=op==1 && b[31] ? a : 0;
        p00<=a[15:0]*b[15:0]; p01<=a[15:0]*b[31:16];
        p10<=a[31:16]*b[15:0]; p11<=a[31:16]*b[31:16];
        low_result_reg<={low_high,p00[15:0]};
        cross_sum<={1'b0,p01}+{1'b0,p10}; low<=p00; high<=p11;
        product<={32'b0,low}+{15'b0,cross_sum,16'b0}+{high,32'b0};
        final_result<=ops[2]==0 ? product[31:0] : product[63:32]-ca[2]-cb[2];
        for(n=1;n<4;n=n+1) begin tags[n]<=tags[n-1]; ops[n]<=ops[n-1]; ca[n]<=ca[n-1]; cb[n]<=cb[n-1]; end
    end
    assign low_done=v[1] && ops[1]==0; assign low_tag=tags[1]; assign low_result=low_result_reg;
    assign done=v[3] && ops[3]!=0; assign result_tag=tags[3]; assign result=final_result;
endmodule
