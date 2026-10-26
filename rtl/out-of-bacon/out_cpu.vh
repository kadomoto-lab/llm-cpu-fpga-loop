// Out-of-Bacon: 16-entry renamed scheduling window and precise two-wide ROB.
// ROB slots are speculative physical registers; committed values live in ARF.
// RAM loads are ordered, stores/MMIO occur only at retirement, flush restores RAT.
module out_cpu #(parameter CODE_FILE="code.hex", DATA_FILE="data.hex",
    parameter integer CLOCK_HZ=100000000)(
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
        for(i=0;i<32;i=i+1) begin arf[i]=0; rat[i]=0; end
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
    genvar g;
    generate for(g=0;g<2;g=g+1) begin: decoders
        decode dec(di[g],ds1[g],ds2[g],dd[g],dimm[g],dop[g],dt1[g],dt2[g],dwe[g],dl[g],ds[g],dbad[g]);
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
            dta[d]=rat[ds1[d]]; dtb[d]=rat[ds2[d]];
            dra[d]=!rat_valid[ds1[d]] || ds1[d]==0 || done[dta[d]];
            drb[d]=!rat_valid[ds2[d]] || ds2[d]==0 || done[dtb[d]];
            da[d]=ds1[d]==0 ? 0 : rat_valid[ds1[d]] ? value[dta[d]] : arf[ds1[d]];
            db[d]=ds2[d]==0 ? 0 : rat_valid[ds2[d]] ? value[dtb[d]] : arf[ds2[d]];
            if(d==1 && dwe[0] && dd[0]!=0) begin
                if(ds1[1]==dd[0]) begin dra[1]=0; dta[1]=tail; end
                if(ds2[1]==dd[0]) begin drb[1]=0; dtb[1]=tail; end
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
    reg [15:0] pending_memory, ready_bits, simple_bits;
    wire [15:0] rotated_mem=(pending_memory>>head) | (pending_memory<<(5'd16-{1'b0,head}));
    wire [3:0] oldest_mem=head+first_bit(rotated_mem);
    integer s;
    always @* begin
        pending_memory=0; ready_bits=0; simple_bits=0;
        for(s=0;s<16;s=s+1) begin
            pending_memory[s]=valid[s] && (kind[s]==3 || (kind[s]==2 && !issued[s]) || (kind[s]==7 && !done[s]));
            ready_bits[s]=valid[s] && !issued[s] && ra[s] && rb[s] &&
                (kind[s]!=5 || !divbusy) && (kind[s]!=7 || s[3:0]==head);
            if((kind[s]==2 || kind[s]==3 || kind[s]==7) && ev[0] && counter && et[0]!=head) ready_bits[s]=0;
            simple_bits[s]=ready_bits[s] && kind[s]==0;
        end
    end
    wire [15:0] memory_mask=16'b1<<oldest_mem;
    wire [15:0] eligible_bits=ready_bits & (~pending_memory | memory_mask);
    wire [15:0] rotated_ready=(eligible_bits>>head) | (eligible_bits<<(5'd16-{1'b0,head}));
    wire pick0=|rotated_ready;
    wire [3:0] sel0=head+first_bit(rotated_ready);
    wire [15:0] second_bits=simple_bits & ~(16'b1<<sel0);
    wire [15:0] rotated_second=(second_bits>>head) | (second_bits<<(5'd16-{1'b0,head}));
    wire pick1=|rotated_second;
    wire [3:0] sel1=head+first_bit(rotated_second);
    wire issue0=resetn && !halted && !flush && pick0;
    wire issue1=resetn && !halted && !flush && pick1;
    wire direct0=issue0 && (kind[sel0]==0 || kind[sel0]==1);
    wire direct1=issue1;
    wire [31:0] selected_addr=va[sel0]+imm[sel0];
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
        wire [31:0] a=ot1[q]==`OP_TYPE_REG ? va[q] : ot1[q]==`OP_TYPE_IMM ? imm[q] : 0;
        wire [31:0] b=ot2[q]==`OP_TYPE_REG ? vb[q] : ot2[q]==`OP_TYPE_IMM ? imm[q] : ot2[q]==`OP_TYPE_PC ? qp[q] : 0;
        alu arithmetic(op[q],a,b,direct_result[g],direct_branch[g]);
    end endgenerate
    // Branch operands bypass the general ALU type/result multiplexers. Direct
    // target addition belongs to dispatch, outside the selection/compare path.
    wire [31:0] branch_a=va[sel0], branch_b=vb[sel0];
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
    muldiv divider(clk,resetn && !flush,ev[0] && ek[0]==5 && !flush,ei[0][14:12],ea[0],eb[0],divdone,divresult);
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
    reg [31:0] last_pc=32'h8000,last_i=32'h13,fault_pc=0,fault_i=0;
    reg [31:0] last_result=0;
    integer j,k;
    reg [3:0] idx;
    always @(posedge clk) begin
        if(!resetn) begin
            valid<=0; done<=0; issued<=0; fault<=0; ra<=0; rb<=0;
            rat_valid<=0; head<=0; tail<=0; count<=0; dv<=0; dv2<=0;
            fpc<=32'h8000; bv<=0; halted<=0; cycles<=0; ev<=0; lv<=0; divbusy<=0;
            last_pc<=32'h8000;
            for(j=0;j<32;j=j+1) arf[j]<=0;
        end else if(!halted) begin
            cycles<=cycles+1'b1;
            if(retire) begin
                last_pc<=qp[head]; last_i<=qi[head]; last_result<=value[head];
                valid[head]<=0;
                if(rd[head]!=0) begin
                    arf[rd[head]]<=value[head];
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
                    arf[rd[h1]]<=value[h1];
                    if(rat_valid[rd[h1]] && rat[rd[h1]]==h1) rat_valid[rd[h1]]<=0;
                end
            end
            if(flush) begin
                valid<=0; done<=0; issued<=0; fault<=0; rat_valid<=0;
                head<=0; tail<=0; count<=0; dv<=0; ev<=0; lv<=0; divbusy<=0;
                fpc<=actual[head];
                if(trap) begin halted<=1; fault_pc<=qp[head]; fault_i<=qi[head]; end
            end else begin
                count<=count+{3'b0,nd}-{3'b0,nr}; head<=head+{2'b0,nr}; tail<=tail+{2'b0,nd};
                // Physical-result readiness is registered, separating selection
                // and ALU execution from the associative wakeup path.
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
                    va[idx]<=da[k]; vb[idx]<=db[k]; ra[idx]<=dra[k]; rb[idx]<=drb[k];
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
                    et[k]<=idx; ea[k]<=va[idx]; eb[k]<=vb[idx]; ei[k]<=qi[idx]; ep[k]<=qp[idx];
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
                        done[et[0]]<=1; value[et[0]]<=counter ? cycles : eres[0];
                        addr[et[0]]<=eaddr; storeval[et[0]]<=eb[0]; actual[et[0]]<=enext;
                        fault[et[0]]<=ek[0]==6 || ((ek[0]==2 || ek[0]==3 || ek[0]==7) && efault);
                    end
                    if(ek[0]==5) divtag<=et[0];
                end
                if(ev[1]) begin done[et[1]]<=1; value[et[1]]<=eres[1]; end
                if(direct0) begin
                    done[sel0]<=1; value[sel0]<=direct_result[0]; actual[sel0]<=direct_target;
                    fault[sel0]<=kind[sel0]==1 && direct_target[1:0]!=0;
                end
                if(direct1) begin done[sel1]<=1; value[sel1]<=direct_result[1]; end
                if(directstore) begin
                    done[sel0]<=1; fault[sel0]<=storefault;
                    addr[sel0]<=selected_addr; storeval[sel0]<=vb[sel0];
                end
                if(lv) begin done[lt]<=1; value[lt]<=loaded; end
                if(muldone) begin done[multag]<=1; value[multag]<=mulresult; end
                if(mullowdone) begin done[mullowtag]<=1; value[mullowtag]<=mullowresult; end
                if(divdone) begin done[divtag]<=1; value[divtag]<=divresult; divbusy<=0; end
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
    assign debug_reg_data=debug_reg_addr==0 ? 0 : arf[debug_reg_addr];
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
