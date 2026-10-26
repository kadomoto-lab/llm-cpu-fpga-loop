// One- or two-issue fetch/decode/execute/memory/writeback pipeline. Synchronous
// block RAM, registered load forwarding, and one-cycle load/use interlocks.
// Stores retire in memory; same-edge store/load collisions bypass byte lanes.
module fast_cpu #(parameter CODE_FILE="code.hex", DATA_FILE="data.hex",
    parameter integer CLOCK_HZ=100000000, SUPER=0)(
    input wire clk, resetn, output wire uart_tx, output reg halted=0,
    output wire [31:0] debug_pc, debug_insn, debug_wdata,
    output wire debug_commit, debug_commit2, debug_we,
    output wire [4:0] debug_rd,
    input wire [4:0] debug_reg_addr, output wire [31:0] debug_reg_data,
    output wire debug_uart_valid, output wire [7:0] debug_uart_data,
    output reg [31:0] cycles=0
);
    (* rom_style="block" *) reg [31:0] imem[0:8191];
    (* ram_style="block" *) reg [31:0] dmem[0:16383];
    (* ram_style="distributed" *) reg [31:0] regs[0:31];
    integer i;
    initial begin
        for(i=0;i<8192;i=i+1) imem[i]=32'h13;
        for(i=0;i<16384;i=i+1) dmem[i]=0;
        for(i=0;i<32;i=i+1) regs[i]=0;
        $readmemh(CODE_FILE,imem); $readmemh(DATA_FILE,dmem);
    end
    reg [31:0] fpc=32'h8000;
    reg dv=0, xv=0, mv=0, wv=0;
    reg [31:0] di=32'h13, dp=32'h8000, dn=32'h8004;
    reg [31:0] xi=32'h13, xp=32'h8000, xn=32'h8004;
    reg [31:0] xa=0, xb=0, ximm=0;
    reg [31:0] xtarget=32'h8004, xfallthrough=32'h8004;
    reg dpt=0, xpt=0;
    reg [4:0] xs1=0, xs2=0, xd=0;
    reg [5:0] xalu=0;
    reg [1:0] xt1=0, xt2=0;
    reg xwe=0, xl=0, xs=0, xbad=0, xm=0;
    reg [31:0] wp=32'h8000, wi=32'h13, wr=0;
    reg [4:0] wd=0;
    reg wwe=0;
    reg [31:0] memory_read;
    reg [31:0] mp=32'h8000, mi=32'h13, ma=0, mb=0, mr=0;
    reg [4:0] md=0;
    reg mwe=0, ml=0, ms=0, mbad=0;
    reg mcontrol=0, mtaken=0, mjalr=0;
    reg [31:0] mpred=0;
    reg [31:0] mnext=0;
    reg [5:0] mal=0;
    reg [3:0] read_bypass_mask=0;
    reg [31:0] read_bypass_data=0;
    reg [13:0] read_bypass_addr=0;
    reg read_bypass_valid=0;
    wire store_collision=read_bypass_valid && read_bypass_addr==ma[15:2];
    wire [31:0] memory_data={
        (store_collision && read_bypass_mask[3]) ? read_bypass_data[31:24] : memory_read[31:24],
        (store_collision && read_bypass_mask[2]) ? read_bypass_data[23:16] : memory_read[23:16],
        (store_collision && read_bypass_mask[1]) ? read_bypass_data[15:8] : memory_read[15:8],
        (store_collision && read_bypass_mask[0]) ? read_bypass_data[7:0] : memory_read[7:0]};
    reg [63:0] bv=0;
    (* ram_style="distributed" *) reg [23:0] tags[0:63];
    (* ram_style="distributed" *) reg [31:0] targets[0:63];
    (* ram_style="distributed" *) reg [1:0] counters[0:63];
    // Decode consumes either one instruction or an independent integer pair.
    // The second synchronous ROM port looks one word ahead. On pairing, skip
    // that word in the next fetch; predictor lookup uses the selected base.
    wire [31:0] fetch_plus4=fpc+4, fetch_plus8=fpc+8;
    wire [31:0] fetch_base=pair_decode ? fetch_plus4 : fpc;
    wire [31:0] fetch_second=pair_decode ? fetch_plus8 : fetch_plus4;
    wire [5:0] fi=fpc[7:2], fi2=fetch_plus4[7:2], ei=mp[7:2];
    // Look up both possible fetch addresses in parallel. Pair selection must
    // not precede a table read, tag comparison, and next-PC increment.
    wire predicted1=bv[fi] && tags[fi]==fpc[31:8] && counters[fi][1];
    wire predicted2=bv[fi2] && tags[fi2]==fetch_plus4[31:8] && counters[fi2][1];
    wire predicted=pair_decode ? predicted2 : predicted1;
    wire [31:0] fn1=predicted1 ? targets[fi] : fetch_plus4;
    wire [31:0] fn2=predicted2 ? targets[fi2] : fetch_plus8;
    wire [31:0] fn=pair_decode ? fn2 : fn1;
    wire [4:0] ds1,ds2,dd;
    wire [31:0] dimm;
    wire [5:0] dalu;
    wire [1:0] dt1,dt2;
    wire dwe,dl,ds,dbad;
    decode decoder(di,ds1,ds2,dd,dimm,dalu,dt1,dt2,dwe,dl,ds,dbad);
    reg [31:0] di2=32'h13;
    wire [4:0] ds12,ds22,dd2;
    wire [31:0] dimm2;
    wire [5:0] dalu2;
    wire [1:0] dt12,dt22;
    wire dwe2,dl2,ds2store,dbad2;
    decode decoder2(di2,ds12,ds22,dd2,dimm2,dalu2,dt12,dt22,dwe2,dl2,ds2store,dbad2);
    wire simple2=di2[6:0]==7'h13 || di2[6:0]==7'h37 || di2[6:0]==7'h17 ||
        (di2[6:0]==7'h33 && di2[31:25]!=1);
    wire pair_decode=SUPER!=0 && dv && !dbad && dp<32'hfffc && dp>=32'h8000 &&
        dp[1:0]==0 && di[6:0]!=7'h63 && di[6:0]!=7'h6f && di[6:0]!=7'h67 &&
        simple2 && !dbad2 && (!dwe || di[11:7]==0 ||
        (di[11:7]!=di2[11:7] &&
         (!(di2[6:0]==7'h13 || di2[6:0]==7'h33) || di[11:7]!=di2[19:15]) &&
         (di2[6:0]!=7'h33 || di[11:7]!=di2[24:20])));
    reg xv2=0,mv2=0,wv2=0;
    reg [31:0] xa2=0,xb2=0,ximm2=0,xp2=0,mr2=0,wr2=0;
    reg [4:0] xs12=0,xs22=0,xd2=0,md2=0,wd2=0;
    reg [5:0] xalu2=0;
    reg [1:0] xt12=0,xt22=0;
    reg xwe2=0,mwe2=0,wwe2=0;
    wire writing2=wv2 && wwe2 && wd2!=0;
    wire memory_forward2=mv2 && mwe2 && md2!=0;
    wire [31:0] da2=ds12==0 ? 0 : writing2 && wd2==ds12 ? wr2 : writing && wd==ds12 ? wb : regs[ds12];
    wire [31:0] db2=ds22==0 ? 0 : writing2 && wd2==ds22 ? wr2 : writing && wd==ds22 ? wb : regs[ds22];
    wire [31:0] a2=xs12==0 ? 0 : memory_forward2 && md2==xs12 ? mr2 : memory_forward && md==xs12 ? mr :
        writing2 && wd2==xs12 ? wr2 : writing && wd==xs12 ? wb : xa2;
    wire [31:0] b2=xs22==0 ? 0 : memory_forward2 && md2==xs22 ? mr2 : memory_forward && md==xs22 ? mr :
        writing2 && wd2==xs22 ? wr2 : writing && wd==xs22 ? wb : xb2;
    wire [31:0] op12=xt12==`OP_TYPE_REG ? a2 : xt12==`OP_TYPE_IMM ? ximm2 : 0;
    wire [31:0] op22=xt22==`OP_TYPE_REG ? b2 : xt22==`OP_TYPE_IMM ? ximm2 : xt22==`OP_TYPE_PC ? xp2 : 0;
    wire [31:0] result2;
    wire unused_branch2;
    alu arithmetic2(xalu2,op12,op22,result2,unused_branch2);
    wire dm=di[6:0]==7'h33 && di[31:25]==1;
    wire [31:0] shifted=memory_data >> {ma[1:0],3'b0};
    reg [31:0] loaded;
    always @* begin
        case(mal)
            `ALU_LB: loaded={{24{shifted[7]}},shifted[7:0]};
            `ALU_LBU: loaded={24'b0,shifted[7:0]};
            `ALU_LH: loaded={{16{shifted[15]}},shifted[15:0]};
            `ALU_LHU: loaded={16'b0,shifted[15:0]};
            default: loaded=shifted;
        endcase
    end
    wire [31:0] wb=wr;
    wire memory_forward=mv && mwe && !ml && md!=0;
    wire writing=wv && wwe && wd!=0;
    wire [31:0] da=ds1==0 ? 0 : writing2 && wd2==ds1 ? wr2 : writing && wd==ds1 ? wb : regs[ds1];
    wire [31:0] db=ds2==0 ? 0 : writing2 && wd2==ds2 ? wr2 : writing && wd==ds2 ? wb : regs[ds2];
    wire [31:0] a=xs1==0 ? 0 : memory_forward2 && md2==xs1 ? mr2 : memory_forward && md==xs1 ? mr : writing2 && wd2==xs1 ? wr2 : writing && wd==xs1 ? wb : xa;
    wire [31:0] b=xs2==0 ? 0 : memory_forward2 && md2==xs2 ? mr2 : memory_forward && md==xs2 ? mr : writing2 && wd2==xs2 ? wr2 : writing && wd==xs2 ? wb : xb;
    wire [31:0] op1=xt1==`OP_TYPE_REG ? a : xt1==`OP_TYPE_IMM ? ximm : 0;
    wire [31:0] op2=xt2==`OP_TYPE_REG ? b : xt2==`OP_TYPE_IMM ? ximm : xt2==`OP_TYPE_PC ? xp : 0;
    wire [31:0] result;
    wire alu_branch;
    wire branch;
    wire [31:0] address=a+ximm;
    alu arithmetic(xalu,op1,op2,result,alu_branch);
    // Branch operands are registers. Compare forwarded values directly,
    // bypassing the general ALU operand-type and operation multiplexers.
    wire less_low=a[30:0]<b[30:0];
    wire less_unsigned=a[31]!=b[31] ? b[31] : less_low;
    wire less_signed=a[31]!=b[31] ? a[31] : less_low;
    reg conditional_taken;
    always @* case(xi[14:12])
        0: conditional_taken=a==b;
        1: conditional_taken=a!=b;
        4: conditional_taken=less_signed;
        5: conditional_taken=!less_signed;
        6: conditional_taken=less_unsigned;
        7: conditional_taken=!less_unsigned;
        default: conditional_taken=0;
    endcase
    assign branch=SUPER==0 ? alu_branch : xi[6:0]==7'h63 ? conditional_taken :
        (xi[6:0]==7'h6f || xi[6:0]==7'h67);
    wire [31:0] jalr_target=(a+ximm)&32'hfffffffe;
    wire [31:0] nextpc=xi[6:0]==7'h67 ? jalr_target : branch ? xtarget : xfallthrough;
    wire control=xi[6:0]==7'h63 || xi[6:0]==7'h6f || xi[6:0]==7'h67;
    wire data_range=ma[31:16]==1;
    wire uart_store=ms && ma==`UART_ADDR;
    wire counter_load=ml && ma==`HARDWARE_COUNTER_ADDR && mal==`ALU_LW;
    wire bad_align=((mal==`ALU_LW || mal==`ALU_SW) && ma[1:0]!=0) ||
        ((mal==`ALU_LH || mal==`ALU_LHU || mal==`ALU_SH) && ma[1:0]==3);
    wire fault=mbad || ((ml || ms) && (bad_align || !(data_range || uart_store || counter_load)));
    wire mdone, uart_ready;
    wire [31:0] mresult;
    wire load_hazard=mv && ml && md!=0 && (md==xs1 || md==xs2 || (xv2 && (md==xs12 || md==xs22)));
    wire memory_stall=mv && uart_store && !uart_ready;
    fast_muldiv munit(clk,resetn,resetn && xv && xm && !mdone && !halted && !memory_stall && !load_hazard,xi[14:12],a,b,mdone,mresult);
    wire stall=xv && ((xm && !mdone) || load_hazard);
    wire memory_commit=resetn && mv && !memory_stall && !fault && !halted;
    // Resolve indirect jumps from registered targets in memory. A miss
    // cancels execute before it can enter memory or have architectural effects.
    wire memory_redirect=memory_commit && mjalr && mnext!=mpred;
    wire execute=resetn && xv && !stall && !memory_stall && !(mv && fault) && !memory_redirect && !halted;
    // ROM is immutable: tagged conditional/JAL targets are stable. Only
    // JALR needs a target comparison; branches compare direction directly.
    wire redirect=execute && ((xi[6:0]==7'h63 && branch!=xpt) || (xi[6:0]==7'h6f && !xpt));
    wire [3:0] mask=(mal==`ALU_SB ? 4'b0001 : mal==`ALU_SH ? 4'b0011 : 4'b1111) << ma[1:0];
    wire [31:0] store_data=mb << {ma[1:0],3'b0};
    always @(posedge clk) if(!memory_stall) memory_read<=dmem[address[15:2]];
    always @(posedge clk) begin
        if(memory_commit && ms && data_range) begin
            if(mask[0]) dmem[ma[15:2]][7:0]<=store_data[7:0];
            if(mask[1]) dmem[ma[15:2]][15:8]<=store_data[15:8];
            if(mask[2]) dmem[ma[15:2]][23:16]<=store_data[23:16];
            if(mask[3]) dmem[ma[15:2]][31:24]<=store_data[31:24];
        end
        if(resetn && writing2) regs[wd2]<=wr2;
        if(resetn && writing) regs[wd]<=wb;
        if(!resetn) begin
            fpc<=32'h8000; dv<=0; xv<=0; mv<=0; wv<=0; bv<=0; cycles<=0; halted<=0; xv2<=0; mv2<=0; wv2<=0;
        end else if(!halted) begin
            cycles<=cycles+1'b1;
            wv<=memory_commit;
            wv2<=memory_commit && mv2;
            if(memory_commit) begin wd2<=md2; wwe2<=mwe2; wr2<=mr2; end
            if(memory_commit) begin
                wp<=mp; wi<=mi; wd<=md; wwe<=mwe;
                wr<=ml ? (counter_load ? cycles : loaded) : mr;
            end
            if(!memory_stall) begin
                mv<=execute;
                mv2<=execute && xv2; md2<=xd2; mwe2<=xwe2; mr2<=result2;
                mp<=xp; mi<=xi; md<=xd; mwe<=xwe;
                ma<=address; mb<=b; mr<=xm ? mresult : result;
                ml<=xl; ms<=xs; mal<=xalu; mbad<=xbad;
                mcontrol<=control; mtaken<=branch; mnext<=nextpc;
                mjalr<=xi[6:0]==7'h67; mpred<=xn;
                read_bypass_mask<=mask;
                read_bypass_valid<=memory_commit && ms && data_range;
                read_bypass_addr<=ma[15:2];
                read_bypass_data<=store_data;
            end
            if(memory_commit && mcontrol) begin
                bv[ei]<=1; tags[ei]<=mp[31:8];
                if(mtaken) targets[ei]<=mnext;
                if(!bv[ei] || tags[ei]!=mp[31:8]) counters[ei]<=mtaken ? 2 : 1;
                else if(mtaken && counters[ei]!=3) counters[ei]<=counters[ei]+1'b1;
                else if(!mtaken && counters[ei]!=0) counters[ei]<=counters[ei]-1'b1;
            end
            // Payload registers need not be gated by branch resolution.
            // Valid bits cancel wrong-path instructions at the same edge.
            if(!stall && !memory_stall) begin
                di<=imem[fetch_base[14:2]]; di2<=imem[fetch_second[14:2]]; dp<=fetch_base; dn<=fn; dpt<=predicted;
                xv2<=pair_decode;
                xa2<=da2; xb2<=db2; xs12<=ds12; xs22<=ds22; xd2<=dd2;
                ximm2<=dimm2; xp2<=dp+4; xalu2<=dalu2; xt12<=dt12; xt22<=dt22; xwe2<=dwe2;
                xi<=di; xp<=dp; xn<=dn; xpt<=dpt;
                xtarget<=dp+dimm; xfallthrough<=dp+4;
                xa<=da; xb<=db; xs1<=ds1; xs2<=ds2; xd<=dd;
                ximm<=dimm; xalu<=dalu; xt1<=dt1; xt2<=dt2;
                xwe<=dwe; xl<=dl; xs<=ds; xm<=dm;
                xbad<=(dbad && !dm) || dp<32'h8000 || dp>=32'h10000 || dp[1:0]!=0;
            end else begin xa<=a; xb<=b; xa2<=a2; xb2<=b2; end
            if(mv && fault) begin halted<=1; dv<=0; xv<=0; mv<=0; end
            else if(memory_redirect) begin fpc<=mnext; dv<=0; xv<=0; end
            else if(redirect) begin fpc<=nextpc; dv<=0; xv<=0; end
            else if(!stall && !memory_stall) begin dv<=1; fpc<=fn; xv<=dv; end
        end
    end
    uart_tx #(.CLOCK_HZ(CLOCK_HZ)) serial(clk,resetn,memory_commit && uart_store,mb[7:0],uart_ready,uart_tx);
    assign debug_pc=halted ? mp : wp;
    assign debug_insn=halted ? mi : wi;
    assign debug_wdata=wb;
    assign debug_commit=resetn && wv;
    assign debug_commit2=resetn && wv2;
    assign debug_we=wwe;
    assign debug_rd=wd;
    assign debug_reg_data=debug_reg_addr==0 ? 0 : regs[debug_reg_addr];
    assign debug_uart_valid=memory_commit && uart_store;
    assign debug_uart_data=mb[7:0];
endmodule

// Four registered 16x16 DSP products. Carry propagation and signed-high
// correction occur in subsequent stages, never on the operand-capture path.
module fast_muldiv(input wire clk,resetn,start,
    input wire [2:0] operation,input wire [31:0] a,b,
    output reg done=0,output reg [31:0] result=0);
    reg [2:0] state=0, op=0;
    reg [31:0] aa=0,bb=0;
    (* use_dsp="yes" *) reg [31:0] p00,p01,p10,p11;
    reg [32:0] cross_sum;
    reg [31:0] low_partial,high_partial;
    reg [63:0] product;
    wire div_done;
    wire [31:0] div_result;
    muldiv divider(clk,resetn,state==1 && op[2],{1'b1,op[1:0]},aa,bb,div_done,div_result);
    wire [31:0] correction_a=(op==1 || op==2) && aa[31] ? bb : 0;
    wire [31:0] correction_b=op==1 && bb[31] ? aa : 0;
    // MUL only needs the low half: a 16-bit sum is sufficient. Avoid waiting
    // for the 64-bit carry and signed correction used by MULH variants.
    wire [15:0] low_high=p00[31:16]+p01[15:0]+p10[15:0];
    always @(posedge clk) begin
        done<=0;
        if(!resetn) begin state<=0; done<=0; end
        else case(state)
            0: if(start && !done) begin aa<=a; bb<=b; op<=operation; state<=1; end
            1: if(op[2]) state<=5;
               else begin
                   p00<=aa[15:0]*bb[15:0]; p01<=aa[15:0]*bb[31:16];
                   p10<=aa[31:16]*bb[15:0]; p11<=aa[31:16]*bb[31:16]; state<=2;
               end
            2: if(op==0) begin result<={low_high,p00[15:0]}; done<=1; state<=0; end
               else begin cross_sum<={1'b0,p01}+{1'b0,p10}; low_partial<=p00; high_partial<=p11; state<=3; end
            3: begin product<={32'b0,low_partial}+{15'b0,cross_sum,16'b0}+{high_partial,32'b0}; state<=4; end
            4: begin result<=op==0 ? product[31:0] : product[63:32]-correction_a-correction_b; done<=1; state<=0; end
            5: if(div_done) begin result<=div_result; done<=1; state<=0; end
            default: state<=0;
        endcase
    end
endmodule
