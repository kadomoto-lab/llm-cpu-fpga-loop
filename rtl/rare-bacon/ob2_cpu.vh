// Out-of-Bacon II (FAST=4): three-wide renamed out-of-order RV32IM core.
//
// F0  aligned 16-byte fetch block; BTB (4x64) and gshare PHT (4x1024) pick next PC
// F1  four instructions from block ROM enter a 16-entry fetch queue
// D1  decode three queue entries, intra-group dependency comparisons
// D2  rename (RAT, free list), allocate ROB/RS/SQ entries, build dependency vectors
// S   per-RS (3 ALU + 1 memory, 8 entries each) oldest-ready select. Wakeup uses a
//     dependency matrix whose columns are RS rows: one-cycle producers clear their
//     column at grant, loads one cycle later, multiplies three cycles later.
// R   64-entry physical register file read, bypass from results being written
// E   ALU/branch (ports 0-2), multiplier/divider (port 0), AGU + RAM read (port 3)
// W   register write; ROB completion; branch check and fetch redirect
// Commit up to three in order (one store, one control). Stores write RAM and
// UART only after commit. A mispredicted branch redirects fetch from W, and the
// backend is flushed once that branch commits (rename waits meanwhile).
// Loads run out of order after all older store addresses are known; a load that
// overlaps an older uncommitted store, or a counter read before reaching the ROB
// head, is replayed from its PC and its PC is remembered by a small predictor.
module ob2_cpu #(parameter CODE_FILE="code.hex", DATA_FILE="data.hex",
    parameter integer CLOCK_HZ=100000000, NA=8, NM=8, F2STAGE=0)(
    input wire clk, resetn_in, output wire uart_tx, output reg halted=0,
    output wire [31:0] debug_pc, debug_insn, debug_wdata,
    output wire debug_commit, debug_commit2, debug_we,
    output wire [4:0] debug_rd,
    input wire [4:0] debug_reg_addr, output wire [31:0] debug_reg_data,
    output wire debug_uart_valid, output wire [7:0] debug_uart_data,
    output reg [31:0] cycles=0,
    output wire debug_timer_valid, output wire [7:0] profile_flags,
    output wire [2:0] profile_issued,
    output wire [31:0] debug_pc2, debug_insn2, debug_wdata2,
    output wire [4:0] debug_rd2, output wire debug_we2,
    output wire debug_commit3, output wire [31:0] debug_pc3, debug_insn3, debug_wdata3,
    output wire [4:0] debug_rd3, output wire debug_we3, output wire debug_ooo_issue,
    output wire [31:0] debug_timer_sample);

    // Local registered reset keeps the board's reset tree out of logic paths.
    (* max_fanout=16 *) reg resetn=0;
    always @(posedge clk) resetn<=resetn_in;
    // Index widths follow the RS-geometry parameters.
    /* verilator lint_off WIDTH */
    // Reservation-station payload layout.
    localparam P_F3=0, P_BIMM=3, P_ASEL=4, P_AOP=6, P_CLS=10, P_T1=13, P_T2=19, P_U1=25, P_U2=26,
               P_DST=27, P_WE=33, P_ROB=34, P_IMM=39, P_PC=71, P_PN=103, P_GHR=135, P_SQ=145,
               P_LW=149, P_MM=150, P_FWD=151, P_FIDX=152, PW=155;
    // RS k (k<3: ALU, k=3: memory) owns rows [BASE(k), BASE(k)+SIZE(k)); rows are wakeup columns.
    localparam NR=3*NA+NM;
    localparam C_ALU=0, C_BR=1, C_JAL=2, C_JALR=3, C_LD=4, C_ST=5, C_MUL=6, C_DIV=7;
    localparam A_ADD=0, A_SUB=1, A_SLL=2, A_SLT=3, A_SLTU=4, A_XOR=5, A_SRL=6, A_SRA=7, A_OR=8, A_AND=9, A_B=10;

    (* rom_style="block" *) reg [31:0] imem[0:8191];
    (* ram_style="block" *) reg [31:0] dmem[0:16383];
    integer i, j, k, q, r;
    initial begin
        for(i=0;i<8192;i=i+1) imem[i]=32'h13;
        for(i=0;i<16384;i=i+1) dmem[i]=0;
        $readmemh(CODE_FILE,imem); $readmemh(DATA_FILE,dmem);
    end

    // Decoded instruction: {fault,nop,cls[3],rd[5],rs1[5],rs2[5],u1,u2,imm[32],aop[4],asel[2],bimm,f3[3]}
    function [63:0] dec;
        input [31:0] in; input bad;
        reg f, nop, u1, u2, bimm; reg [2:0] cls, f3; reg [4:0] rd, r1, r2;
        reg [31:0] imm; reg [3:0] aop; reg [1:0] asel;
        begin
            f=0; nop=0; cls=C_ALU; rd=0; r1=in[19:15]; r2=in[24:20]; u1=0; u2=0; imm=0;
            aop=A_ADD; asel=0; bimm=0; f3=in[14:12];
            case(in[6:0])
                7'h37: begin rd=in[11:7]; imm={in[31:12],12'b0}; aop=A_B; asel=2; bimm=1; end
                7'h17: begin rd=in[11:7]; imm={in[31:12],12'b0}; asel=1; bimm=1; end
                7'h6f: begin cls=C_JAL; rd=in[11:7]; imm={{12{in[31]}},in[19:12],in[20],in[30:21],1'b0}; end
                7'h67: begin cls=C_JALR; rd=in[11:7]; u1=1; bimm=1; imm={{20{in[31]}},in[31:20]}; if(f3!=0) f=1; end
                7'h63: begin cls=C_BR; u1=1; u2=1; imm={{20{in[31]}},in[7],in[30:25],in[11:8],1'b0};
                             if(f3==2 || f3==3) f=1; end
                7'h03: begin cls=C_LD; rd=in[11:7]; u1=1; imm={{20{in[31]}},in[31:20]};
                             if(f3==3 || f3==6 || f3==7) f=1; end
                7'h23: begin cls=C_ST; u1=1; u2=1; imm={{20{in[31]}},in[31:25],in[11:7]}; if(f3>2) f=1; end
                7'h13: begin rd=in[11:7]; u1=1; bimm=1; imm={{20{in[31]}},in[31:20]};
                    case(f3)
                        0: aop=A_ADD; 2: aop=A_SLT; 3: aop=A_SLTU; 4: aop=A_XOR; 6: aop=A_OR; 7: aop=A_AND;
                        1: begin aop=A_SLL; if(in[31:25]!=0) f=1; end
                        default: begin aop=in[30] ? A_SRA : A_SRL; if(in[31:25]!=0 && in[31:25]!=7'h20) f=1; end
                    endcase
                end
                7'h33: begin rd=in[11:7]; u1=1; u2=1;
                    if(in[31:25]==1) cls=in[14] ? C_DIV : C_MUL;
                    else begin
                        if(in[31:25]!=0 && !(in[31:25]==7'h20 && (f3==0 || f3==5))) f=1;
                        case(f3)
                            0: aop=in[30] ? A_SUB : A_ADD; 1: aop=A_SLL; 2: aop=A_SLT; 3: aop=A_SLTU;
                            4: aop=A_XOR; 5: aop=in[30] ? A_SRA : A_SRL; 6: aop=A_OR; default: aop=A_AND;
                        endcase
                    end
                end
                7'h0f: nop=1;   // FENCE: memory is ordered at commit; no instruction cache
                default: f=1;
            endcase
            if(bad) f=1;
            if(f) begin rd=0; u1=0; u2=0; cls=C_ALU; nop=0; end
            if(!f && cls==C_ALU && rd==0) nop=1;   // integer op to x0 has no effect
            if(!u1) r1=0;
            if(!u2) r2=0;
            dec={f,nop,cls,rd,r1,r2,u1,u2,imm,aop,asel,bimm,f3};
        end
    endfunction

    function integer BASE; input [1:0] k; begin BASE=k*NA; end endfunction
    function integer SIZE; input [1:0] k; begin SIZE=k<3 ? NA : NM; end endfunction
    // Slots of RS k as a 16-bit busy vector; nonexistent slots read as busy.
    function [15:0] rsbusy;
        input [NR-1:0] v; input [1:0] k; integer t;
        begin rsbusy=16'hffff; for(t=0;t<16;t=t+1) if(t<SIZE(k)) rsbusy[t]=v[BASE(k)+t]; end
    endfunction
    function [3:0] nth_free;
        input [15:0] busy; input [1:0] n;
        integer t; reg [2:0] c; reg found;
        begin
            nth_free=0; c=0; found=0;
            for(t=0;t<16;t=t+1) if(!busy[t] && !found) begin
                if(c[1:0]==n) begin nth_free=t[3:0]; found=1; end
                c=c+1;
            end
        end
    endfunction
    function [4:0] free_count;
        input [15:0] busy; integer t;
        begin free_count=0; for(t=0;t<16;t=t+1) free_count=free_count+{4'b0,!busy[t]}; end
    endfunction
    function [15:0] nth_free_oh;   // one-hot n-th free slot, zero if none
        input [15:0] busy; input [1:0] n;
        integer t; reg [2:0] c;
        begin
            nth_free_oh=0; c=0;
            for(t=0;t<16;t=t+1) if(!busy[t]) begin
                if(c=={1'b0,n}) nth_free_oh[t]=1'b1;
                c=c+1;
            end
        end
    endfunction
    function [5:0] rowof; input [1:0] k; input [3:0] sl; begin rowof=BASE(k)+sl; end endfunction
    
    // ------------------------------------------------------------------ state
    reg flush_q=0, rd_v=0, pend_v=0;
    reg [31:0] rd_pc=0;
    reg [9:0] rd_ghr=0;
    reg [4:0] pend_rob=0;

    // Frontend
    reg [31:0] fpc=32'h8000;
    reg [9:0] ghr=0;
    reg f1_v=0, f1_taken=0;
    reg [31:0] f1_pc=0;
    reg [3:0] f1_mask=0;
    reg [1:0] f1_tslot=0;
    reg [13:0] f1_tgt=0;
    reg [9:0] f1_ghr=0;
    reg [31:0] f1_w[0:3];
    // Optional F2: registered instruction-RAM output before decode and queueing.
    reg f2_v=0, f2_taken=0;
    reg [31:0] f2_pc=0;
    reg [3:0] f2_mask=0;
    reg [1:0] f2_tslot=0;
    reg [13:0] f2_tgt=0;
    reg [9:0] f2_ghr=0;
    reg [31:0] f2_w[0:3];
    wire fw_v=F2STAGE ? f2_v : f1_v, fw_taken=F2STAGE ? f2_taken : f1_taken;
    wire [31:0] fw_pc=F2STAGE ? f2_pc : f1_pc;
    wire [3:0] fw_mask=F2STAGE ? f2_mask : f1_mask;
    wire [1:0] fw_tslot=F2STAGE ? f2_tslot : f1_tslot;
    wire [13:0] fw_tgt=F2STAGE ? f2_tgt : f1_tgt;
    wire [9:0] fw_ghr=F2STAGE ? f2_ghr : f1_ghr;
    wire [31:0] fw_w[0:3];
    assign fw_w[0]=F2STAGE ? f2_w[0] : f1_w[0]; assign fw_w[1]=F2STAGE ? f2_w[1] : f1_w[1];
    assign fw_w[2]=F2STAGE ? f2_w[2] : f1_w[2]; assign fw_w[3]=F2STAGE ? f2_w[3] : f1_w[3];
    reg [31:0] fq_insn[0:15], fq_pc[0:15], fq_pn[0:15];
    reg [63:0] fq_dec[0:15];
    reg [4:0] fq_mp[0:15];
    reg [9:0] fq_ghr[0:15];
    reg [15:0] fq_bad=0;
    reg [3:0] fq_head=0, fq_tail=0;
    reg [4:0] fq_cnt=0;
    // Predictor training, registered from commit
    reg tq_btb=0, tq_pht=0, tq_tk=0;
    reg [1:0] tq_bank=0;
    reg [5:0] tq_idx=0;
    reg [9:0] tq_row=0;
    reg [20:0] tq_btbd=0;
    // Load-wait and counter predictors: one bit per ROM word, so no aliasing.
    // {counter, conflict, distance}: distance 0-6 selects the store to forward
    // from (0 = youngest older store); 7 waits for older stores to commit.
    // One bank per fetch slot, indexed by block address: one ROM word per entry.
    reg [4:0] f1_mp[0:3];

    // Decode/rename
    reg [2:0] d2_v=0, d2_fault=0, d2_nop=0, d2_u1=0, d2_u2=0, d2_bimm=0, d2_lw=0, d2_mm=0;
    reg [2:0] d2_fd[0:2];
    reg [31:0] d2_pc[0:2], d2_insn[0:2], d2_pn[0:2], d2_imm[0:2];
    reg [9:0] d2_ghr[0:2];
    reg [2:0] d2_cls[0:2], d2_f3[0:2];
    reg [4:0] d2_rd[0:2], d2_r1[0:2], d2_r2[0:2];
    reg [3:0] d2_aop[0:2];
    reg [1:0] d2_asel[0:2];
    reg [2:0] d2_m1[0:2], d2_m2[0:2], d2_wm[0:2];
    reg [5:0] rat_p[0:31], crat[0:31];
    reg [5:0] rat_c[0:31];
    reg [31:0] rat_pend=0;
    reg [5:0] fl[0:31];
    reg [5:0] fl_head=0, fl_tail=6'd32, fl_chead=0;
    reg [5:0] fl_cnt=6'd32;       // free registers available to rename (fl_tail-fl_head)
    reg [1:0] rr=0;

    // ROB
    reg [4:0] rob_head=0, rob_tail=0;
    reg [5:0] rob_cnt=0;
    reg [31:0] rob_cpl=0, rob_flt=0, rob_mis=0, rob_rep=0, rob_st=0, rob_ct=0, rob_cond=0, rob_atk=0, rob_tcnt=0;
    reg [31:0] rob_tval[0:31];   // sampled counter value (simulation trace only)
    reg [31:0] rob_pc[0:31], rob_insn[0:31];
    reg [4:0] rob_rd[0:31];
    reg [5:0] rob_pd[0:31], rob_po[0:31];
    reg [13:0] rob_atgt[0:31];
    reg [9:0] rob_ghr[0:31];
    reg [2:0] rob_cls[0:31];   // profiling/debug only

    // Reservation stations: row = rs*8+slot; rows are also wakeup columns.
    reg [NR-1:0] rs_v=0, rs_iss=0, rs_ath=0, rs_sqok=0;
    reg [NR-1:0] rs_dep[0:NR-1];
    reg [15:0] rs_age[0:NR-1];        // bit t: slot t of the same RS is older
    reg [PW-1:0] rs_pay[0:NR-1];
    reg [1:0] rs_wc[0:NR-1];
    reg [1:0] rs_k[0:NR-1];           // constant: owning RS of each row
    reg [3:0] rs_s[0:NR-1];           // constant: slot within its RS
    initial for(i=0;i<NR;i=i+1) begin
        rs_k[i]=i<NA ? 0 : i<2*NA ? 1 : i<3*NA ? 2 : 3;
        rs_s[i]=i<3*NA ? i%NA : i-3*NA;
    end
    
    // Physical registers; phys 0 is x0 and never allocated.
    // Physical register file: one LUTRAM bank per write port plus a live-value
    // table naming the bank that holds each register's latest value.
    reg [2:0] lvt[0:63];
    initial for(i=0;i<64;i=i+1) lvt[i]=0;
    wire [5:0] prf_rt[0:11];      // 8 operand read ports, 4 trace/debug ports
    wire [31:0] prf_rd[0:11];
    wire [31:0] bank_rd[0:4][0:11];

    // Issue pipelines (ports 0-3)
    reg [3:0] r_v=0, e_v=0;
    reg [PW-1:0] r_pay[0:3], e_pay[0:3];
    reg [31:0] e_a[0:3], e_b[0:3], e_tgt[0:3], e_link[0:3];
    reg [4:0] e_eba[0:3], e_ebb[0:3];
    // Results being written: 0-2 ALU, 3 load, 4 multiply/divide
    (* max_fanout=32 *) reg [4:0] res_v=0;
    (* max_fanout=32 *) reg [5:0] res_tag[0:4];
    (* max_fanout=32 *) reg [31:0] res_val[0:4];
    // ALU write-back bookkeeping
    reg [2:0] wb_v=0, wb_ct=0, wb_cond=0, wb_flt=0, wb_tk=0, wb_jalr=0, wb_brj=0, wb_ptk=0, wb_tgtbad=0;
    reg [3:0] e_ptk=0, e_tgtbad=0;
    // JALR targets need the adder result, so their check completes one cycle later.
    reg [2:0] jc_v=0, jc_mis=0, jc_flt=0;
    reg [4:0] jc_rob[0:2];
    reg [31:0] jc_pc[0:2];
    reg [9:0] jc_ghr[0:2];
    reg [4:0] wb_rob[0:2];
    reg [31:0] wb_pn[0:2], wb_sum[0:2], wb_tgt[0:2], wb_link[0:2];
    // Next PC is selected in W from registered E results (JALR sum, branch target, link).
    reg [31:0] wb_next[0:2];
    always @* for(k=0;k<3;k=k+1)
        wb_next[k]=wb_jalr[k] ? {wb_sum[k][31:1],1'b0} : wb_tk[k] ? wb_tgt[k] : wb_link[k];
    reg [11:0] e_ops[0:3];    // one-hot ALU result select, decoded in R
    reg [9:0] wb_ghr[0:2];
    // Memory pipeline
    reg [31:0] ldata=0;
    reg m2_v=0, m2_we=0, m2_cntrep=0;
    reg [31:0] m2_addr=0, st_addr=0;
    reg [2:0] st_f3=0, st_sqi=0;
    reg [5:0] m2_tag=0;
    reg [4:0] m2_rob=0;
    reg [2:0] m2_f3=0;
    reg [1:0] m2_off=0;
    reg [13:0] m2_word=0;
    reg [3:0] m2_lmask=0, m2_snap=0, m2_fmask=0;
    reg m2_fwd=0;
    reg [2:0] m2_fidx=0;
    reg [13:0] m2_fword=0;
    reg [31:0] m2_fdata=0;
    reg [7:0] lwb_ov=0, lwb_cov=0;
    reg [3:0] lwb_snap=0;
    reg [31:0] m2_pc=0;
    reg [9:0] m2_ghr=0;
    reg st_v=0;
    reg [4:0] st_rob=0;
    reg lwb_v=0, lwb_flt=0, lwb_rep=0, lwb_cnt=0, lwb_iscnt=0;
    reg [31:0] lwb_tval=0;
    reg [4:0] lwb_rob=0;
    reg [31:0] lwb_pc=0;
    reg [9:0] lwb_ghr=0;
    // Store queue (program order, written to RAM after commit)
    reg [31:0] sq_addr[0:7], sq_data[0:7];
    reg [3:0] sq_mask[0:7];
    reg [7:0] sq_av=0, sq_uart=0;
    reg [3:0] sq_head=0, sq_tail=0, sq_cnt=0;
    reg wq_v=0, sh_v=0, uq_v=0;
    reg [13:0] wq_word=0, sh_word=0;
    reg [3:0] wq_mask=0, sh_mask=0;
    reg [31:0] wq_data=0, sh_data=0;
    reg [7:0] uq_data=0;
    // Multiplier (4-cycle) and divider
    reg mq1_v=0, mq2_v=0, mq3_v=0, mq1_we=0, mq2_we=0, mq3_we=0, mq1_hi=0, mq2_hi=0, mq3_hi=0;
    reg [5:0] mq1_tag=0, mq2_tag=0, mq3_tag=0;
    reg [4:0] mq1_rob=0, mq2_rob=0, mq3_rob=0;
    reg signed [32:0] mA=0, mB=0;
    reg signed [65:0] mp1=0, mp2=0;
    reg div_fin=0;
    reg div_busy=0, div_run=0, div_negq=0, div_negr=0, div_rem=0, div_bz=0, div_we=0;
    reg [4:0] div_cnt=0, div_rob=0;
    reg [5:0] div_row=0;
    reg [5:0] div_tag=0;
    reg [31:0] dq=0, dr=0, dd=0, da0=0;
    reg mwb_v=0;
    reg [4:0] mwb_rob=0;
    wire [32:0] dr_trial={dr,dq[31]};
    wire dr_sub=dr_trial>={1'b0,dd};
    wire [32:0] dr_red=dr_sub ? dr_trial-{1'b0,dd} : dr_trial;
    wire [31:0] dr_next=dr_red[31:0];
    wire [31:0] dq_next={dq[30:0],dr_sub};

    // Commit packet: committed-map, free-list and predictor updates are applied
    // one cycle after the in-order commit decision.
    reg [2:0] cp_w=0;
    reg [4:0] cp_rd[0:2];
    reg [5:0] cp_pd[0:2], cp_po[0:2];
    // Training data for all three head slots; the committed control instruction
    // (at most one per cycle) is selected in the packet stage.
    reg [2:0] cs_ct=0, cs_atk=0, cs_cond=0;
    reg [13:0] cs_pc[0:2], cs_atgt[0:2];
    reg [9:0] cs_ghr[0:2];
    wire [1:0] cs_sel=cs_ct[0] ? 2'd0 : cs_ct[1] ? 2'd1 : 2'd2;
    wire cp_ct=|cs_ct, cp_atk=cs_atk[cs_sel], cp_cond=cs_cond[cs_sel];
    wire [31:0] cp_pc={16'b0,cs_pc[cs_sel],2'b0};
    wire [9:0] cp_ghr=cs_ghr[cs_sel];
    wire [13:0] cp_atgt=cs_atgt[cs_sel];
    wire [1:0] cp_n={1'b0,cp_w[0]}+{1'b0,cp_w[1]}+{1'b0,cp_w[2]};
    reg [5:0] crat_now[0:31];   // committed mapping including the pending packet
    integer cn, cm;
    always @* for(cn=0;cn<32;cn=cn+1) begin
        crat_now[cn]=crat[cn];
        for(cm=0;cm<3;cm=cm+1) if(cp_w[cm] && cp_rd[cm]==cn[4:0]) crat_now[cn]=cp_pd[cm];
    end
    reg [31:0] last_pc=32'h8000, last_insn=32'h13, fault_pc=0, fault_insn=0;

    // ------------------------------------------------------------------ F0
    wire f_inrom=fpc[31:15]==17'd1;
    wire [3:0] f_from=4'b1111<<fpc[3:2];
    wire [3:0] f_take;
    wire [13:0] f_tgt[0:3];
    genvar g;
    generate for(g=0;g<4;g=g+1) begin: bp
        (* ram_style="distributed" *) reg [20:0] btb[0:63];   // {valid,tag[14:10],cond,target[15:2]}
        (* ram_style="distributed" *) reg [1:0] pht[0:1023];
        integer n;
        initial begin
            for(n=0;n<64;n=n+1) btb[n]=0;
            for(n=0;n<1024;n=n+1) pht[n]=2'b01;
        end
        localparam [1:0] G=g;
        wire [20:0] e=btb[fpc[9:4]];
        wire [1:0] c=pht[fpc[13:4]^ghr];
        assign f_take[g]=f_inrom && f_from[g] && e[20] && e[19:15]==fpc[14:10] && (!e[14] || c[1]);
        assign f_tgt[g]=e[13:0];
        wire [1:0] old=pht[tq_row];
        always @(posedge clk) begin
            if(tq_btb && tq_bank==G) btb[tq_idx]<=tq_btbd;
            if(tq_pht && tq_bank==G)
                pht[tq_row]<=tq_tk ? (old==3 ? 2'd3 : old+2'd1) : (old==0 ? 2'd0 : old-2'd1);
        end
    end endgenerate
    wire f_any=|f_take;
    wire [1:0] f_tslot=f_take[0] ? 2'd0 : f_take[1] ? 2'd1 : f_take[2] ? 2'd2 : 2'd3;
    wire [13:0] f_ttgt=f_tgt[f_tslot];
    wire [31:0] f_next=f_any ? {16'b0,f_ttgt,2'b0} : {fpc[31:4]+28'd1,4'b0};
    reg [3:0] f_mask;
    always @* for(i=0;i<4;i=i+1) f_mask[i]=f_from[i] && (!f_any || i[1:0]<=f_tslot);
    // Conservative queue space: the F1 block and this block must both fit.
    // Conservative queue space: every block in flight and this one must fit.
    wire [2:0] f_inflight={2'b0,f1_v}+{2'b0,F2STAGE!=0 && f2_v};
    wire f0_go=!halted && !rd_v && {1'b0,fq_cnt}+{f_inflight,2'b0}<=6'd12;

    // ------------------------------------------------------------------ F1 -> fetch queue
    wire [2:0] f1_n={2'b0,fw_mask[0]}+{2'b0,fw_mask[1]}+{2'b0,fw_mask[2]}+{2'b0,fw_mask[3]};
    wire f1_bad=fw_pc[31:15]!=17'd1;
    wire [31:0] f1_seq=fw_pc[31:4]+28'd1;
    wire [31:0] f1_pn[0:3];
    generate for(g=0;g<4;g=g+1) begin: slot_next
        localparam [1:0] G=g;
        assign f1_pn[g]=(fw_taken && fw_tslot==G) ? {16'b0,fw_tgt,2'b0} : G==3 ? {f1_seq[27:0],4'b0} : {fw_pc[31:4],G+2'd1,2'b0};
    end endgenerate

    // ------------------------------------------------------------------ D1
    wire [3:0] fqi[0:2];
    assign fqi[0]=fq_head; assign fqi[1]=fq_head+4'd1; assign fqi[2]=fq_head+4'd2;
    // Instructions are decoded in F1; D1 only selects queue entries.
    wire [63:0] f1_dec[0:3];
    generate for(g=0;g<4;g=g+1) begin: decoders
        assign f1_dec[g]=dec(fw_w[g],f1_bad);
    end endgenerate
    wire [63:0] dk[0:2];
    assign dk[0]=fq_dec[fqi[0]]; assign dk[1]=fq_dec[fqi[1]]; assign dk[2]=fq_dec[fqi[2]];
    // RS assignment for the next group: ALU-class work rotates over RS0-2,
    // multiply/divide uses RS0 and memory RS3. Ranks count earlier group members.
    reg [2:0] g_inrs, g_alu, g_wr, g_st;
    reg [1:0] g_rsid[0:2], g_rsrank[0:2], g_alurank[0:2], g_wrank[0:2], g_strank[0:2], g_need[0:3];
    always @* begin
        for(j=0;j<3;j=j+1) begin
            g_inrs[j]=j<d1_n && !dk[j][63] && !dk[j][62];
            g_alu[j]=g_inrs[j] && dk[j][61:59]<=C_JALR;
            g_wr[j]=j<d1_n && dk[j][58:54]!=0;
            g_st[j]=g_inrs[j] && dk[j][61:59]==C_ST;
        end
        for(j=0;j<3;j=j+1) begin
            g_alurank[j]=0; g_wrank[j]=0; g_strank[j]=0;
            for(i=0;i<3;i=i+1) if(i<j) begin
                g_alurank[j]=g_alurank[j]+{1'b0,g_alu[i]};
                g_wrank[j]=g_wrank[j]+{1'b0,g_wr[i]};
                g_strank[j]=g_strank[j]+{1'b0,g_st[i]};
            end
            if(dk[j][61:59]==C_LD || dk[j][61:59]==C_ST) g_rsid[j]=3;
            else if(dk[j][61:59]==C_MUL || dk[j][61:59]==C_DIV) g_rsid[j]=0;
            else case({rr,g_alurank[j]})
                4'b0000, 4'b0110, 4'b1001: g_rsid[j]=0;
                4'b0001, 4'b0100, 4'b1010: g_rsid[j]=1;
                default: g_rsid[j]=2;
            endcase
        end
        for(j=0;j<3;j=j+1) begin
            g_rsrank[j]=0;
            for(i=0;i<3;i=i+1) if(i<j && g_inrs[i] && g_rsid[i]==g_rsid[j]) g_rsrank[j]=g_rsrank[j]+2'd1;
        end
        for(k=0;k<4;k=k+1) begin
            g_need[k]=0;
            for(j=0;j<3;j=j+1) if(g_inrs[j] && g_rsid[j]==k[1:0]) g_need[k]=g_need[k]+2'd1;
        end
    end
    wire [2:0] rr_sum={1'b0,rr}+{2'b0,g_alu[0]}+{2'b0,g_alu[1]}+{2'b0,g_alu[2]};
    wire [1:0] rr_next=rr_sum>=3'd6 ? rr_sum[1:0]-2'd2 : rr_sum>=3'd3 ? rr_sum[1:0]-2'd3 : rr_sum[1:0];
    reg [2:0] d2_inrs=0, d2_alu=0, d2_wr=0, d2_st=0;
    reg [1:0] d2_rsid[0:2], d2_rsrank[0:2], d2_wrank[0:2], d2_strank[0:2], d2_need[0:3];
    wire [1:0] d1_n=fq_cnt>=3 ? 2'd3 : fq_cnt[1:0];

    // ------------------------------------------------------------------ D2 rename/dispatch
    reg [2:0] inrs, isalu, isst, isld, wr;
    reg [1:0] rsid[0:2], rsrank[0:2], alurank[0:2], wrank[0:2], strank[0:2];
    reg [3:0] slot[0:2];
    reg [15:0] slot_oh[0:2], ff_oh[0:3][0:2];
    reg [NR-1:0] row_oh[0:2];
    always @* for(k=0;k<4;k=k+1) for(i=0;i<3;i=i+1) ff_oh[k][i]=nth_free_oh(rsbusy(rs_v,k[1:0]),i[1:0]);
    always @* alloc_cols=row_oh[0] | row_oh[1] | row_oh[2];
    reg [1:0] gai, gbi;
    reg [5:0] row[0:2];
    reg [5:0] phys[0:2], oldp[0:2], t1[0:2], t2[0:2];
    reg [NR-1:0] newdep[0:2], grpdep[0:2];
    reg [NR-1:0] clear_q=0;
    // sat[c]: column c's producer has woken (cleared at reallocation). The select
    // loop is sat -> request -> grant -> sat; dependence bits clear a cycle later.
    reg [NR-1:0] sat=0;
    reg [4:0] rsfree_q[0:3];      // conservative free rows: frees counted a cycle late
    reg [NR-1:0] alloc_cols;
    reg ga, gb;
    reg [PW-1:0] newpay[0:2];
    reg [15:0] newage[0:2];
    reg [1:0] need[0:3];
    reg [4:0] rsfree[0:3];
    reg [2:0] nst, nd;
    reg p1, p2;
    reg [5:0] c1, c2;
    reg [NR-1:0] st_pend;
    reg [1:0] gst;
    reg [3:0] snap;
    reg rs_ok;
    always @* begin
        for(j=0;j<3;j=j+1) begin
            inrs[j]=d2_inrs[j]; isalu[j]=d2_alu[j]; isst[j]=d2_st[j]; wr[j]=d2_wr[j];
            isld[j]=inrs[j] && d2_cls[j]==C_LD;
            rsid[j]=d2_rsid[j]; rsrank[j]=d2_rsrank[j]; wrank[j]=d2_wrank[j]; strank[j]=d2_strank[j];
        end
        for(k=0;k<4;k=k+1) begin
            need[k]=d2_need[k];
            rsfree[k]=rsfree_q[k];
        end
        rs_ok=1;
        for(k=0;k<4;k=k+1) if({3'b0,need[k]}>rsfree[k]) rs_ok=0;
        nst={2'b0,isst[0]}+{2'b0,isst[1]}+{2'b0,isst[2]};
        nd={2'b0,d2_v[0]}+{2'b0,d2_v[1]}+{2'b0,d2_v[2]};
        for(r=0;r<NR;r=r+1) st_pend[r]=rs_v[r] && rs_pay[r][P_CLS+:3]==C_ST;
        for(j=0;j<3;j=j+1) begin
            slot_oh[j]=inrs[j] ? ff_oh[rsid[j]][rsrank[j]] : 16'b0;
            row_oh[j]=0; row[j]=0;
            for(r=0;r<NR;r=r+1) if(rs_k[r]==rsid[j] && slot_oh[j][rs_s[r]]) begin row_oh[j][r]=1'b1; row[j]=row[j] | r[5:0]; end
            phys[j]=fl[fl_head[4:0]+{3'b0,wrank[j]}];
        end
        for(j=0;j<3;j=j+1) begin
            t1[j]=rat_p[d2_r1[j]]; c1=rat_c[d2_r1[j]]; p1=rat_pend[d2_r1[j]] && d2_r1[j]!=0;
            t2[j]=rat_p[d2_r2[j]]; c2=rat_c[d2_r2[j]]; p2=rat_pend[d2_r2[j]] && d2_r2[j]!=0;
            oldp[j]=rat_p[d2_rd[j]];
            ga=0; gb=0; gai=0; gbi=0;
            for(i=0;i<3;i=i+1) if(i<j) begin   // later producers in the group win
                if(d2_m1[j][i]) begin t1[j]=phys[i]; p1=1; ga=1; gai=i[1:0]; end
                if(d2_m2[j][i]) begin t2[j]=phys[i]; p2=1; gb=1; gbi=i[1:0]; end
                if(d2_wm[j][i]) oldp[j]=phys[i];
            end
            // RAT pending bits lag column clears by a cycle, so RAT-derived bits
            // are also masked by last cycle's clears. Group producers are new rows.
            newdep[j]=0; grpdep[j]=0;
            if(p1) begin if(ga) grpdep[j]=grpdep[j] | row_oh[gai]; else newdep[j][c1]=1'b1; end
            if(p2) begin if(gb) grpdep[j]=grpdep[j] | row_oh[gbi]; else newdep[j][c2]=1'b1; end
            if(d2_cls[j]==C_LD) begin
                newdep[j]=newdep[j] | st_pend;
                for(i=0;i<3;i=i+1) if(i<j && isst[i]) grpdep[j]=grpdep[j] | row_oh[i];
            end
            newdep[j]=(newdep[j] & ~clear_q) | grpdep[j];
            snap=sq_tail+{2'b0,strank[j]};
            newage[j]=~rsbusy(~rs_v,rsid[j]);
            for(i=0;i<3;i=i+1) if(i<j && rsid[i]==rsid[j]) newage[j]=newage[j] | slot_oh[i];
            newpay[j]=0;
            newpay[j][P_F3+:3]=d2_f3[j];
            newpay[j][P_BIMM]=d2_bimm[j];
            newpay[j][P_ASEL+:2]=d2_asel[j];
            newpay[j][P_AOP+:4]=d2_aop[j];
            newpay[j][P_CLS+:3]=d2_cls[j];
            newpay[j][P_T1+:6]=t1[j];
            newpay[j][P_T2+:6]=t2[j];
            newpay[j][P_U1]=d2_u1[j];
            newpay[j][P_U2]=d2_u2[j];
            newpay[j][P_DST+:6]=phys[j];
            newpay[j][P_WE]=wr[j];
            newpay[j][P_ROB+:5]=rob_tail+j[4:0];
            newpay[j][P_IMM+:32]=d2_imm[j];
            newpay[j][P_PC+:32]=d2_pc[j];
            newpay[j][P_PN+:32]=d2_pn[j];
            newpay[j][P_GHR+:10]=d2_ghr[j];
            newpay[j][P_SQ+:4]=snap;
            newpay[j][P_LW]=d2_lw[j] && d2_cls[j]==C_LD && d2_fd[j]==7;
            newpay[j][P_FWD]=d2_lw[j] && d2_cls[j]==C_LD && d2_fd[j]!=7;
            newpay[j][P_FIDX+:3]=snap[2:0]-3'd1-d2_fd[j];
            newpay[j][P_MM]=d2_mm[j] && d2_cls[j]==C_LD;
        end
    end
    wire fire=!halted && |d2_v && !pend_v && !rd_v && !flush_q &&
        ({1'b0,rob_cnt}+{4'b0,nd}<=7'd32) && ({1'b0,sq_cnt}+{2'b0,nst}<=5'd8) && rs_ok &&
        fl_cnt>={4'b0,wrank[2]}+{5'b0,wr[2]};
    wire d2_free=!(|d2_v) || fire;


    // ------------------------------------------------------------------ select
    reg [NR-1:0] req, grant, clear, lat0;
    reg [15:0] rsreq;
    reg [3:0] gany;
    reg [2:0] cls_r;
    wire div_wake=div_run && div_cnt==5'd30;
    always @* begin
        for(r=0;r<NR;r=r+1) begin
            cls_r=rs_pay[r][P_CLS+:3];
            lat0[r]=!(cls_r==C_LD || cls_r==C_MUL || cls_r==C_DIV);
            req[r]=rs_v[r] && !rs_iss[r] && (rs_dep[r] & ~sat)==0 &&
                (!(cls_r==C_MUL || cls_r==C_DIV) || !div_busy) &&
                (!rs_pay[r][P_MM] || rs_ath[r]) && (!rs_pay[r][P_LW] || rs_sqok[r]);
        end
        for(r=0;r<NR;r=r+1) begin
            rsreq=~rsbusy(~req,rs_k[r]);
            grant[r]=req[r] && (rsreq & rs_age[r])==0;
        end
        for(k=0;k<4;k=k+1) gany[k]=rsbusy(~grant,k[1:0])!=16'hffff;
        for(r=0;r<NR;r=r+1)
            clear[r]=(grant[r] && lat0[r]) || (rs_v[r] && rs_iss[r] && rs_wc[r]==2'd1) ||
                (div_wake && r==div_row);
    end
    reg [15:0] age_keep[0:3];
    always @* for(k=0;k<4;k=k+1) age_keep[k]=~rsbusy(~rs_v,k[1:0]);
    // Select carries tags/control only; immediate, PC, predicted next PC and
    // history (bits P_IMM..P_GHR+9) are read in R through the registered grant.
    // Only operand tags, operand-use flags and operand-source selects pass through the
    // select mux; all other fields are read in R from the registered one-hot grant.
    localparam [PW-1:0] PSMALL=({{(PW-6){1'b0}},6'h3f}<<P_T1) | ({{(PW-6){1'b0}},6'h3f}<<P_T2) |
        ({{(PW-1){1'b0}},1'b1}<<P_U1) | ({{(PW-1){1'b0}},1'b1}<<P_U2) |
        ({{(PW-2){1'b0}},2'b11}<<P_ASEL) | ({{(PW-1){1'b0}},1'b1}<<P_BIMM);
    localparam [PW-1:0] PBULK=~PSMALL;
    reg [NR-1:0] r_gnt[0:3];
    reg [PW-1:0] r_full[0:3];
    always @* for(k=0;k<4;k=k+1) begin
        r_full[k]=r_pay[k];
        for(r=0;r<NR;r=r+1) if(rs_k[r]==k[1:0]) r_full[k]=r_full[k] | ({PW{r_gnt[k][r]}} & rs_pay[r] & PBULK);
    end
    reg [PW-1:0] gpay[0:3];
    always @* for(k=0;k<4;k=k+1) begin
        gpay[k]=0;
        for(r=0;r<NR;r=r+1) if(rs_k[r]==k[1:0]) gpay[k]=gpay[k] | ({PW{grant[r]}} & rs_pay[r] & ~PBULK);
    end

    // ------------------------------------------------------------------ R
    // Producers whose results enter res_* at the end of this cycle.
    reg [4:0] prod_v;
    reg [5:0] prod_tag[0:4];
    wire div_last=div_fin;   // result written from registered quotient/remainder
    always @* begin
        for(q=0;q<3;q=q+1) begin prod_v[q]=e_v[q] && e_pay[q][P_WE]; prod_tag[q]=e_pay[q][P_DST+:6]; end
        prod_v[3]=m2_v && m2_we; prod_tag[3]=m2_tag;
        prod_v[4]=(mq3_v && mq3_we) || (div_last && div_we); prod_tag[4]=mq3_v ? mq3_tag : div_tag;
    end
    genvar pq, pr;
    generate for(pq=0;pq<5;pq=pq+1) begin: prf_bank
        (* ram_style="distributed" *) reg [31:0] bank[0:63];
        integer n;
        initial for(n=0;n<64;n=n+1) bank[n]=0;
        always @(posedge clk) if(res_v[pq]) bank[res_tag[pq]]<=res_val[pq];
        for(pr=0;pr<12;pr=pr+1) begin: rd
            assign bank_rd[pq][pr]=bank[prf_rt[pr]];
        end
    end endgenerate
    generate for(pr=0;pr<12;pr=pr+1) begin: prf_read
        assign prf_rd[pr]=bank_rd[lvt[prf_rt[pr]]][pr];
    end endgenerate
    generate for(pr=0;pr<4;pr=pr+1) begin: prf_ops
        assign prf_rt[2*pr]=r_pay[pr][P_T1+:6];
        assign prf_rt[2*pr+1]=r_pay[pr][P_T2+:6];
    end endgenerate
    assign prf_rt[8]=rob_pd[h0]; assign prf_rt[9]=rob_pd[h1]; assign prf_rt[10]=rob_pd[h2];
    assign prf_rt[11]=crat_now[debug_reg_addr];
    function [31:0] wbyp;   // register value including results being written now
        input [5:0] t; input [31:0] fileval; integer n; reg hit;
        begin
            wbyp=0; hit=0;
            for(n=0;n<5;n=n+1) if(res_v[n] && res_tag[n]==t) begin wbyp=wbyp | res_val[n]; hit=1; end
            if(!hit) wbyp=fileval;
        end
    endfunction
    reg [31:0] r_a[0:3], r_b[0:3];
    reg [4:0] r_eba[0:3], r_ebb[0:3];
    always @* for(k=0;k<4;k=k+1) begin
        r_a[k]=r_pay[k][P_ASEL+:2]==1 ? r_full[k][P_PC+:32] : r_pay[k][P_ASEL+:2]==2 || !r_pay[k][P_U1] ? 32'b0 :
            wbyp(r_pay[k][P_T1+:6],prf_rd[2*k]);
        r_b[k]=r_pay[k][P_BIMM] ? r_full[k][P_IMM+:32] : !r_pay[k][P_U2] ? 32'b0 : wbyp(r_pay[k][P_T2+:6],prf_rd[2*k+1]);
        for(q=0;q<5;q=q+1) begin
            r_eba[k][q]=r_pay[k][P_U1] && r_pay[k][P_ASEL+:2]==0 && prod_v[q] && prod_tag[q]==r_pay[k][P_T1+:6];
            r_ebb[k][q]=r_pay[k][P_U2] && !r_pay[k][P_BIMM] && prod_v[q] && prod_tag[q]==r_pay[k][P_T2+:6];
        end
    end

    // ------------------------------------------------------------------ E
    function [31:0] bmux;
        input [4:0] s; integer n;
        begin bmux=0; for(n=0;n<5;n=n+1) bmux=bmux | ({32{s[n]}} & res_val[n]); end
    endfunction
    reg [31:0] x_a[0:3], x_b[0:3], x_res[0:2], x_sum[0:2];
    reg [2:0] x_tk, x_flt, x_ct;
    reg [31:0] dif, shl, shr;
    reg lts, ltu, cond;
    always @* begin
        for(k=0;k<4;k=k+1) begin
            x_a[k]=|e_eba[k] ? bmux(e_eba[k]) : e_a[k];
            x_b[k]=|e_ebb[k] ? bmux(e_ebb[k]) : e_b[k];
        end
        for(k=0;k<3;k=k+1) begin
            x_sum[k]=x_a[k]+x_b[k];
            dif=x_a[k]-x_b[k];
            ltu=x_a[k]<x_b[k];
            lts=$signed(x_a[k])<$signed(x_b[k]);
            shl=x_a[k]<<x_b[k][4:0];
            shr=$signed({e_ops[k][A_SRA] & x_a[k][31],x_a[k]})>>>x_b[k][4:0];
            // One-hot select: unit results are masked and ORed (bit 11 = link).
            x_res[k]=({32{e_ops[k][A_ADD]}} & x_sum[k]) | ({32{e_ops[k][A_SUB]}} & dif) |
                     ({32{e_ops[k][A_SLL]}} & shl) | ({32{e_ops[k][A_SLT]}} & {31'b0,lts}) |
                     ({32{e_ops[k][A_SLTU]}} & {31'b0,ltu}) | ({32{e_ops[k][A_XOR]}} & (x_a[k]^x_b[k])) |
                     ({32{e_ops[k][A_SRL] | e_ops[k][A_SRA]}} & shr) | ({32{e_ops[k][A_OR]}} & (x_a[k]|x_b[k])) |
                     ({32{e_ops[k][A_AND]}} & (x_a[k]&x_b[k])) | ({32{e_ops[k][A_B]}} & x_b[k]) |
                     ({32{e_ops[k][11]}} & e_link[k]);
            case(e_pay[k][P_F3+:3])
                0: cond=x_a[k]==x_b[k];
                1: cond=x_a[k]!=x_b[k];
                4: cond=lts;
                5: cond=!lts;
                6: cond=ltu;
                default: cond=!ltu;
            endcase
            x_ct[k]=e_pay[k][P_CLS+:3]>=C_BR && e_pay[k][P_CLS+:3]<=C_JALR;
            x_tk[k]=e_pay[k][P_CLS+:3]==C_BR ? cond : x_ct[k];
            // Branch/JAL targets are known in R; JALR faults are checked in W.
            x_flt[k]=x_tk[k] && e_pay[k][P_CLS+:3]!=C_JALR && e_tgt[k][1];
        end
    end
    // Memory port E1
    wire [31:0] m_addr=x_a[3]+e_pay[3][P_IMM+:32];
    wire [2:0] m_f3=e_pay[3][P_F3+:3];
    wire m_ld=e_v[3] && e_pay[3][P_CLS+:3]==C_LD;
    wire m_st=e_v[3] && e_pay[3][P_CLS+:3]==C_ST;
    // Address classification happens in E2 from the registered address.
    wire m2_cnt=m2_addr==`HARDWARE_COUNTER_ADDR && m2_f3==3'd2;
    wire m2_flt=(m2_f3[1:0]==2'd2 && m2_addr[1:0]!=0) || (m2_f3[1:0]==2'd1 && m2_addr[1:0]==2'd3) ||
                !(m2_addr[31:16]==16'd1 || m2_cnt);
    wire st_uart=st_addr==`UART_ADDR;
    wire st_flt=(st_f3[1:0]==2'd2 && st_addr[1:0]!=0) || (st_f3[1:0]==2'd1 && st_addr[1:0]==2'd3) ||
                !(st_addr[31:16]==16'd1 || st_uart);
    wire [3:0] m_mask=(m_f3[1:0]==0 ? 4'b0001 : m_f3[1:0]==1 ? 4'b0011 : 4'b1111)<<m_addr[1:0];
    wire m_head=e_pay[3][P_ROB+:5]==rob_head;
    wire [2:0] m_sqi=e_pay[3][P_SQ+:3];
    // E2
    // Predicted store-to-load forwarding: byte merge before alignment.
    // Committed stores still in the write queue (wq) or written on the same edge
    // as this read (sh) are merged too, so they never cause a replay.
    wire l_wqhit=wq_v && wq_word==m2_word, l_shhit=sh_v && sh_word==m2_word;
    reg [31:0] l_word;
    always @* for(i=0;i<4;i=i+1)
        l_word[i*8+:8]=m2_fwd && m2_fmask[i] ? m2_fdata[i*8+:8] : l_wqhit && wq_mask[i] ? wq_data[i*8+:8] :
                       l_shhit && sh_mask[i] ? sh_data[i*8+:8] : ldata[i*8+:8];
    wire [31:0] l_sh=l_word>>{m2_off,3'b0};
    reg [31:0] l_val;
    always @* case(m2_f3)
        0: l_val={{24{l_sh[7]}},l_sh[7:0]};
        1: l_val={{16{l_sh[15]}},l_sh[15:0]};
        4: l_val={24'b0,l_sh[7:0]};
        5: l_val={16'b0,l_sh[15:0]};
        default: l_val=l_sh;
    endcase
    // A load conflicts if an older uncommitted store (or one committed but not
    // yet in RAM) overlaps it, unless a fully covering forwarded store is the
    // youngest such store.
    reg l_conf;
    reg [7:0] l_ov, l_cov;
    reg [2:0] sqd, fdist, ldist;
    reg [3:0] sqlim;
    always @* begin
        sqlim=m2_snap-sq_head;
        ldist=0;
        fdist=m2_snap[2:0]-3'd1-m2_fidx;
        for(i=0;i<8;i=i+1) begin
            sqd=i[2:0]-sq_head[2:0];
            l_cov[i]=(sq_mask[i] & m2_lmask)==m2_lmask;
            l_ov[i]={1'b0,sqd}<sqlim && sq_av[i] && !sq_uart[i] && sq_addr[i][15:2]==m2_word && |(sq_mask[i] & m2_lmask);
        end
        if(m2_fwd) begin
            l_conf=m2_fword!=m2_word || (m2_lmask & ~m2_fmask)!=0;
            for(i=0;i<8;i=i+1) begin
                ldist=m2_snap[2:0]-3'd1-i[2:0];
                if(l_ov[i] && ldist<fdist) l_conf=1;
            end
        end else
            l_conf=l_ov!=0;
    end
    // Training: distance to the youngest overlapping older store if it covers
    // every loaded byte; otherwise wait for older stores to commit.
    reg [2:0] tr_d, tr_best;
    reg tr_found, tr_cov;
    always @* begin
        tr_best=7; tr_found=0; tr_cov=0;
        for(i=7;i>=0;i=i-1) if(lwb_ov[i] && (!tr_found || lwb_snap[2:0]-3'd1-i[2:0]<tr_best)) begin
            tr_best=lwb_snap[2:0]-3'd1-i[2:0]; tr_found=1; tr_cov=lwb_cov[i];
        end
        tr_d=tr_found && tr_cov && tr_best!=7 ? tr_best : 3'd7;
    end

    // ------------------------------------------------------------------ W: branch check / redirect
    // Seven candidates (branch/JAL per ALU port, last cycle's JALR per port, load
    // replay). The oldest wins by pairwise age comparison, not a serial scan.
    reg [6:0] cand_v, win;
    reg [4:0] cand_rob[0:6], cand_rel[0:6];
    reg [31:0] cand_pc[0:6], rd_pc_n;
    reg [9:0] cand_ghr[0:6], rd_ghr_n;
    reg [4:0] rd_rob_n;
    reg [2:0] w_mis;
    reg acc;
    wire [4:0] pend_rel=pend_rob-rob_head;
    integer ca, cb;
    always @* begin
        for(k=0;k<3;k=k+1) begin
            w_mis[k]=wb_v[k] && wb_brj[k] && !wb_flt[k] && (wb_tk[k]!=wb_ptk[k] || (wb_tk[k] && wb_tgtbad[k]));
            cand_v[k]=w_mis[k]; cand_rob[k]=wb_rob[k]; cand_pc[k]=wb_next[k]; cand_ghr[k]={wb_ghr[k][8:0],wb_tk[k]};
            cand_v[k+3]=jc_v[k] && jc_mis[k]; cand_rob[k+3]=jc_rob[k]; cand_pc[k+3]=jc_pc[k]; cand_ghr[k+3]=jc_ghr[k];
        end
        cand_v[6]=lwb_v && lwb_rep; cand_rob[6]=lwb_rob; cand_pc[6]=lwb_pc; cand_ghr[6]=lwb_ghr;
        for(ca=0;ca<7;ca=ca+1) cand_rel[ca]=cand_rob[ca]-rob_head;
        acc=0; rd_pc_n=0; rd_ghr_n=0; rd_rob_n=0;
        for(ca=0;ca<7;ca=ca+1) begin
            win[ca]=cand_v[ca] && (!pend_v || cand_rel[ca]<pend_rel);
            for(cb=0;cb<7;cb=cb+1) if(cb!=ca && cand_v[cb] && !(cand_rel[ca]<cand_rel[cb])) win[ca]=0;
            acc=acc | win[ca];
            rd_pc_n=rd_pc_n | ({32{win[ca]}} & cand_pc[ca]);
            rd_ghr_n=rd_ghr_n | ({10{win[ca]}} & cand_ghr[ca]);
            rd_rob_n=rd_rob_n | ({5{win[ca]}} & cand_rob[ca]);
        end
        acc=acc && !halted && !flush_q;
    end

    // ------------------------------------------------------------------ commit
    wire [4:0] h0=rob_head, h1=rob_head+5'd1, h2=rob_head+5'd2;
    wire [2:0] sqh=sq_head[2:0];
    wire uart_ready;
    wire run=!halted && !flush_q;
    wire su=sq_uart[sqh];
    wire stok=!su || (uart_ready && !uq_v);
    wire ok0=rob_cnt!=0 && rob_cpl[h0] && !rob_flt[h0] && !rob_rep[h0];
    wire ok1=rob_cnt>1 && rob_cpl[h1] && !rob_flt[h1] && !rob_rep[h1];
    wire ok2=rob_cnt>2 && rob_cpl[h2] && !rob_flt[h2] && !rob_rep[h2];
    wire s0=rob_st[h0], s1=rob_st[h1], s2=rob_st[h2], t0=rob_ct[h0], tc1=rob_ct[h1], tc2=rob_ct[h2];
    wire cm0=run && ok0 && (!s0 || stok);
    // A counter read commits as the oldest of its group, so the profiler's
    // retirement count at that edge covers exactly the older instructions.
    wire cm1=cm0 && !rob_mis[h0] && ok1 && !rob_tcnt[h1] && !(s0 && s1) && !(t0 && tc1) && (!s1 || stok);
    wire cm2=cm1 && !rob_mis[h1] && ok2 && !rob_tcnt[h2] && !((s0 || s1) && s2) && !((t0 || tc1) && tc2) && (!s2 || stok);
    wire trap=run && rob_cnt!=0 && rob_cpl[h0] && rob_flt[h0];
    wire replay=run && rob_cnt!=0 && rob_cpl[h0] && rob_rep[h0] && !rob_flt[h0];
    wire misflush=(cm0 && rob_mis[h0]) || (cm1 && rob_mis[h1]) || (cm2 && rob_mis[h2]);
    wire [1:0] ncm={1'b0,cm0}+{1'b0,cm1}+{1'b0,cm2};
    wire cst=(cm0 && s0) || (cm1 && s1) || (cm2 && s2);
    wire cw0=cm0 && rob_rd[h0]!=0, cw1=cm1 && rob_rd[h1]!=0, cw2=cm2 && rob_rd[h2]!=0;
    wire [1:0] ncw={1'b0,cw0}+{1'b0,cw1}+{1'b0,cw2};
    wire [4:0] cth=(cm0 && t0) ? h0 : (cm1 && tc1) ? h1 : h2;
    wire ctv=(cm0 && t0) || (cm1 && tc1) || (cm2 && tc2);

    // ------------------------------------------------------------------ sequential
    reg [3:0] fo;
    reg [1:0] fs;
    reg [4:0] ri;
    generate for(g=0;g<4;g=g+1) begin: mpred
        localparam [1:0] G=g;
        (* ram_style="distributed" *) reg [4:0] mem_pred[0:2047];
        integer n;
        initial for(n=0;n<2048;n=n+1) mem_pred[n]=0;
        always @* f1_mp[g]=mem_pred[fw_pc[14:4]];
        always @(posedge clk) if(resetn && !halted && lwb_v && lwb_rep && lwb_pc[3:2]==G)
            mem_pred[lwb_pc[14:4]]<={1'b0,1'b1,tr_d};
    end endgenerate
    always @(posedge clk) begin
        // Register file write port (results computed in the previous cycle).
        for(q=0;q<5;q=q+1) if(res_v[q]) lvt[res_tag[q]]<=q[2:0];
        // RAM: one read expression and one byte-enabled write port.
        if(wq_v) begin
            if(wq_mask[0]) dmem[wq_word][7:0]<=wq_data[7:0];
            if(wq_mask[1]) dmem[wq_word][15:8]<=wq_data[15:8];
            if(wq_mask[2]) dmem[wq_word][23:16]<=wq_data[23:16];
            if(wq_mask[3]) dmem[wq_word][31:24]<=wq_data[31:24];
        end
        if(m_ld) ldata<=dmem[m_addr[15:2]];
        for(k=0;k<4;k=k+1) f1_w[k]<=imem[{fpc[14:4],k[1:0]}];
        for(k=0;k<4;k=k+1) f2_w[k]<=f1_w[k];
    end

    always @(posedge clk) begin
        if(!resetn) begin
            halted<=0; cycles<=0; flush_q<=0; rd_v<=0; pend_v<=0;
            fpc<=32'h8000; ghr<=0; f1_v<=0; f2_v<=0; fq_head<=0; fq_tail<=0; fq_cnt<=0; d2_v<=0;
            tq_btb<=0; tq_pht<=0;
            for(i=0;i<32;i=i+1) begin rat_p[i]<=i[5:0]; crat[i]<=i[5:0]; fl[i]<=6'd32+i[5:0]; end
            rat_pend<=0; fl_head<=0; fl_tail<=6'd32; fl_chead<=0; fl_cnt<=6'd32; rr<=0; cp_w<=0; cs_ct<=0;
            rob_head<=0; rob_tail<=0; rob_cnt<=0;
            rs_v<=0; rs_iss<=0; r_v<=0; e_v<=0; res_v<=0; wb_v<=0; jc_v<=0;
            m2_v<=0; st_v<=0; lwb_v<=0; sq_head<=0; sq_tail<=0; sq_cnt<=0; wq_v<=0; sh_v<=0; uq_v<=0;
            mq1_v<=0; mq2_v<=0; mq3_v<=0; div_busy<=0; div_run<=0; div_fin<=0; mwb_v<=0; clear_q<=0; sat<=0;
            for(k=0;k<4;k=k+1) rsfree_q[k]<=SIZE(k[1:0]);
            last_pc<=32'h8000;
        end else if(!halted) begin
            cycles<=cycles+1'b1;

            // ---------------- frontend
            if(rd_v) begin
                fpc<=rd_pc; ghr<=rd_ghr; f1_v<=0;
            end else if(f0_go) begin
                fpc<=f_next; ghr<={ghr[8:0],f_any}; f1_v<=1; f1_pc<=fpc; f1_mask<=f_mask;
                f1_tslot<=f_tslot; f1_taken<=f_any; f1_tgt<=f_ttgt; f1_ghr<=ghr;
            end else f1_v<=0;
            f2_v<=f1_v && !rd_v; f2_pc<=f1_pc; f2_mask<=f1_mask; f2_tslot<=f1_tslot; f2_taken<=f1_taken;
            f2_tgt<=f1_tgt; f2_ghr<=f1_ghr;
            if(rd_v) begin
                fq_head<=0; fq_tail<=0; fq_cnt<=0; d2_v<=0;
            end else begin
                if(fw_v) for(i=0;i<16;i=i+1) begin
                    fo=i[3:0]-fq_tail;
                    if({1'b0,fo}<{2'b0,f1_n}) begin
                        fs=fw_pc[3:2]+fo[1:0];
                        fq_insn[i]<=f1_w[fs]; fq_dec[i]<=f1_dec[fs]; fq_mp[i]<=f1_mp[fs];
                        fq_pc[i]<={fw_pc[31:4],fs,2'b0};
                        fq_pn[i]<=f1_pn[fs];
                        fq_ghr[i]<=fw_ghr;
                        fq_bad[i]<=f1_bad;
                    end
                end
                fq_tail<=fq_tail+(fw_v ? {1'b0,f1_n} : 4'd0);
                fq_head<=fq_head+(d2_free ? {2'b0,d1_n} : 4'd0);
                fq_cnt<=fq_cnt+(fw_v ? {2'b0,f1_n} : 5'd0)-(d2_free ? {3'b0,d1_n} : 5'd0);
                if(d2_free) for(j=0;j<3;j=j+1) begin
                    d2_v[j]<=j<d1_n;
                    {d2_fault[j],d2_nop[j],d2_cls[j],d2_rd[j],d2_r1[j],d2_r2[j],d2_u1[j],d2_u2[j],
                     d2_imm[j],d2_aop[j],d2_asel[j],d2_bimm[j],d2_f3[j]}<=dk[j];
                    d2_pc[j]<=fq_pc[fqi[j]]; d2_insn[j]<=fq_insn[fqi[j]]; d2_pn[j]<=fq_pn[fqi[j]];
                    d2_ghr[j]<=fq_ghr[fqi[j]];
                    {d2_mm[j],d2_lw[j],d2_fd[j]}<=fq_mp[fqi[j]];
                    d2_inrs[j]<=g_inrs[j]; d2_alu[j]<=g_alu[j]; d2_wr[j]<=g_wr[j]; d2_st[j]<=g_st[j];
                    d2_rsid[j]<=g_rsid[j]; d2_rsrank[j]<=g_rsrank[j]; d2_wrank[j]<=g_wrank[j]; d2_strank[j]<=g_strank[j];
                    for(i=0;i<3;i=i+1) begin
                        d2_m1[j][i]<=i<j && dk[j][43] && dk[j][53:49]!=0 && dk[j][53:49]==dk[i][58:54];
                        d2_m2[j][i]<=i<j && dk[j][42] && dk[j][48:44]!=0 && dk[j][48:44]==dk[i][58:54];
                        d2_wm[j][i]<=i<j && dk[j][58:54]!=0 && dk[j][58:54]==dk[i][58:54];
                    end
                end
            end
            if(!rd_v && d2_free) begin
                rr<=rr_next;
                for(k=0;k<4;k=k+1) d2_need[k]<=g_need[k];
            end
            if(tq_btb || tq_pht) begin tq_btb<=0; tq_pht<=0; end

            // ---------------- rename/dispatch
            clear_q<=clear;
            sat<=(sat | clear) & ~alloc_cols;
            for(k=0;k<4;k=k+1)
                rsfree_q[k]<=rsfree_q[k]-(fire ? {3'b0,need[k]} : 5'd0)+free_count(rsbusy(~clear_q,k[1:0]));
            for(r=0;r<32;r=r+1) if(rat_pend[r] && clear_q[rat_c[r]]) rat_pend[r]<=0;
            // Entries/rows beyond the allocated ones are free, so their contents
            // are written whether or not the group dispatches; only valid bits,
            // pointers and the RAT depend on the late dispatch decision.
            for(j=0;j<3;j=j+1) if({1'b0,rob_cnt}+j<32) begin
                ri=rob_tail+j[4:0];
                rob_pc[ri]<=d2_pc[j]; rob_insn[ri]<=d2_insn[j]; rob_rd[ri]<=d2_rd[j];
                rob_pd[ri]<=phys[j]; rob_po[ri]<=oldp[j]; rob_ghr[ri]<=d2_ghr[j];
                rob_st[ri]<=isst[j]; rob_ct[ri]<=inrs[j] && d2_cls[j]>=C_BR && d2_cls[j]<=C_JALR;
                rob_cond[ri]<=d2_cls[j]==C_BR; rob_cls[ri]<=d2_cls[j]; rob_tcnt[ri]<=0;
                rob_cpl[ri]<=d2_nop[j] || d2_fault[j]; rob_flt[ri]<=d2_fault[j];
                rob_mis[ri]<=0; rob_rep[ri]<=0;
            end
            for(j=0;j<3;j=j+1) if(isst[j] && {1'b0,sq_cnt}+strank[j]<8) sq_av[sq_tail[2:0]+strank[j]]<=0;
            if(fire) begin
                fl_head<=fl_head+{4'b0,wrank[2]}+{5'b0,wr[2]};
                for(j=0;j<3;j=j+1) if(wr[j]) begin rat_p[d2_rd[j]]<=phys[j]; rat_c[d2_rd[j]]<=row[j]; rat_pend[d2_rd[j]]<=1; end
                rob_tail<=rob_tail+{2'b0,nd};
                sq_tail<=sq_tail+{1'b0,nst};
            end

            // ---------------- reservation stations
            for(r=0;r<NR;r=r+1) begin
                if(clear[r]) begin rs_v[r]<=0; rs_iss[r]<=0; end
                else if(grant[r]) begin
                    rs_iss[r]<=1;
                    rs_wc[r]<=rs_pay[r][P_CLS+:3]==C_LD ? 2'd1 : rs_pay[r][P_CLS+:3]==C_MUL ? 2'd3 : 2'd0;
                end else if(rs_iss[r] && rs_wc[r]>2'd1) rs_wc[r]<=rs_wc[r]-2'd1;
                rs_dep[r]<=rs_dep[r] & ~clear_q;
                rs_age[r]<=rs_age[r] & age_keep[rs_k[r]];   // empty slots are younger
                rs_ath[r]<=rs_pay[r][P_ROB+:5]==rob_head;
                rs_sqok[r]<=rs_pay[r][P_SQ+:4]==sq_head;
            end
            for(r=0;r<NR;r=r+1) for(j=0;j<3;j=j+1) if(row_oh[j][r]) begin
                rs_dep[r]<=newdep[j]; rs_pay[r]<=newpay[j]; rs_age[r]<=newage[j];
                rs_ath[r]<=0; rs_sqok[r]<=0;
                if(fire) begin rs_v[r]<=1; rs_iss[r]<=0; end
            end
            for(r=0;r<NA;r=r+1) if(grant[r] && rs_pay[r][P_CLS+:3]==C_DIV) begin div_busy<=1; div_row<=r[5:0]; end

            // ---------------- R -> E
            for(k=0;k<4;k=k+1) begin
                r_v[k]<=gany[k]; r_pay[k]<=gpay[k];
                for(r=0;r<NR;r=r+1) r_gnt[k][r]<=grant[r] && rs_k[r]==k[1:0];
                e_v[k]<=r_v[k]; e_pay[k]<=r_full[k]; e_a[k]<=r_a[k]; e_b[k]<=r_b[k];
                e_eba[k]<=r_eba[k]; e_ebb[k]<=r_ebb[k];
                e_ops[k]<=0;
                if(r_full[k][P_CLS+:3]==C_JAL || r_full[k][P_CLS+:3]==C_JALR) e_ops[k][11]<=1'b1;
                else e_ops[k][r_full[k][P_AOP+:4]]<=1'b1;
                e_tgt[k]<=r_full[k][P_PC+:32]+r_full[k][P_IMM+:32];
                e_ptk[k]<=r_full[k][P_PN+:32]!=r_full[k][P_PC+:32]+32'd4;
                e_tgtbad[k]<=r_full[k][P_PN+:32]!=r_full[k][P_PC+:32]+r_full[k][P_IMM+:32];
                e_link[k]<=r_full[k][P_PC+:32]+32'd4;
            end

            // ---------------- E -> W (ALU ports)
            for(k=0;k<3;k=k+1) begin
                res_v[k]<=e_v[k] && e_pay[k][P_WE] && e_pay[k][P_CLS+:3]<=C_JALR;
                res_tag[k]<=e_pay[k][P_DST+:6]; res_val[k]<=x_res[k];
                wb_v[k]<=e_v[k] && e_pay[k][P_CLS+:3]<=C_JALR; wb_rob[k]<=e_pay[k][P_ROB+:5];
                wb_ct[k]<=x_ct[k]; wb_cond[k]<=e_pay[k][P_CLS+:3]==C_BR; wb_flt[k]<=x_flt[k]; wb_tk[k]<=x_tk[k];
                wb_sum[k]<=x_sum[k]; wb_tgt[k]<=e_tgt[k]; wb_link[k]<=e_link[k];
                // Branch/JAL targets are PC-relative: decide from direction and
                // the precomputed target without a late 32-bit comparison.
                wb_brj[k]<=e_pay[k][P_CLS+:3]==C_BR || e_pay[k][P_CLS+:3]==C_JAL;
                wb_ptk[k]<=e_ptk[k]; wb_tgtbad[k]<=e_tgtbad[k];
                wb_jalr[k]<=e_pay[k][P_CLS+:3]==C_JALR;
                jc_v[k]<=wb_v[k] && wb_jalr[k]; jc_flt[k]<=wb_sum[k][1];
                jc_mis[k]<=wb_v[k] && wb_jalr[k] && !wb_sum[k][1] && wb_next[k]!=wb_pn[k];
                jc_rob[k]<=wb_rob[k]; jc_pc[k]<=wb_next[k]; jc_ghr[k]<={wb_ghr[k][8:0],1'b1};
                wb_pn[k]<=e_pay[k][P_PN+:32]; wb_ghr[k]<=e_pay[k][P_GHR+:10];
            end
            // Memory E1
            m2_v<=m_ld; m2_we<=e_pay[3][P_WE]; m2_tag<=e_pay[3][P_DST+:6]; m2_rob<=e_pay[3][P_ROB+:5];
            m2_f3<=m_f3; m2_off<=m_addr[1:0]; m2_word<=m_addr[15:2]; m2_lmask<=m_mask;
            m2_snap<=e_pay[3][P_SQ+:4]; m2_addr<=m_addr;
            m2_fidx<=e_pay[3][P_FIDX+:3];
            m2_fwd<=e_pay[3][P_FWD] && sq_av[e_pay[3][P_FIDX+:3]] &&
                {1'b0,e_pay[3][P_FIDX+:3]-sq_head[2:0]}<e_pay[3][P_SQ+:4]-sq_head;
            m2_fdata<=sq_data[e_pay[3][P_FIDX+:3]]; m2_fmask<=sq_mask[e_pay[3][P_FIDX+:3]];
            m2_fword<=sq_addr[e_pay[3][P_FIDX+:3]][15:2];
            // The free-running counter has no read side effect; it is sampled
            // when the load executes, including speculatively.
            m2_cntrep<=0;
            m2_pc<=e_pay[3][P_PC+:32]; m2_ghr<=e_pay[3][P_GHR+:10];
            st_v<=m_st; st_rob<=e_pay[3][P_ROB+:5]; st_addr<=m_addr; st_f3<=m_f3; st_sqi<=m_sqi;
            if(st_v) sq_uart[st_sqi]<=st_uart;
            if(m_st) begin
                sq_addr[m_sqi]<=m_addr; sq_data[m_sqi]<=x_b[3]<<{m_addr[1:0],3'b0}; sq_mask[m_sqi]<=m_mask;
                sq_av[m_sqi]<=1;
            end
            // E2
            res_v[3]<=m2_v && m2_we; res_tag[3]<=m2_tag; res_val[3]<=m2_cnt ? cycles : l_val;
            lwb_v<=m2_v; lwb_rob<=m2_rob; lwb_flt<=m2_flt; lwb_rep<=!m2_flt && (m2_cntrep || l_conf);
            lwb_cnt<=m2_cntrep; lwb_iscnt<=m2_cnt && !m2_flt; lwb_tval<=cycles; lwb_pc<=m2_pc; lwb_ghr<=m2_ghr; lwb_ov<=l_ov; lwb_cov<=l_cov; lwb_snap<=m2_snap;
            // Multiplier
            mq1_v<=e_v[0] && e_pay[0][P_CLS+:3]==C_MUL; mq1_we<=e_pay[0][P_WE]; mq1_tag<=e_pay[0][P_DST+:6];
            mq1_rob<=e_pay[0][P_ROB+:5]; mq1_hi<=e_pay[0][P_F3+:3]!=0;
            mA<={(e_pay[0][P_F3+:3]==1 || e_pay[0][P_F3+:3]==2) && x_a[0][31],x_a[0]};
            mB<={e_pay[0][P_F3+:3]==1 && x_b[0][31],x_b[0]};
            mq2_v<=mq1_v; mq2_we<=mq1_we; mq2_tag<=mq1_tag; mq2_rob<=mq1_rob; mq2_hi<=mq1_hi; mp1<=mA*mB;
            mq3_v<=mq2_v; mq3_we<=mq2_we; mq3_tag<=mq2_tag; mq3_rob<=mq2_rob; mq3_hi<=mq2_hi; mp2<=mp1;
            res_v[4]<=(mq3_v && mq3_we) || (div_last && div_we);
            res_tag[4]<=mq3_v ? mq3_tag : div_tag;
            res_val[4]<=mq3_v ? (mq3_hi ? mp2[63:32] : mp2[31:0]) :
                div_bz ? (div_rem ? da0 : 32'hffffffff) :
                div_rem ? (div_negr ? -dr : dr) : (div_negq ? -dq : dq);
            mwb_v<=mq3_v || div_last; mwb_rob<=mq3_v ? mq3_rob : div_rob;
            // Divider: setup in E, 32 restoring iterations
            if(e_v[0] && e_pay[0][P_CLS+:3]==C_DIV) begin
                dq<=!e_pay[0][P_F3] && x_a[0][31] ? -x_a[0] : x_a[0];
                dd<=!e_pay[0][P_F3] && x_b[0][31] ? -x_b[0] : x_b[0];
                dr<=0; da0<=x_a[0]; div_cnt<=0; div_run<=1;
                div_negq<=!e_pay[0][P_F3] && (x_a[0][31]^x_b[0][31]);
                div_negr<=!e_pay[0][P_F3] && x_a[0][31];
                div_rem<=e_pay[0][P_F3+1]; div_bz<=x_b[0]==0;
                div_we<=e_pay[0][P_WE]; div_tag<=e_pay[0][P_DST+:6]; div_rob<=e_pay[0][P_ROB+:5];
            end else if(div_run) begin
                dq<=dq_next; dr<=dr_next; div_cnt<=div_cnt+5'd1;
                if(div_cnt==5'd31) begin div_run<=0; div_fin<=1; end
            end
            if(div_fin) begin div_fin<=0; div_busy<=0; end

            // ---------------- ROB completion
            for(k=0;k<3;k=k+1) if(wb_v[k] && !wb_jalr[k]) begin
                rob_cpl[wb_rob[k]]<=1; rob_flt[wb_rob[k]]<=wb_flt[k]; rob_mis[wb_rob[k]]<=w_mis[k];
                rob_atk[wb_rob[k]]<=wb_tk[k]; rob_atgt[wb_rob[k]]<=wb_next[k][15:2];
            end
            for(k=0;k<3;k=k+1) if(jc_v[k]) begin
                rob_cpl[jc_rob[k]]<=1; rob_flt[jc_rob[k]]<=jc_flt[k]; rob_mis[jc_rob[k]]<=jc_mis[k];
                rob_atk[jc_rob[k]]<=1; rob_atgt[jc_rob[k]]<=jc_pc[k][15:2];
            end
            if(st_v) begin rob_cpl[st_rob]<=1; rob_flt[st_rob]<=st_flt; end
            if(lwb_v) begin rob_cpl[lwb_rob]<=1; rob_flt[lwb_rob]<=lwb_flt; rob_rep[lwb_rob]<=lwb_rep;
                rob_tcnt[lwb_rob]<=lwb_iscnt; rob_tval[lwb_rob]<=lwb_tval; end
            if(mwb_v) rob_cpl[mwb_rob]<=1;

            // ---------------- redirect
            rd_v<=acc;
            if(acc) begin rd_pc<=rd_pc_n; rd_ghr<=rd_ghr_n; pend_v<=1; pend_rob<=rd_rob_n; end

            // ---------------- commit
            wq_v<=cst && !su; wq_word<=sq_addr[sqh][15:2]; wq_data<=sq_data[sqh]; wq_mask<=sq_mask[sqh];
            sh_v<=wq_v; sh_word<=wq_word; sh_mask<=wq_mask; sh_data<=wq_data;
            uq_v<=cst && su; uq_data<=sq_data[sqh][7:0];
            if(cst) begin sq_head<=sq_head+4'd1; end
            sq_cnt<=sq_cnt+(fire ? {1'b0,nst} : 4'd0)-{3'b0,cst};
            cp_w<={cw2,cw1,cw0};
            cp_rd[0]<=rob_rd[h0]; cp_rd[1]<=rob_rd[h1]; cp_rd[2]<=rob_rd[h2];
            cp_pd[0]<=rob_pd[h0]; cp_pd[1]<=rob_pd[h1]; cp_pd[2]<=rob_pd[h2];
            cp_po[0]<=rob_po[h0]; cp_po[1]<=rob_po[h1]; cp_po[2]<=rob_po[h2];
            cs_ct<={cm2 && tc2, cm1 && tc1, cm0 && t0};
            cs_atk<={rob_atk[h2],rob_atk[h1],rob_atk[h0]}; cs_cond<={rob_cond[h2],rob_cond[h1],rob_cond[h0]};
            cs_pc[0]<=rob_pc[h0][15:2]; cs_pc[1]<=rob_pc[h1][15:2]; cs_pc[2]<=rob_pc[h2][15:2];
            cs_ghr[0]<=rob_ghr[h0]; cs_ghr[1]<=rob_ghr[h1]; cs_ghr[2]<=rob_ghr[h2];
            cs_atgt[0]<=rob_atgt[h0]; cs_atgt[1]<=rob_atgt[h1]; cs_atgt[2]<=rob_atgt[h2];
            for(k=0;k<3;k=k+1) if(cp_w[k]) crat[cp_rd[k]]<=cp_pd[k];
            if(cp_w[0]) fl[fl_tail[4:0]]<=cp_po[0];
            if(cp_w[1]) fl[fl_tail[4:0]+{4'b0,cp_w[0]}]<=cp_po[1];
            if(cp_w[2]) fl[fl_tail[4:0]+{4'b0,cp_w[0]}+{4'b0,cp_w[1]}]<=cp_po[2];
            fl_tail<=fl_tail+{4'b0,cp_n}; fl_chead<=fl_chead+{4'b0,cp_n};
            fl_cnt<=fl_cnt+{4'b0,cp_n}-(fire ? {4'b0,wrank[2]}+{5'b0,wr[2]} : 6'd0);
            rob_head<=rob_head+{3'b0,ncm};
            rob_cnt<=rob_cnt+(fire ? {3'b0,nd} : 6'd0)-{4'b0,ncm};
            if(cm0) begin
                last_pc<=cm2 ? rob_pc[h2] : cm1 ? rob_pc[h1] : rob_pc[h0];
                last_insn<=cm2 ? rob_insn[h2] : cm1 ? rob_insn[h1] : rob_insn[h0];
            end
            if(cp_ct) begin
                tq_btb<=cp_atk; tq_pht<=cp_cond; tq_tk<=cp_atk;
                tq_bank<=cp_pc[3:2]; tq_idx<=cp_pc[9:4]; tq_row<=cp_pc[13:4]^cp_ghr;
                tq_btbd<={1'b1,cp_pc[14:10],cp_cond,cp_atgt};
            end
            if(trap) begin halted<=1; fault_pc<=rob_pc[h0]; fault_insn<=rob_insn[h0]; end
            flush_q<=replay || misflush;

            // ---------------- commit-time flush: restore committed state
            if(flush_q) begin
                rs_v<=0; rs_iss<=0; r_v<=0; e_v<=0; res_v<=0; wb_v<=0; jc_v<=0; m2_v<=0; st_v<=0; lwb_v<=0;
                mq1_v<=0; mq2_v<=0; mq3_v<=0; div_run<=0; div_fin<=0; div_busy<=0; mwb_v<=0;
                rob_tail<=rob_head; rob_cnt<=0; sq_tail<=sq_head; sq_cnt<=0;
                for(i=0;i<32;i=i+1) rat_p[i]<=crat_now[i];
                rat_pend<=0; fl_head<=fl_chead+{4'b0,cp_n}; fl_cnt<=fl_tail-fl_chead; pend_v<=0; rd_v<=0;
                clear_q<=0; sat<=0;
                for(k=0;k<4;k=k+1) rsfree_q[k]<=SIZE(k[1:0]);
            end
        end
    end

    uart_tx #(.CLOCK_HZ(CLOCK_HZ)) serial(clk,resetn,uq_v,uq_data,uart_ready,uart_tx);

    // ------------------------------------------------------------------ debug and profile
    assign debug_pc=halted ? fault_pc : cm0 ? rob_pc[h0] : last_pc;
    assign debug_insn=halted ? fault_insn : cm0 ? rob_insn[h0] : last_insn;
    assign debug_commit=cm0; assign debug_commit2=cm1; assign debug_commit3=cm2;
    assign debug_we=rob_rd[h0]!=0; assign debug_rd=rob_rd[h0]; assign debug_wdata=prf_rd[8];
    assign debug_pc2=rob_pc[h1]; assign debug_insn2=rob_insn[h1]; assign debug_we2=rob_rd[h1]!=0;
    assign debug_rd2=rob_rd[h1]; assign debug_wdata2=prf_rd[9];
    assign debug_pc3=rob_pc[h2]; assign debug_insn3=rob_insn[h2]; assign debug_we3=rob_rd[h2]!=0;
    assign debug_rd3=rob_rd[h2]; assign debug_wdata3=prf_rd[10];
    assign debug_reg_data=debug_reg_addr==0 ? 32'b0 : prf_rd[11];
    assign debug_uart_valid=uq_v; assign debug_uart_data=uq_data;
    assign debug_timer_valid=cm0 && rob_tcnt[h0];
    assign debug_timer_sample=rob_tval[h0];
    wire [2:0] hcls=rob_ct[h0] ? 3'd1 : 3'd0;
    // head load waiting; head mul/div waiting; rename held for recovery; redirect;
    // empty ROB; UART-blocked commit; ROB full; fetch queue empty. Predicates overlap.
    assign profile_flags={fq_cnt==0, rob_cnt>=30, rob_cnt!=0 && rob_st[h0] && rob_cpl[h0] && su && !stok,
        rob_cnt==0, rd_v, pend_v, 1'b0, 1'b0};
    // Younger work issued while the oldest instruction is still incomplete.
    reg head_granted;
    always @* begin
        head_granted=0;
        for(r=0;r<NR;r=r+1) if(grant[r] && rs_pay[r][P_ROB+:5]==rob_head) head_granted=1;
    end
    assign debug_ooo_issue=|grant && rob_cnt!=0 && !rob_cpl[rob_head] && !head_granted;
    assign profile_issued={2'b0,gany[0]}+{2'b0,gany[1]}+{2'b0,gany[2]}+{2'b0,gany[3]};
`ifdef VERILATOR
    always @(posedge clk) if(resetn && !halted) begin
        for(q=0;q<5;q=q+1) for(k=0;k<5;k=k+1) if(q<k && res_v[q] && res_v[k] && res_tag[q]==res_tag[k])
            $fatal(1,"two results for one physical register");
        if(fire) for(j=0;j<3;j=j+1) if(inrs[j] && rs_v[row[j]]) $fatal(1,"allocated a busy RS row");
        if(fire) for(j=0;j<3;j=j+1) if(inrs[j] && slot_oh[j]==0) $fatal(1,"dispatch without a free RS row");
        for(k=0;k<4;k=k+1) if(rsfree_q[k]>free_count(rsbusy(rs_v,k[1:0]))) $fatal(1,"RS free count above actual");
        if(fl_tail-fl_head>6'd32) $fatal(1,"free list overflow");
        if(fl_cnt!=fl_tail-fl_head) $fatal(1,"free-list count mismatch");
    end
`endif
`ifdef VERILATOR
    // Simulation-only accounting over the timed interval [first, second counter read).
    reg prof_on=0;
    reg [1:0] prof_reads=0;
    longint pc_cyc=0, pc_d2empty=0, pc_hold=0, pc_robfull=0, pc_sqfull=0, pc_rsfull[0:3], pc_cm[0:3];
    longint pc_robempty=0, pc_headwait[0:7], pc_headbr=0, pc_grant[0:3], pc_rsocc[0:3], pc_robocc=0;
    longint pc_hflag=0, pc_flagrows=0, pc_flaggrants=0, pc_lgrants=0;
    longint pc_lst=0, pc_lreg=0, pc_lrdy=0, pc_sreg=0, pc_srdy=0, pc_alurdy=0, pc_alu_wait=0;
    longint pc_hw[0:7], pc_hstate[0:3], pc_ldwait_dep=0, pc_ldwait_st=0;
    longint pc_redirect=0, pc_replay=0, pc_flush=0, pc_fqempty=0, pc_f0stall=0, pc_headdone_notc=0;
    integer pj;
    initial for(pj=0;pj<4;pj=pj+1) begin pc_rsfull[pj]=0; pc_cm[pj]=0; pc_grant[pj]=0; pc_rsocc[pj]=0; end
    initial for(pj=0;pj<8;pj=pj+1) begin pc_headwait[pj]=0; pc_hw[pj]=0; end
    initial for(pj=0;pj<4;pj=pj+1) pc_hstate[pj]=0;
    reg hin_rs, hissued; reg [5:0] hrow;
    always @* begin hin_rs=0; hissued=0; hrow=0; for(pj=0;pj<NR;pj=pj+1) if(rs_v[pj] && rs_pay[pj][P_ROB+:5]==rob_head) begin hin_rs=1; hissued=rs_iss[pj]; hrow=pj; end end
    always @(posedge clk) if(resetn && !halted) begin
        if(debug_timer_valid) begin prof_reads<=prof_reads+1; prof_on<=prof_reads==0; end
        if(prof_on) begin
            pc_cyc<=pc_cyc+1;
            if(!(|d2_v)) pc_d2empty<=pc_d2empty+1;
            else if(!fire) begin
                if(pend_v || flush_q || rd_v) pc_hold<=pc_hold+1;
                else if({1'b0,rob_cnt}+{4'b0,nd}>7'd32) pc_robfull<=pc_robfull+1;
                else if({1'b0,sq_cnt}+{2'b0,nst}>5'd8) pc_sqfull<=pc_sqfull+1;
                else for(pj=0;pj<4;pj=pj+1) if({2'b0,need[pj]}>rsfree[pj]) pc_rsfull[pj]<=pc_rsfull[pj]+1;
            end
            pc_cm[ncm]<=pc_cm[ncm]+1;
            if(rob_cnt==0) pc_robempty<=pc_robempty+1;
            else if(!rob_cpl[h0]) pc_headwait[rob_ct[h0] ? 1 : rob_st[h0] ? 5 : {1'b0,rob_rd[h0]!=0}+2]<=pc_headwait[rob_ct[h0] ? 1 : rob_st[h0] ? 5 : {1'b0,rob_rd[h0]!=0}+2]+1;
            else if(!cm0) pc_headdone_notc<=pc_headdone_notc+1;
            if(rob_cnt!=0 && !rob_cpl[h0]) begin
                pc_hw[rob_cls[h0]]<=pc_hw[rob_cls[h0]]+1;
                // 0: waiting in RS on dependences, 1: ready in RS but not granted, 2: issued/in flight
                if(hin_rs && !hissued && rs_dep[hrow]!=0) begin pc_hstate[0]<=pc_hstate[0]+1;
                    if(rob_cls[h0]==C_LD) begin if((rs_dep[hrow] & st_pend)!=0) pc_ldwait_st<=pc_ldwait_st+1; else pc_ldwait_dep<=pc_ldwait_dep+1; end end
                else if(hin_rs && !hissued) pc_hstate[1]<=pc_hstate[1]+1;
                else pc_hstate[2]<=pc_hstate[2]+1;
            end
            for(pj=0;pj<4;pj=pj+1) begin
                pc_grant[pj]<=pc_grant[pj]+gany[pj];
                pc_rsocc[pj]<=pc_rsocc[pj]+(SIZE(pj)-free_count(rsbusy(rs_v,pj)));
            end
            pc_robocc<=pc_robocc+rob_cnt;
            if(hin_rs && !hissued && rs_pay[hrow][P_LW]) pc_hflag<=pc_hflag+1;
            for(pj=0;pj<NR;pj=pj+1) begin
                if(rs_v[pj] && !rs_iss[pj] && rs_pay[pj][P_LW]) pc_flagrows=pc_flagrows+1;
                if(grant[pj] && rs_pay[pj][P_LW]) pc_flaggrants=pc_flaggrants+1;
                if(grant[pj] && rs_pay[pj][P_CLS+:3]==C_LD) pc_lgrants=pc_lgrants+1;
            end
            for(pj=3*NA;pj<NR;pj=pj+1) if(rs_v[pj] && !rs_iss[pj]) begin
                if(rs_pay[pj][P_CLS+:3]==C_LD) begin
                    if(rs_dep[pj]==0) pc_lrdy=pc_lrdy+1;
                    else if((rs_dep[pj] & ~st_pend)==0) pc_lst=pc_lst+1;
                    else pc_lreg=pc_lreg+1;
                end else begin
                    if(rs_dep[pj]==0) pc_srdy=pc_srdy+1; else pc_sreg=pc_sreg+1;
                end
            end
            for(pj=0;pj<3*NA;pj=pj+1) if(rs_v[pj] && !rs_iss[pj]) begin
                if(rs_dep[pj]==0) pc_alurdy=pc_alurdy+1; else pc_alu_wait=pc_alu_wait+1;
            end
            if(rd_v) pc_redirect<=pc_redirect+1;
            if(lwb_v && lwb_rep) pc_replay<=pc_replay+1;
            if(flush_q) pc_flush<=pc_flush+1;
            if(fq_cnt==0) pc_fqempty<=pc_fqempty+1;
            if(!f0_go && !rd_v) pc_f0stall<=pc_f0stall+1;
        end
    end
    // Whole-run event counts for directed tests.
    longint ev_fwd=0, ev_merge=0, ev_replay=0, ev_cntrep=0, ev_redirect=0;
    always @(posedge clk) if(resetn && !halted) begin
        if(m2_v && m2_fwd && !l_conf) ev_fwd<=ev_fwd+1;
        if(m2_v && !m2_fwd && ((l_wqhit && |(wq_mask & m2_lmask)) || (l_shhit && |(sh_mask & m2_lmask)))) ev_merge<=ev_merge+1;
        if(lwb_v && lwb_rep) begin ev_replay<=ev_replay+1; if(lwb_cnt) ev_cntrep<=ev_cntrep+1; end
        if(rd_v) ev_redirect<=ev_redirect+1;
    end
    final $fwrite(32'h80000002,"OB2_EVENTS forwarded=%0d merged=%0d replays=%0d counter_replays=%0d redirects=%0d\n",
        ev_fwd,ev_merge,ev_replay,ev_cntrep,ev_redirect);
    // Optional pipeline trace: +ob2trace=<first cycle> prints 400 cycles.
    longint tr_from=0; reg tr_en=0; integer tq;
    initial tr_en=$value$plusargs("ob2trace=%d",tr_from);
    always @(posedge clk) if(tr_en && cycles>=tr_from && cycles<tr_from+400) begin
        $fwrite(32'h80000002,"%0d D:", cycles);
        for(tq=0;tq<3;tq=tq+1) if(fire && d2_v[tq]) $fwrite(32'h80000002," %h",d2_pc[tq][15:0]); else $fwrite(32'h80000002,"     ");
        $fwrite(32'h80000002," |G:");
        for(tq=0;tq<4;tq=tq+1) if(gany[tq]) $fwrite(32'h80000002," %h",gpay[tq][P_PC+15:P_PC]); else $fwrite(32'h80000002,"     ");
        $fwrite(32'h80000002," |C:");
        if(cm0) $fwrite(32'h80000002," %h",rob_pc[h0][15:0]); if(cm1) $fwrite(32'h80000002," %h",rob_pc[h1][15:0]); if(cm2) $fwrite(32'h80000002," %h",rob_pc[h2][15:0]);
        $fwrite(32'h80000002," |rob=%0d rs=%0d/%0d/%0d/%0d fq=%0d%s%s%s\n", rob_cnt, NA-free_count(rsbusy(rs_v,0)), NA-free_count(rsbusy(rs_v,1)),
            NA-free_count(rsbusy(rs_v,2)), NM-free_count(rsbusy(rs_v,3)), fq_cnt, rd_v?" REDIRECT":"", pend_v?" hold":"", flush_q?" FLUSH":"");
    end
    final if(pc_cyc!=0) begin
        $fwrite(32'h80000002,"OB2_PROFILE cycles=%0d d2_empty=%0d dispatch_hold=%0d rob_full=%0d sq_full=%0d rs_full=%0d/%0d/%0d/%0d\n",
            pc_cyc,pc_d2empty,pc_hold,pc_robfull,pc_sqfull,pc_rsfull[0],pc_rsfull[1],pc_rsfull[2],pc_rsfull[3]);
        $fwrite(32'h80000002,"OB2_PROFILE commit0=%0d commit1=%0d commit2=%0d commit3=%0d rob_empty=%0d head_done_blocked=%0d\n",
            pc_cm[0],pc_cm[1],pc_cm[2],pc_cm[3],pc_robempty,pc_headdone_notc);
        $fwrite(32'h80000002,"OB2_PROFILE head_wait ctrl=%0d nordinst=%0d writer=%0d store=%0d\n",pc_headwait[1],pc_headwait[2],pc_headwait[3],pc_headwait[5]);
        $fwrite(32'h80000002,"OB2_PROFILE grants=%0d/%0d/%0d/%0d rs_occ=%0d/%0d/%0d/%0d rob_occ=%0d\n",pc_grant[0],pc_grant[1],pc_grant[2],pc_grant[3],
            pc_rsocc[0],pc_rsocc[1],pc_rsocc[2],pc_rsocc[3],pc_robocc);
        $fwrite(32'h80000002,"OB2_PROFILE head_incomplete_by_class alu=%0d br=%0d jal=%0d jalr=%0d ld=%0d st=%0d mul=%0d div=%0d\n",
            pc_hw[0],pc_hw[1],pc_hw[2],pc_hw[3],pc_hw[4],pc_hw[5],pc_hw[6],pc_hw[7]);
        $fwrite(32'h80000002,"OB2_PROFILE head_state waiting_deps=%0d ready_not_granted=%0d in_flight=%0d head_load_waits_store=%0d head_load_waits_reg=%0d\n",
            pc_hstate[0],pc_hstate[1],pc_hstate[2],pc_ldwait_st,pc_ldwait_dep);
        $fwrite(32'h80000002,"OB2_PROFILE memrs_rowcycles load_ready=%0d load_wait_store=%0d load_wait_reg=%0d store_ready=%0d store_wait=%0d alu_ready=%0d alu_wait=%0d\n",
            pc_lrdy,pc_lst,pc_lreg,pc_srdy,pc_sreg,pc_alurdy,pc_alu_wait);
        $fwrite(32'h80000002,"OB2_PROFILE flagged_load: head_waiting=%0d row_cycles=%0d grants=%0d all_load_grants=%0d\n",pc_hflag,pc_flagrows,pc_flaggrants,pc_lgrants);
        $fwrite(32'h80000002,"OB2_PROFILE redirects=%0d replays=%0d flushes=%0d fq_empty=%0d f0_stall=%0d\n",pc_redirect,pc_replay,pc_flush,pc_fqempty,pc_f0stall);
    end
    `endif
endmodule
