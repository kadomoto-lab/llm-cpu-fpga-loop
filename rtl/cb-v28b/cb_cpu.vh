// Crispy Bacon (FAST=5): frequency-focused successor of Out-of-Bacon II (FAST=4).
// Started from the post-v7 working OB2 RTL (results/crispy-bacon/origin); OB2 v7 stays frozen.
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
module cb_cpu #(parameter CODE_FILE="code.hex", DATA_FILE="data.hex",
    parameter integer CLOCK_HZ=100000000, NA=8, NM=8, SELPOS=0)(
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
    // Active-high copy for the main state reset: no inverter, so the synchronous
    // reset net can be replicated instead of feeding hundreds of loads from one LUT.
    (* max_fanout=24 *) reg rst=1;
    always @(posedge clk) rst<=!resetn_in;
    // Index widths follow the RS-geometry parameters.
    /* verilator lint_off WIDTH */
    // Reservation-station payload layout.
    localparam P_F3=0, P_BIMM=3, P_ASEL=4, P_AOP=6, P_CLS=10, P_T1=13, P_T2=19, P_U1=25, P_U2=26,
               P_DST=27, P_WE=33, P_ROB=34, P_SQ=39,
               P_LW=43, P_MM=44, P_FWD=45, P_FIDX=46, P_B1=49, P_B2=52, P_RW=55, PW=56;
    // P_B1/P_B2: register-file bank (= result port) holding each source operand.
    // P_RW: ROB wrap bit; {P_RW, P_ROB} orders any two live instructions by the sign
    // of their 6-bit difference, with no reference to the ROB head.
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
                7'h17: begin rd=in[11:7]; imm={in[31:12],12'b0}; aop=A_B; asel=2; bimm=1; end   // pc+imm precomputed in the side table
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
            // The fault bit is a separate shallow function; it does not mask the
            // other fields (D1 qualifies rd and in-RS by it), keeping F1 decode short.
            if(cls==C_ALU && rd==0) nop=1;   // integer op to x0 has no effect
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
    reg pend_rw=0;

    // Frontend
    reg [31:0] fpc=32'h8000;
    reg [9:0] ghr=0;
    reg [9:0] pidx=0;   // PHT row fpc[13:4]^ghr, registered with fpc
    reg f1_v=0, f1_taken=0;
    reg [31:0] f1_pc=0;
    reg [3:0] f1_mask=0;
    reg [1:0] f1_tslot=0;
    reg [13:0] f1_tgt=0;
    reg [9:0] f1_ghr=0;
    // f1_w[k] is the instruction for fetch-queue bank k: each ROM copy is addressed
    // with the block slot that bank k receives (slot = bank + f1_rot), so the raw
    // ROM outputs are decoded in place and no rotation follows the slow RAM output.
    reg [31:0] f1_w[0:3];
    reg [1:0] f1_rot=0;
    wire fw_v=f1_v, fw_taken=f1_taken;
    wire [31:0] fw_pc=f1_pc;
    wire [3:0] fw_mask=f1_mask;
    wire [1:0] fw_tslot=f1_tslot;
    wire [13:0] fw_tgt=f1_tgt;
    wire [9:0] fw_ghr=f1_ghr;
    // Fetch queue: four LUTRAM banks by index[1:0]. A block writes at most one
    // entry per bank and D1's three reads use three distinct banks.
    // Record {decoded, pc, predicted next, history, memory prediction, taken, insn}.
    localparam FQW=64+32+32+10+5+1+32;
    wire [FQW-1:0] fq_out[0:2];
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
    reg [2:0] d2_pt=0;
    reg [31:0] d2_pc[0:2], d2_insn[0:2], d2_pn[0:2], d2_imm[0:2];
    reg [9:0] d2_ghr[0:2];
    reg [2:0] d2_cls[0:2], d2_f3[0:2];
    reg [4:0] d2_rd[0:2], d2_r1[0:2], d2_r2[0:2];
    reg [3:0] d2_aop[0:2];
    reg [1:0] d2_asel[0:2];
    reg [2:0] d2_m1[0:2], d2_m2[0:2], d2_wm[0:2];
    // RAT results read in D1: source tags, old destination mapping, and per-source
    // dependence columns (cleared like RS dependences while the group waits in D2).
    reg [5:0] d2_t1[0:2], d2_t2[0:2], d2_op[0:2];
    reg [2:0] d2_b1[0:2], d2_b2[0:2];
    // Writers of the group that dispatched as this group entered D2 (RAT bypass).
    reg [2:0] pg_v=0;
    reg [4:0] pg_rd[0:2];
    reg [5:0] pg_phys[0:2];
    reg [2:0] pg_bank[0:2];
    reg [NR-1:0] pg_dep[0:2];
    reg [5:0] pg_row[0:2];
    reg [2:0] pg_p=0;            // previous-group producer not yet woken
    // [slot j][writer i]: source 1 / source 2 / destination of slot j names the
    // register written by previous-group writer i (compared in D1, registered).
    reg [2:0] d2_pm1[0:2], d2_pm2[0:2], d2_pmo[0:2];
    reg [5:0] rat_p[0:31], crat[0:31];
    // Bank (result port 0-4) that writes each mapping's physical register. A physical
    // register is written only by its producer's port, so R reads that bank directly
    // and the live-value table is needed only by the debug read ports.
    reg [2:0] rat_b[0:31], crat_b[0:31];
    reg [5:0] rat_c[0:31];
    reg [31:0] rat_pend=0;
    reg [5:0] fl[0:31];
    reg [5:0] fl_head=0, fl_tail=6'd32, fl_chead=0;
    reg [5:0] fl_cnt=6'd32;       // free registers available to rename (fl_tail-fl_head)
    // Prefetched free-list window: fl_win[i]=fl[old head+i] read a cycle early (with
    // registers freed on that edge bypassed in), and fl_used = registers taken at that
    // edge, so rename reads fl_win[fl_used+rank] without an adder or a 32-way read.
    reg [5:0] fl_win[0:5];
    reg [1:0] fl_used=0;
    reg [5:0] fl_base;
    reg [4:0] fl_pos;
    initial for(i=0;i<6;i=i+1) fl_win[i]=6'd32+i[5:0];
    reg [1:0] rr=0;

    // ROB
    reg [4:0] rob_head=0, rob_tail=0;
    reg rob_hw=0, rob_tw=0;   // wrap bits of head and tail
    // rob_head+1 and +2 as registers: head indices and head updates need no adders.
    reg [4:0] rob_h1=1, rob_h2=2;
    reg [5:0] rob_cnt=0;
    reg [31:0] rob_cpl=0, rob_flt=0, rob_mis=0, rob_rep=0, rob_st=0, rob_ct=0, rob_cond=0, rob_atk=0, rob_tcnt=0;
    reg [31:0] rob_tval[0:31];   // sampled counter value (simulation trace only)
    reg [31:0] rob_pc[0:31], rob_insn[0:31];
    reg [4:0] rob_rd[0:31];
    reg [5:0] rob_pd[0:31], rob_po[0:31];
    reg [13:0] rob_atgt[0:31];
    // Actual-target writes are registered and applied a cycle late; the target is read
    // for training in the commit-packet stage (index cs_hi), after any such write.
    reg [5:0] aw_v=0;
    reg [4:0] aw_i[0:5];
    reg [13:0] aw_d[0:5];
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
    // Encoded bypass selects (0-4: result port, 7: register value) and per-port
    // copies of the results, so each E-stage operand is one 8:1 mux of local flops.
    // v28-A: one-hot E-stage operand source selects (bit 0..4 = bypass column,
    // bit 5 = bank/default). The R stage already resolves bypass to the
    // one-hot r_eba/r_ebb; encoding to 3 bits and re-decoding behind the E
    // operand muxes gave the fo=65 e_sa net feeding 800 res_e/res_vw endpoints
    // (routed -0.477 arc). Strict priority (lowest column wins) is preserved
    // by masking in the R->E decode, so the AND-OR below is bit-identical to
    // the old case() for ANY r_eba value, not only one-hots.
    reg [5:0] e_so_a[0:3], e_so_b[0:3];
    (* keep="true", equivalent_register_removal="no" *) reg [31:0] res_e[0:3][0:4];
    reg [31:0] res_nx[0:4];   // next results (written to res_val, res_vw, res_e)
    // Results being written: 0-2 ALU, 3 load, 4 multiply/divide
    (* max_fanout=32 *) reg [4:0] res_v=0;
    (* max_fanout=32 *) reg [5:0] res_tag[0:4];
    (* max_fanout=32 *) reg [31:0] res_val[0:4];
    // Copy of the results for register-file writes and the R-stage bypass, so the
    // E-stage bypass copy (res_val) drives only the operand muxes.
    (* max_fanout=32, keep="true", equivalent_register_removal="no" *) reg [31:0] res_vw[0:4];
    // ALU write-back bookkeeping
    reg [2:0] wb_v=0, wb_ct=0, wb_cond=0, wb_flt=0, wb_tk=0, wb_jalr=0, wb_brj=0, wb_ptk=0, wb_tgtbad=0;
    reg [3:0] e_ptk=0, e_tgtbad=0;
    // JALR targets need the adder result, so their check completes one cycle later.
    reg [2:0] jc_v=0, jc_mis=0, jc_flt=0;
    reg [4:0] jc_rob[0:2];
    reg [2:0] jc_rw=0, wb_rw=0;
    reg [31:0] jc_pc[0:2];
    reg [9:0] jc_ghr[0:2];
    reg [4:0] wb_rob[0:2];
    reg [31:0] wb_pn[0:2], wb_sum[0:2], wb_tgt[0:2], wb_link[0:2];
    // Next PC is selected in W from registered E results (JALR sum, branch target, link).
    reg [31:0] wb_next[0:2];
    always @* for(k=0;k<3;k=k+1)
        wb_next[k]=wb_jalr[k] ? {wb_sum[k][31:1],1'b0} : wb_tk[k] ? wb_tgt[k] : wb_link[k];
    reg [11:0] e_ops[0:3];    // one-hot ALU result select, decoded in R
    reg [3:0] e_inv=0;        // subtract (SUB, SLT, SLTU, conditional branch), decoded in R
    reg [5:0] e_bop[0:3];
    // Static per-instruction fields live in a ROB-indexed LUTRAM side table (four
    // banks by ROB index[1:0]; a dispatch group's consecutive entries use distinct
    // banks). It is written one cycle after D2 (first read in R, two cycles after
    // dispatch), and that write stage precomputes every R-stage sum so R has no adders.
    // Layout {pc[15:2], operand imm (pc+imm for AUIPC), target, link[16:2],
    // predicted next[16:2], predicted taken, target differs from prediction, history}.
    // Valid fetch PCs lie in 0x8000-0xfffc, so pc, link and predicted next are narrow.
    localparam XW=14+32+32+15+15+1+1+10;
    localparam X_PC=XW-1, X_IMM=XW-15, X_TGT=XW-47, X_LINK=XW-79, X_PN=XW-94, X_PT=11, X_TB=10;
    reg [XW-1:0] xw_data[0:3];    // per-bank write data (built from xs_*)
    reg [31:0] xs_pc[0:3], xs_imm[0:3], xs_pn[0:3];
    reg [9:0] xs_ghr[0:3];
    reg [3:0] xs_pt=0, xs_au=0;
    reg [31:0] xs_tgt, xs_link;
    reg [3:0] xw_en;
    reg [2:0] xw_idx[0:3];
    wire [XW-1:0] xr_port[0:3];   // R-stage read for issue port k
    wire [XW-1:0] xr_commit[0:3]; // commit-side read, one entry per bank
    reg [31:0] e_pc[0:3], e_imm[0:3], e_pn[0:3];
    reg [9:0] e_ghr[0:3];
    reg [9:0] wb_ghr[0:2];
    // Memory pipeline
    reg [31:0] ldata=0;
    reg m2_v=0, m2_we=0, m2_cntrep=0;
    reg [31:0] m2_addr=0, st_addr=0;
    reg [2:0] st_f3=0, st_sqi=0;
    reg [5:0] m2_tag=0;
    reg [4:0] m2_rob=0;
    reg m2_rw=0, lwb_rw=0;
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
    // Multiplier (3-cycle, v27) and divider
    reg mq1_v=0, mq2_v=0, mq1_we=0, mq2_we=0, mq1_hi=0, mq2_hi=0;
    reg [5:0] mq1_tag=0, mq2_tag=0;
    reg [4:0] mq1_rob=0, mq2_rob=0;
    reg signed [32:0] mA=0, mB=0;
    reg signed [65:0] mp1=0;
    reg div_fin=0;
    reg div_setup=0, div_sgn=0, div_new=0;   // div_new: a divide was granted last cycle
    reg [31:0] db0=0;
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
    // The packet feeds the committed map, its merge (crat_now) and the free list.
    (* max_fanout=16 *) reg [2:0] cp_w=0;
    (* max_fanout=16 *) reg [4:0] cp_rd[0:2];
    (* max_fanout=16 *) reg [5:0] cp_pd[0:2];
    reg [5:0] cp_po[0:2];
    (* max_fanout=16 *) reg [2:0] cp_b[0:2];
    // Training data for all three head slots; the committed control instruction
    // (at most one per cycle) is selected in the packet stage.
    reg [2:0] cs_ct=0, cs_atk=0, cs_cond=0;
    reg [13:0] cs_pc[0:2];
    reg [4:0] cs_hi[0:2];
    reg [9:0] cs_ghr[0:2];
    wire [1:0] cs_sel=cs_ct[0] ? 2'd0 : cs_ct[1] ? 2'd1 : 2'd2;
    wire cp_ct=|cs_ct, cp_atk=cs_atk[cs_sel], cp_cond=cs_cond[cs_sel];
    wire [31:0] cp_pc={16'b0,cs_pc[cs_sel],2'b0};
    wire [9:0] cp_ghr=cs_ghr[cs_sel];
    wire [13:0] cp_atgt=rob_atgt[cs_hi[cs_sel]];
    wire [1:0] cp_n={1'b0,cp_w[0]}+{1'b0,cp_w[1]}+{1'b0,cp_w[2]};
    reg [5:0] crat_now[0:31];   // committed mapping including the pending packet
    reg [2:0] crat_now_b[0:31];
    integer cn, cm;
    always @* for(cn=0;cn<32;cn=cn+1) begin
        crat_now[cn]=crat[cn]; crat_now_b[cn]=crat_b[cn];
        for(cm=0;cm<3;cm=cm+1) if(cp_w[cm] && cp_rd[cm]==cn[4:0]) begin crat_now[cn]=cp_pd[cm]; crat_now_b[cn]=cp_b[cm]; end
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
        wire [1:0] c=pht[pidx];
        // Hit (registered-address BTB) is early; the PHT bit enters one LUT late.
        wire hit=f_inrom && f_from[g] && e[20] && e[19:15]==fpc[14:10];
        assign f_take[g]=(hit && !e[14]) || (hit && e[14] && c[1]);
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
    // Next PC as a priority mux on the takes (sequential address precomputed).
    wire [27:0] f_seq=fpc[31:4]+28'd1;
    wire [31:0] f_next=f_take[0] ? {16'b0,f_tgt[0],2'b0} : f_take[1] ? {16'b0,f_tgt[1],2'b0} :
                       f_take[2] ? {16'b0,f_tgt[2],2'b0} : f_take[3] ? {16'b0,f_tgt[3],2'b0} : {f_seq,4'b0};
    // Next PHT row for each candidate (taken slot g: history bit 1; sequential: 0),
    // precomputed so the row register is a priority mux of the takes.
    wire [9:0] f_pidx_n=f_take[0] ? f_tgt[0][11:2]^{ghr[8:0],1'b1} : f_take[1] ? f_tgt[1][11:2]^{ghr[8:0],1'b1} :
                        f_take[2] ? f_tgt[2][11:2]^{ghr[8:0],1'b1} : f_take[3] ? f_tgt[3][11:2]^{ghr[8:0],1'b1} :
                        f_seq[9:0]^{ghr[8:0],1'b0};
    reg [3:0] f_mask;
    always @* for(i=0;i<4;i=i+1) f_mask[i]=f_from[i] && (!f_any || i[1:0]<=f_tslot);
    // Conservative queue space: the F1 block and this block must both fit.
    // Conservative queue space: every block in flight and this one must fit.
    wire [2:0] f_inflight={2'b0,f1_v};
    wire f0_go=!halted && !rd_v && {1'b0,fq_cnt}+{f_inflight,2'b0}<=6'd12;

    // ------------------------------------------------------------------ F1 -> fetch queue
    wire [2:0] f1_n={2'b0,fw_mask[0]}+{2'b0,fw_mask[1]}+{2'b0,fw_mask[2]}+{2'b0,fw_mask[3]};
    // Queue head next cycle (the fetch-queue read rows are registered from it).
    wire [3:0] fq_head_n=rd_v ? 4'd0 : fq_head+(d2_free ? {2'b0,d1_n} : 4'd0);
    // Queue tail when the block now being read from ROM is written (next cycle).
    wire [3:0] fq_tail_n=rd_v ? 4'd0 : fq_tail+(fw_v ? {1'b0,f1_n} : 4'd0);
    wire f1_bad=fw_pc[31:15]!=17'd1;
    reg [27:0] f1_sq=0;   // fw_pc[31:4]+1, computed in F0
    wire [31:0] f1_seq={4'b0,f1_sq};
    wire [31:0] f1_pn[0:3];
    generate for(g=0;g<4;g=g+1) begin: slot_next
        localparam [1:0] G=g;
        assign f1_pn[g]=(fw_taken && fw_tslot==G) ? {16'b0,fw_tgt,2'b0} : G==3 ? {f1_seq[27:0],4'b0} : {fw_pc[31:4],G+2'd1,2'b0};
    end endgenerate

    reg [FQW-1:0] fqw_data[0:3];
    reg [3:0] fqw_en;
    reg [1:0] fqw_idx[0:3];
    reg [1:0] fqo, fqs;
    reg [3:0] fqw_pos;
    always @* for(k=0;k<4;k=k+1) begin
        fqo=k[1:0]-fq_tail[1:0];
        fqs=fw_pc[3:2]+fqo;
        fqw_en[k]=fw_v && !rd_v && {1'b0,fqo}<f1_n;
        fqw_pos=fq_tail+{2'b0,fqo};
        fqw_idx[k]=fqw_pos[3:2];
        fqw_data[k]={f1_dec[k],fw_pc[31:4],fqs,2'b0,f1_pn[fqs],fw_ghr,f1_mp[fqs],fw_taken && fqs==fw_tslot,f1_w[k]};
    end
    wire [FQW-1:0] fqb_out[0:3];
    generate for(g=0;g<4;g=g+1) begin: fqbank
        localparam [1:0] G=g;
        (* ram_style="distributed" *) reg [FQW-1:0] mem[0:3];
        always @(posedge clk) if(fqw_en[g]) mem[fqw_idx[g]]<=fqw_data[g];
        // Read row registered from the next head, so the RAM address is a flop.
        wire [3:0] re=fq_head_n+{2'b0,G-fq_head_n[1:0]};
        reg [1:0] ra=0;
        always @(posedge clk) if(rst || !halted) ra<=rst ? 2'd0 : re[3:2];
        assign fqb_out[g]=mem[ra];
    end endgenerate
    assign fq_out[0]=fqb_out[fq_head[1:0]];
    assign fq_out[1]=fqb_out[fq_head[1:0]+2'd1];
    assign fq_out[2]=fqb_out[fq_head[1:0]+2'd2];

    // ------------------------------------------------------------------ D1
    wire [3:0] fqi[0:2];
    assign fqi[0]=fq_head; assign fqi[1]=fq_head+4'd1; assign fqi[2]=fq_head+4'd2;
    // Instructions are decoded in F1; D1 only selects queue entries.
    wire [63:0] f1_dec[0:3];
    generate for(g=0;g<4;g=g+1) begin: decoders
        assign f1_dec[g]=dec(f1_w[g],f1_bad);
    end endgenerate
    wire [63:0] dk[0:2];
    assign dk[0]=fq_out[0][FQW-1-:64]; assign dk[1]=fq_out[1][FQW-1-:64]; assign dk[2]=fq_out[2][FQW-1-:64];
    // RS assignment for the next group: ALU-class work rotates over RS0-2,
    // multiply/divide uses RS0 and memory RS3. Ranks count earlier group members.
    // g_inrs_a/g_alu_a ignore the (deep) fault bit: they only size and rank the RS
    // assignment (a faulting instruction reserves a slot it never takes, and halts the
    // machine at commit). Allocation, RAT, store-queue and ROB use the exact bits.
    reg [2:0] g_inrs, g_alu, g_wr, g_st, g_inrs_a, g_alu_a;
    reg [1:0] g_rsid[0:2], g_rsrank[0:2], g_alurank[0:2], g_wrank[0:2], g_strank[0:2], g_need[0:3];
    always @* begin
        for(j=0;j<3;j=j+1) begin
            g_inrs[j]=j<d1_n && !dk[j][63] && !dk[j][62];
            g_alu[j]=g_inrs[j] && dk[j][61:59]<=C_JALR;
            g_inrs_a[j]=j<d1_n && !dk[j][62];
            g_alu_a[j]=g_inrs_a[j] && dk[j][61:59]<=C_JALR;
            g_wr[j]=j<d1_n && !dk[j][63] && dk[j][58:54]!=0;
            g_st[j]=g_inrs[j] && dk[j][61:59]==C_ST;
        end
        for(j=0;j<3;j=j+1) begin
            g_alurank[j]=0; g_wrank[j]=0; g_strank[j]=0;
            for(i=0;i<3;i=i+1) if(i<j) begin
                g_alurank[j]=g_alurank[j]+{1'b0,g_alu_a[i]};
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
            for(i=0;i<3;i=i+1) if(i<j && g_inrs_a[i] && g_rsid[i]==g_rsid[j]) g_rsrank[j]=g_rsrank[j]+2'd1;
        end
        for(k=0;k<4;k=k+1) begin
            g_need[k]=0;
            for(j=0;j<3;j=j+1) if(g_inrs_a[j] && g_rsid[j]==k[1:0]) g_need[k]=g_need[k]+2'd1;
        end
    end
    wire [2:0] rr_sum={1'b0,rr}+{2'b0,g_alu_a[0]}+{2'b0,g_alu_a[1]}+{2'b0,g_alu_a[2]};
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
    // Free-row choices are registered: ff_oh(t) is computed at t-1 from the rows free
    // then and not allocated by that cycle's dispatch. Rows freed on that edge are not
    // offered (as in the rsfree_q counts), so every dispatch still finds its rows.
    // Both outcomes of this cycle's dispatch are computed; fire selects at the end.
    genvar fk, fi;
    generate for(fk=0;fk<4;fk=fk+1) for(fi=0;fi<3;fi=fi+1) begin: ffsel
        always @(posedge clk)
            if(rst || flush_q) ff_oh[fk][fi]<=nth_free_oh(rsbusy({NR{1'b0}},fk),fi);
            else if(!halted) ff_oh[fk][fi]<=fire ? nth_free_oh(rsbusy(rs_v | alloc_cols,fk),fi) : nth_free_oh(rsbusy(rs_v,fk),fi);
    end endgenerate
    always @* alloc_cols=row_oh[0] | row_oh[1] | row_oh[2];
    reg [1:0] gai, gbi;
    reg [5:0] row[0:2];
    reg [5:0] phys[0:2], oldp[0:2], t1[0:2], t2[0:2];
    reg [2:0] mybank[0:2], b1[0:2], b2[0:2];
    reg [5:0] pt1[0:2], pt2[0:2], pop[0:2];
    reg [NR-1:0] pdep1[0:2], pdep2[0:2];
    reg [NR-1:0] newdep[0:2], grpdep[0:2];
    // RAT-derived producer column and pending bit per source. d2_fresh: loaded last
    // cycle, so last cycle's clears (clear_qq) still apply; while waiting, clears are
    // folded into the pending bits.
    reg [5:0] d2_c1[0:2], d2_c2[0:2];
    reg [2:0] d2_p1=0, d2_p2=0;
    reg d2_fresh=0;
    reg [NR-1:0] clear_qq=0;
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
    reg [5:0] rseq;
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
            phys[j]=fl_win[{1'b0,fl_used}+{1'b0,wrank[j]}];
            mybank[j]=d2_cls[j]==C_MUL || d2_cls[j]==C_DIV ? 3'd4 : {1'b0,rsid[j]};
        end
        for(j=0;j<3;j=j+1) begin
            // RAT values were read in D1; producers earlier in this group override them.
            pt1[j]=d2_t1[j]; pt2[j]=d2_t2[j]; pop[j]=d2_op[j]; b1[j]=d2_b1[j]; b2[j]=d2_b2[j];
            // Bitwise (decode, then mask) rather than a lookup of clear_qq by column.
            pdep1[j]=0; pdep2[j]=0;
            if(d2_p1[j]) pdep1[j][d2_c1[j]]=1'b1;
            if(d2_p2[j]) pdep2[j][d2_c2[j]]=1'b1;
            if(d2_fresh) begin pdep1[j]=pdep1[j] & ~clear_qq; pdep2[j]=pdep2[j] & ~clear_qq; end
            for(i=0;i<3;i=i+1) begin   // previous group's writers (later ones win)
                if(d2_pm1[j][i]) begin pt1[j]=pg_phys[i]; b1[j]=pg_bank[i]; pdep1[j]=pg_dep[i]; end
                if(d2_pm2[j][i]) begin pt2[j]=pg_phys[i]; b2[j]=pg_bank[i]; pdep2[j]=pg_dep[i]; end
                if(d2_pmo[j][i]) pop[j]=pg_phys[i];
            end
            t1[j]=pt1[j]; t2[j]=pt2[j]; oldp[j]=pop[j];
            ga=0; gb=0; gai=0; gbi=0;
            for(i=0;i<3;i=i+1) if(i<j) begin   // later producers in the group win
                if(d2_m1[j][i]) begin t1[j]=phys[i]; b1[j]=mybank[i]; ga=1; gai=i[1:0]; end
                if(d2_m2[j][i]) begin t2[j]=phys[i]; b2[j]=mybank[i]; gb=1; gbi=i[1:0]; end
                if(d2_wm[j][i]) oldp[j]=phys[i];
            end
            // RAT-derived columns are masked by this cycle's clears (they were masked
            // by earlier clears in D1 or while waiting). Group producers are new rows.
            newdep[j]=(ga ? {NR{1'b0}} : pdep1[j]) | (gb ? {NR{1'b0}} : pdep2[j]);
            grpdep[j]=0;
            if(ga) grpdep[j]=grpdep[j] | row_oh[gai];
            if(gb) grpdep[j]=grpdep[j] | row_oh[gbi];
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
            newpay[j][P_B1+:3]=b1[j];
            newpay[j][P_B2+:3]=b2[j];
            newpay[j][P_U1]=d2_u1[j];
            newpay[j][P_U2]=d2_u2[j];
            newpay[j][P_DST+:6]=phys[j];
            newpay[j][P_WE]=wr[j];
            newpay[j][P_ROB+:5]=rob_tail+j[4:0];
            rseq={rob_tw,rob_tail}+{4'b0,j[1:0]}; newpay[j][P_RW]=rseq[5];
            newpay[j][P_SQ+:4]=snap;
            newpay[j][P_LW]=d2_lw[j] && d2_cls[j]==C_LD && d2_fd[j]==7;
            newpay[j][P_FWD]=d2_lw[j] && d2_cls[j]==C_LD && d2_fd[j]!=7;
            newpay[j][P_FIDX+:3]=snap[2:0]-3'd1-d2_fd[j];
            newpay[j][P_MM]=d2_mm[j] && d2_cls[j]==C_LD;
        end
    end
    // ------------------------------------------------------------------ D1 RAT read
    // The group entering D2 reads the RAT a cycle early. Pending bits cleared by
    // clear_q this cycle are masked here; at a flush the restored committed map is
    // read. RAT writes by the group dispatching from D2 on the same edge are applied
    // in D2 from registered copies of that group's writers (pg_*), and a group waiting
    // in D2 at a flush re-reads the committed map for its own sources.
    reg [4:0] rr_a;
    reg [5:0] rr_tag[0:2][0:2];   // [slot][r1, r2, rd]
    reg [2:0] rr_bk[0:2][0:1];
    reg rr_pend[0:2][0:1];
    reg [5:0] rr_col[0:2][0:1];
    integer rq, rm;
    reg [5:0] rr_ct;
    reg [2:0] rr_cb;
    always @* for(j=0;j<3;j=j+1) for(rq=0;rq<3;rq=rq+1) begin
        rr_a=rq==0 ? dk[j][53:49] : rq==1 ? dk[j][48:44] : dk[j][58:54];
        // D1 does not advance in a flush cycle, so the RAT is read directly.
        rr_tag[j][rq]=rat_p[rr_a];
        if(rq<2) rr_bk[j][rq]=rat_b[rr_a];
        if(rq<2) begin rr_pend[j][rq]=rat_pend[rr_a] && rr_a!=0; rr_col[j][rq]=rat_c[rr_a]; end
    end
    // Committed map for the sources of the group waiting in D2 (used at a flush).
    reg [5:0] fr_t1[0:2], fr_t2[0:2], fr_op[0:2];
    reg [2:0] fr_b1[0:2], fr_b2[0:2];
    always @* for(j=0;j<3;j=j+1) begin
        fr_t1[j]=crat_now[d2_r1[j]]; fr_b1[j]=crat_now_b[d2_r1[j]];
        fr_t2[j]=crat_now[d2_r2[j]]; fr_b2[j]=crat_now_b[d2_r2[j]];
        fr_op[j]=crat_now[d2_rd[j]];
    end

    wire fire=!halted && |d2_v && !pend_v && !rd_v && !flush_q &&
        rob_cnt<=6'd29 && ({1'b0,sq_cnt}+{2'b0,nst}<=5'd8) && rs_ok && fl_cnt>=6'd3;
    // ROB and free-list space are checked conservatively for a full group (no adder
    // on the dispatch decision); measured cycle-identical on the 10-iteration run.
    // D1 does not advance during a flush cycle (its RAT read is the speculative map);
    // a group already waiting in D2 re-reads the committed map instead.
    wire d2_free=(!(|d2_v) && !flush_q) || fire;


    reg [4:0] xri, xrj;
    genvar gx;
    always @* begin
        for(k=0;k<4;k=k+1) begin xw_en[k]=0; xw_idx[k]=0; end
        for(j=0;j<3;j=j+1) begin
            xri=rob_tail+j[4:0];
            if({1'b0,rob_cnt}+j<32) begin
                xw_en[xri[1:0]]=1'b1; xw_idx[xri[1:0]]=xri[4:2];
            end
        end
        for(k=0;k<4;k=k+1) begin
            xs_tgt=xs_pc[k]+xs_imm[k]; xs_link=xs_pc[k]+32'd4;
            xw_data[k]={xs_pc[k][15:2],xs_au[k] ? xs_tgt : xs_imm[k],xs_tgt,xs_link[16:2],xs_pn[k][16:2],
                        xs_pt[k],xs_tgt!=xs_pn[k],xs_ghr[k]};
        end
    end
    // Side-table write stage registers (bank-ordered copies of the D2 group).
    reg [4:0] xsj;
    always @(posedge clk) begin
        for(j=0;j<3;j=j+1) begin
            xsj=rob_tail+j[4:0];
            xs_pc[xsj[1:0]]<=d2_pc[j]; xs_imm[xsj[1:0]]<=d2_imm[j]; xs_pn[xsj[1:0]]<=d2_pn[j];
            xs_ghr[xsj[1:0]]<=d2_ghr[j]; xs_pt[xsj[1:0]]<=d2_pt[j]; xs_au[xsj[1:0]]<=d2_insn[j][6:0]==7'h17;
        end
    end
    wire [4:0] r_rob[0:3];
    assign r_rob[0]=r_pay[0][P_ROB+:5]; assign r_rob[1]=r_pay[1][P_ROB+:5];
    assign r_rob[2]=r_pay[2][P_ROB+:5]; assign r_rob[3]=r_pay[3][P_ROB+:5];
    wire [XW-1:0] xbank_rd[0:3][0:3];
    generate for(g=0;g<4;g=g+1) begin: xtab
        localparam [1:0] G=g;
        // One copy per reader (issue ports 0-3, commit), each with its own write
        // enable/address registers so no write-address net spans every copy.
        for(gx=0;gx<5;gx=gx+1) begin: rd
            (* ram_style="distributed" *) reg [XW-1:0] mem[0:7];
            (* keep="true", equivalent_register_removal="no" *) reg we=0;
            (* keep="true", equivalent_register_removal="no" *) reg [2:0] wi=0;
            always @(posedge clk) begin we<=resetn && xw_en[g]; wi<=xw_idx[g]; end
            always @(posedge clk) if(we) mem[wi]<=xw_data[g];
            if(gx<4) begin: port
                assign xbank_rd[g][gx]=mem[r_rob[gx][4:2]];
            end else begin: commit
                assign xr_commit[g]=mem[cra[g]];
            end
        end
    end endgenerate
    // Commit-side {rd, new phys, old phys}, same four-bank layout.
    wire [19:0] crd_bank[0:3];
    reg [19:0] crw[0:3];
    always @* begin
        for(k=0;k<4;k=k+1) crw[k]=0;
        for(j=0;j<3;j=j+1) begin xrj=rob_tail+j[4:0]; crw[xrj[1:0]]={mybank[j],d2_rd[j],phys[j],oldp[j]}; end
    end
    generate for(gx=0;gx<4;gx=gx+1) begin: ctab
        localparam [1:0] G=gx;
        (* ram_style="distributed" *) reg [19:0] mem[0:7];
        always @(posedge clk) if(xw_en[gx]) mem[xw_idx[gx]]<=crw[gx];
        assign crd_bank[gx]=mem[cra[gx]];
    end endgenerate
    wire [4:0] cr_rd[0:2];
    wire [5:0] cr_pd[0:2], cr_po[0:2];
    wire [2:0] cr_b[0:2];
    // Bank selects use a local copy of the head's low bits (rob_head fans out widely).
    (* keep="true", equivalent_register_removal="no" *) reg [1:0] crsel=0;
    assign {cr_b[0],cr_rd[0],cr_pd[0],cr_po[0]}=crd_bank[crsel];
    assign {cr_b[1],cr_rd[1],cr_pd[1],cr_po[1]}=crd_bank[crsel+2'd1];
    assign {cr_b[2],cr_rd[2],cr_pd[2],cr_po[2]}=crd_bank[crsel+2'd2];
    generate for(gx=0;gx<4;gx=gx+1) begin: xread
        assign xr_port[gx]=xbank_rd[r_rob[gx][1:0]][gx];
    end endgenerate

    // ------------------------------------------------------------------ select
    reg [NR-1:0] req, grant, clear, lat0, req_ref;
    reg [15:0] rsreq;
    reg [3:0] gany;
    reg [2:0] cls_r;
    wire div_wake=div_run && div_cnt==5'd30;
    // Requests are registered: req_q holds this cycle's request, computed last cycle
    // from registered state and last cycle's grants/clears (see req_n). The select
    // loop is req_q -> grant -> clear -> req_q; the payload mux follows the grant.
    reg [NR-1:0] req_q=0, new_q=0;   // new_q: rows allocated by last cycle's dispatch
    always @* begin
        for(r=0;r<NR;r=r+1) begin
            cls_r=rs_pay[r][P_CLS+:3];
            lat0[r]=!(cls_r==C_LD || cls_r==C_MUL || cls_r==C_DIV);
            // A divide issues only as the oldest instruction, so no two divides can
            // overlap and a multiply need not see a divide granted one cycle earlier
            // (their results cannot collide on port 4): the request loop then has no
            // grant -> divider-busy term.
            req_ref[r]=rs_v[r] && !rs_iss[r] && (rs_dep[r] & ~sat)==0 &&
                (!(cls_r==C_MUL) || !div_busy || div_new) && (!(cls_r==C_DIV) || (!div_busy && rs_ath[r])) &&
                (!rs_pay[r][P_MM] || rs_ath[r]) && (!rs_pay[r][P_LW] || rs_sqok[r]);
            req[r]=req_q[r];
        end
        for(r=0;r<NR;r=r+1) begin
            rsreq=~rsbusy(~req,rs_k[r]);
            // SELPOS=1 grants the lowest ready slot instead of the oldest.
            grant[r]=req[r] && (rsreq & (SELPOS!=0 ? ~(16'hffff<<rs_s[r]) : rs_age[r]))==0;
        end
        for(k=0;k<4;k=k+1) gany[k]=rsbusy(~grant,k[1:0])!=16'hffff;
        for(r=0;r<NR;r=r+1)
            clear[r]=(grant[r] && lat0[r]) || (rs_v[r] && rs_iss[r] && rs_wc[r]==2'd1) ||
                (div_wake && r==div_row);
    end
    reg [15:0] age_keep[0:3];
    always @* for(k=0;k<4;k=k+1) age_keep[k]=~rsbusy(~rs_v,k[1:0]);
    // Next cycle's requests. Existing rows: dependences still unsatisfied after this
    // cycle (rs_dep & ~clear_q & ~sat, all registered) must all be woken by this
    // cycle's clears. Rows allocated now: the same test on the new dependence vector
    // (RAT columns read in D1, pending stores for loads); a group producer or a group
    // store blocks the first cycle, as do the head/store-queue load conditions.
    reg [NR-1:0] req_n, xwait, cl_st, cl_g, wk, wkf, w0;
    reg [NR-1:0] rq_xl[0:NR-1];
    reg [NR-1:0] rq_base;
    reg [2:0] nreq;
    reg div_busy_n, n_ga, n_gb, n_gs, n_q1, n_q2;
    reg [5:0] n_c1, n_c2;
    always @* begin
        div_busy_n=div_fin ? 1'b0 : div_busy;   // registered part only (see req_ref)
        // Clears not caused by this cycle's grants, from registers; the grant part
        // enters only as one term per column.
        for(r=0;r<NR;r=r+1) cl_st[r]=(rs_v[r] && rs_iss[r] && rs_wc[r]==2'd1) || (div_wake && r==div_row);
        cl_g=grant & lat0;
        w0=clear_q | sat | cl_st;
        wk=w0 | cl_g;   // column woken by the end of this cycle
        // A row dispatched now is woken only by registered state in its first cycle
        // (not by this cycle's grants), keeping dispatch out of the select loop.
        wkf=w0 | (d2_fresh ? clear_qq : {NR{1'b0}});
        for(r=0;r<NR;r=r+1) begin
            cls_r=rs_pay[r][P_CLS+:3];
            // Two parts: everything independent of this cycle's grants (rq_base,
            // including dependences that no grant can wake: not lat-0), and one
            // reduction over the grants of the remaining lat-0 producer columns.
            xwait=rs_dep[r] & ~w0;
            rq_xl[r]=xwait & lat0;
            rq_base[r]=!flush_q && rs_v[r] && !rs_iss[r] && (xwait & ~lat0)==0 &&
                (!(cls_r==C_MUL) || !div_busy_n) && (!(cls_r==C_DIV) || (!div_busy_n && rs_pay[r][P_ROB+:5]==rob_head)) &&
                (!rs_pay[r][P_MM] || rs_pay[r][P_ROB+:5]==rob_head) && (!rs_pay[r][P_LW] || rs_pay[r][P_SQ+:4]==sq_head);
            req_n[r]=rq_base[r] && !grant[r] && (rq_xl[r] & ~grant)==0;
        end
        for(j=0;j<3;j=j+1) begin
            n_ga=|d2_m1[j]; n_gb=|d2_m2[j]; n_gs=0;
            for(i=0;i<3;i=i+1) if(i<j && isst[i]) n_gs=1'b1;
            // Each source has at most one producer column: look up whether it has
            // woken (registered wake state, or a lat-0 grant now) instead of reducing
            // a dependence vector.
            // (Freshly loaded RAT columns may also have woken last cycle: clear_qq.)
            n_c1=d2_c1[j]; n_q1=d2_p1[j]; n_c2=d2_c2[j]; n_q2=d2_p2[j];
            for(i=0;i<3;i=i+1) begin
                if(d2_pm1[j][i]) begin n_c1=pg_row[i]; n_q1=pg_p[i]; end
                if(d2_pm2[j][i]) begin n_c2=pg_row[i]; n_q2=pg_p[i]; end
            end
            nreq[j]=!n_ga && !n_gb && !(isld[j] && n_gs) && !(n_q1 && !wkf[n_c1]) && !(n_q2 && !wkf[n_c2]) &&
                !(isld[j] && (st_pend & ~w0)!=0) &&
                (!(d2_cls[j]==C_MUL) || !div_busy_n) && d2_cls[j]!=C_DIV &&
                !(d2_mm[j] && d2_cls[j]==C_LD) && !(d2_lw[j] && d2_cls[j]==C_LD && d2_fd[j]==7);
        end
        // A row being allocated is free (its existing-row term is 0), so the new-row
        // request is ORed in.
        for(r=0;r<NR;r=r+1) for(j=0;j<3;j=j+1) req_n[r]=req_n[r] | (!flush_q && row_oh[j][r] && fire && nreq[j]);
    end
    // Select carries tags/control only; immediate, PC, predicted next PC and
    // Only operand tags, operand-use flags and operand-source selects pass through the
    // select mux; all other fields are read in R from the registered one-hot grant.
    localparam [PW-1:0] PSMALL=({{(PW-6){1'b0}},6'h3f}<<P_T1) | ({{(PW-6){1'b0}},6'h3f}<<P_T2) |
        ({{(PW-3){1'b0}},3'h7}<<P_B1) | ({{(PW-3){1'b0}},3'h7}<<P_B2) |
        ({{(PW-1){1'b0}},1'b1}<<P_U1) | ({{(PW-1){1'b0}},1'b1}<<P_U2) |
        ({{(PW-2){1'b0}},2'b11}<<P_ASEL) | ({{(PW-1){1'b0}},1'b1}<<P_BIMM) | ({{(PW-5){1'b0}},5'h1f}<<P_ROB);
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
        prod_v[4]=(mq2_v && mq2_we) || (div_last && div_we); prod_tag[4]=mq2_v ? mq2_tag : div_tag;
    end
    genvar pq, pr;
    // Operand reads: each bank has its own registered copy of every operand tag (the
    // tags otherwise fan out to every LUTRAM of all five banks), and the operand's
    // bank field selects the result; no live-value-table lookup on this path.
    wire [31:0] bank_op[0:4][0:7];
    generate for(pq=0;pq<5;pq=pq+1) begin: prf_bank
        (* ram_style="distributed" *) reg [31:0] bank[0:63];
        integer n;
        initial for(n=0;n<64;n=n+1) bank[n]=0;
        always @(posedge clk) if(res_v[pq]) bank[res_tag[pq]]<=res_vw[pq];
        for(pr=0;pr<12;pr=pr+1) begin: rd
            assign bank_rd[pq][pr]=bank[prf_rt[pr]];
        end
        for(pr=0;pr<8;pr=pr+1) begin: op
            (* keep="true", equivalent_register_removal="no" *) reg [5:0] t=0;
            always @(posedge clk) if(!rst && !halted) t<=gpay[pr/2][(pr%2 ? P_T2 : P_T1)+:6];
            assign bank_op[pq][pr]=bank[t];
        end
    end endgenerate
    generate for(pr=0;pr<12;pr=pr+1) begin: prf_read
        assign prf_rd[pr]=bank_rd[lvt[prf_rt[pr]]][pr];
    end endgenerate
    generate for(pr=0;pr<4;pr=pr+1) begin: prf_ops
        assign prf_rt[2*pr]=r_pay[pr][P_T1+:6];
        assign prf_rt[2*pr+1]=r_pay[pr][P_T2+:6];
    end endgenerate
    assign prf_rt[8]=cr_pd[0]; assign prf_rt[9]=cr_pd[1]; assign prf_rt[10]=cr_pd[2];
    assign prf_rt[11]=crat_now[debug_reg_addr];
    function [31:0] wbyp;   // register value including results being written now
        input [5:0] t; input [31:0] fileval; integer n; reg hit;
        begin
            wbyp=0; hit=0;
            for(n=0;n<5;n=n+1) if(res_v[n] && res_tag[n]==t) begin wbyp=wbyp | res_val[n]; hit=1; end
            if(!hit) wbyp=fileval;
        end
    endfunction
    reg [31:0] r_a[0:3], r_b[0:3], r_fa, r_fb;
    reg [4:0] r_eba[0:3], r_ebb[0:3];
    reg [2:0] r_ba, r_bb;
    reg [5:0] r_ta, r_tb;
    always @* for(k=0;k<4;k=k+1) begin
        r_ta=r_pay[k][P_T1+:6]; r_tb=r_pay[k][P_T2+:6]; r_ba=r_pay[k][P_B1+:3]; r_bb=r_pay[k][P_B2+:3];
        // Only the operand's own bank can hold or be writing its value.
        r_fa=res_v[r_ba] && res_tag[r_ba]==r_ta ? res_vw[r_ba] : bank_op[r_ba][2*k];
        r_fb=res_v[r_bb] && res_tag[r_bb]==r_tb ? res_vw[r_bb] : bank_op[r_bb][2*k+1];
        r_a[k]=r_pay[k][P_ASEL+:2]==2 || !r_pay[k][P_U1] ? 32'b0 : r_fa;
        r_b[k]=r_pay[k][P_BIMM] ? xr_port[k][X_IMM-:32] : !r_pay[k][P_U2] ? 32'b0 : r_fb;
        for(q=0;q<5;q=q+1) begin
            r_eba[k][q]=r_pay[k][P_U1] && r_pay[k][P_ASEL+:2]==0 && r_ba==q && prod_v[q] && prod_tag[q]==r_ta;
            r_ebb[k][q]=r_pay[k][P_U2] && !r_pay[k][P_BIMM] && r_bb==q && prod_v[q] && prod_tag[q]==r_tb;
        end
    end
`ifdef VERILATOR
    // The bank-field read must equal the live-value-table read it replaces.
    always @(posedge clk) if(resetn && !halted) for(k=0;k<4;k=k+1) if(r_v[k]) begin
        if(r_pay[k][P_U1] && r_pay[k][P_ASEL+:2]==0 && r_eba[k]==0 && r_a[k]!=wbyp(r_pay[k][P_T1+:6],prf_rd[2*k])) $fatal(1,"bank read A mismatch");
        if(r_pay[k][P_U2] && !r_pay[k][P_BIMM] && r_ebb[k]==0 && r_b[k]!=wbyp(r_pay[k][P_T2+:6],prf_rd[2*k+1])) $fatal(1,"bank read B mismatch");
        for(q=0;q<5;q=q+1) begin
            if(r_pay[k][P_U1] && r_pay[k][P_ASEL+:2]==0 && prod_v[q] && prod_tag[q]==r_pay[k][P_T1+:6] && !r_eba[k][q]) $fatal(1,"bank bypass A mismatch");
            if(r_pay[k][P_U2] && !r_pay[k][P_BIMM] && prod_v[q] && prod_tag[q]==r_pay[k][P_T2+:6] && !r_ebb[k][q]) $fatal(1,"bank bypass B mismatch");
        end
    end
`endif

    // ------------------------------------------------------------------ E
    function [31:0] bmux;
        input [4:0] s; integer n;
        begin bmux=0; for(n=0;n<5;n=n+1) bmux=bmux | ({32{s[n]}} & res_val[n]); end
    endfunction
    reg [31:0] x_a[0:3], x_b[0:3], x_res[0:2], x_sum[0:2];
    reg [2:0] x_tk, x_flt, x_ct;
    reg [31:0] shl, shr, x_rest;
    // The non-adder part of each result is a kept net, so the late sum enters only
    // the final LUT of the result.
    /* verilator lint_off UNOPTFLAT */
    reg [31:0] x_restv[0:2];   // (x_a and x_res share an always block; not a real loop)
    /* verilator lint_on UNOPTFLAT */
    integer xk;
    reg [32:0] sh_sx;
    always @* for(xk=0;xk<3;xk=xk+1) begin
        // Two explicit stages: by 0/8/16/24 (4:1), then by 0-7 (8:1, LUT6+MUXF7).
        shl=(x_a[xk]<<{x_b[xk][4:3],3'b0})<<x_b[xk][2:0];
        sh_sx=$signed({e_ops[xk][A_SRA] & x_a[xk][31],x_a[xk]})>>>{x_b[xk][4:3],3'b0};
        shr=sh_sx[31:0]>>x_b[xk][2:0] | ~({32{1'b1}}>>x_b[xk][2:0]) & {32{sh_sx[32]}};
        x_rest=({32{e_ops[xk][A_SLL]}} & shl) | ({32{e_ops[xk][A_XOR]}} & (x_a[xk]^x_b[xk])) |
               ({32{e_ops[xk][A_SRL] | e_ops[xk][A_SRA]}} & shr) | ({32{e_ops[xk][A_OR]}} & (x_a[xk]|x_b[xk])) |
               ({32{e_ops[xk][A_AND]}} & (x_a[xk]&x_b[xk])) | ({32{e_ops[xk][A_B]}} & x_b[xk]) |
               ({32{e_ops[xk][11]}} & e_link[xk]);
        x_restv[xk]=x_rest;
    end
    /* verilator lint_off UNOPTFLAT */
    (* keep="true" *) wire [31:0] x_rest_k[0:2];
    /* verilator lint_on UNOPTFLAT */
    assign x_rest_k[0]=x_restv[0]; assign x_rest_k[1]=x_restv[1]; assign x_rest_k[2]=x_restv[2];
    reg [32:0] x_as;
    reg lts, ltu, cond;
    reg [3:0] e_isbr=0, e_isct=0, e_brt1=0, e_jt1=0, e_sx=0;   // decoded in R (e_sx: signed compare)
    reg [3:0] e_blt=0, e_bneg=0;   // branch uses less-than / result inverted (f3[2], f3[0])
    /* verilator lint_off UNOPTFLAT */
    reg [2:0] x_eqv;
    /* verilator lint_on UNOPTFLAT */
    /* verilator lint_off UNOPTFLAT */
    (* keep="true" *) wire [2:0] x_eqk=x_eqv;
    /* verilator lint_on UNOPTFLAT */
    always @* begin
        for(k=0;k<4;k=k+1) begin
            // v28-A: bit-identical rewrite of the case() priority mux using the
            // registered strict one-hot (bit 5 = default bank value e_a/e_b).
            x_a[k]=({32{e_so_a[k][0]}}&res_e[k][0])|({32{e_so_a[k][1]}}&res_e[k][1])|({32{e_so_a[k][2]}}&res_e[k][2])|
                   ({32{e_so_a[k][3]}}&res_e[k][3])|({32{e_so_a[k][4]}}&res_e[k][4])|({32{e_so_a[k][5]}}&e_a[k]);
            x_b[k]=({32{e_so_b[k][0]}}&res_e[k][0])|({32{e_so_b[k][1]}}&res_e[k][1])|({32{e_so_b[k][2]}}&res_e[k][2])|
                   ({32{e_so_b[k][3]}}&res_e[k][3])|({32{e_so_b[k][4]}}&res_e[k][4])|({32{e_so_b[k][5]}}&e_b[k]);
        end
        for(k=0;k<3;k=k+1) begin
            // One carry chain per port: a+b, or a-b whose carry/sign give the
            // unsigned/signed less-than results for SLT(U) and branches.
            // Operands are sign- (e_sx) or zero-extended to 33 bits, so bit 32 of
            // a-b is the less-than result for SLT/SLTU and branches.
            x_as={e_sx[k] & x_a[k][31],x_a[k]}+({e_sx[k] & x_b[k][31],x_b[k]} ^ {33{e_inv[k]}})+{32'b0,e_inv[k]};
            x_sum[k]=x_as[31:0];
            ltu=x_as[32]; lts=x_as[32];
            // One-hot select: unit results are masked and ORed (bit 11 = link). The
            // carry-chain results (sum, SLT/SLTU) are ORed in last.
            x_res[k]=x_rest_k[k] | ({32{e_ops[k][A_ADD] | e_ops[k][A_SUB]}} & x_sum[k]) |
                     {31'b0,(e_ops[k][A_SLT] | e_ops[k][A_SLTU]) & x_as[32]};
            // One-hot branch type {geu,ltu,ge,lt,ne,eq} decoded in R.
            // Less-than branches use bit 32 of a-b; f3[0] inverts (BNE/BGE/BGEU). The
            // equality term is a kept net, so taken/fault are one LUT after the carry.
            x_eqv[k]=(x_a[k]==x_b[k]) ^ e_bneg[k];
            cond=e_blt[k] ? (x_as[32] ^ e_bneg[k]) : x_eqk[k];
            x_ct[k]=e_isct[k];
            x_tk[k]=e_isbr[k] ? cond : e_isct[k];
            // Branch/JAL targets are known in R (misaligned-target flags decoded there);
            // JALR faults are checked in W.
            x_flt[k]=(e_brt1[k] && cond) || e_jt1[k];
        end
    end
    // Memory port E1
    wire [31:0] m_addr=x_a[3]+e_imm[3];
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
    // Store-merge hits are decided in E1 (registered): wq/sh in E2 are exactly the
    // commit store and wq of the E1 cycle. (base+imm)[15:2]==w is tested carry-free.
    reg m2_wqhit=0, m2_shhit=0;
    wire l_wqhit=m2_wqhit, l_shhit=m2_shhit;
    function addr_eq;
        input [31:0] a, b; input [13:0] w;
        reg [16:2] rc; reg c2, ok; integer n;
        begin
            c2=(a[1] & b[1]) | ((a[1] ^ b[1]) & a[0] & b[0]);
            for(n=2;n<16;n=n+1) rc[n]=a[n] ^ b[n] ^ w[n-2];
            ok=rc[2]==c2;
            for(n=2;n<15;n=n+1) ok=ok && (((a[n] & b[n]) | ((a[n] ^ b[n]) & rc[n]))==rc[n+1]);
            addr_eq=ok;
        end
    endfunction
    // Byte sources are chosen from registered compares (ready before the RAM output),
    // then one level aligns the first byte(s) and one extends: no shifter after RAM.
    reg [7:0] l_byte[0:3], l_b0, l_b1;
    reg [31:0] l_val;
    reg l_sx;
    always @* begin
        for(i=0;i<4;i=i+1)
            l_byte[i]=m2_fwd && m2_fmask[i] ? m2_fdata[i*8+:8] : l_wqhit && wq_mask[i] ? wq_data[i*8+:8] :
                      l_shhit && sh_mask[i] ? sh_data[i*8+:8] : ldata[i*8+:8];
        l_b0=l_byte[m2_off];
        l_b1=m2_off==2'd0 ? l_byte[1] : m2_off==2'd1 ? l_byte[2] : l_byte[3];   // halfword offsets 0-2
        l_sx=!m2_f3[2];
        l_val[7:0]=l_b0;
        l_val[15:8]=m2_f3[1:0]==2'd0 ? {8{l_sx && l_b0[7]}} : l_b1;
        l_val[31:16]=m2_f3[1:0]==2'd2 ? {l_byte[3],l_byte[2]} :
                     m2_f3[1:0]==2'd0 ? {16{l_sx && l_b0[7]}} : {16{l_sx && l_b1[7]}};
    end
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

    always @* begin
        for(k=0;k<3;k=k+1) res_nx[k]=x_res[k];
        res_nx[3]=m2_cnt ? cycles : l_val;
        // v27: mul result taken from registered mp1 one cycle earlier (mq2 stage);
        // mp2/mq3 removed. A mul never shares its writeback cycle with div_last:
        // a mul may enter E only when div_busy_n is low (div_fin cycle at the
        // latest, so its writeback is >=2 cycles later) and a div's first possible
        // finish is >30 cycles after it could have been granted behind any in-flight
        // mul. Asserted sim-only at the mq2 collision point.
        res_nx[4]=mq2_v ? (mq2_hi ? mp1[63:32] : mp1[31:0]) :
                div_bz ? (div_rem ? da0 : 32'hffffffff) :
                div_rem ? (div_negr ? -dr : dr) : (div_negq ? -dq : dq);
    end
    genvar ek, eq;
    generate for(ek=0;ek<4;ek=ek+1) for(eq=0;eq<5;eq=eq+1) begin: rescopy
        always @(posedge clk) if(!rst && !halted) res_e[ek][eq]<=res_nx[eq];
    end endgenerate
`ifdef VERILATOR
    // v28-A sim-only: the registered one-hot selects are exact and the AND-OR
    // mux equals the original e_sa/e_sb case() priority mux for every value.
    reg [31:0] x_aold[0:3], x_bold[0:3];
    integer vka;
    always @* begin
      for(vka=0;vka<4;vka=vka+1) begin
        x_aold[vka]=e_eba[vka][0] ? res_e[vka][0] : e_eba[vka][1] ? res_e[vka][1] : e_eba[vka][2] ? res_e[vka][2] :
                   e_eba[vka][3] ? res_e[vka][3] : e_eba[vka][4] ? res_e[vka][4] : e_a[vka];
        x_bold[vka]=e_ebb[vka][0] ? res_e[vka][0] : e_ebb[vka][1] ? res_e[vka][1] : e_ebb[vka][2] ? res_e[vka][2] :
                   e_ebb[vka][3] ? res_e[vka][3] : e_ebb[vka][4] ? res_e[vka][4] : e_b[vka];
      end
      if(!rst) begin
        for(vka=0;vka<4;vka=vka+1) begin
            if($countones(e_so_a[vka])!=1 || $countones(e_so_b[vka])!=1) begin
                $fwrite(32'h80000002,"v28-A select not onehot k=%0d a=%b b=%b\n", vka, e_so_a[vka], e_so_b[vka]);
                $fatal(1,"v28-A select not onehot");
            end
            if(x_aold[vka]!==x_a[vka] || x_bold[vka]!==x_b[vka]) begin
                $fwrite(32'h80000002,"v28-A mux mismatch k=%0d cycle=%0d\n", vka, cycles);
                $fatal(1,"v28-A operand mux mismatch");
            end
        end
      end
    end
`endif

    // ------------------------------------------------------------------ W: branch check / redirect
    // Seven candidates (branch/JAL per ALU port, last cycle's JALR per port, load
    // replay). The oldest wins by pairwise age comparison, not a serial scan.
    reg [6:0] cand_v, win;
    reg [4:0] cand_rob[0:6];
    reg [5:0] cand_seq[0:6], age_d;
    reg [6:0] cand_old[0:6], cand_opend;   // [a][b]: candidate a is older than b / than the pending redirect
    // Ages are compared a cycle early from the registers that become next cycle's
    // candidates (E-stage branches, W-stage JALRs, E2 loads) and registered.
    reg [6:0] cold_q[0:6], copend_q;
    reg [5:0] nseq[0:6], cseq_d;
    reg [6:0] p1n, p2n[0:6];
    reg [31:0] cand_pc[0:6], rd_pc_n;
    reg [9:0] cand_ghr[0:6], rd_ghr_n;
    reg [4:0] rd_rob_n;
    reg rd_rw_n;
    reg [2:0] w_mis;
    reg acc;
    integer ca, cb;
    always @* begin
        for(k=0;k<3;k=k+1) begin
            w_mis[k]=wb_v[k] && wb_brj[k] && !wb_flt[k] && (wb_tk[k]!=wb_ptk[k] || (wb_tk[k] && wb_tgtbad[k]));
            cand_v[k]=w_mis[k]; cand_rob[k]=wb_rob[k]; cand_pc[k]=wb_next[k]; cand_ghr[k]={wb_ghr[k][8:0],wb_tk[k]};
            cand_seq[k]={wb_rw[k],wb_rob[k]}; cand_seq[k+3]={jc_rw[k],jc_rob[k]};
            cand_v[k+3]=jc_v[k] && jc_mis[k]; cand_rob[k+3]=jc_rob[k]; cand_pc[k+3]=jc_pc[k]; cand_ghr[k+3]=jc_ghr[k];
        end
        cand_v[6]=lwb_v && lwb_rep; cand_rob[6]=lwb_rob; cand_pc[6]=lwb_pc; cand_ghr[6]=lwb_ghr;
        cand_seq[6]={lwb_rw,lwb_rob};
        // Ages from registered sequence numbers only (live entries differ by < 32),
        // compared last cycle (cold_q/copend_q).
        for(ca=0;ca<7;ca=ca+1) begin cand_opend[ca]=copend_q[ca]; cand_old[ca]=cold_q[ca]; end
        acc=0; rd_pc_n=0; rd_ghr_n=0; rd_rob_n=0; rd_rw_n=0;
        for(ca=0;ca<7;ca=ca+1) begin
            win[ca]=cand_v[ca] && (!pend_v || cand_opend[ca]);
            for(cb=0;cb<7;cb=cb+1) if(cb!=ca && cand_v[cb] && !cand_old[ca][cb]) win[ca]=0;
            rd_rw_n=rd_rw_n | (win[ca] && cand_seq[ca][5]);
            acc=acc | win[ca];
            rd_pc_n=rd_pc_n | ({32{win[ca]}} & cand_pc[ca]);
            rd_ghr_n=rd_ghr_n | ({10{win[ca]}} & cand_ghr[ca]);
            rd_rob_n=rd_rob_n | ({5{win[ca]}} & cand_rob[ca]);
        end
        acc=acc && !halted && !flush_q;
    end
    always @* begin
        for(k=0;k<3;k=k+1) begin nseq[k]={e_pay[k][P_RW],e_pay[k][P_ROB+:5]}; nseq[k+3]={wb_rw[k],wb_rob[k]}; end
        nseq[6]={m2_rw,m2_rob};
        for(ca=0;ca<7;ca=ca+1) begin
            cseq_d=nseq[ca]-{pend_rw,pend_rob}; p1n[ca]=cseq_d[5];
            for(cb=0;cb<7;cb=cb+1) begin cseq_d=nseq[ca]-cand_seq[cb]; p2n[ca][cb]=cseq_d[5]; end
        end
    end
    always @(posedge clk) if(!rst && !halted) for(ca=0;ca<7;ca=ca+1) begin
        for(cb=0;cb<7;cb=cb+1) begin cseq_d=nseq[ca]-nseq[cb]; cold_q[ca][cb]<=cseq_d[5]; end
        copend_q[ca]<=acc ? |(p2n[ca] & win) : p1n[ca];
    end

    // ------------------------------------------------------------------ commit
    wire [4:0] h0=rob_head, h1=rob_h1, h2=rob_h2;
    // Head window: status {tcnt, ct, st, mis, rep, flt, cpl} of entries head..head+2,
    // held in registers. Next cycle's window is chosen by the commit count from the
    // six entries head..head+5 as they will be after this edge's ROB writes.
    reg [6:0] hw[0:2];
    // Commit-window row of each four-way bank (entries head..head+3), registered from
    // the next head so the commit-side table reads start at a flop.
    reg [2:0] cra[0:3];
    initial for(i=0;i<4;i=i+1) cra[i]=0;
    function [2:0] crow; input [4:0] hd; input [1:0] gb; reg [4:0] e;
        begin e=hd+{3'b0,gb-hd[1:0]}; crow=e[4:2]; end
    endfunction
    reg [6:0] hcand[0:5];
    // Entry indices head+0..5 are registers (rob_hm). An entry receives at most one
    // write per cycle (dispatch writes free entries, each instruction completes once),
    // so the post-write status is an AND-OR of the hitting writer, not a priority chain.
    reg [4:0] rob_hm[0:5];
    initial for(i=0;i<6;i=i+1) rob_hm[i]=i[4:0];
    function [6:0] rob_next;   // status of entry e after this edge's writes
        input [4:0] e; integer n; reg [4:0] di; reg cd, cc, cmh, crp; reg [6:0] old, dv; reg [2:0] fl;
        begin
            old={rob_tcnt[e],rob_ct[e],rob_st[e],rob_mis[e],rob_rep[e],rob_flt[e],rob_cpl[e]};
            cd=0; dv=0;
            for(n=0;n<3;n=n+1) begin
                di=rob_tail+n[4:0];
                if({1'b0,rob_cnt}+n<32 && di==e) begin cd=1;
                    dv=dv | {1'b0,inrs[n] && d2_cls[n]>=C_BR && d2_cls[n]<=C_JALR,isst[n],1'b0,1'b0,d2_fault[n],d2_nop[n] || d2_fault[n]}; end
            end
            if(cd) old=dv;
            cc=0; cmh=0; crp=0; fl=0; rob_next=old;
            for(n=0;n<3;n=n+1) if(wb_v[n] && !wb_jalr[n] && wb_rob[n]==e) begin cc=1; cmh=1; fl[0]=fl[0] | wb_flt[n]; rob_next[3]=w_mis[n]; end
            for(n=0;n<3;n=n+1) if(jc_v[n] && jc_rob[n]==e) begin cc=1; cmh=1; fl[1]=fl[1] | jc_flt[n]; rob_next[3]=jc_mis[n]; end
            if(st_v && st_rob==e) begin cc=1; fl[2]=fl[2] | st_flt; end
            if(lwb_v && lwb_rob==e) begin cc=1; crp=1; fl[2]=fl[2] | lwb_flt; end
            if(mwb_v && mwb_rob==e) rob_next[0]=1'b1;
            if(cc) begin rob_next[0]=1'b1; rob_next[1]=|fl; end
            if(crp) begin rob_next[2]=lwb_rep; rob_next[6]=lwb_iscnt; end
            if(!cmh) rob_next[3]=old[3];
        end
    endfunction
    always @* for(k=0;k<6;k=k+1) hcand[k]=rob_next(rob_hm[k]);
    wire [2:0] sqh=sq_head[2:0];
    wire uart_ready;
    wire run=!halted && !flush_q;
    // UART flag of the head store, registered (next head's flag with this edge's
    // store-address write bypassed), so the commit decision does not start with an
    // eight-way read.
    reg su_q=0;
    wire su=su_q;
    function sq_uart_nx; input [2:0] e; begin sq_uart_nx=st_v && st_sqi==e ? st_uart : sq_uart[e]; end endfunction
    wire stok=!su || (uart_ready && !uq_v);
    wire ok0=rob_cnt!=0 && hw[0][0] && !hw[0][1] && !hw[0][2];
    wire ok1=rob_cnt>1 && hw[1][0] && !hw[1][1] && !hw[1][2];
    wire ok2=rob_cnt>2 && hw[2][0] && !hw[2][1] && !hw[2][2];
    wire s0=hw[0][4], s1=hw[1][4], s2=hw[2][4], t0=hw[0][5], tc1=hw[1][5], tc2=hw[2][5];
    wire cm0=run && ok0 && (!s0 || stok);
    // A counter read commits as the oldest of its group, so the profiler's
    // retirement count at that edge covers exactly the older instructions.
    wire cm1=cm0 && !hw[0][3] && ok1 && !hw[1][6] && !(s0 && s1) && !(t0 && tc1) && (!s1 || stok);
    wire cm2=cm1 && !hw[1][3] && ok2 && !hw[2][6] && !((s0 || s1) && s2) && !((t0 || tc1) && tc2) && (!s2 || stok);
    wire trap=run && rob_cnt!=0 && hw[0][0] && hw[0][1];
    wire replay=run && rob_cnt!=0 && hw[0][0] && hw[0][2] && !hw[0][1];
    wire misflush=(cm0 && hw[0][3]) || (cm1 && hw[1][3]) || (cm2 && hw[2][3]);
    wire [1:0] ncm={1'b0,cm0}+{1'b0,cm1}+{1'b0,cm2};
    wire cst=(cm0 && s0) || (cm1 && s1) || (cm2 && s2);
    wire cw0=cm0 && cr_rd[0]!=0, cw1=cm1 && cr_rd[1]!=0, cw2=cm2 && cr_rd[2]!=0;
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
        // Read with the F0 address and registered (like the ROM); a training write on
        // the same edge is bypassed, so F1 sees exactly the asynchronous-read value.
        reg [4:0] mpq=0;
        always @* f1_mp[g]=mpq;
        // Registered per-bank training write keeps the address fanout local.
        (* max_fanout=24 *) reg mpw_v=0; (* max_fanout=24 *) reg [10:0] mpw_i=0; (* max_fanout=24 *) reg [2:0] mpw_d=0;
        always @(posedge clk) begin
            mpw_v<=resetn && !halted && lwb_v && lwb_rep && lwb_pc[3:2]==G;
            mpw_i<=lwb_pc[14:4]; mpw_d<=tr_d;
            // T1c MW-A: distance 7 ("wait for older stores to commit") is an
            // absorbing state — a load that waits never mispredicts, so the
            // entry is never retrained. Writing it when the replaying load
            // found no covering store (tr_d defaults to 7 exactly then) costs
            // the load store-queue ordering forever. Clear the entry instead:
            // optimistic forwarding is verified by l_conf and retrains on
            // mismatch. Same treatment for the read bypass.
            if(mpw_v) mem_pred[mpw_i]<= mpw_d==3'd7 ? 5'd0 : {1'b0,1'b1,mpw_d};
            mpq<=mpw_v && mpw_i==fpc[14:4] ? (mpw_d==3'd7 ? 5'd0 : {1'b0,1'b1,mpw_d}) : mem_pred[fpc[14:4]];
        end
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
        for(k=0;k<4;k=k+1) f1_w[k]<=imem[{fpc[14:4],fpc[3:2]+k[1:0]-fq_tail_n[1:0]}];
        f1_rot<=fpc[3:2]-fq_tail_n[1:0];
    end

    // Registers without a reset value are not gated by reset (they are don't-care until
    // the valid bits and pointers below are reset); reset is applied last and wins.
    always @(posedge clk) begin
        if(!halted) begin
            cycles<=cycles+1'b1;

            // ---------------- frontend
            if(rd_v) begin
                fpc<=rd_pc; ghr<=rd_ghr; pidx<=rd_pc[13:4]^rd_ghr; f1_v<=0;
            end else if(f0_go) begin
                fpc<=f_next; ghr<={ghr[8:0],f_any}; pidx<=f_pidx_n; f1_v<=1; f1_pc<=fpc; f1_sq<=fpc[31:4]+28'd1; f1_mask<=f_mask;
                f1_tslot<=f_tslot; f1_taken<=f_any; f1_tgt<=f_ttgt; f1_ghr<=ghr;
            end else f1_v<=0;
            if(rd_v) begin
                fq_head<=0; fq_tail<=0; fq_cnt<=0; d2_v<=0;
            end else begin
                fq_tail<=fq_tail+(fw_v ? {1'b0,f1_n} : 4'd0);
                fq_head<=fq_head+(d2_free ? {2'b0,d1_n} : 4'd0);
                fq_cnt<=fq_cnt+(fw_v ? {2'b0,f1_n} : 5'd0)-(d2_free ? {3'b0,d1_n} : 5'd0);
                if(d2_free) for(j=0;j<3;j=j+1) begin
                    d2_v[j]<=j<d1_n;
                    {d2_fault[j],d2_nop[j],d2_cls[j],d2_rd[j],d2_r1[j],d2_r2[j],d2_u1[j],d2_u2[j],
                     d2_imm[j],d2_aop[j],d2_asel[j],d2_bimm[j],d2_f3[j]}<=dk[j];
                    {d2_pc[j],d2_pn[j],d2_ghr[j],d2_mm[j],d2_lw[j],d2_fd[j],d2_pt[j],d2_insn[j]}<=fq_out[j][FQW-65:0];
                    d2_inrs[j]<=g_inrs[j]; d2_alu[j]<=g_alu[j]; d2_wr[j]<=g_wr[j]; d2_st[j]<=g_st[j];
                    d2_t1[j]<=rr_tag[j][0]; d2_t2[j]<=rr_tag[j][1]; d2_op[j]<=rr_tag[j][2];
                    d2_b1[j]<=rr_bk[j][0]; d2_b2[j]<=rr_bk[j][1];
                    d2_c1[j]<=rr_col[j][0]; d2_c2[j]<=rr_col[j][1]; d2_p1[j]<=rr_pend[j][0]; d2_p2[j]<=rr_pend[j][1];
                    for(i=0;i<3;i=i+1) begin
                        d2_pm1[j][i]<=fire && wr[i] && d2_rd[i]==dk[j][53:49];
                        d2_pm2[j][i]<=fire && wr[i] && d2_rd[i]==dk[j][48:44];
                        d2_pmo[j][i]<=fire && wr[i] && d2_rd[i]==dk[j][58:54];
                    end
                    d2_rsid[j]<=g_rsid[j]; d2_rsrank[j]<=g_rsrank[j]; d2_wrank[j]<=g_wrank[j]; d2_strank[j]<=g_strank[j];
                    for(i=0;i<3;i=i+1) begin
                        d2_m1[j][i]<=i<j && !dk[i][63] && dk[j][43] && dk[j][53:49]!=0 && dk[j][53:49]==dk[i][58:54];
                        d2_m2[j][i]<=i<j && !dk[i][63] && dk[j][42] && dk[j][48:44]!=0 && dk[j][48:44]==dk[i][58:54];
                        d2_wm[j][i]<=i<j && !dk[i][63] && dk[j][58:54]!=0 && dk[j][58:54]==dk[i][58:54];
                    end
                end else for(j=0;j<3;j=j+1) begin
                    if(flush_q) begin   // group waiting in D2 re-reads the restored map
                        d2_t1[j]<=fr_t1[j]; d2_t2[j]<=fr_t2[j]; d2_op[j]<=fr_op[j];
                        d2_b1[j]<=fr_b1[j]; d2_b2[j]<=fr_b2[j];
                        d2_p1[j]<=0; d2_p2[j]<=0; pg_v[j]<=0; d2_pm1[j]<=0; d2_pm2[j]<=0; d2_pmo[j]<=0;
                    end else begin
                        d2_p1[j]<=d2_p1[j] && !(d2_fresh && clear_qq[d2_c1[j]]) && !clear_q[d2_c1[j]];
                        d2_p2[j]<=d2_p2[j] && !(d2_fresh && clear_qq[d2_c2[j]]) && !clear_q[d2_c2[j]];
                        pg_dep[j]<=pg_dep[j] & ~clear_q;
                        pg_p[j]<=pg_p[j] && !clear_q[pg_row[j]];
                    end
                end
                if(d2_free) for(j=0;j<3;j=j+1) begin
                    pg_v[j]<=fire && wr[j]; pg_rd[j]<=d2_rd[j]; pg_phys[j]<=phys[j]; pg_bank[j]<=mybank[j]; pg_dep[j]<=row_oh[j];
                    pg_row[j]<=row[j]; pg_p[j]<=1'b1;
                end
            end
            d2_fresh<=!rd_v && d2_free;
            clear_qq<=clear_q;
            if(!rd_v && d2_free) begin
                rr<=rr_next;
                for(k=0;k<4;k=k+1) d2_need[k]<=g_need[k];
            end
            if(tq_btb || tq_pht) begin tq_btb<=0; tq_pht<=0; end

            // ---------------- rename/dispatch
            clear_q<=clear;
            req_q<=req_n;
            new_q<=fire ? alloc_cols : {NR{1'b0}};
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
                for(j=0;j<3;j=j+1) if(wr[j]) begin rat_p[d2_rd[j]]<=phys[j]; rat_b[d2_rd[j]]<=mybank[j]; rat_c[d2_rd[j]]<=row[j]; rat_pend[d2_rd[j]]<=1; end
                {rob_tw,rob_tail}<={rob_tw,rob_tail}+{3'b0,nd};
                sq_tail<=sq_tail+{1'b0,nst};
            end

            // ---------------- reservation stations
            for(r=0;r<NR;r=r+1) begin
                if(clear[r]) begin rs_v[r]<=0; rs_iss[r]<=0; end
                else if(grant[r]) begin
                    rs_iss[r]<=1;
                    rs_wc[r]<=rs_pay[r][P_CLS+:3]==C_LD ? 2'd1 : rs_pay[r][P_CLS+:3]==C_MUL ? 2'd2 : 2'd0;
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
            div_new<=0;
            for(r=0;r<NA;r=r+1) if(grant[r] && rs_pay[r][P_CLS+:3]==C_DIV) begin div_busy<=1; div_new<=1; div_row<=r[5:0]; end

            // ---------------- R -> E
            for(k=0;k<4;k=k+1) begin
                r_v[k]<=gany[k]; r_pay[k]<=gpay[k];
                for(r=0;r<NR;r=r+1) r_gnt[k][r]<=grant[r] && rs_k[r]==k[1:0];
                e_v[k]<=r_v[k]; e_pay[k]<=r_full[k]; e_a[k]<=r_a[k]; e_b[k]<=r_b[k];
                e_eba[k]<=r_eba[k]; e_ebb[k]<=r_ebb[k];
                e_so_a[k]<={!(|r_eba[k]), r_eba[k][4]&~(|r_eba[k][3:0]), r_eba[k][3]&~(|r_eba[k][2:0]),
                            r_eba[k][2]&~(|r_eba[k][1:0]), r_eba[k][1]&~r_eba[k][0], r_eba[k][0]};
                e_so_b[k]<={!(|r_ebb[k]), r_ebb[k][4]&~(|r_ebb[k][3:0]), r_ebb[k][3]&~(|r_ebb[k][2:0]),
                            r_ebb[k][2]&~(|r_ebb[k][1:0]), r_ebb[k][1]&~r_ebb[k][0], r_ebb[k][0]};
                e_ops[k]<=0;
                e_isbr[k]<=r_full[k][P_CLS+:3]==C_BR;
                e_blt[k]<=r_full[k][P_F3+2]; e_bneg[k]<=r_full[k][P_F3];
                e_sx[k]<=r_full[k][P_CLS+:3]==C_BR ? (r_full[k][P_F3+:3]==3'd4 || r_full[k][P_F3+:3]==3'd5) :
                         r_full[k][P_AOP+:4]==A_SLT;
                e_isct[k]<=r_full[k][P_CLS+:3]>=C_BR && r_full[k][P_CLS+:3]<=C_JALR;
                e_brt1[k]<=r_full[k][P_CLS+:3]==C_BR && xr_port[k][X_TGT-30];
                e_jt1[k]<=r_full[k][P_CLS+:3]==C_JAL && xr_port[k][X_TGT-30];
                e_inv[k]<=r_full[k][P_CLS+:3]==C_BR || r_full[k][P_AOP+:4]==A_SUB ||
                          r_full[k][P_AOP+:4]==A_SLT || r_full[k][P_AOP+:4]==A_SLTU;
                case(r_full[k][P_F3+:3])
                    3'd0: e_bop[k]<=6'b000001; 3'd1: e_bop[k]<=6'b000010; 3'd4: e_bop[k]<=6'b000100;
                    3'd5: e_bop[k]<=6'b001000; 3'd6: e_bop[k]<=6'b010000; default: e_bop[k]<=6'b100000;
                endcase
                if(r_full[k][P_CLS+:3]==C_JAL || r_full[k][P_CLS+:3]==C_JALR) e_ops[k][11]<=1'b1;
                else e_ops[k][r_full[k][P_AOP+:4]]<=1'b1;
                // {pc, imm, target, predicted target, predicted taken, history}
                e_pc[k]<={16'b0,xr_port[k][X_PC-:14],2'b0}; e_imm[k]<=xr_port[k][X_IMM-:32];
                e_tgt[k]<=xr_port[k][X_TGT-:32];
                e_ptk[k]<=xr_port[k][X_PT];
                e_tgtbad[k]<=xr_port[k][X_TB];
                e_link[k]<={15'b0,xr_port[k][X_LINK-:15],2'b0};
                e_pn[k]<={15'b0,xr_port[k][X_PN-:15],2'b0};
                e_ghr[k]<=xr_port[k][9:0];
            end

            // ---------------- E -> W (ALU ports)
            for(k=0;k<3;k=k+1) begin
                res_v[k]<=e_v[k] && e_pay[k][P_WE] && e_pay[k][P_CLS+:3]<=C_JALR;
                res_tag[k]<=e_pay[k][P_DST+:6]; res_val[k]<=res_nx[k]; res_vw[k]<=res_nx[k];
                wb_v[k]<=e_v[k] && e_pay[k][P_CLS+:3]<=C_JALR; wb_rob[k]<=e_pay[k][P_ROB+:5]; wb_rw[k]<=e_pay[k][P_RW];
                wb_ct[k]<=x_ct[k]; wb_cond[k]<=e_pay[k][P_CLS+:3]==C_BR; wb_flt[k]<=x_flt[k]; wb_tk[k]<=x_tk[k];
                wb_sum[k]<=x_sum[k]; wb_tgt[k]<=e_tgt[k]; wb_link[k]<=e_link[k];
                // Branch/JAL targets are PC-relative: decide from direction and
                // the precomputed target without a late 32-bit comparison.
                wb_brj[k]<=e_pay[k][P_CLS+:3]==C_BR || e_pay[k][P_CLS+:3]==C_JAL;
                wb_ptk[k]<=e_ptk[k]; wb_tgtbad[k]<=e_tgtbad[k];
                wb_jalr[k]<=e_pay[k][P_CLS+:3]==C_JALR;
                jc_v[k]<=wb_v[k] && wb_jalr[k]; jc_flt[k]<=wb_sum[k][1];
                jc_mis[k]<=wb_v[k] && wb_jalr[k] && !wb_sum[k][1] && wb_next[k]!=wb_pn[k];
                jc_rob[k]<=wb_rob[k]; jc_rw[k]<=wb_rw[k]; jc_pc[k]<=wb_next[k]; jc_ghr[k]<={wb_ghr[k][8:0],1'b1};
                wb_pn[k]<=e_pn[k]; wb_ghr[k]<=e_ghr[k];
            end
            // Memory E1
            m2_v<=m_ld; m2_we<=e_pay[3][P_WE]; m2_tag<=e_pay[3][P_DST+:6]; m2_rob<=e_pay[3][P_ROB+:5]; m2_rw<=e_pay[3][P_RW];
            m2_f3<=m_f3; m2_off<=m_addr[1:0]; m2_word<=m_addr[15:2]; m2_lmask<=m_mask;
            m2_snap<=e_pay[3][P_SQ+:4]; m2_addr<=m_addr;
            m2_shhit<=wq_v && addr_eq(x_a[3],e_imm[3],wq_word);
            m2_wqhit<=cst && !su && addr_eq(x_a[3],e_imm[3],sq_addr[sqh][15:2]);
            m2_fidx<=e_pay[3][P_FIDX+:3];
            m2_fwd<=e_pay[3][P_FWD] && sq_av[e_pay[3][P_FIDX+:3]] &&
                {1'b0,e_pay[3][P_FIDX+:3]-sq_head[2:0]}<e_pay[3][P_SQ+:4]-sq_head;
            m2_fdata<=sq_data[e_pay[3][P_FIDX+:3]]; m2_fmask<=sq_mask[e_pay[3][P_FIDX+:3]];
            m2_fword<=sq_addr[e_pay[3][P_FIDX+:3]][15:2];
            // The free-running counter has no read side effect; it is sampled
            // when the load executes, including speculatively.
            m2_cntrep<=0;
            m2_pc<=e_pc[3]; m2_ghr<=e_ghr[3];
            st_v<=m_st; st_rob<=e_pay[3][P_ROB+:5]; st_addr<=m_addr; st_f3<=m_f3; st_sqi<=m_sqi;
            if(st_v) sq_uart[st_sqi]<=st_uart;
            if(m_st) begin
                sq_addr[m_sqi]<=m_addr; sq_data[m_sqi]<=x_b[3]<<{m_addr[1:0],3'b0}; sq_mask[m_sqi]<=m_mask;
                sq_av[m_sqi]<=1;
            end
            // E2
            res_v[3]<=m2_v && m2_we; res_tag[3]<=m2_tag; res_val[3]<=res_nx[3]; res_vw[3]<=res_nx[3];
            lwb_v<=m2_v; lwb_rob<=m2_rob; lwb_rw<=m2_rw; lwb_flt<=m2_flt; lwb_rep<=!m2_flt && (m2_cntrep || l_conf);
            lwb_cnt<=m2_cntrep; lwb_iscnt<=m2_cnt && !m2_flt; lwb_tval<=cycles; lwb_pc<=m2_pc; lwb_ghr<=m2_ghr; lwb_ov<=l_ov; lwb_cov<=l_cov; lwb_snap<=m2_snap;
            // Multiplier
            mq1_v<=e_v[0] && e_pay[0][P_CLS+:3]==C_MUL; mq1_we<=e_pay[0][P_WE]; mq1_tag<=e_pay[0][P_DST+:6];
            mq1_rob<=e_pay[0][P_ROB+:5]; mq1_hi<=e_pay[0][P_F3+:3]!=0;
            mA<={(e_pay[0][P_F3+:3]==1 || e_pay[0][P_F3+:3]==2) && x_a[0][31],x_a[0]};
            mB<={e_pay[0][P_F3+:3]==1 && x_b[0][31],x_b[0]};
            mq2_v<=mq1_v; mq2_we<=mq1_we; mq2_tag<=mq1_tag; mq2_rob<=mq1_rob; mq2_hi<=mq1_hi; mp1<=mA*mB;
            // v27: mp2/mq3 removed — the result, wake, and ROB completion now ride
            // mq2 with the registered mp1; rs_wc for C_MUL is 2 to match.
            res_v[4]<=(mq2_v && mq2_we) || (div_last && div_we);
            res_tag[4]<=mq2_v ? mq2_tag : div_tag;
            res_val[4]<=res_nx[4]; res_vw[4]<=res_nx[4];
            mwb_v<=mq2_v || div_last; mwb_rob<=mq2_v ? mq2_rob : div_rob;
            // Divider: operands captured in E, signs/magnitudes set up from registers
            // the next cycle (no negation adder behind the E-stage bypass), then 32
            // restoring iterations.
            if(e_v[0] && e_pay[0][P_CLS+:3]==C_DIV) begin
                div_setup<=1; da0<=x_a[0]; db0<=x_b[0]; div_sgn<=!e_pay[0][P_F3];
                div_rem<=e_pay[0][P_F3+1];
                div_we<=e_pay[0][P_WE]; div_tag<=e_pay[0][P_DST+:6]; div_rob<=e_pay[0][P_ROB+:5];
            end else if(div_setup) begin
                div_setup<=0;
                dq<=div_sgn && da0[31] ? -da0 : da0;
                dd<=div_sgn && db0[31] ? -db0 : db0;
                dr<=0; div_cnt<=0; div_run<=1;
                div_negq<=div_sgn && (da0[31]^db0[31]);
                div_negr<=div_sgn && da0[31];
                div_bz<=db0==0;
            end else if(div_run) begin
                dq<=dq_next; dr<=dr_next; div_cnt<=div_cnt+5'd1;
                if(div_cnt==5'd31) begin div_run<=0; div_fin<=1; end
            end
            if(div_fin) begin div_fin<=0; div_busy<=0; end

            // ---------------- ROB completion
            for(k=0;k<3;k=k+1) if(wb_v[k] && !wb_jalr[k]) begin
                rob_cpl[wb_rob[k]]<=1; rob_flt[wb_rob[k]]<=wb_flt[k]; rob_mis[wb_rob[k]]<=w_mis[k];
                rob_atk[wb_rob[k]]<=wb_tk[k];
            end
            for(k=0;k<3;k=k+1) if(jc_v[k]) begin
                rob_cpl[jc_rob[k]]<=1; rob_flt[jc_rob[k]]<=jc_flt[k]; rob_mis[jc_rob[k]]<=jc_mis[k];
                rob_atk[jc_rob[k]]<=1;
            end
            if(st_v) begin rob_cpl[st_rob]<=1; rob_flt[st_rob]<=st_flt; end
            if(lwb_v) begin rob_cpl[lwb_rob]<=1; rob_flt[lwb_rob]<=lwb_flt; rob_rep[lwb_rob]<=lwb_rep;
                rob_tcnt[lwb_rob]<=lwb_iscnt; rob_tval[lwb_rob]<=lwb_tval; end
            if(mwb_v) rob_cpl[mwb_rob]<=1;

            // ---------------- redirect
            rd_v<=acc;
            if(acc) begin rd_pc<=rd_pc_n; rd_ghr<=rd_ghr_n; pend_v<=1; pend_rob<=rd_rob_n; pend_rw<=rd_rw_n; end

            // ---------------- commit
            wq_v<=cst && !su; wq_word<=sq_addr[sqh][15:2]; wq_data<=sq_data[sqh]; wq_mask<=sq_mask[sqh];
            sh_v<=wq_v; sh_word<=wq_word; sh_mask<=wq_mask; sh_data<=wq_data;
            uq_v<=cst && su; uq_data<=sq_data[sqh][7:0];
            if(cst) begin sq_head<=sq_head+4'd1; end
            su_q<=cst ? sq_uart_nx(sqh+3'd1) : sq_uart_nx(sqh);
            sq_cnt<=sq_cnt+(fire ? {1'b0,nst} : 4'd0)-{3'b0,cst};
            cp_w<={cw2,cw1,cw0};
            cp_rd[0]<=cr_rd[0]; cp_rd[1]<=cr_rd[1]; cp_rd[2]<=cr_rd[2];
            cp_pd[0]<=cr_pd[0]; cp_pd[1]<=cr_pd[1]; cp_pd[2]<=cr_pd[2];
            cp_po[0]<=cr_po[0]; cp_po[1]<=cr_po[1]; cp_po[2]<=cr_po[2];
            cp_b[0]<=cr_b[0]; cp_b[1]<=cr_b[1]; cp_b[2]<=cr_b[2];
            cs_ct<={cm2 && tc2, cm1 && tc1, cm0 && t0};
            cs_atk<={rob_atk[h2],rob_atk[h1],rob_atk[h0]}; cs_cond<={rob_cond[h2],rob_cond[h1],rob_cond[h0]};
            cs_pc[0]<=xr_commit[h0[1:0]][X_PC-:14]; cs_pc[1]<=xr_commit[h1[1:0]][X_PC-:14];
            cs_pc[2]<=xr_commit[h2[1:0]][X_PC-:14];
            cs_ghr[0]<=xr_commit[h0[1:0]][9:0]; cs_ghr[1]<=xr_commit[h1[1:0]][9:0]; cs_ghr[2]<=xr_commit[h2[1:0]][9:0];
            cs_hi[0]<=h0; cs_hi[1]<=h1; cs_hi[2]<=h2;
            for(k=0;k<3;k=k+1) begin
                aw_v[k]<=wb_v[k] && !wb_jalr[k]; aw_i[k]<=wb_rob[k]; aw_d[k]<=wb_next[k][15:2];
                aw_v[k+3]<=jc_v[k]; aw_i[k+3]<=jc_rob[k]; aw_d[k+3]<=jc_pc[k][15:2];
            end
            for(k=0;k<6;k=k+1) if(aw_v[k]) rob_atgt[aw_i[k]]<=aw_d[k];
            for(k=0;k<3;k=k+1) if(cp_w[k]) begin crat[cp_rd[k]]<=cp_pd[k]; crat_b[cp_rd[k]]<=cp_b[k]; end
            if(cp_w[0]) fl[fl_tail[4:0]]<=cp_po[0];
            if(cp_w[1]) fl[fl_tail[4:0]+{4'b0,cp_w[0]}]<=cp_po[1];
            if(cp_w[2]) fl[fl_tail[4:0]+{4'b0,cp_w[0]}+{4'b0,cp_w[1]}]<=cp_po[2];
            fl_tail<=fl_tail+{4'b0,cp_n}; fl_chead<=fl_chead+{4'b0,cp_n};
            fl_cnt<=fl_cnt+{4'b0,cp_n}-(fire ? {4'b0,wrank[2]}+{5'b0,wr[2]} : 6'd0);
            fl_used<=fire ? wrank[2]+{1'b0,wr[2]} : 2'd0;
            fl_base=flush_q ? fl_chead+{4'b0,cp_n} : fl_head;
            for(i=0;i<6;i=i+1) begin
                fl_pos=fl_base[4:0]+i[4:0];
                fl_win[i]<=cp_w[0] && fl_pos==fl_tail[4:0] ? cp_po[0] :
                    cp_w[1] && fl_pos==fl_tail[4:0]+{4'b0,cp_w[0]} ? cp_po[1] :
                    cp_w[2] && fl_pos==fl_tail[4:0]+{4'b0,cp_w[0]}+{4'b0,cp_w[1]} ? cp_po[2] : fl[fl_pos];
            end
            for(k=0;k<3;k=k+1) hw[k]<=cm2 ? hcand[k+3] : cm1 ? hcand[k+2] : cm0 ? hcand[k+1] : hcand[k];
            crsel<=cm2 ? rob_head[1:0]+2'd3 : cm1 ? rob_head[1:0]+2'd2 : cm0 ? rob_head[1:0]+2'd1 : crsel;
            for(k=0;k<4;k=k+1) cra[k]<=cm2 ? crow(rob_head+5'd3,k[1:0]) : cm1 ? crow(rob_h2,k[1:0]) : cm0 ? crow(rob_h1,k[1:0]) : cra[k];
            for(k=0;k<6;k=k+1) rob_hm[k]<=cm2 ? rob_head+k[4:0]+5'd3 : cm1 ? rob_head+k[4:0]+5'd2 : cm0 ? rob_head+k[4:0]+5'd1 : rob_hm[k];
            // Late commit/dispatch decisions select sums precomputed from registers.
            rob_head<=cm2 ? rob_head+5'd3 : cm1 ? rob_h2 : cm0 ? rob_h1 : rob_head;
            rob_hw<=rob_hw ^ (cm2 ? rob_head>=5'd29 : cm1 ? rob_head>=5'd30 : cm0 && rob_head==5'd31);
            rob_h1<=cm2 ? rob_head+5'd4 : cm1 ? rob_head+5'd3 : cm0 ? rob_h2 : rob_h1;
            rob_h2<=cm2 ? rob_head+5'd5 : cm1 ? rob_head+5'd4 : cm0 ? rob_head+5'd3 : rob_h2;
            rob_cnt<=fire ? (cm2 ? rob_cnt+{3'b0,nd}-6'd3 : cm1 ? rob_cnt+{3'b0,nd}-6'd2 : cm0 ? rob_cnt+{3'b0,nd}-6'd1 : rob_cnt+{3'b0,nd}) :
                            (cm2 ? rob_cnt-6'd3 : cm1 ? rob_cnt-6'd2 : cm0 ? rob_cnt-6'd1 : rob_cnt);
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
                mq1_v<=0; mq2_v<=0; div_run<=0; div_setup<=0; div_fin<=0; div_busy<=0; mwb_v<=0;
                rob_tail<=rob_head; rob_tw<=rob_hw; rob_cnt<=0; sq_tail<=sq_head; sq_cnt<=0;
                for(i=0;i<32;i=i+1) begin rat_p[i]<=crat_now[i]; rat_b[i]<=crat_now_b[i]; end
                rat_pend<=0; fl_head<=fl_chead+{4'b0,cp_n}; fl_cnt<=fl_tail-fl_chead; pend_v<=0; rd_v<=0;
                fl_used<=0;
                clear_q<=0; sat<=0;
                for(k=0;k<4;k=k+1) rsfree_q[k]<=SIZE(k[1:0]);
            end
        end
        if(rst) begin
            halted<=0; cycles<=0; flush_q<=0; rd_v<=0; pend_v<=0;
            fpc<=32'h8000; ghr<=0; pidx<=10'd0; f1_v<=0; fq_head<=0; fq_tail<=0; fq_cnt<=0; d2_v<=0;
            tq_btb<=0; tq_pht<=0;
            for(i=0;i<32;i=i+1) begin rat_p[i]<=i[5:0]; crat[i]<=i[5:0]; rat_b[i]<=0; crat_b[i]<=0; fl[i]<=6'd32+i[5:0]; end
            rat_pend<=0; fl_head<=0; fl_tail<=6'd32; fl_chead<=0; fl_cnt<=6'd32; fl_used<=0; rr<=0;
            for(i=0;i<6;i=i+1) fl_win[i]<=6'd32+i[5:0]; cp_w<=0; cs_ct<=0;
            rob_head<=0; rob_h1<=1; rob_h2<=2; rob_hw<=0; rob_tw<=0;
            for(k=0;k<6;k=k+1) rob_hm[k]<=k[4:0];
            for(k=0;k<4;k=k+1) cra[k]<=0;
            crsel<=0;
            for(k=0;k<3;k=k+1) hw[k]<={rob_tcnt[k],rob_ct[k],rob_st[k],rob_mis[k],rob_rep[k],rob_flt[k],rob_cpl[k]}; rob_tail<=0; rob_cnt<=0;
            rs_v<=0; rs_iss<=0; r_v<=0; e_v<=0; res_v<=0; wb_v<=0; jc_v<=0;
            m2_v<=0; st_v<=0; lwb_v<=0; sq_head<=0; sq_tail<=0; sq_cnt<=0; su_q<=sq_uart_nx(3'd0); wq_v<=0; sh_v<=0; uq_v<=0;
            mq1_v<=0; mq2_v<=0; div_busy<=0; div_run<=0; div_setup<=0; div_fin<=0; mwb_v<=0; clear_q<=0; sat<=0; req_q<=0;
            for(k=0;k<4;k=k+1) rsfree_q[k]<=SIZE(k[1:0]);
            last_pc<=32'h8000;
        end
    end

    uart_tx #(.CLOCK_HZ(CLOCK_HZ)) serial(clk,resetn,uq_v,uq_data,uart_ready,uart_tx);

    // ------------------------------------------------------------------ debug and profile
    assign debug_pc=halted ? fault_pc : cm0 ? rob_pc[h0] : last_pc;
    assign debug_insn=halted ? fault_insn : cm0 ? rob_insn[h0] : last_insn;
    assign debug_commit=cm0; assign debug_commit2=cm1; assign debug_commit3=cm2;
    assign debug_we=cr_rd[0]!=0; assign debug_rd=cr_rd[0]; assign debug_wdata=prf_rd[8];
    assign debug_pc2=rob_pc[h1]; assign debug_insn2=rob_insn[h1]; assign debug_we2=cr_rd[1]!=0;
    assign debug_rd2=cr_rd[1]; assign debug_wdata2=prf_rd[9];
    assign debug_pc3=rob_pc[h2]; assign debug_insn3=rob_insn[h2]; assign debug_we3=cr_rd[2]!=0;
    assign debug_rd3=cr_rd[2]; assign debug_wdata3=prf_rd[10];
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
        // v27: the res_nx[4] mux is steered by mq2_v regardless of write-enable,
        // so ANY mq2-valid cycle coinciding with a divider finish would silently
        // lose the div result. Timing makes this impossible (a mul may enter E no
        // later than the div_fin cycle, and the divider's earliest finish is more
        // than 30 cycles after its grant, so no in-flight mul mq2 can reach it).
        if(mq2_v && div_last) $fatal(1,"v27 port-4 collision: mul mq2 vs div_fin cycle=%0d", cycles);
        if(fire) for(j=0;j<3;j=j+1) if(inrs[j] && rs_v[row[j]]) $fatal(1,"allocated a busy RS row");
        if(fire) for(j=0;j<3;j=j+1) if(inrs[j] && slot_oh[j]==0) $fatal(1,"dispatch without a free RS row");
        for(k=0;k<4;k=k+1) if(rsfree_q[k]>free_count(rsbusy(rs_v,k[1:0]))) $fatal(1,"RS free count above actual");
        if(fl_tail-fl_head>6'd32) $fatal(1,"free list overflow");
        if(fl_cnt!=fl_tail-fl_head) $fatal(1,"free-list count mismatch");
        // Existing rows request exactly per the original formula; a row in its first
        // cycle may request later than it (never earlier).
        if((req_q & ~new_q)!=(req_ref & ~new_q) || (req_q & ~req_ref)!=0) $fatal(1,"registered request mismatch");
        if(pidx!=(fpc[13:4]^ghr)) $fatal(1,"PHT index register mismatch");
        if(su_q!=sq_uart[sqh]) $fatal(1,"head-store UART flag mismatch");
        if(crsel!=rob_head[1:0]) $fatal(1,"commit bank select copy mismatch");
        for(j=0;j<3;j=j+1) for(i=0;i<3;i=i+1) if(d2_v[j] && (d2_pm1[j][i]!=(pg_v[i] && pg_rd[i]==d2_r1[j]) ||
            d2_pm2[j][i]!=(pg_v[i] && pg_rd[i]==d2_r2[j]) || d2_pmo[j][i]!=(pg_v[i] && pg_rd[i]==d2_rd[j]))) $fatal(1,"D1 previous-group match mismatch");
        if(m2_v && (m2_wqhit!=(wq_v && wq_word==m2_word) || m2_shhit!=(sh_v && sh_word==m2_word))) $fatal(1,"E1 merge hit mismatch");
        // (In a flush cycle the pending branch has just retired; no redirect is taken.)
        if(!flush_q) begin
            for(ca=0;ca<7;ca=ca+1) if(pend_v && cand_v[ca] && cand_opend[ca]!=(((cand_seq[ca]-{pend_rw,pend_rob}) & 6'h20)!=0))
                $fatal(1,"registered pending age mismatch");
            for(ca=0;ca<7;ca=ca+1) for(cb=0;cb<7;cb=cb+1) if(ca!=cb && cand_v[ca] && cand_v[cb] &&
                cand_old[ca][cb]!=((cand_rob[ca]-rob_head)<(cand_rob[cb]-rob_head))) $fatal(1,"redirect age mismatch");
            for(ca=0;ca<7;ca=ca+1) if(pend_v && cand_v[ca] && cand_opend[ca]!=((cand_rob[ca]-rob_head)<(pend_rob-rob_head)))
                $fatal(1,"pending redirect age mismatch");
        end
        if(hw[0]!={rob_tcnt[h0],rob_ct[h0],rob_st[h0],rob_mis[h0],rob_rep[h0],rob_flt[h0],rob_cpl[h0]} ||
           hw[1]!={rob_tcnt[h1],rob_ct[h1],rob_st[h1],rob_mis[h1],rob_rep[h1],rob_flt[h1],rob_cpl[h1]} ||
           hw[2]!={rob_tcnt[h2],rob_ct[h2],rob_st[h2],rob_mis[h2],rob_rep[h2],rob_flt[h2],rob_cpl[h2]}) $fatal(1,"head window mismatch");
        // The D1 RAT read must equal a direct D2 read of the current RAT.
        if(fire) for(j=0;j<3;j=j+1) if(inrs[j]) begin
            if(d2_u1[j] && !(d2_m1[j]!=0) && pt1[j]!=rat_p[d2_r1[j]]) $fatal(1,"D1 RAT tag 1 mismatch");
            if(d2_u2[j] && !(d2_m2[j]!=0) && pt2[j]!=rat_p[d2_r2[j]]) $fatal(1,"D1 RAT tag 2 mismatch");
            if(wr[j] && !(d2_wm[j]!=0) && pop[j]!=rat_p[d2_rd[j]]) $fatal(1,"D1 RAT old mapping mismatch");
            if(d2_u1[j] && !(d2_m1[j]!=0) && d2_r1[j]!=0 && b1[j]!=rat_b[d2_r1[j]]) $fatal(1,"D1 RAT bank 1 mismatch");
            if(d2_u2[j] && !(d2_m2[j]!=0) && d2_r2[j]!=0 && b2[j]!=rat_b[d2_r2[j]]) $fatal(1,"D1 RAT bank 2 mismatch");
            for(r=0;r<NR;r=r+1) begin
                if(!(d2_m1[j]!=0) && (pdep1[j][r] & ~clear_q[r])!=(rat_pend[d2_r1[j]] && d2_r1[j]!=0 && rat_c[d2_r1[j]]==r && !clear_q[r]))
                    $fatal(1,"D1 RAT dependence 1 mismatch");
                if(!(d2_m2[j]!=0) && (pdep2[j][r] & ~clear_q[r])!=(rat_pend[d2_r2[j]] && d2_r2[j]!=0 && rat_c[d2_r2[j]]==r && !clear_q[r]))
                    $fatal(1,"D1 RAT dependence 2 mismatch");
            end
        end
        if(fire) for(j=0;j<3;j=j+1) if(wr[j] && phys[j]!=fl[fl_head[4:0]+{3'b0,wrank[j]}]) $fatal(1,"free-list window mismatch");
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
        for(tq=0;tq<4;tq=tq+1) if(gany[tq]) $fwrite(32'h80000002," %h",rob_pc[gpay[tq][P_ROB+:5]][15:0]); else $fwrite(32'h80000002,"     ");
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
