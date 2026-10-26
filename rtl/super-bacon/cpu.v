`include "define.vh"
`include "fast_cpu.vh"
// Configurable RV32IM core: fetch/execute pipeline, prediction, two-wide
// integer issue, optional ALU lookahead, and synchronous or distributed RAM.
module cpu #(parameter CODE_FILE = "code.hex", DATA_FILE = "data.hex",
             parameter integer CLOCK_HZ = 25000000,
             parameter integer PIPELINE = 0, PREDICT = 0, ENABLE_M = 1,
             parameter integer DUAL = 0, LOOKAHEAD = 0, BRAM = 0, OOO = 0, FAST = 0)(
    input wire clk, resetn, output wire uart_tx,
    output wire halted,
    output wire [31:0] debug_pc, debug_insn, debug_wdata,
    output wire debug_commit, debug_commit2, debug_we,
    output wire [4:0] debug_rd,
    input wire [4:0] debug_reg_addr, output wire [31:0] debug_reg_data,
    output wire debug_uart_valid, output wire [7:0] debug_uart_data,
    output wire [31:0] cycles
);
    generate if (FAST != 0) begin: fast
        fast_cpu #(.CODE_FILE(CODE_FILE), .DATA_FILE(DATA_FILE), .CLOCK_HZ(CLOCK_HZ), .SUPER((FAST==2) ? 1 : 0)) core(.clk(clk), .resetn(resetn), .uart_tx(uart_tx), .halted(halted), .debug_pc(debug_pc), .debug_insn(debug_insn), .debug_wdata(debug_wdata), .debug_commit(debug_commit), .debug_commit2(debug_commit2), .debug_we(debug_we), .debug_rd(debug_rd), .debug_reg_addr(debug_reg_addr), .debug_reg_data(debug_reg_data), .debug_uart_valid(debug_uart_valid), .debug_uart_data(debug_uart_data), .cycles(cycles));
    end else begin: legacy
        legacy_cpu #(.CODE_FILE(CODE_FILE), .DATA_FILE(DATA_FILE), .CLOCK_HZ(CLOCK_HZ), .PIPELINE(PIPELINE), .PREDICT(PREDICT), .ENABLE_M(ENABLE_M), .DUAL(DUAL), .LOOKAHEAD(LOOKAHEAD), .BRAM(BRAM), .OOO(OOO)) core(.clk(clk), .resetn(resetn), .uart_tx(uart_tx), .halted(halted), .debug_pc(debug_pc), .debug_insn(debug_insn), .debug_wdata(debug_wdata), .debug_commit(debug_commit), .debug_commit2(debug_commit2), .debug_we(debug_we), .debug_rd(debug_rd), .debug_reg_addr(debug_reg_addr), .debug_reg_data(debug_reg_data), .debug_uart_valid(debug_uart_valid), .debug_uart_data(debug_uart_data), .cycles(cycles));
    end endgenerate
endmodule
`include "define.vh"
// Configurable RV32IM core: fetch/execute pipeline, prediction, two-wide
// integer issue, optional ALU lookahead, and synchronous or distributed RAM.
module legacy_cpu #(parameter CODE_FILE = "code.hex", DATA_FILE = "data.hex",
             parameter integer CLOCK_HZ = 25000000,
             parameter integer PIPELINE = 0, PREDICT = 0, ENABLE_M = 1,
             parameter integer DUAL = 0, LOOKAHEAD = 0, BRAM = 0, OOO = 0)(
    input wire clk, resetn, output wire uart_tx,
    output reg halted = 0,
    output wire [31:0] debug_pc, debug_insn, debug_wdata,
    output wire debug_commit, debug_commit2, debug_we,
    output wire [4:0] debug_rd,
    input wire [4:0] debug_reg_addr, output wire [31:0] debug_reg_data,
    output wire debug_uart_valid, output wire [7:0] debug_uart_data,
    output reg [31:0] cycles = 0
);
    reg [31:0] pc = 32'h8000;
    (* ram_style = "distributed" *) reg [31:0] regs [0:31];
    (* rom_style = "distributed" *) reg [31:0] imem [0:8191];
    (* ram_style = (BRAM != 0) ? "block" : "distributed" *) reg [31:0] dmem [0:16383];
    integer i;
    initial begin
        for (i=0;i<32;i=i+1) regs[i]=0;
        for (i=0;i<8192;i=i+1) imem[i]=32'h00000013;
        for (i=0;i<16384;i=i+1) dmem[i]=0;
        $readmemh(CODE_FILE,imem);
        $readmemh(DATA_FILE,dmem);
    end
    reg [31:0] fetch_pc = 32'h8000, fetched_insn = 32'h13;
    reg valid = 0;
    reg [31:0] second_insn = 32'h13, second_pc = 32'h8004;
    reg second_issued = 0;
    reg [31:0] second_saved_result = 0;
    wire [31:0] insn = (PIPELINE != 0) ? fetched_insn : imem[pc[14:2]];
    reg [63:0] btb_valid = 0;
    reg [23:0] btb_tag [0:63];
    reg [31:0] btb_target [0:63];
    reg [1:0] btb_counter [0:63];
    wire [5:0] fetch_index = fetch_pc[7:2], execute_index = pc[7:2];
    wire predict_taken = (PREDICT != 0) && btb_valid[fetch_index] &&
        btb_tag[fetch_index] == fetch_pc[31:8] && btb_counter[fetch_index][1];
    wire [31:0] predicted_pc = predict_taken ? btb_target[fetch_index] : fetch_pc + 4;
    wire [5:0] second_index = second_pc[7:2];
    wire second_predict_taken = (PREDICT != 0) && btb_valid[second_index] &&
        btb_tag[second_index] == second_pc[31:8] && btb_counter[second_index][1];
    wire [31:0] second_predicted_pc = second_predict_taken ? btb_target[second_index] : second_pc + 4;
    wire control_instruction = insn[6:0] == 7'h63 || insn[6:0] == 7'h6f || insn[6:0] == 7'h67;
    wire execute_valid = (PIPELINE == 0) || valid;
    wire [4:0] rs1, rs2, rd;
    wire [31:0] imm;
    wire [5:0] alucode;
    wire [1:0] op1_type, op2_type;
    wire reg_we, is_load, is_store, illegal;
    decode decoder(insn,rs1,rs2,rd,imm,alucode,op1_type,op2_type,reg_we,is_load,is_store,illegal);
    wire [31:0] a = rs1 == 0 ? 32'b0 : regs[rs1];
    wire [31:0] b = rs2 == 0 ? 32'b0 : regs[rs2];
    wire [31:0] op1 = op1_type == `OP_TYPE_REG ? a : op1_type == `OP_TYPE_IMM ? imm : 32'b0;
    wire [31:0] op2 = op2_type == `OP_TYPE_REG ? b : op2_type == `OP_TYPE_IMM ? imm : op2_type == `OP_TYPE_PC ? pc : 32'b0;
    wire [31:0] result;
    wire branch;
    alu arithmetic(alucode,op1,op2,result,branch);
    wire [31:0] address = result;
    wire data_range = address >= 32'h10000 && address < 32'h20000;
    wire uart_store = is_store && address == `UART_ADDR;
    wire counter_load = is_load && address == `HARDWARE_COUNTER_ADDR && alucode == `ALU_LW;
    wire uart_ready;
    wire m_instruction = (ENABLE_M != 0) && insn[6:0] == 7'h33 && insn[31:25] == 7'h01;
    wire m_done;
    wire [31:0] m_result;
    muldiv m_unit(.clk(clk), .resetn(resetn),
        .start(resetn && execute_valid && !halted && m_instruction && !m_done),
        .operation(insn[14:12]), .a(a), .b(b), .done(m_done), .result(m_result));
    reg load_ready = 0;
    wire load_wait = (BRAM != 0) && is_load && !counter_load && !load_ready;
    wire stall = load_wait || (uart_store && !uart_ready) || (m_instruction && !m_done);
    wire bad_fetch = pc < 32'h8000 || pc >= 32'h10000 || pc[1:0] != 0;
    // Support byte accesses and halfwords at offsets 0,1,2, as required by
    // the course ISA. Crossing word boundaries and misaligned LW/SW halt.
    wire bad_align = ((alucode == `ALU_LW || alucode == `ALU_SW) && address[1:0] != 0) ||
                     ((alucode == `ALU_LH || alucode == `ALU_LHU || alucode == `ALU_SH) && address[1:0] == 3);
    wire fault = (illegal && !m_instruction) || bad_fetch || ((is_load || is_store) &&
                 (bad_align || !(data_range || uart_store || counter_load)));
    wire commit = resetn && execute_valid && !halted && !stall && !fault;
    reg [31:0] memory_read;
    generate if (BRAM != 0) begin: synchronous_memory
        always @(posedge clk) memory_read <= dmem[address[15:2]];
    end else begin: asynchronous_memory
        always @* memory_read = dmem[address[15:2]];
    end endgenerate
    wire [31:0] shifted = memory_read >> {address[1:0],3'b0};
    reg [31:0] loaded;
    always @* begin
        case (alucode)
            `ALU_LB: loaded = {{24{shifted[7]}},shifted[7:0]};
            `ALU_LBU: loaded = {24'b0,shifted[7:0]};
            `ALU_LH: loaded = {{16{shifted[15]}},shifted[15:0]};
            `ALU_LHU: loaded = {16'b0,shifted[15:0]};
            default: loaded = shifted;
        endcase
    end
    wire [31:0] wdata = m_instruction ? m_result : is_load ? (counter_load ? cycles : loaded) : result;
    // Second slot only executes legal integer ALU operations. Dependencies
    // and WAW conflicts block pairing; no speculative memory/MMIO effects.
    wire [4:0] rs1_second, rs2_second, rd_second;
    wire [31:0] imm_second;
    wire [5:0] alu_second;
    wire [1:0] type1_second, type2_second;
    wire we_second, load_second, store_second, illegal_second;
    decode second_decoder(second_insn,rs1_second,rs2_second,rd_second,
        imm_second,alu_second,type1_second,type2_second,we_second,
        load_second,store_second,illegal_second);
    wire [31:0] a_second = rs1_second == 0 ? 32'b0 : regs[rs1_second];
    wire [31:0] b_second = rs2_second == 0 ? 32'b0 : regs[rs2_second];
    wire [31:0] operand1_second = type1_second == `OP_TYPE_REG ? a_second :
        type1_second == `OP_TYPE_IMM ? imm_second : 32'b0;
    wire [31:0] operand2_second = type2_second == `OP_TYPE_REG ? b_second :
        type2_second == `OP_TYPE_IMM ? imm_second :
        type2_second == `OP_TYPE_PC ? second_pc : 32'b0;
    wire [31:0] result_second;
    wire unused_branch_second;
    alu second_arithmetic(alu_second,operand1_second,operand2_second,
        result_second,unused_branch_second);
    wire second_simple = second_insn[6:0] == 7'h13 || second_insn[6:0] == 7'h37 ||
        second_insn[6:0] == 7'h17 || (second_insn[6:0] == 7'h33 && second_insn[31:25] != 1);
    wire second_independent = !reg_we || rd == 0 ||
        (rs1_second != rd && rs2_second != rd && rd_second != rd);
    wire can_pair = (DUAL != 0) && valid && !control_instruction &&
        ((OOO != 0) || second_pc == pc + 4) && second_simple && !illegal_second && second_independent && pc < 32'hfffc;
    // A legal M instruction cannot trap in this core. Capture its operands
    // before permitting younger, independent ALU writes; its destination is
    // scoreboarding the entire time it waits. Memory/control stay blocked.
    wire ooo_issue = (OOO != 0) && resetn && execute_valid && !halted && !fault &&
        m_instruction && !m_done && can_pair;
    wire pair_commit = commit && can_pair;
    wire [31:0] second_wdata = second_issued && (OOO == 0) ? second_saved_result : result_second;
    wire [31:0] next_pc = !branch ? pc+4 : alucode == `ALU_JALR ? ((a+imm)&32'hfffffffe) : pc+imm;
    wire [3:0] base_mask = alucode == `ALU_SB ? 4'b0001 : alucode == `ALU_SH ? 4'b0011 : 4'b1111;
    wire [3:0] byte_mask = base_mask << address[1:0];
    wire [31:0] store_data = b << {address[1:0],3'b0};
    always @(posedge clk) begin
        if (commit && is_store && data_range) begin
            if (byte_mask[0]) dmem[address[15:2]][7:0] <= store_data[7:0];
            if (byte_mask[1]) dmem[address[15:2]][15:8] <= store_data[15:8];
            if (byte_mask[2]) dmem[address[15:2]][23:16] <= store_data[23:16];
            if (byte_mask[3]) dmem[address[15:2]][31:24] <= store_data[31:24];
        end
        if ((pair_commit || ooo_issue) && we_second && rd_second != 0) regs[rd_second] <= second_wdata;
        if (commit && reg_we && rd != 0) regs[rd] <= wdata;
        if (!resetn) begin
            pc <= 32'h8000; fetch_pc <= 32'h8000; valid <= 0;
            cycles <= 0; halted <= 0; btb_valid <= 0; second_issued <= 0; load_ready <= 0;
        end
        else begin
            cycles <= cycles + 1'b1;
            if (commit) load_ready <= 0;
            else if (execute_valid && !halted && is_load && !counter_load) load_ready <= 1;
            if ((PREDICT != 0) && commit && control_instruction) begin
                btb_valid[execute_index] <= 1;
                btb_tag[execute_index] <= pc[31:8];
                if (branch) btb_target[execute_index] <= next_pc;
                if (!btb_valid[execute_index] || btb_tag[execute_index] != pc[31:8])
                    btb_counter[execute_index] <= branch ? 2'b10 : 2'b01;
                else if (branch && btb_counter[execute_index] != 3)
                    btb_counter[execute_index] <= btb_counter[execute_index] + 1'b1;
                else if (!branch && btb_counter[execute_index] != 0)
                    btb_counter[execute_index] <= btb_counter[execute_index] - 1'b1;
            end
            if ((LOOKAHEAD != 0) && (OOO == 0) && can_pair && m_instruction && !m_done && !second_issued) begin
                // Execute a younger independent ALU while divide is busy;
                // retain its result and retire both in program order.
                second_saved_result <= result_second;
                second_issued <= 1;
            end
            if (commit) second_issued <= 0;
            if (execute_valid && fault) halted <= 1;
            if (PIPELINE == 0) begin
                if (commit) pc <= next_pc;
            end else if ((DUAL != 0) && !halted) begin
                if (!valid) begin
                    pc <= fetch_pc;
                    fetched_insn <= imem[fetch_pc[14:2]];
                    second_insn <= imem[predicted_pc[14:2]];
                    second_pc <= predicted_pc;
                    fetch_pc <= predicted_pc + 4;
                    valid <= 1;
                end else if (ooo_issue) begin
                    second_insn <= imem[second_predicted_pc[14:2]];
                    second_pc <= second_predicted_pc;
                    fetch_pc <= second_predicted_pc + 4;
                end else if (commit) begin
                    if (next_pc != second_pc && !((OOO != 0) && m_instruction)) begin
                        fetch_pc <= next_pc; valid <= 0;
                    end else if (pair_commit) begin
                        pc <= fetch_pc;
                        fetched_insn <= imem[fetch_pc[14:2]];
                        second_insn <= imem[predicted_pc[14:2]];
                        second_pc <= predicted_pc;
                        fetch_pc <= predicted_pc + 4;
                    end else begin
                        pc <= second_pc;
                        fetched_insn <= second_insn;
                        second_insn <= imem[second_predicted_pc[14:2]];
                        second_pc <= second_predicted_pc;
                        fetch_pc <= second_predicted_pc + 4;
                    end
                end
            end else if (!halted) begin
                if (!valid || commit) begin
                    if (valid && next_pc != fetch_pc) begin
                        // Discard the speculative fetch on a redirect.
                        fetch_pc <= next_pc;
                        valid <= 0;
                    end else begin
                        pc <= fetch_pc;
                        fetched_insn <= imem[fetch_pc[14:2]];
                        fetch_pc <= predicted_pc;
                        valid <= 1;
                    end
                end
            end
        end
    end
    uart_tx #(.CLOCK_HZ(CLOCK_HZ)) serial(clk,resetn,commit && uart_store,b[7:0],uart_ready,uart_tx);
    assign debug_pc = pc;
    assign debug_insn = insn;
    assign debug_commit = commit;
    assign debug_commit2 = pair_commit || ooo_issue;
    assign debug_we = reg_we;
    assign debug_rd = rd;
    assign debug_wdata = wdata;
    assign debug_reg_data = debug_reg_addr == 0 ? 32'b0 : regs[debug_reg_addr];
    assign debug_uart_valid = commit && uart_store;
    assign debug_uart_data = b[7:0];
endmodule
