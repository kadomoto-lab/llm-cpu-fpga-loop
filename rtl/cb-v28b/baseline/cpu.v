`include "define.vh"
// Baseline single-cycle Harvard RV32I CPU. Asynchronous reads deliberately
// use distributed memory; register and memory writes occur on the rising edge.
module cpu #(parameter CODE_FILE = "code.hex", DATA_FILE = "data.hex",
             parameter integer CLOCK_HZ = 25000000)(
    input wire clk, resetn, output wire uart_tx,
    output reg halted = 0,
    output wire [31:0] debug_pc, debug_insn, debug_wdata,
    output wire debug_commit, debug_we,
    output wire [4:0] debug_rd,
    input wire [4:0] debug_reg_addr, output wire [31:0] debug_reg_data,
    output wire debug_uart_valid, output wire [7:0] debug_uart_data,
    output reg [31:0] cycles = 0
);
    reg [31:0] pc = 32'h8000;
    (* ram_style = "distributed" *) reg [31:0] regs [0:31];
    (* rom_style = "distributed" *) reg [31:0] imem [0:8191];
    (* ram_style = "distributed" *) reg [31:0] dmem [0:16383];
    integer i;
    initial begin
        for (i=0;i<32;i=i+1) regs[i]=0;
        for (i=0;i<8192;i=i+1) imem[i]=32'h00000013;
        for (i=0;i<16384;i=i+1) dmem[i]=0;
        $readmemh(CODE_FILE,imem);
        $readmemh(DATA_FILE,dmem);
    end
    wire [31:0] insn = imem[pc[14:2]];
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
    wire stall = uart_store && !uart_ready;
    wire bad_fetch = pc < 32'h8000 || pc >= 32'h10000 || pc[1:0] != 0;
    // Support byte accesses and halfwords at offsets 0,1,2, as required by
    // the course ISA. Crossing word boundaries and misaligned LW/SW halt.
    wire bad_align = ((alucode == `ALU_LW || alucode == `ALU_SW) && address[1:0] != 0) ||
                     ((alucode == `ALU_LH || alucode == `ALU_LHU || alucode == `ALU_SH) && address[1:0] == 3);
    wire fault = illegal || bad_fetch || ((is_load || is_store) &&
                 (bad_align || !(data_range || uart_store || counter_load)));
    wire commit = resetn && !halted && !stall && !fault;
    wire [31:0] shifted = dmem[address[15:2]] >> {address[1:0],3'b0};
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
    wire [31:0] wdata = is_load ? (counter_load ? cycles : loaded) : result;
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
        if (commit && reg_we && rd != 0) regs[rd] <= wdata;
        if (!resetn) begin pc <= 32'h8000; cycles <= 0; halted <= 0; end
        else begin
            cycles <= cycles + 1'b1;
            if (fault) halted <= 1;
            if (commit) pc <= next_pc;
        end
    end
    uart_tx #(.CLOCK_HZ(CLOCK_HZ)) serial(clk,resetn,commit && uart_store,b[7:0],uart_ready,uart_tx);
    assign debug_pc = pc;
    assign debug_insn = insn;
    assign debug_commit = commit;
    assign debug_we = reg_we;
    assign debug_rd = rd;
    assign debug_wdata = wdata;
    assign debug_reg_data = debug_reg_addr == 0 ? 32'b0 : regs[debug_reg_addr];
    assign debug_uart_valid = commit && uart_store;
    assign debug_uart_data = b[7:0];
endmodule
