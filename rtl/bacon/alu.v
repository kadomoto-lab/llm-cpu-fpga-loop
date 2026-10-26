`include "define.vh"
module alu(input wire [5:0] alucode, input wire [31:0] op1, op2,
           output reg [31:0] alu_result, output reg br_taken);
    always @* begin
        alu_result = 0; br_taken = 0;
        case (alucode)
            `ALU_ADD, `ALU_LB, `ALU_LH, `ALU_LW, `ALU_LBU, `ALU_LHU,
            `ALU_SB, `ALU_SH, `ALU_SW: alu_result = op1 + op2;
            `ALU_SUB: alu_result = op1 - op2;
            `ALU_SLT: alu_result = {31'b0,($signed(op1) < $signed(op2))};
            `ALU_SLTU: alu_result = {31'b0,(op1 < op2)};
            `ALU_XOR: alu_result = op1 ^ op2;
            `ALU_OR: alu_result = op1 | op2;
            `ALU_AND: alu_result = op1 & op2;
            `ALU_SLL: alu_result = op1 << op2[4:0];
            `ALU_SRL: alu_result = op1 >> op2[4:0];
            `ALU_SRA: alu_result = $signed(op1) >>> op2[4:0];
            `ALU_LUI: alu_result = op2;
            `ALU_JAL, `ALU_JALR: begin alu_result = op2 + 4; br_taken = 1; end
            `ALU_BEQ: br_taken = op1 == op2;
            `ALU_BNE: br_taken = op1 != op2;
            `ALU_BLT: br_taken = $signed(op1) < $signed(op2);
            `ALU_BGE: br_taken = $signed(op1) >= $signed(op2);
            `ALU_BLTU: br_taken = op1 < op2;
            `ALU_BGEU: br_taken = op1 >= op2;
            default: begin end
        endcase
    end
endmodule
