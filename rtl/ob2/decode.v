`include "define.vh"
module decode(
    input wire [31:0] ir,
    output reg [4:0] srcreg1_num, srcreg2_num, dstreg_num,
    output reg [31:0] imm,
    output reg [5:0] alucode,
    output reg [1:0] aluop1_type, aluop2_type,
    output reg reg_we, is_load, is_store, is_halt
);
    wire [6:0] opcode = ir[6:0];
    wire [2:0] funct3 = ir[14:12];
    always @* begin
        srcreg1_num = 0; srcreg2_num = 0; dstreg_num = 0;
        imm = 0; alucode = `ALU_NOP;
        aluop1_type = `OP_TYPE_NONE; aluop2_type = `OP_TYPE_NONE;
        reg_we = 0; is_load = 0; is_store = 0; is_halt = 0;
        case (opcode)
            `LUI, `AUIPC: begin
                dstreg_num = ir[11:7]; reg_we = (ir[11:7] != 0);
                imm = {ir[31:12],12'b0};
                aluop1_type = opcode == `LUI ? `OP_TYPE_NONE : `OP_TYPE_IMM;
                aluop2_type = opcode == `LUI ? `OP_TYPE_IMM : `OP_TYPE_PC;
                alucode = opcode == `LUI ? `ALU_LUI : `ALU_ADD;
            end
            `JAL: begin
                dstreg_num = ir[11:7]; reg_we = (ir[11:7] != 0);
                imm = {{11{ir[31]}},ir[31],ir[19:12],ir[20],ir[30:21],1'b0};
                alucode = `ALU_JAL; aluop2_type = `OP_TYPE_PC;
            end
            `JALR: begin
                srcreg1_num = ir[19:15]; dstreg_num = ir[11:7];
                reg_we = (ir[11:7] != 0); imm = {{20{ir[31]}},ir[31:20]};
                alucode = `ALU_JALR;
                aluop1_type = `OP_TYPE_REG; aluop2_type = `OP_TYPE_PC;
                if (funct3 != 0) is_halt = 1;
            end
            `BRANCH: begin
                srcreg1_num = ir[19:15]; srcreg2_num = ir[24:20];
                imm = {{19{ir[31]}},ir[31],ir[7],ir[30:25],ir[11:8],1'b0};
                aluop1_type = `OP_TYPE_REG; aluop2_type = `OP_TYPE_REG;
                case (funct3)
                    0: alucode = `ALU_BEQ;
                    1: alucode = `ALU_BNE;
                    4: alucode = `ALU_BLT;
                    5: alucode = `ALU_BGE;
                    6: alucode = `ALU_BLTU;
                    7: alucode = `ALU_BGEU;
                    default: is_halt = 1;
                endcase
            end
            `LOAD, `STORE: begin
                srcreg1_num = ir[19:15];
                aluop1_type = `OP_TYPE_REG; aluop2_type = `OP_TYPE_IMM;
                if (opcode == `LOAD) begin
                    dstreg_num = ir[11:7]; reg_we = (ir[11:7] != 0); is_load = 1;
                    imm = {{20{ir[31]}},ir[31:20]};
                    case (funct3)
                        0: alucode = `ALU_LB;
                        1: alucode = `ALU_LH;
                        2: alucode = `ALU_LW;
                        4: alucode = `ALU_LBU;
                        5: alucode = `ALU_LHU;
                        default: is_halt = 1;
                    endcase
                end else begin
                    srcreg2_num = ir[24:20]; is_store = 1;
                    imm = {{20{ir[31]}},ir[31:25],ir[11:7]};
                    case (funct3)
                        0: alucode = `ALU_SB;
                        1: alucode = `ALU_SH;
                        2: alucode = `ALU_SW;
                        default: is_halt = 1;
                    endcase
                end
            end
            `OP, `OPIMM: begin
                srcreg1_num = ir[19:15]; dstreg_num = ir[11:7];
                reg_we = (ir[11:7] != 0); aluop1_type = `OP_TYPE_REG;
                if (opcode == `OP) begin
                    srcreg2_num = ir[24:20]; aluop2_type = `OP_TYPE_REG;
                    if (ir[31:25] != 0 && !(ir[31:25] == 7'h20 && (funct3 == 0 || funct3 == 5))) is_halt = 1;
                end else begin
                    aluop2_type = `OP_TYPE_IMM;
                    imm = {{20{ir[31]}},ir[31:20]};
                    if (funct3 == 1 || funct3 == 5) begin
                        imm = {27'b0,ir[24:20]};
                        if (ir[31:25] != 0 && !(funct3 == 5 && ir[31:25] == 7'h20)) is_halt = 1;
                    end
                end
                case (funct3)
                    0: alucode = (opcode == `OP && ir[30]) ? `ALU_SUB : `ALU_ADD;
                    1: alucode = `ALU_SLL;
                    2: alucode = `ALU_SLT;
                    3: alucode = `ALU_SLTU;
                    4: alucode = `ALU_XOR;
                    5: alucode = ir[30] ? `ALU_SRA : `ALU_SRL;
                    6: alucode = `ALU_OR;
                    7: alucode = `ALU_AND;
                endcase
            end
            // In-order execution makes FENCE a no-op. ECALL/EBREAK halt;
            // this bare-metal teaching CPU has no privileged trap handler.
            7'b0001111: begin end
            default: is_halt = 1;
        endcase
    end
endmodule
