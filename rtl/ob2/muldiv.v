// Registered DSP multiply and 32-step restoring divide. No combinational divider.
module muldiv(input wire clk, resetn, start,
              input wire [2:0] operation, input wire [31:0] a, b,
              output reg done = 0, output reg [31:0] result = 0);
    reg busy = 0;
    reg [5:0] count = 0;
    reg [31:0] quotient, remainder, divisor;
    reg negate_q, negate_r, return_remainder;
    wire signed_div = !operation[0];
    wire [31:0] magnitude_a = signed_div && a[31] ? -a : a;
    wire [31:0] magnitude_b = signed_div && b[31] ? -b : b;
    (* use_dsp = "yes" *) wire [63:0] product = {32'b0,a} * {32'b0,b};
    wire [31:0] high_product = product[63:32]
        - ((operation == 1 || operation == 2) && a[31] ? b : 32'b0)
        - (operation == 1 && b[31] ? a : 32'b0);
    wire [32:0] trial = {remainder,quotient[31]};
    wire subtract = trial >= {1'b0,divisor};
    wire [32:0] reduced = subtract ? trial - {1'b0,divisor} : trial;
    wire [31:0] next_remainder = reduced[31:0];
    wire [31:0] next_quotient = {quotient[30:0],subtract};
    always @(posedge clk) begin
        done <= 0;
        if (!resetn) begin busy <= 0; done <= 0; end
        else if (busy) begin
            remainder <= next_remainder;
            quotient <= next_quotient;
            count <= count + 1'b1;
            if (count == 31) begin
                busy <= 0; done <= 1;
                result <= return_remainder ? (negate_r ? -next_remainder : next_remainder)
                                          : (negate_q ? -next_quotient : next_quotient);
            end
        end else if (start && !done) begin
            if (!operation[2]) begin
                result <= operation == 0 ? product[31:0] : high_product;
                done <= 1;
            end else if (b == 0) begin
                result <= operation[1] ? a : 32'hffffffff;
                done <= 1;
            end else begin
                quotient <= magnitude_a; divisor <= magnitude_b; remainder <= 0;
                negate_q <= signed_div && (a[31] != b[31]);
                negate_r <= signed_div && a[31]; return_remainder <= operation[1];
                count <= 0; busy <= 1;
            end
        end
    end
endmodule
