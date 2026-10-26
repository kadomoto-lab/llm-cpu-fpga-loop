// 8N1 transmitter; clock enables keep all sequential logic on the CPU clock.
module uart_tx #(parameter integer CLOCK_HZ = 25000000, BAUD = 115200)(
    input wire clk, resetn, valid, input wire [7:0] data,
    output wire ready, output wire tx
);
    localparam integer DIVISOR = (CLOCK_HZ + BAUD/2) / BAUD;
    localparam [15:0] RELOAD = DIVISOR[15:0]-16'd1;
    reg [15:0] timer = 0;
    reg [3:0] remaining = 0;
    reg [9:0] frame = 10'h3ff;
    assign ready = remaining == 0;
    assign tx = frame[0];
    always @(posedge clk) begin
        if (!resetn) begin
            timer <= 0; remaining <= 0; frame <= 10'h3ff;
        end else if (ready) begin
            if (valid) begin
                frame <= {1'b1,data,1'b0}; remaining <= 10; timer <= RELOAD;
            end
        end else if (timer == 0) begin
            frame <= {1'b1,frame[9:1]}; remaining <= remaining - 1'b1; timer <= RELOAD;
        end else timer <= timer - 1'b1;
    end
endmodule
