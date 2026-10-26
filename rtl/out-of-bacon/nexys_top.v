module nexys_top #(parameter integer CLOCK_HZ = 25000000,
    parameter integer PIPELINE = 0, PREDICT = 0, ENABLE_M = 1, DUAL = 0, LOOKAHEAD = 0, BRAM = 0, OOO = 0, FAST = 0)(input wire sysclk, cpu_resetn, output wire uart_tx,
                 output wire [7:0] led);
    wire cpu_clk, clk_raw, feedback, feedback_buf, locked;
    // 100 MHz input, 1 GHz MMCM VCO, exact configurable CPU output clock.
    MMCME2_BASE #(.CLKIN1_PERIOD(10.0), .CLKFBOUT_MULT_F(10.0),
                  .DIVCLK_DIVIDE(1), .CLKOUT0_DIVIDE_F(1000000000.0 / CLOCK_HZ)) clock_manager (
        .CLKIN1(sysclk), .CLKFBIN(feedback_buf), .CLKFBOUT(feedback),
        .CLKOUT0(clk_raw), .LOCKED(locked), .PWRDWN(1'b0), .RST(1'b0)
    );
    BUFG feedback_buffer(.I(feedback),.O(feedback_buf));
    BUFG clock_buffer(.I(clk_raw),.O(cpu_clk));
    (* ASYNC_REG = "TRUE" *) reg [1:0] reset_sync = 0;
    reg [17:0] startup = 0;
    always @(posedge cpu_clk) begin
        reset_sync <= {reset_sync[0],cpu_resetn && locked};
        if (!reset_sync[1]) startup <= 0;
        else if (!(&startup)) startup <= startup + 1'b1;
    end
    wire resetn = &startup;
    wire halted;
    wire [31:0] cycles;
    cpu #(.CLOCK_HZ(CLOCK_HZ), .PIPELINE(PIPELINE), .PREDICT(PREDICT),
          .ENABLE_M(ENABLE_M), .DUAL(DUAL), .LOOKAHEAD(LOOKAHEAD), .BRAM(BRAM), .OOO(OOO), .FAST(FAST)) processor(.clk(cpu_clk),.resetn(resetn),.uart_tx(uart_tx),
        .halted(halted),.debug_reg_addr(5'b0),.cycles(cycles));
    // Keep status-pin timing independent of the startup reduction tree.
    (* IOB = "TRUE" *) reg [7:0] led_state = 0;
    always @(posedge cpu_clk) led_state <= {halted,resetn,cycles[25:20]};
    assign led = led_state;
endmodule
