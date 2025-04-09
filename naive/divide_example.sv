`default_nettype none
`timescale 1 ps / 1 ps

module divide_tb;
    logic [15:0] quotient;
    logic clk;

    division div_inst (
            .denom(16'd5),
            .numer(16'd20),
            .quotient(quotient),
            .remain()
    );

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk; // 100 MHz clock
    end

    // Wait some time for result to stabilize
    initial begin
        #50; // Wait 50 ps (enough for simple combinational driver)
        $display("20 divided by 5 is %d", quotient);
        $finish;
    end

endmodule : divide_tb