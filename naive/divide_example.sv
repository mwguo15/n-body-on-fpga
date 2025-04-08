`timescale 1 ps / 1 ps
`default_nettype none

module divide_tb;
    logic [15:0] quotient;

    division(.denom(16'd1),
             .numer(16'd2),
             .quotient(quotient),
             .remain()
    );

    initial begin
        $display("2 divided by 1 is %d", quotient);
        $finish;
    end

endmodule : divide_tb