`default_nettype none

module NBodySim_tb;
    logic clock, reset, start, done;

    NBodySim(.clk(clock),
             .reset(reset),
             .start(start),
             .done(done)
    );

    initial begin
        clock = 1'b0;

        forever #5 clock = ~clock;
    end

endmodule : NBodySim_tb