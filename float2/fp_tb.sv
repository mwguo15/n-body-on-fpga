`timescale 1ns/1ps

module fp_testbench;

  // DUT signals
  logic clk = 0;
  logic reset = 0;
  logic clk_en = 1;
  logic start = 0;
  logic reset_req = 0;

  logic [31:0] dataa, datab;
  logic [2:0]  n;  // operation select
  logic [31:0] result;
  logic done;

  // Instantiate the DUT
  FP dut (
    .clk(clk),
    .clk_en(clk_en),
    .dataa(dataa),
    .datab(datab),
    .n(n),
    .reset(reset),
    .reset_req(reset_req),
    .start(start),
    .done(done),
    .result(result)
  );

  // Clock generation
  always #5 clk = ~clk;  // 100MHz

  // IEEE-754 conversion
  function [31:0] float_to_bits(input real f);
    return $realtobits(f);
  endfunction

  function real bits_to_float(input [31:0] b);
    return $bitstoreal(b);
  endfunction

  // Test procedure
  initial begin
    $display("==== Floating Point Unit Test ====");
    reset <= 1;
    #20;
    reset <= 0;

    // Set inputs (e.g., 2.5 + 3.0)
    dataa <= float_to_bits(2.5);
    datab <= float_to_bits(3.0);
    n     <= 3'd0;  // ADD
    start <= 1;

    #10;
    start <= 0;

    // Wait for done
    wait (done);

    $display("Result (raw bits): %h", result);
    $display("Result (float): %f", bits_to_float(result));

    #20;
    $stop;
  end

endmodule
