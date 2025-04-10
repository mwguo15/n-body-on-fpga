`timescale 1ns/1ps

module tb_ram_2_port;
  // Testbench signals
  reg         clock;
  reg         wren;
  reg  [14:0] wraddress, rdaddress;
  reg  [79:0] data;
  wire [79:0] q;

  // Instantiate your DUT (Device Under Test)
  ram_2_port dut (
    .clock    (clock),
    .data     (data),
    .rdaddress(rdaddress),
    .wraddress(wraddress),
    .wren     (wren),
    .q        (q)
  );

  logic [79:0] output_dummy;
  ram_2_port dummy (
    .clock(clock),
    .data(),
    .rdaddress(15'd400),
    .wraddress(),
    .wren(1'b0),
    .q(output_dummy)
  );

  // Generate a simple clock
  initial begin
    clock = 1'b0;
    forever #5 clock = ~clock;  // 10ns period
  end

  // Test sequence
  initial begin
    // Initialize signals
    wren      = 0;
    wraddress = 15'd0;
    rdaddress = 15'd0;
    data      = 80'h0;

    // Let clock stabilize for a few cycles
    repeat(2) @(posedge clock);

    // Write a test value into the RAM at address 15'd10
    wren        = 1;
    wraddress   = 15'd400;
    data        = 80'hDEAD_BEEF_CAFE_0123_4567;  // 80-bit data example
    @(posedge clock);

    // Read back from the same address
    rdaddress = 15'd400;
    @(posedge clock);  // Wait one cycle for the data to appear at q
    @(posedge clock);

    // Display the read value
    $display("Time %0t, Read data at address %0d = %h (expected %h)",
             $time, rdaddress, q, data);

    // Simple check for correctness
    if (q == data) begin
      $display("PASS: Data matches!");
    end else begin
      $display("FAIL: Data mismatch!");
    end

    @(posedge clock);
    $display("Time %0t, Dummy read data at address %0d = %h",
             $time, rdaddress, output_dummy);

    $finish;
  end
endmodule
