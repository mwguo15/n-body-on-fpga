`default_nettype none

`timescale 1ns/1ps

module NBodySim_tb();
    logic clk, reset, start, done;
    logic [79:0] read_data;
    logic [14:0] addr;

    // Instantiate NBodySim
    NBodySim #(.N(2)) sim(
        .clk(clk),
        .reset(reset),
        .start(start),
        .done(done)
    );

    ram_2_port BRAM(.clock(clk),
                    .data(),
                    .rdaddress(addr),
                    .wraddress(),
                    .wren(),
                    .q(read_data)
                   );  

    // Clock generation (100 MHz)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Test sequence
    initial begin
        // Initialize signals
        reset = 1;
        start = 0;
        
        // Apply reset
        #20 reset = 0;
        
        // Wait a cycle, then start simulation
        @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;
        
        // Wait for completion
        wait(done == 1);
        $display("Simulation completed at time %0t", $time);
        
        // Read and print forces
        $display("Reading forces from memory...");
        print_forces();
        
        $finish;
    end

    task print_forces();
        logic [31:0] force_body0, force_body1, force_body2, force_body3, force_body4;
        
        // Read address 400 (0x190) - Force on body 0
        // ram_read(32'h190, force_body0);
        // $display("Force on body 0: x=%0d, y=%0d", 
        //          force_body0[31:16], force_body0[15:0]);
        
        // // Read address 401 (0x191) - Force on body 1
        // ram_read(32'h191, force_body1);
        // $display("Force on body 1: x=%0d, y=%0d", 
        //          force_body1[31:16], force_body1[15:0]);

        // // Read address 402 (0x192) - Force on body 0
        // ram_read(32'h192, force_body0);
        // $display("Force on body 2: x=%0d, y=%0d", 
        //          force_body2[31:16], force_body2[15:0]);
        
        // // Read address 403 (0x193) - Force on body 1
        // ram_read(32'h193, force_body1);
        // $display("Force on body 3: x=%0d, y=%0d", 
        //          force_body3[31:16], force_body3[15:0]);

        // // Read address 404 (0x194) - Force on body 1
        // ram_read(32'h194, force_body1);
        // $display("Force on body 4: x=%0d, y=%0d", 
        //          force_body4[31:16], force_body4[15:0]);
        ram_read(32'h0, force_body0);
        $display("Force on body 0: x=%0d, y=%0d", 
                 force_body0[31:16], force_body0[15:0]);
        
        // Read address 401 (0x191) - Force on body 1
        ram_read(32'h1, force_body1);
        $display("Force on body 1: x=%0d, y=%0d", 
                 force_body1[31:16], force_body1[15:0]);

        // Read address 402 (0x192) - Force on body 0
        ram_read(32'h2, force_body0);
        $display("Force on body 2: x=%0d, y=%0d", 
                 force_body2[31:16], force_body2[15:0]);
        
        // Read address 403 (0x193) - Force on body 1
        ram_read(32'h3, force_body1);
        $display("Force on body 3: x=%0d, y=%0d", 
                 force_body3[31:16], force_body3[15:0]);

        // Read address 404 (0x194) - Force on body 1
        ram_read(32'h4, force_body1);
        $display("Force on body 4: x=%0d, y=%0d", 
                 force_body4[31:16], force_body4[15:0]);
    endtask

    // Helper task to read from RAM (replace with your actual RAM interface)
    task ram_read(input logic [14:0] rd_addr, output logic [79:0] data);
        @(posedge clk);
        addr = rd_addr;      
        @(posedge clk);
        data <= read_data;
        @(posedge clk); 
    endtask

endmodule: NBodySim_tb