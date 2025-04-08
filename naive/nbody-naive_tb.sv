`default_nettype none

`timescale 1ns/1ps

module NBodySim_tb();
    logic clk, reset, start, done;

    // Instantiate NBodySim
    NBodySim sim (
        .clk(clk),
        .reset(reset),
        .start(start),
        .done(done)
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
        logic [31:0] force_body0, force_body1;
        
        // Read address 400 (0x190) - Force on body 0
        force_ram_read(32'h190, force_body0);
        $display("Force on body 0: x=%0d, y=%0d", 
                 force_body0[31:16], force_body0[15:0]);
        
        // Read address 401 (0x191) - Force on body 1
        force_ram_read(32'h191, force_body1);
        $display("Force on body 1: x=%0d, y=%0d", 
                 force_body1[31:16], force_body1[15:0]);
    endtask

    // Helper task to read from RAM (replace with your actual RAM interface)
    task force_ram_read(input logic [31:0] addr, output logic [31:0] data);
        // This assumes your RAM returns 32-bit force values at these addresses
        // Replace with your actual RAM read protocol
        @(posedge clk);
        sim.addr = addr;      // Replace with actual RAM address input
        sim.rd_en = 1;        // Replace with actual RAM read enable
        @(posedge clk);
        data = sim.read_body; // Replace with actual RAM data output
        sim.rd_en = 0;
    endtask

    // Initialize RAM with body data (from .mif)
    initial begin
        // Body 0: x=100, y=100, vx=200, vy=200, mass=5
        // Body 1: x=200, y=200, vx=100, vy=100, mass=2
        // Wait for reset to complete
        @(negedge reset);
        
        // Write to RAM (replace with your actual RAM write protocol)
        @(posedge clk);
        sim.wr_en = 1;
        sim.addr = 0;
        sim.write_data = 80'h0064006400C800C80005; // Body 0 data
        @(posedge clk);
        sim.addr = 1;
        sim.write_data = 80'h00C800C8006400640002; // Body 1 data
        @(posedge clk);
        sim.wr_en = 0;
    end
endmodule
