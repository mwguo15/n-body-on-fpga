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
        logic [79:0] data0, data1, data2, data3, data4;
        
        ram_read(32'h190, data0);
        $display("Force 0: %20h", data0);
        
        ram_read(32'h191, data1);
        $display("Force 1: %20h", data1);

        ram_read(32'h192, data2);
        $display("Force 2: %20h", data2);
        
        ram_read(32'h193, data3);
        $display("Force 3: %20h", data3);

        ram_read(32'h194, data4);
        $display("Force 4: %20h", data4);

        ram_read(32'h0, data0);
        $display("Body 0: %20h", data0);
        
        ram_read(32'h1, data1);
        $display("Body 1: %20h", data1);

        ram_read(32'h2, data2);
        $display("Body 2: %20h", data2);
        
        ram_read(32'h3, data3);
        $display("Body 3: %20h", data3);

        ram_read(32'h4, data4);
        $display("Body 4: %20h", data4);

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