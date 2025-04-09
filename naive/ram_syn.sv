module ram_test_fpga (
    input wire CLOCK_50,         // 50 MHz input clock
    input wire [14:0] SW,        // Switches [14:0] for address selection
    input wire [0:0] KEY,        // KEY[0] as active-low reset
    output wire [6:0] HEX0,
    output wire [6:0] HEX1,
    output wire [6:0] HEX2,
    output wire [6:0] HEX3
);

    wire [14:0] rd_addr = SW;
    wire [79:0] q;
    reg  [79:0] q_reg;

    // Tie off write port (we're only reading for now)
    wire [14:0] wr_addr = 15'd0;
    wire [79:0] wr_data = 80'd0;
    wire        wren    = 1'b0;

    // RAM instantiation
    ram_2_port ram_inst (
        .clock     (CLOCK_50),
        .data      (wr_data),
        .rdaddress (rd_addr),
        .wraddress (wr_addr),
        .wren      (wren),
        .q         (q)
    );

    // Register the output since q is registered (outdata_reg_b = CLOCK0)
    always @(posedge CLOCK_50) begin
        q_reg <= q;
    end

    // Show lowest 16 bits of output on HEX displays
    hex_display h0(.in(q_reg[3:0]),   .out(HEX0));
    hex_display h1(.in(q_reg[7:4]),   .out(HEX1));
    hex_display h2(.in(q_reg[11:8]),  .out(HEX2));
    hex_display h3(.in(q_reg[15:12]), .out(HEX3));

endmodule


// Simple hex digit to 7-segment decoder
module hex_display (
    input  wire [3:0] in,
    output reg  [6:0] out
);
    always @(*) begin
        case (in)
            4'h0: out = 7'b1000000;
            4'h1: out = 7'b1111001;
            4'h2: out = 7'b0100100;
            4'h3: out = 7'b0110000;
            4'h4: out = 7'b0011001;
            4'h5: out = 7'b0010010;
            4'h6: out = 7'b0000010;
            4'h7: out = 7'b1111000;
            4'h8: out = 7'b0000000;
            4'h9: out = 7'b0010000;
            4'hA: out = 7'b0001000;
            4'hB: out = 7'b0000011;
            4'hC: out = 7'b1000110;
            4'hD: out = 7'b0100001;
            4'hE: out = 7'b0000110;
            4'hF: out = 7'b0001110;
            default: out = 7'b1111111;
        endcase
    end
endmodule
