module top_divide_fpga (
    input  wire [17:0] SW,     // Switches
    output wire [6:0] HEX0,   // Least significant hex digit
    output wire [6:0] HEX1,
    output wire [6:0] HEX2,
    output wire [6:0] HEX3    // Most significant hex digit
);

    wire [7:0] numer = SW[15:8];
    wire [7:0] denom = SW[7:0];

    wire [15:0] quotient;
    wire [15:0] remain;  // unused here, but can be displayed too

    division div_inst (
        .numer({8'b0, numer}),   // Extend to 16 bits
        .denom({8'b0, denom}),
        .quotient(quotient),
        .remain(remain)
    );

    // Display quotient on HEX0–HEX3 (4 hex digits)
    hex_display h0(.in(quotient[3:0]),   .out(HEX0));
    hex_display h1(.in(quotient[7:4]),   .out(HEX1));
    hex_display h2(.in(quotient[11:8]),  .out(HEX2));
    hex_display h3(.in(quotient[15:12]), .out(HEX3));

endmodule

module hex_display (
    input wire [3:0] in,
    output reg [6:0] out
);
    always @(*) begin
        case (in)
            4'h0: out = 7'b100_0000;
            4'h1: out = 7'b111_1001;
            4'h2: out = 7'b010_0100;
            4'h3: out = 7'b011_0000;
            4'h4: out = 7'b001_1001;
            4'h5: out = 7'b001_0010;
            4'h6: out = 7'b000_0010;
            4'h7: out = 7'b111_1000;
            4'h8: out = 7'b000_0000;
            4'h9: out = 7'b001_0000;
            4'hA: out = 7'b000_1000;
            4'hB: out = 7'b000_0011;
            4'hC: out = 7'b100_0110;
            4'hD: out = 7'b010_0001;
            4'hE: out = 7'b000_0110;
            4'hF: out = 7'b000_1110;
            default: out = 7'b111_1111;
        endcase
    end
endmodule
