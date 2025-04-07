`default_nettype none

`include "RAM_2_PORT.v"

typedef struct packed {
    logic signed [15:0] x;
    logic signed [15:0] y;
    logic signed [15:0] vx;
    logic signed [15:0] vy;
    logic        [15:0] mass;
} body_t;

module NBodySim #(parameter int N = 16,
                  parameter ADDR_WIDTH = $clog(N)
                  parameter DT = 32'h3C23D70A,         // 0.01 in IEEE 754 - timestep
                  parameter G = 32'h3A83126F,          // 0.001 in IEEE 754 - gravitatonal constant
                  parameter SOFTENING = 32'h3A83126F)  // 0.001 in IEEE 754 - prevents infinite force

(
    input  logic clk,
    input  logic reset,
    input  logic start,
    output logic done
);

logic body_t body_i, body_j;
logic [31:0] total_force_x, total_force_y; // Accumulated force for body_i

logic [15:0] bram_address_a;

RAM_2_PORT bram(
    .aclr(1'b0),
    .address_a(bram_address_a),
    .address_b(),
    .clock(clk),
    .data_a(),      // Write data a
    .data_b(16'b0), // Write data b
    .rden_a(),
    .rden_b(),
    .wren_a(),
    .wren_b(),
    .q_a(),         // Read data a
    .q_b()          // Read data b
);

enum logic [3:0] {
    IDLE,           
    LOAD_BODY_I,    // Set RAM address = i to load body i
    INIT_BODY_I,    // Register body i TODO: maybe not necessary
    LOAD_BODY_J,    // Set RAM address = j to load body j (inner loop)
    COMPUTE_FORCE,  // Compute force contribution from body i
    INCR_J,         // Increment j and check loop condition
    UPDATE_BODY,    // Compute updated velocity and position for body i
    WRITE_BACK,     // Write updated body back to BRAM
    INCR_I,         // Increment i (outer loop)
    DONE
} currState, nextState;

always_ff @(posedge clk) begin
    if (reset)
        currState <= IDLE;
    else
        currState <= nextState;
end

always_comb begin
    case (currState)
        IDLE: begin
            if (start)
                nextState = LOAD_BODY_I;
            else
                nextState = IDLE;
        end
        LOAD_BODY_I: begin
            bram_address_a <= i;
        end
        INIT_BODY_I: begin
            // TODO: might remove
        end 
        LOAD_BODY_J: begin
            bram_address_a <= j;
        end
        COMPUTE_FORCE: begin
            // Use force_calculator module
        end
        INCR_J:        
        UPDATE_BODY:    
        WRITE_BACK:     
        INCR_I:
        DONE:
    endcase
end


endmodule : NBodySim


module force_calculator #(
    parameter G = 32'h3A83126F,         // 0.001 in IEEE 754
    parameter SOFTENING = 32'h3A83126F)   // 0.001 in IEEE 754
(
    input  logic enable,    
    input  body_t body_i,  
    input  body_t body_j, 
    input  logic valid_in, // Input data valid    
    output logic [31:0] force_x, force_y,  // Force components
    output logic valid_out // Output data valid
);

// Internal signals
logic signed [15:0] pos_x_i, pos_x_j, pos_y_i, pos_y_j;
logic [15:0] mass_i, mass_j
logic [31:0] dx, dy;
logic [31:0] dist_sq, inv_dist, inv_dist3;
logic [31:0] magnitude;

always_comb begin
    pos_x_i = body_i.x;
    pos_y_i = body_i.y;
    mass_i  = body_i.mass;
    pos_x_j = body_j.x;
    pos_y_j = body_j.y;
    mass_j  = body_j.mass;
end

always_comb begin
    // Calculate distance components
    dx = pos_x_j - pos_x_i;
    dy = pos_y_j - pos_y_i;
    
    // Calculate distance squared with softening
    dist_sq = dx * dx + dy * dy + SOFTENING * SOFTENING;
    
    // Inverse distance calculation
    inv_dist = 1.0 / $sqrt(dist_sq);
    inv_dist3 = inv_dist * inv_dist * inv_dist;
    
    // Force magnitude
    magnitude = G * mass_j * inv_dist3;
    
    // Force components
    force_x = magnitude * dx;
    force_y = magnitude * dy;
end

endmodule: force_calculator