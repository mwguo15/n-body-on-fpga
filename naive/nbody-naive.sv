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
                  parameter ADDR_WIDTH = $clog2(N),
                  parameter DT = 32'h3C23D70A,         // 0.01 in IEEE 754 - timestep
                  parameter G = 32'h3A83126F,          // 0.001 in IEEE 754 - gravitatonal constant
                  parameter SOFTENING = 32'h3A83126F)  // 0.001 in IEEE 754 - prevents infinite force

(
    input  logic clk,
    input  logic reset,
    input  logic start,
    output logic done
);

body_t read_body;
body_t body_i, body_j;
logic register_i, register_j;
logic fc_en, fc_done, fc_valid;
logic [31:0] force_x, force_y;
logic [31:0] total_force_x, total_force_y; // Accumulated force for body_i
logic [7:0] i, j; // i and j counters
logic incr_i, incr_j;
logic [15:0] addr;
logic [79:0] write_data;
logic rd_en, wr_en;

force_calculator FORCE(.clk, .reset, .enable(fc_en), 
                       .body_i, .body_j, .valid_in(fc_valid), 
                       .force_x, .force_y, .valid_out(fc_done));

RAM_2_PORT BRAM(.aclr(1'b0),
                .address_a(addr),
                .address_b(addr),
                .clock(clk),
                .data_a(write_data),      // Write data a
                .data_b(write_data),                // Write data b
                .rden_a(rd_en),
                .rden_b(rd_en),
                .wren_a(wr_en),
                .wren_b(wr_en),
                .q_a(read_body),          // Read data a
                .q_b(read_body)                    // Read data b
               );

enum logic [3:0] {
    IDLE,   
    LOAD_BODY_I,    // Set RAM address = i to load body i
    INIT_BODY_I,    // Register body_i
    LOAD_BODY_J,    // Set RAM address = j to load body j (inner loop)
    INIT_BODY_J,    // Register body_j
    COMPUTE_FORCE,  // Compute force contribution from body j
    WAIT_FORCE,     // Wait for force calculation to finish
    INCR_J,         // Increment j and check loop condition
    UPDATE_BODY,    // Compute updated velocity and position for body i
    WRITE_BACK,     // Write updated body back to BRAM
    INCR_I,         // Increment i (outer loop)
    DONE
} currState, nextState;

// State register
always_ff @(posedge clk) begin
    if (reset)
        currState <= IDLE;
    else
        currState <= nextState;
end

// Body register
always_ff @(posedge clk) begin
    if (reset) begin
        body_i <= 0;
        body_j <= 0;
    end
    else if (register_i)
        body_i <= read_body;
    else if (register_j)
        body_j <= read_body;
end

// i and j register
always_ff @(posedge clk) begin
    if (reset) begin
        i <= 0;
        j <= 0;
    end
    else if (incr_i) begin
        i <= i + 8'b1;
        j <= 0; // Next outer loop --> reset j
    end
    else if (incr_j) begin
        j <= j + 8'b1;
    end
end

// Force accumulator 
always_ff @(posedge clk) begin
    if (reset | incr_i) begin // If changing base body, reset force
        total_force_x <= 0;
        total_force_y <= 0;
    end
    else if (fc_done) begin
        total_force_x <= total_force_x + force_x;
        total_force_y <= total_force_y + force_y;
    end
end

// Next state and control signal generation 
always_comb begin
    addr = 0;
    rd_en = 0;
    wr_en = 0;
    fc_en = 0;
    fc_valid = 0;
    incr_i = 0;
    incr_j = 0;
    register_i = 0;
    register_j = 0;
    done = 0;
    write_data = '0;

    case (currState)
        IDLE: begin
            nextState = start ? LOAD_BODY_I : IDLE;
        end
        LOAD_BODY_I: begin
            rd_en = 1;
            addr = i; // Reading body_i
            nextState = INIT_BODY_I;
        end
        INIT_BODY_I: begin
            register_i = 1;     // Register body_i
            nextState = LOAD_BODY_J;
        end 
        LOAD_BODY_J: begin
            rd_en = 1;
            addr = j; // Reading body_j
            nextState = INIT_BODY_J;
        end
        INIT_BODY_J: begin
            register_j = 1;     // Register body_j
            nextState = COMPUTE_FORCE;
        end
        COMPUTE_FORCE: begin
            fc_en = 1; // Begin force calculation
            fc_valid = 1; // Signal that the input is valid for 1 cycle
            nextState = j < N - 1 ? WAIT_FORCE : UPDATE_BODY;
        end
        WAIT_FORCE: begin
            fc_en = 1; // Keeping force calculation enabled while it runs 
            nextState = fc_done ? INCR_J : WAIT_FORCE;
        end
        INCR_J: begin
            incr_j = 1'b1;
            nextState = LOAD_BODY_J;
        end
        UPDATE_BODY: begin
            // probably update some velocity position stuff here lol 
            nextState = WRITE_BACK;
        end
        WRITE_BACK: begin
            wr_en = 1;
            write_data = {16'd0, total_force_x, total_force_y};
            addr = i + 400; // 400 is some arbitrary offset
            nextState = INCR_I;
        end
        INCR_I: begin
            incr_i = 1'b1;
            nextState = i < N - 1 ? LOAD_BODY_I : DONE;
        end
        DONE: begin
            done = 1;
            nextState = DONE;
        end
        default: nextState = IDLE;
    endcase
end

endmodule : NBodySim


module force_calculator #(
    parameter G = 32'h66,          // 0.001 in IEEE 754
    parameter SOFTENING = 32'h66,  // 0.001 in IEEE 754
    parameter LATENCY = 6                // Number of pipeline stages
)(
    input  logic clk,                   // Clock
    input  logic reset,                 // Active-high reset
    input  logic enable,                // Module enable
    input  body_t body_i,               // Body i
    input  body_t body_j,               // Body j
    input  logic valid_in,              // Input valid
    output logic [31:0] force_x,        // Force x-component
    output logic [31:0] force_y,        // Force y-component
    output logic valid_out              // Output valid
);

// Pipeline stage registers
typedef struct packed {
    logic signed [15:0] pos_x_i, pos_x_j, pos_y_i, pos_y_j;
    logic [15:0] mass_i, mass_j;
    logic valid;
} stage1_t;

typedef struct packed {
    logic signed [15:0] dx, dy;
    logic [15:0] mass_j;
    logic valid;
} stage2_t;

typedef struct packed {
    logic signed [15:0] dx, dy;
    logic [31:0] dx_sq, dy_sq;
    logic [31:0] softening_sq;
    logic [15:0] mass_j;
    logic valid;
} stage3_t;

typedef struct packed {
    logic signed [15:0] dx, dy;
    logic [31:0] dist_sq;
    logic [15:0] mass_j;
    logic valid;
} stage4_t;

typedef struct packed {
    logic signed [15:0] dx, dy;
    logic [31:0] inv_dist;
    logic [31:0] mass_j_scaled;
    logic valid;
} stage5_t;

typedef struct packed {
    logic [31:0] inv_dist3;
    logic [31:0] mass_j_scaled;
    logic valid;
} stage6_t;

// Pipeline registers
stage1_t s1;
stage2_t s2;
stage3_t s3;
stage4_t s4;
stage5_t s5;
stage6_t s6;

// Softening squared (constant)
logic [31:0] softening_sq;
assign softening_sq = SOFTENING * SOFTENING;

// =============================================
// Pipeline Stage 1: Input Registration
// =============================================
always_ff @(posedge clk) begin
    if (reset | valid_out) 
        s1 <= '0;
    else if (enable) begin
        s1.pos_x_i <= body_i.x;
        s1.pos_y_i <= body_i.y;
        s1.mass_i  <= body_i.mass;
        s1.pos_x_j <= body_j.x;
        s1.pos_y_j <= body_j.y;
        s1.mass_j  <= body_j.mass;
        s1.valid   <= valid_in;
    end
end

// =============================================
// Pipeline Stage 2: Delta Calculation
// =============================================
always_ff @(posedge clk) begin
    if (reset | valid_out) 
        s2 <= '0;
    else if (enable) begin
        s2.dx    <= s1.pos_x_j - s1.pos_x_i;
        s2.dy    <= s1.pos_y_j - s1.pos_y_i;
        s2.mass_j <= s1.mass_j;
        s2.valid <= s1.valid;
    end
end

// =============================================
// Pipeline Stage 3: Squared Calculations
// =============================================
always_ff @(posedge clk) begin
    if (reset | valid_out) 
        s3 <= '0;
    else if (enable) begin
        s3.dx          <= s2.dx;
        s3.dy          <= s2.dy;
        s3.dx_sq       <= s2.dx * s2.dx;
        s3.dy_sq       <= s2.dy * s2.dy;
        s3.softening_sq <= softening_sq;
        s3.mass_j      <= s2.mass_j;
        s3.valid       <= s2.valid;
    end
end

// =============================================
// Pipeline Stage 4: Distance Squared
// =============================================
always_ff @(posedge clk) begin
    if (reset | valid_out) 
        s4 <= '0;
    else if (enable) begin
        s4.dx      <= s3.dx;
        s4.dy      <= s3.dy;
        s4.dist_sq <= s3.dx_sq + s3.dy_sq + s3.softening_sq;
        s4.mass_j  <= s3.mass_j;
        s4.valid   <= s3.valid;
    end
end

// =============================================
// Pipeline Stage 5: Inverse Distance
// =============================================
logic [31:0] sqrt_dist_sq;
logic [31:0] inv_dist;

assign sqrt_dist_sq = $sqrt(s4.dist_sq);
assign inv_dist = 1.0 / sqrt_dist_sq;

always_ff @(posedge clk) begin
    if (reset | valid_out) 
        s5 <= '0;
    else if (enable) begin
        s5.dx          <= s4.dx;  
        s5.dy          <= s4.dy;  
        s5.inv_dist     <= inv_dist;
        s5.mass_j_scaled <= G * s4.mass_j;
        s5.valid       <= s4.valid;
    end
end

// =============================================
// Pipeline Stage 6: Final Force Calculation
// =============================================
logic [31:0] inv_dist3;
logic [31:0] magnitude;

assign inv_dist3 = s5.inv_dist * s5.inv_dist * s5.inv_dist;
assign magnitude = s5.mass_j_scaled * inv_dist3;

always_ff @(posedge clk or posedge reset) begin
    if (reset | valid_out) begin
        s6 <= '0;
        force_x <= '0;
        force_y <= '0;
        valid_out <= '0;
    end else if (enable) begin
        s6.inv_dist3    <= inv_dist3;
        s6.mass_j_scaled <= s5.mass_j_scaled;
        s6.valid        <= s5.valid;
        
        // Final outputs
        force_x <= magnitude * s5.dx;
        force_y <= magnitude * s5.dy;
        valid_out <= s5.valid;
    end
end

endmodule: force_calculator


// module force_calculator_waa #(
//     parameter logic [31:0] G = 32'd66,
//     parameter LATENCY = 6
// )(
//     input  logic clk,
//     input  logic reset,
//     input  logic enable,
//     input  body_t body_i,
//     input  body_t body_j,
//     output logic [31:0] force_x,
//     output logic [31:0] force_y,
//     output logic valid_out
// );

// // Pipeline stage structs (unchanged)
// typedef struct packed {
//     logic signed [15:0] pos_x_i, pos_x_j, pos_y_i, pos_y_j;
//     logic [15:0] mass_j;
// } stage1_t;

// typedef struct packed {
//     logic signed [15:0] dx, dy;
//     logic [15:0] mass_j;
// } stage2_t;

// typedef struct packed {
//     logic signed [15:0] dx, dy;
//     logic [31:0] dx_sq, dy_sq;
//     logic [15:0] mass_j;
// } stage3_t;

// typedef struct packed {
//     logic signed [15:0] dx, dy;
//     logic [31:0] dist_sq;
//     logic [15:0] mass_j;
// } stage4_t;

// typedef struct packed {
//     logic signed [15:0] dx, dy;
//     logic [31:0] inv_dist;
//     logic [31:0] mass_j_scaled;
// } stage5_t;

// typedef struct packed {
//     logic [31:0] inv_dist3;
//     logic [31:0] mass_j_scaled;
//     logic signed [15:0] dx, dy;
// } stage6_t;

// // Pipeline registers
// stage1_t s1;
// stage2_t s2;
// stage3_t s3;
// stage4_t s4;
// stage5_t s5;
// stage6_t s6;

// // Stage 1: Register inputs
// always_ff @(posedge clk or posedge reset) begin
//     if (reset) s1 <= '0;
//     else if (enable) begin
//         s1.pos_x_i <= body_i.x;
//         s1.pos_y_i <= body_i.y;
//         s1.pos_x_j <= body_j.x;
//         s1.pos_y_j <= body_j.y;
//         s1.mass_j  <= body_j.mass;
//     end
// end

// // Stage 2: Compute deltas
// always_ff @(posedge clk or posedge reset) begin
//     if (reset) s2 <= '0;
//     else if (enable) begin
//         s2.dx <= s1.pos_x_j - s1.pos_x_i;
//         s2.dy <= s1.pos_y_j - s1.pos_y_i;
//         s2.mass_j <= s1.mass_j;
//     end
// end

// // Stage 3: Square deltas
// always_ff @(posedge clk or posedge reset) begin
//     if (reset) s3 <= '0;
//     else if (enable) begin
//         s3.dx <= s2.dx;
//         s3.dy <= s2.dy;
//         s3.dx_sq <= s2.dx * s2.dx;
//         s3.dy_sq <= s2.dy * s2.dy;
//         s3.mass_j <= s2.mass_j;
//     end
// end

// // Stage 4: Distance squared (no softening)
// always_ff @(posedge clk or posedge reset) begin
//     if (reset) s4 <= '0;
//     else if (enable) begin
//         s4.dx <= s3.dx;
//         s4.dy <= s3.dy;
//         s4.dist_sq <= s3.dx_sq + s3.dy_sq;
//         s4.mass_j <= s3.mass_j;
//     end
// end

// // Stage 5: Inverse distance & scale
// logic [31:0] inv_dist;

// assign inv_dist = 32'd4294967295 / ($sqrt(s4.dist_sq)); // Fixed-point approximation

// always_ff @(posedge clk or posedge reset) begin
//     if (reset) s5 <= '0;
//     else if (enable) begin
//         s5.dx <= s4.dx;
//         s5.dy <= s4.dy;
//         s5.inv_dist <= inv_dist;
//         s5.mass_j_scaled <= G * s4.mass_j;
//     end
// end

// // Stage 6: inv_dist^3 and final force vector
// logic [31:0] inv_dist3;
// logic [63:0] mag_long;
// logic [31:0] magnitude;

// assign inv_dist3 = (s5.inv_dist * s5.inv_dist) >> 16;
// assign inv_dist3 = (inv_dist3 * s5.inv_dist) >> 16;

// assign mag_long = s5.mass_j_scaled * inv_dist3;
// assign magnitude = mag_long >> 16;

// always_ff @(posedge clk or posedge reset) begin
//     if (reset) begin
//         force_x <= 0;
//         force_y <= 0;
//         valid_out <= 0;
//     end else if (enable) begin
//         force_x <= (magnitude * s5.dx) >>> 16;
//         force_y <= (magnitude * s5.dy) >>> 16;
//         valid_out <= 1;
//     end
// end

// endmodule


// module force_calculator #(
//     parameter G = 32'h3A83126F,         // 0.001 in IEEE 754
//     parameter SOFTENING = 32'h3A83126F)   // 0.001 in IEEE 754
// (
//     input  logic enable,    
//     input  body_t body_i,  
//     input  body_t body_j, 
//     input  logic valid_in, // Input data valid    
//     output logic [31:0] force_x, force_y,  // Force components
//     output logic valid_out // Output data valid
// );

// // Internal signals
// logic signed [15:0] pos_x_i, pos_x_j, pos_y_i, pos_y_j;
// logic [15:0] mass_i, mass_j
// logic [31:0] dx, dy;
// logic [31:0] dist_sq, inv_dist, inv_dist3;
// logic [31:0] magnitude;

// always_comb begin
//     pos_x_i = body_i.x;
//     pos_y_i = body_i.y;
//     mass_i  = body_i.mass;
//     pos_x_j = body_j.x;
//     pos_y_j = body_j.y;
//     mass_j  = body_j.mass;
// end

// always_comb begin
//     // Calculate distance components
//     dx = pos_x_j - pos_x_i;
//     dy = pos_y_j - pos_y_i;
    
//     // Calculate distance squared with softening
//     dist_sq = dx * dx + dy * dy + SOFTENING * SOFTENING;
    
//     // Inverse distance calculation
//     inv_dist = 1.0 / $sqrt(dist_sq);
//     inv_dist3 = inv_dist * inv_dist * inv_dist;
    
//     // Force magnitude
//     magnitude = G * mass_j * inv_dist3;
    
//     // Force components
//     force_x = magnitude * dx;
//     force_y = magnitude * dy;
// end

// endmodule: force_calculator