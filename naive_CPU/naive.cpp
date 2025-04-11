#include <stdio.h>
#include <stdint.h>
#include <math.h>

// Fixed-point precision (Q8.8)
#define SCALE 256
#define SCALE_SQ 65536

typedef struct {
    int16_t x, y;     // Q8.8 positions
    uint16_t mass;    // Integer mass
} Body;

// Fixed-point multiply (matches Verilog rounding)
int16_t mul_fixed(int16_t a, int16_t b) {
    return (int16_t)(((int32_t)a * b + (SCALE/2)) >> 8);
}

void compute_force(const Body* a, const Body* b, int16_t* fx, int16_t* fy) {
    const int16_t G = 1 * SCALE;         // 0x0100
    const int16_t SOFTENING = 1 * SCALE;  // 0x0100

    // 1. Delta calculation (Q8.8)
    int16_t dx = b->x - a->x;
    int16_t dy = b->y - a->y;

    // 2. Distance squared (Verilog: (dx*dx)>>8)
    int32_t dx_sq = ((int32_t)dx * dx) >> 8;
    int32_t dy_sq = ((int32_t)dy * dy) >> 8;
    int32_t dist_sq = dx_sq + dy_sq + (SOFTENING * SOFTENING >> 8);

    // 3. Avoid division by zero
    if (dist_sq < 1) dist_sq = 1;

    // 4. 1/sqrt(dist_sq) approximation
    int32_t inv_dist_sq = SCALE_SQ / dist_sq;

    // 5. Force magnitude (Q8.8)
    int32_t magnitude = ((int32_t)G * b->mass * inv_dist_sq) >> 16;

    // 6. Force components
    *fx = mul_fixed(magnitude, dx);
    *fy = mul_fixed(magnitude, dy);
}

// Print float as both decimal and exact hex representation
void print_force(const char* label, int16_t fx, int16_t fy) {
    // Convert Q8.8 to float
    float fx_float = (float)fx / SCALE;
    float fy_float = (float)fy / SCALE;

    // Get raw hex bits (for Verilog comparison)
    uint16_t fx_hex = (uint16_t)fx;
    uint16_t fy_hex = (uint16_t)fy;

    printf("%s:\n", label);
    printf("  Float: x=%10.6f, y=%10.6f\n", fx_float, fy_float);
    printf("  Hex:   x=0x%04X (%5d), y=0x%04X (%5d)\n", 
           fx_hex, fx, fy_hex, fy);
}

int main() {
    // Initialize bodies (Q8.8 positions)
    Body bodies[2] = {
        {100 * SCALE, 100 * SCALE, 500},   // Body 0
        {105 * SCALE, 105 * SCALE, 120}   // Body 1
    };

    // Compute forces
    int16_t fx0, fy0, fx1, fy1;
    compute_force(&bodies[0], &bodies[1], &fx0, &fy0);  // Force on body 0
    compute_force(&bodies[1], &bodies[0], &fx1, &fy1);  // Force on body 1 (Newton's 3rd law)

    // Print results
    print_force("Force on body 0", fx0, fy0);
    print_force("Force on body 1", fx1, fy1);

    return 0;
}