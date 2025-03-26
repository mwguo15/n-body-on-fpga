/* File: main.cpp */

#include "include/raytracer.h"

int main() {
    // Image parameters
    const int image_width = 400;
    const int image_height = 200;
    const int samples_per_pixel = 100;
    const int max_depth = 50;

    // Create a scene with several spheres.
    Scene scene;
    // Ground: a large sphere acting as a plane.
    scene.add(Sphere(Vec(0, -100.5, -1), 100));
    // Three spheres in the scene.
    scene.add(Sphere(Vec(0, 0, -1), 0.5));
    scene.add(Sphere(Vec(-1, 0, -1), 0.5));
    scene.add(Sphere(Vec(1, 0, -1), 0.5));

    // Open output file
    std::ofstream ofs("image.ppm");
    ofs << "P3\n" << image_width << " " << image_height << "\n255\n";

    // Camera setup (simple pinhole camera)
    Vec lower_left_corner(-2.0, -1.0, -1.0);
    Vec horizontal(4.0, 0.0, 0.0);
    Vec vertical(0.0, 2.0, 0.0);
    Vec origin(0.0, 0.0, 0.0);

    // Loop over image pixels (from top to bottom)
    for (int j = image_height - 1; j >= 0; j--) {
        std::cerr << "\rScanlines remaining: " << j << " " << std::flush;
        for (int i = 0; i < image_width; i++) {
            Color col(0, 0, 0);
            // Accumulate samples for antialiasing and noise reduction.
            for (int s = 0; s < samples_per_pixel; s++) {
                double u = (i + rand() / (double)RAND_MAX) / double(image_width);
                double v = (j + rand() / (double)RAND_MAX) / double(image_height);
                Ray r(origin, lower_left_corner + horizontal * u + vertical * v - origin);
                col += ray_color(r, scene, max_depth);
            }
            col = col / double(samples_per_pixel);
            // Apply gamma correction (gamma=2.0)
            col = Color(std::sqrt(col.x), std::sqrt(col.y), std::sqrt(col.z));
            ofs << to_int(col.x) << " " << to_int(col.y) << " " << to_int(col.z) << "\n";
        }
    }
    ofs.close();
    std::cerr << "\nDone.\n";
    return 0;
}