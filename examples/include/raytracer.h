#ifndef RAYTRACER_H
#define RAYTRACER_H

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

// ----------------------
// Vector and Color Class
// ----------------------
struct Vec {
    double x, y, z;
    
    // Constructors
    inline Vec() : x(0), y(0), z(0) {}
    inline Vec(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    
    // Operator overloads
    inline Vec operator+(const Vec &v) const { return Vec(x + v.x, y + v.y, z + v.z); }
    inline Vec operator-(const Vec &v) const { return Vec(x - v.x, y - v.y, z - v.z); }
    inline Vec operator*(double t) const { return Vec(x * t, y * t, z * t); }
    inline Vec operator/(double t) const { return Vec(x / t, y / t, z / t); }
    
    inline Vec& operator+=(const Vec &v) {
        x += v.x; y += v.y; z += v.z;
        return *this;
    }
    
    // Dot product
    inline double dot(const Vec &v) const { return x * v.x + y * v.y + z * v.z; }
    // Cross product
    inline Vec cross(const Vec &v) const {
        return Vec(y * v.z - z * v.y,
                   z * v.x - x * v.z,
                   x * v.y - y * v.x);
    }
    
    // Magnitude and normalization
    inline double length() const { return std::sqrt(x * x + y * y + z * z); }
    inline Vec normalized() const { return *this / length(); }
};

typedef Vec Color; // We use Vec to represent RGB colors.

// ----------------------
// Ray Structure
// ----------------------
struct Ray {
    Vec origin;
    Vec direction;
    Ray(const Vec &o, const Vec &d);
};

// ----------------------
// Sphere Structure
// ----------------------
struct Sphere {
    Vec center;
    double radius;
    Sphere(const Vec &c, double r);
    
    // Returns true if ray r hits the sphere between t_min and t_max.
    // If so, sets t_hit to the hit distance, and computes the hit point and normal.
    bool hit(const Ray &r, double t_min, double t_max, double &t_hit, Vec &hit_point, Vec &normal) const;
};

// ----------------------
// Scene Structure
// ----------------------
struct Scene {
    std::vector<Sphere> spheres;
    void add(const Sphere &s);
    
    // Check if the ray hits any sphere in the scene.
    bool hit(const Ray &r, double t_min, double t_max, double &closest_t, Vec &hit_point, Vec &normal) const;
};

// ----------------------
// Utility Functions
// ----------------------

// Generates a random point in the unit sphere.
Vec random_in_unit_sphere();

// Computes a background color as a gradient from white to blue.
Color background(const Ray &r);

// Computes the color for a ray by recursively scattering the ray.
// 'depth' limits the number of bounces.
Color ray_color(const Ray &r, const Scene &scene, int depth);

// Converts a double in [0,1] to an integer color value in [0,255].
int to_int(double x);

#endif // RAYTRACER_H
