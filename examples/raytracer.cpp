/* File: raytracer.cpp */

#include "include/raytracer.h"

Ray::Ray(const Vec &o, const Vec &d) : origin(o), direction(d.normalized()) {}

Sphere::Sphere(const Vec &c, double r) : center(c), radius(r) {}

// Returns true if ray r hits the sphere between t_min and t_max.
// If so, sets t_hit to the hit distance, and computes the hit point and normal.
bool Sphere::hit(const Ray &r, double t_min, double t_max, double &t_hit, Vec &hit_point, Vec &normal) const {
    Vec oc = r.origin - center;
    double a = r.direction.dot(r.direction);
    double b = oc.dot(r.direction);
    double c = oc.dot(oc) - radius * radius;
    double discriminant = b * b - a * c;
    if (discriminant > 0) {
        double temp = (-b - std::sqrt(discriminant)) / a;
        if (temp < t_max && temp > t_min) {
            t_hit = temp;
            hit_point = r.origin + r.direction * t_hit;
            normal = (hit_point - center) / radius;
            return true;
        }
        temp = (-b + std::sqrt(discriminant)) / a;
        if (temp < t_max && temp > t_min) {
            t_hit = temp;
            hit_point = r.origin + r.direction * t_hit;
            normal = (hit_point - center) / radius;
            return true;
        }
    }
    return false;
}

void Scene::add(const Sphere &s) { spheres.push_back(s); }
    
// Check if the ray hits any sphere in the scene.
bool Scene::hit(const Ray &r, double t_min, double t_max, double &closest_t, Vec &hit_point, Vec &normal) const {
    bool hit_anything = false;
    closest_t = t_max;
    double t;
    Vec temp_hit, temp_normal;
    for (const auto &s : spheres) {
        if (s.hit(r, t_min, closest_t, t, temp_hit, temp_normal)) {
            hit_anything = true;
            closest_t = t;
            hit_point = temp_hit;
            normal = temp_normal;
        }
    }
    return hit_anything;
}

// Utility function: generate a random point in the unit sphere
Vec random_in_unit_sphere() {
    while (true) {
        double x = 2.0 * rand() / double(RAND_MAX) - 1.0;
        double y = 2.0 * rand() / double(RAND_MAX) - 1.0;
        double z = 2.0 * rand() / double(RAND_MAX) - 1.0;
        Vec p(x, y, z);
        if (p.dot(p) < 1.0) return p;
    }
}

// Background color: a simple gradient from white to blue.
Color background(const Ray &r) {
    Vec unit_dir = r.direction.normalized();
    double t = 0.5 * (unit_dir.y + 1.0);
    return Color(1.0, 1.0, 1.0) * (1.0 - t) + Color(0.5, 0.7, 1.0) * t;
}

// Recursive function to compute the color for a ray.
Color ray_color(const Ray &r, const Scene &scene, int depth) {
    if (depth <= 0)
        return Color(0, 0, 0);
    
    double t_hit;
    Vec hit_point, normal;
    if (scene.hit(r, 0.001, std::numeric_limits<double>::max(), t_hit, hit_point, normal)) {
        // For a simple diffuse material, scatter the ray randomly in the hemisphere.
        Vec target = hit_point + normal + random_in_unit_sphere();
        return ray_color(Ray(hit_point, target - hit_point), scene, depth - 1) * 0.5;
    } else {
        return background(r);
    }
}

int to_int(double x) { return static_cast<int>(255.99 * x); }
