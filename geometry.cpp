

#include "geometry.h"


bool Sphere::intersect(const Ray& ray, float& tHit) const {
    const vec3 oc = ray.origin - center; // vector from ray origin to sphere center
    const float a = dot(ray.direction, ray.direction); // should be 1 if direction is normalized otherwise, its the quadratic coefficient
    const float b = 2.0f * dot(oc, ray.direction); // linear coefficient
    const float c = dot(oc, oc) - radius*radius; // constant coefficient
    const float discriminant = b*b - 4*a*c;

    if (discriminant < 0) return false; // no intersection
    // if discriminant == 0 ray grazes the sphere

    const float sqrtDisc = sqrt(discriminant); // two solutions to the quadratic equation
    const float t0 = (-b - sqrtDisc) / (2*a);
    const float t1 = (-b + sqrtDisc) / (2*a);

    tHit = t0 > 0 ? t0 : t1; // we want the closest positive t value
    return tHit > 0; // return true if tHit is positive (i.e., intersection is in front of the ray origin)
}