

#include "geometry.h"


bool Sphere::intersect(const Ray& ray, float& tHit) const {
    vec3 oc = ray.origin - center;
    float a = dot(ray.direction, ray.direction);
    float b = 2.0f * dot(oc, ray.direction);
    float c = dot(oc, oc) - radius*radius;
    float discriminant = b*b - 4*a*c;

    if (discriminant < 0) return false;

    float sqrtDisc = sqrt(discriminant);
    float t0 = (-b - sqrtDisc) / (2*a);
    float t1 = (-b + sqrtDisc) / (2*a);

    tHit = (t0 > 0) ? t0 : t1;
    return tHit > 0;
}