
#ifndef BIDIRECTINALPATHTRACER_GEOMETRY_H
#define BIDIRECTINALPATHTRACER_GEOMETRY_H
#include <glm/vec3.hpp>

#include "material.h"
#include "ray.h"

using namespace glm;
struct Sphere {
    float radius;
    vec3 center;
    Material material;
    bool intersect(const Ray& ray,float& tHit) const;
};


#endif //BIDIRECTINALPATHTRACER_GEOMETRY_H