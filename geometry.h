
#ifndef BIDIRECTINALPATHTRACER_GEOMETRY_H
#define BIDIRECTINALPATHTRACER_GEOMETRY_H
#include <glm/vec3.hpp>

#include "material.h"
#include "ray.h"
using namespace glm;

struct HitRecord {
    float t;               // distance along ray
    vec3 point;       // hit point in space
    vec3 normal;      // surface normal at hit
    Material material;     // material at hit
};

class Geometry {
public:
    virtual ~Geometry() = default;

    // Pure virtual: must be implemented by all derived classes
    virtual bool intersect(const Ray& ray, HitRecord& rec) const = 0;
};
#endif //BIDIRECTINALPATHTRACER_GEOMETRY_H