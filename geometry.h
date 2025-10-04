
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
    HitRecord() : t(0.0f), point(0.0f), normal(0.0f, 1.0f, 0.0f),
                  material(1.0f, 0.0f, 0.0f, vec3(1.0f)) {}
};

class Geometry {
public:
    Geometry() = default;
    virtual ~Geometry() = default;

    // Pure virtual: must be implemented by all derived classes
    virtual bool intersect(const Ray& ray, HitRecord& rec) const = 0;
    virtual const Material& getMaterial() const = 0;
};
#endif //BIDIRECTINALPATHTRACER_GEOMETRY_H