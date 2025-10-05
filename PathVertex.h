//
// Created by lunas on 2025-10-04.
//

#ifndef PATH_VERTEX_H
#define PATH_VERTEX_H

#include <glm/glm.hpp>

#include "geometry.h"
#include "material.h"
using namespace glm;
struct PathVertex {
    vec3 point;
    vec3 normal;
    vec3 wi;  // Incident direction (from previous vertex)
    vec3 wo;  // Outgoing direction (to next vertex)
    const Material material;
    vec3 throughput;  // Cumulative weight up to this vertex
    float pdf;        // PDF of the sampling strategy that generated this vertex
    bool isLight;     // Whether this vertex is on a light source

    PathVertex(const HitRecord& hit, const vec3& incomingDir, const vec3& weight, float pdf_ = 1.0f)
        : point(hit.point), normal(hit.normal), wi(incomingDir), material(hit.material),
          throughput(weight), pdf(pdf_), isLight(hit.material.isEmissive()) {}

    // Constructor for light vertices
    PathVertex(const vec3& p, const vec3& n, const Material mat, const vec3& weight)
        : point(p), normal(n), material(mat), throughput(weight), isLight(true) {}
};

#endif //PATH_VERTEX_H