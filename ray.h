

#ifndef BIDIRECTINALPATHTRACER_RAY_H
#define BIDIRECTINALPATHTRACER_RAY_H
#include <glm/glm.hpp>
using namespace glm;

struct Ray {
    vec3 origin;
    vec3 direction;
    Ray(const vec3& o, const vec3& d) : origin(o), direction(normalize(d)) {}
};

#endif //BIDIRECTINALPATHTRACER_RAY_H