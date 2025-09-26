
#ifndef BIDIRECTINALPATHTRACER_MATERIAL_H
#define BIDIRECTINALPATHTRACER_MATERIAL_H
#include <glm/vec3.hpp>

using namespace glm;
struct Material {
    float diffuse;
    float specular;
    float shininess;
    vec3 color;
};


#endif //BIDIRECTINALPATHTRACER_MATERIAL_H