
#ifndef BIDIRECTINALPATHTRACER_MATERIAL_H
#define BIDIRECTINALPATHTRACER_MATERIAL_H
#include <glm/vec3.hpp>

using namespace glm;
struct Material {
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    float shininess;
};


#endif //BIDIRECTINALPATHTRACER_MATERIAL_H