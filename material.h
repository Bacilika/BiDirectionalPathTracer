#ifndef BIDIRECTINALPATHTRACER_MATERIAL_H
#define BIDIRECTINALPATHTRACER_MATERIAL_H
#include <glm/vec3.hpp>

using namespace glm;
struct Material {
    float diffuse;
    float specular;
    float shininess;
    vec3 color;

    // Emission properties
    vec3 emissionColor;     // Color of emitted light
    float emissionStrength; // Brightness multiplier

    // Constructor for convenience
    Material(float d, float s, float sh, const vec3& c,
             const vec3& eColor = vec3(0.0f), float eStrength = 0.0f)
        : diffuse(d), specular(s), shininess(sh), color(c),
          emissionColor(eColor), emissionStrength(eStrength) {}

    // Check if material is emissive
    bool isEmissive() const {
        return emissionStrength > 0.0f;
    }
};

#endif //BIDIRECTINALPATHTRACER_MATERIAL_H