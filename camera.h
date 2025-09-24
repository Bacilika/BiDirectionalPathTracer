

#ifndef BIDIRECTINALPATHTRACER_CAMERA_H
#define BIDIRECTINALPATHTRACER_CAMERA_H

#include <glm/glm.hpp>

#include "ray.h"
using namespace glm;
struct Camera {
    vec3 pos  = vec3(0.0f, 1.0f, 5.0f);
    vec3 lookAt = vec3(0.0f, 0.0f, 0.0f);
    vec3 up = vec3(0.0f, 1.0f, 0.0f);
    float fov = 45.f;
    int imageWidth = 800;
    int imageHeight = 800;
    float aspectRatio = static_cast<float>(imageWidth) / static_cast<float>(imageHeight);
    Ray getRay(float u, float v) const;
};


#endif //BIDIRECTINALPATHTRACER_CAMERA_H