
#include "camera.h"


#include <glm/gtc/matrix_transform.hpp>

Ray Camera::getRay(const float u, const float v) const {
    vec3 forward = normalize(lookAt - eye);
    vec3 right   = normalize(cross(forward, up));
    vec3 camUp   = cross(right, forward);

    float tanFov = tan(radians(fov) / 2.0f);
    float px = (2.0f * u - 1.0f) * tanFov * aspectRatio;
    float py = (1.0f - 2.0f * v) * tanFov; // flip Y

    vec3 rayDir = normalize(px * right + py * camUp + forward);
    return Ray(eye, rayDir);
}