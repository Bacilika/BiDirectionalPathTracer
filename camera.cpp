
#include "camera.h"


#include <glm/gtc/matrix_transform.hpp>

Ray Camera::getRay(const float u, const float v) const {
    const vec3 forward = normalize(lookAt - pos);
    const vec3 right   = normalize(cross(forward, up));
    const vec3 camUp   = cross(right, forward);

    const float tanFov = tan(radians(fov) / 2.0f);
    const float px = (2.0f * u - 1.0f) * tanFov * aspectRatio; //2.0f * u - 1.0f to map it [-1,1]
    const float py = (1.0f - 2.0f * v) * tanFov; // map [0,1] to [1,-1] so it doesnt appear flipped

    const vec3 rayDir = normalize(px * right + py * camUp + forward);
    return Ray(pos, rayDir);
}