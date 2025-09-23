
#include "scene.h"
#include <limits>

bool Scene::intersect(const Ray& ray, vec3& hitPoint, vec3& normal, Material& material) const
{
    float closestT = std::numeric_limits<float>::max();
    bool hitAnything = false;

    for (const auto& obj : objects) {
        float tHit;
        if (obj.intersect(ray, tHit)) {
            if (tHit < closestT) {
                closestT = tHit;
                hitPoint = ray.origin + ray.direction * tHit;
                normal = normalize(hitPoint - obj.center);
                material = obj.material;
                hitAnything = true;
            }
        }
    }
    return hitAnything;
}