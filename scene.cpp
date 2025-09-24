
#include "scene.h"
#include <limits>

bool Scene::intersect(const Ray& ray, vec3& hitPoint, vec3& normal, Material& material) const
{
    float closestT = std::numeric_limits<float>::max();
    bool hitAnything = false;

    for (const auto& obj : objects) {
        float tHit; // distance from ray origin to intersection point
        if (obj.intersect(ray, tHit)) { // if there is intersection
            if (tHit < closestT) { // if this intersection is the closest one so far
                closestT = tHit;
                hitPoint = ray.origin + ray.direction * tHit; // calculate hit point
                normal = normalize(hitPoint - obj.center); // assuming all shapes are spheres with center
                material = obj.material;
                hitAnything = true;
            }
        }
    }
    return hitAnything;
}