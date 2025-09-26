
#include "scene.h"
#include <limits>

bool Scene::intersect(const Ray& ray, HitRecord& closestHit) const
{
    bool hitAnything = false;
    float closestSoFar = std::numeric_limits<float>::max();
    HitRecord tempRec{};

    for (const auto& obj : objects) {
        if (obj->intersect(ray, tempRec) && tempRec.t < closestSoFar) {
            closestSoFar = tempRec.t;
            closestHit = tempRec;
            hitAnything = true;
        }
    }
    return hitAnything;
}