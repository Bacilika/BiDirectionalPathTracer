
#ifndef BIDIRECTINALPATHTRACER_SCENE_H
#define BIDIRECTINALPATHTRACER_SCENE_H
#include <memory>
#include <vector>

#include "geometry.h"
#include "light.h"
#include "material.h"


struct Scene {
    std::vector<std::shared_ptr<Geometry>> objects;
    std::vector<Light> lights;

    bool intersect(const Ray& ray,HitRecord& closestHit) const;
};


#endif //BIDIRECTINALPATHTRACER_SCENE_H