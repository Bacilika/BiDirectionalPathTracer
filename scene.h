
#ifndef BIDIRECTINALPATHTRACER_SCENE_H
#define BIDIRECTINALPATHTRACER_SCENE_H
#include <vector>

#include "geometry.h"
#include "light.h"
#include "material.h"


struct Scene {
    std::vector<Sphere> objects;
    std::vector<Light> lights;

    bool intersect(const Ray& ray, vec3& hitPoint, vec3& normal, Material& material) const;
};


#endif //BIDIRECTINALPATHTRACER_SCENE_H