
#include <iostream>
#include <ostream>

#include "camera.h"
#include "scene.h"
#include "geometry.cpp"
#include <glm/glm.hpp>

#include "image.h"

vec3 calculateLight(Camera camera, Light light, HitRecord hitRecord) {
    // Diffuse
    vec3 lightDir = normalize(light.position - hitRecord.point);
    const vec3 reflectDir  = normalize(reflect(-lightDir, hitRecord.normal));
    const vec3 viewDir = normalize(camera.pos - hitRecord.point); // View direction

    float diffuse = dot(hitRecord.normal, lightDir);
    diffuse = max(0.0f, diffuse); // No negative light

    float specular = pow(max(dot(viewDir, reflectDir), 0.0f), hitRecord.material.shininess);
    specular = hitRecord.material.specular * specular;
    diffuse = hitRecord.material.diffuse * diffuse;
    return light.intensity * (diffuse + specular);
}

int main() {
    Camera cam;

    Scene scene;

    //sphere 2
    auto material2 = Material{1, 1.1, 100, vec3(0,0.6,1)};
    auto sphere2 = std::make_shared<Sphere>(vec3(-1, 0, 0),1,material2);

    float wallDiffuse = 1.6f;
    float wallSpecular = 0.0f;
    float wallShininess = 100.0f;
    // Materials
    Material floorMat{ wallDiffuse,wallSpecular,wallShininess,vec3(0.5f, 0.5f, 0.7f) }; //blue-gray
    Material roofMat{ wallDiffuse,wallSpecular,wallShininess,vec3(0.8f, 0.8f, 0.8f) }; //gray
    Material leftMat{ wallDiffuse,wallSpecular,wallShininess,vec3(0.8f, 0.2f, 0.2f) }; //red
    Material rightMat{ wallDiffuse,wallSpecular,wallShininess,vec3(0.2f, 0.8f, 0.2f) }; //green
    Material backMat{ wallDiffuse,wallSpecular,wallShininess,vec3(0.6f, 0.2f, 0.4f) }; //pink


    float depth = -3.0f;
    float closest = 1.5f;
    float left = -2.0f;
    float right = 2.0f;
    float down = -2.0f;
    float up = 2.0f;
    float middlex = (left + right) / 2.0f;
    float middley = (down + up) / 2.0f;
    float middlez = (depth + closest) / 2.0f;

    scene.objects.push_back(std::make_shared<XZRectangle>(
        left, right, depth, closest, down, floorMat)); //blue-gray

    scene.objects.push_back(std::make_shared<XZRectangle>(
        left, right, depth, closest, up, roofMat)); //gray

    scene.objects.push_back(std::make_shared<YZRectangle>(
        down, up, depth, closest, left, leftMat)); //red

    scene.objects.push_back(std::make_shared<YZRectangle>(
        down, up, depth, closest, right, rightMat)); //green

    scene.objects.push_back(std::make_shared<XYRectangle>(
        left, right, down, up, depth, backMat)); //pink

    scene.lights.push_back(Light{vec3(right-0.3,up-0.3,depth+1),vec3(1,1,1)});


    //sphere 1
    auto material1 = Material{1, 1.1, 100, vec3(0,0.6,1)};
    auto sphere1 = std::make_shared<Sphere>(vec3(middlex, middley, middlez-2),0.6,material1);
    scene.objects.push_back(sphere1);

    int width = 800, height = 800;
    Image image(width, height);
    vec3 hitColor;
    for(int j = 0; j < height; j++) {
        for(int i = 0; i < width; i++) {
            // to get pixel coordinate in [0,1] range
            float u = (static_cast<float>(i) + 0.5f) / static_cast<float>(width);
            float v = (static_cast<float>(j) + 0.5f) / static_cast<float>(height);

            // generate eye path
            Ray ray = cam.getRay(u, v);
            if (HitRecord hitRecord{}; scene.intersect(ray, hitRecord)) {

                vec3 light = calculateLight(cam, scene.lights[0], hitRecord);
                hitColor = hitRecord.material.color* light;
            }
            else {
                hitColor = vec3(0.3,0.1,0.2);
            }

            //generate light path

            image.pixels[j*width + i] = hitColor;

        }
    }

    image.savePPM("output.ppm");
    return 0;
}
