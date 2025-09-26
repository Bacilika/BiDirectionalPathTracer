
#include "camera.h"
#include "scene.h"
#include <glm/glm.hpp>

#include "image.h"

vec3 calculateLight(vec3 normal, Light light, vec3 surface) {
    // Diffuse
    float diffuse = dot(normalize(normal), normalize(light.position));
    diffuse = max(0.0f, diffuse); // No negative light

    // Specular
    const vec3 r = reflect(-1.f*light.position, normalize(normal));
    const vec3 v = normalize(-1.f*surface); // View direction
    float specular = dot(r, v);
    if (specular > 0.0)
        specular = 1.0f * pow(specular, 150.0);
    specular = max(specular, 0.0f);
    const float shade = 1 * diffuse + 0 * specular;
    return {shade, shade, shade};


}

int main() {
    Camera cam;

    Scene scene;
    auto sphere = Sphere{1,vec3(0,0,0)};
    scene.objects.push_back(sphere);
    scene.lights.push_back(Light{vec3(10,10,10),vec3(1,1,1)});

    int width = 512, height = 512;
    Image image(width, height);
    vec3 hitColor;
    for(int j = 0; j < height; j++) {
        for(int i = 0; i < width; i++) {
            // to get pixel coordinate in [0,1] range
            float u = (static_cast<float>(i) + 0.5f) / static_cast<float>(width);
            float v = (static_cast<float>(j) + 0.5f) / static_cast<float>(height);
            Ray ray = cam.getRay(u, v);
            vec3 hitPoint, normal;
            Material material;
            if (scene.intersect(ray,hitPoint,normal,material)) {
                vec3 light = calculateLight(normal, scene.lights[0], hitPoint);
                hitColor = vec3(0,0.6,1)* light;
            }
            else {
                hitColor = vec3(0.8,0.3,0.7);
            }

            image.pixels[j*width + i] = hitColor ;
        }
    }

    image.savePPM("output.ppm");
    return 0;
}
