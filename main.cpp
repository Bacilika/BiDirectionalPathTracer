
#include "camera.h"
#include "scene.h"
#include <glm/glm.hpp>

#include "image.h"

int main() {
    Camera cam;

    Scene scene;
    scene.objects.push_back(Sphere{10,vec3(0,0,0)});
    scene.lights.push_back(Light{vec3(10,10,10),vec3(1,1,1)});

    int width = 512, height = 512;
    Image image(width, height);

    for(int j = 0; j < height; j++) {
        for(int i = 0; i < width; i++) {
            float u = (static_cast<float>(i) + 0.5f) / static_cast<float>(width);
            float v = (static_cast<float>(j) + 0.5f) / static_cast<float>(height);
            Ray ray = cam.getRay(u, v);

            vec3 hitColor(0,1,0);
            image.pixels[j*width + i] = hitColor;
        }
    }

    image.savePPM("output.ppm");
    return 0;
}
