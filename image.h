
#ifndef BIDIRECTINALPATHTRACER_IMAGE_H
#define BIDIRECTINALPATHTRACER_IMAGE_H
#include <string>
#include <vector>
#include <glm/vec3.hpp>


struct Image {
    int width;
    int height;
    std::vector<glm::vec3> pixels;
    Image(int w, int h) : width(w), height(h), pixels(w*h, glm::vec3(0)) {}
    void savePPM(const std::string& filename) const;
};


#endif //BIDIRECTINALPATHTRACER_IMAGE_H