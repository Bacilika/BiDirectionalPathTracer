

#include "image.h"
#include <fstream>
#include <algorithm>

void Image::savePPM(const std::string& filename) const {
    std::ofstream ofs(filename, std::ios::out | std::ios::binary);
    ofs << "P6\n" << width << " " << height << "\n255\n";
    for (auto& c : pixels) {
        unsigned char r = (unsigned char)(std::clamp(c.r, 0.0f, 1.0f) * 255);
        unsigned char g = (unsigned char)(std::clamp(c.g, 0.0f, 1.0f) * 255);
        unsigned char b = (unsigned char)(std::clamp(c.b, 0.0f, 1.0f) * 255);
        ofs << r << g << b;
    }
    ofs.close();
}