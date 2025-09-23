

#include "image.h"
#include <fstream>
#include <algorithm>

void Image::savePPM(const std::string& filename) const {
    std::ofstream ofs(filename, std::ios::out | std::ios::binary);
    ofs << "P6\n" << width << " " << height << "\n255\n";
    for (auto& c : pixels) {
        const unsigned char r = static_cast<unsigned char>(std::clamp(c.r, 0.0f, 1.0f) * 255);
        const unsigned char g = static_cast<unsigned char>(std::clamp(c.g, 0.0f, 1.0f) * 255);
        const unsigned char b = static_cast<unsigned char>(std::clamp(c.b, 0.0f, 1.0f) * 255);
        ofs << r << g << b;
    }
    ofs.close();
}