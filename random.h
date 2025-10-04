#ifndef BIDIRECTIONALPATHTRACER_RANDOM_H
#define BIDIRECTIONALPATHTRACER_RANDOM_H

#include <random>
#include <glm/glm.hpp>
using namespace glm;
class Random {
private:
    static std::mt19937 generator;
    static std::uniform_real_distribution<float> distribution;

public:
    static float nextFloat() {
        return distribution(generator);
    }
    
    static vec3 randomInUnitSphere() {
        while (true) {
            vec3 p(2.0f * nextFloat() - 1.0f, 
                   2.0f * nextFloat() - 1.0f, 
                   2.0f * nextFloat() - 1.0f);
            if (dot(p, p) < 1.0f) return p;
        }
    }
    
    static vec3 randomInHemisphere(const vec3& normal) {
        vec3 inSphere = randomInUnitSphere();
        if (dot(inSphere, normal) > 0.0f) {
            return inSphere;
        } else {
            return -inSphere;
        }
    }
    
    static vec3 randomCosineDirection() {
        float r1 = nextFloat();
        float r2 = nextFloat();
        float z = sqrt(1 - r2);
        float phi = 2 * M_PI * r1;
        float x = cos(phi) * sqrt(r2);
        float y = sin(phi) * sqrt(r2);
        return vec3(x, y, z);
    }
};



#endif