
#include "random.h"

// Initialize static members
std::mt19937 Random::generator(std::random_device{}());
std::uniform_real_distribution<float> Random::distribution(0.0f, 1.0f);