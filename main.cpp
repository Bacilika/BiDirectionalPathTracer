#include <iostream>
#include <ostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>
#include "camera.h"
#include "scene.h"
#include "geometry.cpp"
#include "random.h"
#include <glm/glm.hpp>
#include "image.h"
#include "PathVertex.h"

const int MAX_DEPTH = 5;
const float RR_START = 3; // Start Russian Roulette after this depth

/**
 * Estimates the surface area of a geometry object for light sampling.
 * This is used to compute the probability density function (PDF) when sampling light sources.
 *
 * @param obj Shared pointer to the geometry object
 * @return Surface area of the object in world units, or 1.0 as default for unknown types
 */
float estimateObjectArea(const std::shared_ptr<Geometry>& obj) {
    // spheres
    if (const auto sphere = std::dynamic_pointer_cast<Sphere>(obj)) {
        return 4.0f * static_cast<float>(M_PI) * sphere->radius * sphere->radius;
    }
    // rectangles
    if (const auto rect = std::dynamic_pointer_cast<XYRectangle>(obj)) {
        return (rect->x1 - rect->x0) * (rect->y1 - rect->y0);
    }
    if (const auto rect = std::dynamic_pointer_cast<XZRectangle>(obj)) {
        return (rect->x1 - rect->x0) * (rect->z1 - rect->z0);
    }
    if (const auto rect = std::dynamic_pointer_cast<YZRectangle>(obj)) {
        return (rect->y1 - rect->y0) * (rect->z1 - rect->z0);
    }

    return 1.0f; // Default area
}

/**
 * Samples a random point uniformly distributed over the surface of a geometry object.
 * Used for area light sampling in direct lighting calculations.
 *
 * @param obj Shared pointer to the geometry object to sample from
 * @param refPoint Reference point (used for fallback, not critical for sampling)
 * @return A random 3D point on the surface of the object
 */
vec3 pickRandomPointOnObject(const std::shared_ptr<Geometry>& obj, const vec3& refPoint) {
    // spheres
    if (const auto sphere = std::dynamic_pointer_cast<Sphere>(obj)) {
        const vec3 localPoint = Random::randomInUnitSphere();
        const vec3 direction = normalize(localPoint);
        return sphere->center + direction * sphere->radius;
    }
    // rectangles
    if (const auto rect = std::dynamic_pointer_cast<XYRectangle>(obj)) {
        float x = rect->x0 + Random::nextFloat() * (rect->x1 - rect->x0);
        float y = rect->y0 + Random::nextFloat() * (rect->y1 - rect->y0);
        return {x, y, rect->k};
    }
    if (const auto rect = std::dynamic_pointer_cast<XZRectangle>(obj)) {
        float x = rect->x0 + Random::nextFloat() * (rect->x1 - rect->x0);
        float z = rect->z0 + Random::nextFloat() * (rect->z1 - rect->z0);
        return {x, rect->k, z};
    }
    if (const auto rect = std::dynamic_pointer_cast<YZRectangle>(obj)) {
        float y = rect->y0 + Random::nextFloat() * (rect->y1 - rect->y0);
        float z = rect->z0 + Random::nextFloat() * (rect->z1 - rect->z0);
        return {rect->k, y, z};
    }

    // Fallback: just return object center or something
    return refPoint + vec3(1.0f, 1.0f, 1.0f);
}
/**
 * Represents a sampled light source with its properties for path generation.
 * Used in bidirectional path tracing to start light subpaths.
 */
struct LightSample {
    PathVertex vertex;
    vec3 direction;
    float pdf;
};

/**
 * Samples a random light source from the scene and a point on its surface.
 * This function implements importance sampling for light sources by selecting
 * lights proportional to their emission characteristics.
 *
 * @param scene The scene containing all objects and lights
 * @return LightSample containing the sampled light vertex, direction, and PDF
 */
LightSample sampleLightSource(const Scene& scene) {
    // Collect lights
    std::vector<std::shared_ptr<Geometry>> lights;
    for (auto& obj : scene.objects) {
        if (obj->getMaterial().isEmissive()) lights.push_back(obj);
    }

    if (lights.empty()) {
        // Return dummy
        return {PathVertex(vec3(0), vec3(0), Material(0,0,0,vec3(0)), vec3(0)), vec3(0), 1.0f};
    }

    // Sample random light
    int lightIndex = static_cast<int>(Random::nextFloat() * lights.size());
    auto& lightObj = lights[lightIndex];
    const Material& lightMat = lightObj->getMaterial();

    // Sample point on light
    vec3 lightPoint = pickRandomPointOnObject(lightObj, vec3(0.0f));

    // Get normal at sampled point
    HitRecord lightHitRec;
    Ray dummyRay(lightPoint, vec3(0,1,0));
    lightObj->intersect(dummyRay, lightHitRec);

    // Sample direction PROPERLY
    vec3 localDir = Random::randomCosineDirection();
    vec3 tangent, bitangent;
    if (fabs(lightHitRec.normal.x) > fabs(lightHitRec.normal.y)) {
        tangent = normalize(vec3(lightHitRec.normal.z, 0, -lightHitRec.normal.x));
    } else {
        tangent = normalize(vec3(0, -lightHitRec.normal.z, lightHitRec.normal.y));
    }
    bitangent = cross(lightHitRec.normal, tangent);
    vec3 lightDir = localDir.x * tangent + localDir.y * bitangent + localDir.z * lightHitRec.normal;

    float area = estimateObjectArea(lightObj);
    float areaPdf = 1.0f / area;
    float lightPdf = 1.0f / static_cast<float>(lights.size());
    float totalPdf = areaPdf * lightPdf;

    // FIX: Throughput should be emission divided by PDF, not multiplied
    vec3 throughput = lightMat.emissionColor * lightMat.emissionStrength / totalPdf;

    PathVertex vertex(lightPoint, lightHitRec.normal, lightMat, throughput);
    return {vertex, lightDir, totalPdf};
}

/**
 * Computes the geometry term G between two surface points.
 * The geometry term converts between solid angle and surface area measures
 * and accounts for foreshortening due to surface orientation.
 *
 * Formula: G(x, y) = (cos?? * cos??) / ||x - y||?
 * where ?? is angle between normal at x and direction to y,
 * and ?? is angle between normal at y and direction to x.
 *
 * @param point1 First surface point
 * @param normal1 Surface normal at point1
 * @param point2 Second surface point
 * @param normal2 Surface normal at point2
 * @return Geometry term value, or 0 if points are co-located or back-facing
 */
float computeGeometryTerm(const vec3& point1, const vec3& normal1, const vec3& point2, const vec3& normal2) {
    const vec3 toPoint2 = point2 - point1;
    const float distance = length(toPoint2);
    if (distance < 1e-5f) return 0.0f;
    const vec3 dir = normalize(toPoint2);

    const float cos1 = max(dot(normal1, dir), 0.0f);
    const float cos2 = max(dot(normal2, -dir), 0.0f);

    if (cos1 < 1e-5f || cos2 < 1e-5f) return 0.0f;

    return (cos1 * cos2) / (distance * distance);
}

/**
 * Samples a new scattering direction based on the material properties at a path vertex.
 * Implements importance sampling by choosing between diffuse (cosine-weighted) and
 * specular (perfect reflection) scattering based on material characteristics.
 *
 * @param vertex The current path vertex containing material and geometric information
 * @param newDirection Output parameter for the sampled scattering direction
 * @param pdf Output parameter for the probability density of the sampled direction
 */
void sampleScattering(const PathVertex& vertex, vec3& newDirection, float& pdf) {
    float diffuseProb = vertex.material.diffuse / (vertex.material.diffuse + vertex.material.specular + 1e-5f);

    if (Random::nextFloat() < diffuseProb) {
        // Diffuse sampling
        vec3 localDir = Random::randomCosineDirection();
        vec3 tangent, bitangent;
        if (fabs(vertex.normal.x) > fabs(vertex.normal.y)) {
            tangent = normalize(vec3(vertex.normal.z, 0, -vertex.normal.x));
        } else {
            tangent = normalize(vec3(0, -vertex.normal.z, vertex.normal.y));
        }
        bitangent = cross(vertex.normal, tangent);
        newDirection = localDir.x * tangent + localDir.y * bitangent + localDir.z * vertex.normal;
        pdf = max(dot(newDirection, vertex.normal), 0.0f) / static_cast<float>(M_PI);
    } else {
        // Pure specular - for glass/mirror materials
        if (vertex.material.specular > 0.9f) {
            // Simple reflection for now - add refraction later for full effect
            newDirection = reflect(-vertex.wi, vertex.normal);
        } else {
            // Glossy reflection for metallic materials
            vec3 perfectReflection = reflect(-vertex.wi, vertex.normal);
            // Add small random perturbation
            vec3 localDir = Random::randomCosineDirection();
            vec3 tangent, bitangent;
            if (fabs(perfectReflection.x) > fabs(perfectReflection.y)) {
                tangent = normalize(vec3(perfectReflection.z, 0, -perfectReflection.x));
            } else {
                tangent = normalize(vec3(0, -perfectReflection.z, perfectReflection.y));
            }
            bitangent = cross(perfectReflection, tangent);

            float roughness = 0.1f;
            vec3 perturbedDir = normalize(perfectReflection + localDir * roughness);
            newDirection = perturbedDir;
        }
        pdf = 1.0f;
    }
}

/**
 * Computes the Bidirectional Reflectance Distribution Function (BRDF) at a path vertex.
 * The BRDF describes how light is reflected at a surface point given incoming and outgoing directions.
 * This implementation combines Lambertian diffuse and Phong-like specular components.
 *
 * @param vertex The path vertex containing material properties and geometry
 * @param outgoingDir The direction in which light is leaving the surface
 * @return The BRDF value representing the ratio of reflected radiance to incident irradiance
 */
vec3 computeBRDF(const PathVertex& vertex, const vec3& outgoingDir) {
    if (vertex.isLight) {
        // CRITICAL FIX: Light sources should NOT contribute BRDF in connections
        // They only emit light, not reflect it
        return vec3(0.0f);
    }

    vec3 brdf(0.0f);

    // Diffuse component
    if (vertex.material.diffuse > 0.0f) {
        brdf += vertex.material.color * vertex.material.diffuse / static_cast<float>(M_PI);
    }

    // Specular component
    if (vertex.material.specular > 0.0f) {
        vec3 perfectReflection = reflect(-vertex.wi, vertex.normal);
        float specularFactor = pow(max(dot(perfectReflection, outgoingDir), 0.0f),
                                 vertex.material.shininess);
        brdf += vec3(1.0f) * vertex.material.specular * specularFactor;
    }

    return brdf;
}

/**
 * Computes the Multiple Importance Sampling (MIS) weight for a path connection.
 * This balances between different sampling strategies to reduce variance.
 * Based on the balance heuristic from Veach's thesis, as used in the paper.
 *
 * @param eyePath The complete eye subpath
 * @param lightPath The complete light subpath
 * @param eyeIndex Index of eye vertex in connection
 * @param lightIndex Index of light vertex in connection
 * @return MIS weight between 0 and 1
 */
float computeMISWeight(const std::vector<PathVertex>& eyePath,
                      const std::vector<PathVertex>& lightPath,
                      int eyeIndex, int lightIndex) {
    // For now, use simple uniform weights
    // In advanced implementation, you'd compute PDF ratios here
    return 0.5f;
}


/**
 * Generates an eye subpath by tracing rays from the camera through the scene.
 * This creates a sequence of path vertices representing light transport from the camera.
 * Each vertex stores the accumulated throughput and is used in bidirectional connections.
 *
 * @param initialRay The initial ray from the camera
 * @param scene The scene containing all geometry
 * @param maxDepth Maximum number of bounces for the path
 * @return Vector of PathVertex objects representing the eye subpath
 */
std::vector<PathVertex> generateEyeSubpath(const Ray& initialRay, const Scene& scene, int maxDepth) {
    std::vector<PathVertex> path;
    Ray ray = initialRay;
    vec3 throughput(1.0f);

    for (int depth = 0; depth < maxDepth; depth++) {
        HitRecord hitRec;
        if (!scene.intersect(ray, hitRec)) break;

        PathVertex vertex(hitRec, -ray.direction, throughput);
        path.push_back(vertex);

        if (static_cast<float>(depth) >= RR_START) {
            float continueProb = std::max(vertex.material.diffuse, vertex.material.specular);
            continueProb = std::min(0.95f, continueProb);
            if (Random::nextFloat() > continueProb) break;
        }

        // Sample next direction (your existing scattering code)
        vec3 newDir;
        float pdf;
        sampleScattering(vertex, newDir, pdf);

        // Update throughput and ray
        vec3 brdf = computeBRDF(vertex, newDir);
        float cosTheta = max(dot(vertex.normal, newDir), 0.0f);
        throughput = throughput * brdf * cosTheta / pdf;
        ray = Ray(vertex.point + vertex.normal * 0.001f, newDir);
    }

    return path;
}

/**
 * Generates a light subpath by tracing rays from a light source through the scene.
 * This creates a sequence of path vertices representing light transport from emissive surfaces.
 * Used in bidirectional path tracing to connect with eye subpaths.
 *
 * @param scene The scene containing all geometry and lights
 * @param maxDepth Maximum number of bounces for the path
 * @return Vector of PathVertex objects representing the light subpath
 */
std::vector<PathVertex> generateLightSubpath(const Scene& scene, int maxDepth) {
    std::vector<PathVertex> path;

    // Sample light source
    auto lightSample = sampleLightSource(scene);
    auto lightVertex = lightSample.vertex;



    // The light vertex itself is the first vertex in the path
    path.push_back(lightVertex);

    // If we want to continue the path from the light, use the sampled direction
    if (maxDepth > 1) {
        Ray ray(lightVertex.point + lightVertex.normal * 0.001f, lightSample.direction);
        vec3 throughput = lightVertex.throughput;

        for (int depth = 1; depth < maxDepth; depth++) {
            HitRecord hitRec;
            if (!scene.intersect(ray, hitRec)) break;

            PathVertex vertex(hitRec, -ray.direction, throughput);
            path.push_back(vertex);

            // Stop if we hit an emissive surface (another light)
            if (vertex.isLight) break;

            // Russian Roulette termination
            if (depth >= RR_START) {
                float continueProb = std::max(vertex.material.diffuse, vertex.material.specular);
                continueProb = std::min(0.95f, continueProb);
                if (Random::nextFloat() > continueProb) break;
            }

            // Sample next direction
            vec3 newDir;
            float pdf;
            sampleScattering(vertex, newDir, pdf);

            if (pdf < 1e-5f) break;

            // Update throughput and ray
            vec3 brdf = computeBRDF(vertex, newDir);
            float cosTheta = max(dot(vertex.normal, newDir), 0.0f);
            throughput = throughput * brdf * cosTheta / pdf;
            ray = Ray(vertex.point + vertex.normal * 0.001f, newDir);
        }
    }

    return path;
}

/**
 * Estimate direct illumination using next event estimation (Strategy s=1)
 * This corresponds to sampling light sources directly
 */
vec3 estimateDirectIllumination(const PathVertex& eyeVertex, const Scene& scene) {
    if (eyeVertex.isLight) return vec3(0.0f); // Already at light

    vec3 L_direct(0.0f);

    // Sample all light sources
    std::vector<std::shared_ptr<Geometry>> lights;
    for (auto& obj : scene.objects) {
        if (obj->getMaterial().isEmissive()) lights.push_back(obj);
    }

    for (auto& light : lights) {
        // Sample point on light source
        vec3 lightPoint = pickRandomPointOnObject(light, eyeVertex.point);
        HitRecord lightHitRec;
        Ray dummyRay(lightPoint, vec3(0,1,0));
        light->intersect(dummyRay, lightHitRec);

        vec3 toLight = lightPoint - eyeVertex.point;
        float distance = length(toLight);
        if (distance < 1e-5f) continue;

        vec3 wi = normalize(toLight);

        // Visibility test
        Ray shadowRay(eyeVertex.point + eyeVertex.normal * 0.001f, wi);
        HitRecord shadowRec;
        if (scene.intersect(shadowRay, shadowRec) && shadowRec.t < distance - 1e-4f) {
            continue;
        }

        // Geometry term
        float cosTheta = max(dot(eyeVertex.normal, wi), 0.0f);
        float cosLight = max(dot(lightHitRec.normal, -wi), 0.0f);
        if (cosTheta < 1e-5f || cosLight < 1e-5f) continue;

        float G = (cosTheta * cosLight) / (distance * distance);

        // BRDF evaluation
        vec3 brdf = computeBRDF(eyeVertex, wi);
        vec3 lightEmission = light->getMaterial().emissionColor * light->getMaterial().emissionStrength;

        // Area measure PDF
        float lightArea = estimateObjectArea(light);
        float areaPdf = 1.0f / lightArea;
        float lightPdf = 1.0f / lights.size();
        float totalPdf = areaPdf * lightPdf;

        L_direct += brdf * lightEmission * G / totalPdf;
    }

    return L_direct / static_cast<float>(lights.size());
}

/**
 * Connect an eye subpath vertex to light sources (for s>1 paths)
 */
vec3 connectToLightSources(const PathVertex& eyeVertex, const Scene& scene) {
    // Similar to estimateDirectIllumination but using the path throughput
    vec3 L(0.0f);

    std::vector<std::shared_ptr<Geometry>> lights;
    for (auto& obj : scene.objects) {
        if (obj->getMaterial().isEmissive()) lights.push_back(obj);
    }

    for (auto& light : lights) {
        vec3 lightPoint = pickRandomPointOnObject(light, eyeVertex.point);
        HitRecord lightHitRec;
        Ray dummyRay(lightPoint, vec3(0,1,0));
        light->intersect(dummyRay, lightHitRec);

        vec3 toLight = lightPoint - eyeVertex.point;
        float distance = length(toLight);
        if (distance < 1e-5f) continue;

        vec3 wi = normalize(toLight);

        // Visibility test
        Ray shadowRay(eyeVertex.point + eyeVertex.normal * 0.001f, wi);
        HitRecord shadowRec;
        if (scene.intersect(shadowRay, shadowRec) && shadowRec.t < distance - 1e-4f) {
            continue;
        }

        // Geometry term
        float cosTheta = max(dot(eyeVertex.normal, wi), 0.0f);
        float cosLight = max(dot(lightHitRec.normal, -wi), 0.0f);
        if (cosTheta < 1e-5f || cosLight < 1e-5f) continue;

        float G = (cosTheta * cosLight) / (distance * distance);

        vec3 brdf = computeBRDF(eyeVertex, wi);
        vec3 lightEmission = light->getMaterial().emissionColor * light->getMaterial().emissionStrength;

        float lightArea = estimateObjectArea(light);
        float areaPdf = 1.0f / lightArea;
        float lightPdf = 1.0f / lights.size();
        float totalPdf = areaPdf * lightPdf;

        L += eyeVertex.throughput * brdf * lightEmission * G / totalPdf;
    }

    return L / static_cast<float>(lights.size());
}
/**
 * Balance heuristic for Multiple Importance Sampling
 * Implements the power heuristic from Veach's thesis as referenced in the paper
 */
float balanceHeuristic(const std::vector<PathVertex>& eyePath, int n,
                      const std::vector<PathVertex>& lightPath, int m) {
    // Simple balance heuristic: 1 / number of strategies
    // In advanced implementation, you'd compute actual PDF ratios
    int numStrategies = n + m + 1;
    return 1.0f / numStrategies;
}
vec3 connectVertices(const PathVertex& eyeVertex, const PathVertex& lightVertex, const Scene& scene) {
    // Skip if both are light sources
    if (eyeVertex.isLight && lightVertex.isLight) return vec3(0.0f);

    vec3 toLight = lightVertex.point - eyeVertex.point;
    float distance = length(toLight);
    if (distance < 1e-5f) return vec3(0.0f);

    vec3 dir = normalize(toLight);

    // Visibility test
    Ray shadowRay(eyeVertex.point + eyeVertex.normal * 0.001f, dir);
    HitRecord shadowRec;
    if (scene.intersect(shadowRay, shadowRec) && shadowRec.t < distance - 1e-4f) {
        return vec3(0.0f);
    }

    // Geometry term
    float cosEye = max(dot(eyeVertex.normal, dir), 0.0f);
    float cosLight = max(dot(lightVertex.normal, -dir), 0.0f);
    if (cosEye < 1e-5f || cosLight < 1e-5f) return vec3(0.0f);

    float G = (cosEye * cosLight) / (distance * distance);

    vec3 contribution(0.0f);

    if (lightVertex.isLight) {
        // Case: Surface -> Light (indirect lighting)
        vec3 brdf = computeBRDF(eyeVertex, dir);
        vec3 lightEmission = lightVertex.material.emissionColor * lightVertex.material.emissionStrength;
        contribution = eyeVertex.throughput * brdf * G * lightEmission * cosEye;

    } else if (eyeVertex.isLight) {
        // Case: Light -> Camera
        vec3 lightEmission = eyeVertex.material.emissionColor * eyeVertex.material.emissionStrength;
        contribution = lightEmission;  // Light directly visible to camera

    } else {
        // Case: Surface -> Surface (color bleeding)
        vec3 brdfEye = computeBRDF(eyeVertex, dir);
        vec3 brdfLight = computeBRDF(lightVertex, -dir);
        contribution = eyeVertex.throughput * brdfEye * G * brdfLight * lightVertex.throughput * cosEye * cosLight;
    }
    float maxBrightness = 10.0f;
    if (length(contribution) > maxBrightness) {
        contribution = normalize(contribution);
    }

    return contribution;
}
/**
 * Bidirectional path tracing main function.
 * Implements the complete algorithm from Lafortune & Willems paper:
 * 1. Generate eye subpath from camera
 * 2. Generate light subpath from light sources
 * 3. Connect all combinations of vertices from both subpaths
 * 4. Apply Multiple Importance Sampling weights
 *
 * @param ray Initial ray from camera
 * @param scene The scene to render
 * @return Radiance value computed using bidirectional path tracing
 */
/**
 * Bidirectional path tracing based on "Bidirectional Path Tracing" by
 * Lafortune and Willems (1993)
 */


/**
 * Continue path tracing from a specular vertex (traditional path tracing approach)
 */
vec3 tracePath(const Ray& ray, const Scene& scene) {
    auto eyePath = generateEyeSubpath(ray, scene, MAX_DEPTH);
    auto lightPath = generateLightSubpath(scene, MAX_DEPTH);

    if (eyePath.empty()) return vec3(0.1f, 0.1f, 0.2f);

    vec3 result(0.0f);

    // Handle direct light visibility (camera sees light directly)
    if (eyePath[0].isLight) {
        result += eyePath[0].material.emissionColor * eyePath[0].material.emissionStrength;
    }

    for (int i = 0; i < eyePath.size(); i++) {
        for (int j = 0; j < lightPath.size(); j++) {
            vec3 contrib = connectVertices(eyePath[i], lightPath[j], scene);

            // Clamp extremely bright contributions to reduce fireflies
            float maxBrightness = 10.0f; // Adjust this value
            if (length(contrib) > maxBrightness) {
                contrib = normalize(contrib) * maxBrightness;
            }

            result += contrib;
        }
    }
    // Add direct lighting for the first eye vertex (important for shadows)
    if (!eyePath.empty() && !eyePath[0].isLight) {
        vec3 directLight = estimateDirectIllumination(eyePath[0], scene);
        result += directLight;
    }

    // Tone mapping to bring values into reasonable range
    //result = result / (result + vec3(1.0f));
    result = 1.0f - exp(-result * 0.3f);
    return result;
}

/**
 * Constructs the Cornell box scene with walls, lights, and test objects.
 * Creates a classic Cornell box setup with colored walls, area lights, and reflective spheres.
 * This scene is commonly used for testing global illumination algorithms.
 *
 * @param scene The scene object to populate with geometry
 */
void createScene(Scene& scene) {
    auto material2 = Material{1, 1.1, 100, vec3(0,0.6,1)};
    auto sphere2 = std::make_shared<Sphere>(vec3(-1, 0, 0),1,material2);

    float wallDiffuse = 1.6f;
    float wallSpecular = 0.0f;
    float wallShininess = 100.0f;
    Material floorMat{ wallDiffuse,wallSpecular,wallShininess,vec3(0.5f, 0.5f, 0.7f) };
    Material roofMat{ wallDiffuse,wallSpecular,wallShininess,vec3(0.8f, 0.8f, 0.8f) };
    Material leftMat{ wallDiffuse,wallSpecular,wallShininess,vec3(0.8f, 0.2f, 0.2f) };
    Material rightMat{ wallDiffuse,wallSpecular,wallShininess,vec3(0.2f, 0.8f, 0.2f) };
    Material backMat{ wallDiffuse,wallSpecular,wallShininess,vec3(0.6f, 0.2f, 0.4f) };
    Material lightMat{0.0f, 0.0f, 0.0f, vec3(1.0f), vec3(1.0f, 1.0f, 1.0f), 10.0f};
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
        left, right, depth, closest, down, floorMat));
    scene.objects.push_back(std::make_shared<XZRectangle>(
        left, right, depth, closest, up, roofMat));
    scene.objects.push_back(std::make_shared<YZRectangle>(
        down, up, depth, closest, left, leftMat));
    scene.objects.push_back(std::make_shared<YZRectangle>(
        down, up, depth, closest, right, rightMat));
    scene.objects.push_back(std::make_shared<XYRectangle>(
        left, right, down, up, depth, backMat));

    scene.objects.push_back(std::make_shared<XZRectangle>(
        right-0.6f, right-0.1f, depth+0.5f, depth+1.0f, up-0.01f, lightMat));

    Material glassMat{0.0f, 1.0f, 100.0f, vec3(1.0f, 1.0f, 1.0f)}; // Pure specular
    auto material1 = Material{1, 1.1, 100, vec3(0,0.6,1)};
    auto sphere1 = std::make_shared<Sphere>(vec3(middlex, middley, middlez-2),0.6,glassMat);
    scene.objects.push_back(sphere1);
    //auto glassSphere = std::make_shared<Sphere>(vec3(0.0f, -1.0f, -1.5f), 0.4f, glassMat);
   //scene.objects.push_back(glassSphere);

}

void createFlashyScene(Scene& scene) {
    float depth = -3.0f;
    float closest = 1.5f;
    float left = -2.0f;
    float right = 2.0f;
    float down = -2.0f;
    float up = 2.0f;
    float middlex = (left + right) / 2.0f;
    float middley = (down + up) / 2.0f;
    float middlez = (depth + closest) / 2.0f;

    // ====== ENHANCED WALLS WITH SOME SPECULARITY ======
    float wallDiffuse = 1.4f;
    float wallSpecular = 0.2f; // Added slight specular for reflections
    Material floorMat{ wallDiffuse, wallSpecular, 500.0f, vec3(0.5f, 0.5f, 0.8f) };
    Material roofMat{ wallDiffuse, wallSpecular, 500.0f, vec3(0.9f, 0.9f, 0.9f) };
    Material leftMat{ wallDiffuse, wallSpecular, 500.0f, vec3(0.9f, 0.2f, 0.2f) };  // Brighter red
    Material rightMat{ wallDiffuse, wallSpecular, 500.0f, vec3(0.2f, 0.9f, 0.2f) }; // Brighter green
    Material backMat{ wallDiffuse, wallSpecular, 500.0f, vec3(0.3f, 0.2f, 0.8f) };  // Deeper purple

    scene.objects.push_back(std::make_shared<XZRectangle>(left, right, depth, closest, down, floorMat));
    scene.objects.push_back(std::make_shared<XZRectangle>(left, right, depth, closest, up, roofMat));
    scene.objects.push_back(std::make_shared<YZRectangle>(down, up, depth, closest, left, leftMat));
    scene.objects.push_back(std::make_shared<YZRectangle>(down, up, depth, closest, right, rightMat));
    scene.objects.push_back(std::make_shared<XYRectangle>(left, right, down, up, depth, backMat));

    // ====== MULTIPLE COLORED LIGHTS FOR COMPLEX ILLUMINATION ======
    Material whiteLightMat{0.0f, 0.0f, 0.0f, vec3(1.0f, 1.0f, 1.0f), vec3(1.0f, 1.0f, 1.0f), 8.0f};
    Material redLightMat{0.0f, 0.0f, 0.0f, vec3(1.0f, 0.3f, 0.3f), vec3(1.0f, 0.3f, 0.3f), 30.0f};
    Material blueLightMat{0.0f, 0.0f, 0.0f, vec3(0.3f, 0.3f, 1.0f), vec3(0.3f, 0.3f, 1.0f), 30.0f};

    // Main white light
    scene.objects.push_back(std::make_shared<XZRectangle>(
        right-0.6f, right-0.1f, depth+0.5f, depth+1.0f, up-0.01f, whiteLightMat));

    // Colored accent lights
    scene.objects.push_back(std::make_shared<XZRectangle>(
        left+0.1f, left+0.3f, depth+1.0f, depth+1.5f, up-0.01f, redLightMat));

    scene.objects.push_back(std::make_shared<YZRectangle>(
        down+0.5f, down+1.0f, depth+0.5f, depth+1.0f, left+0.01f, blueLightMat));

    // ====== SPECTACULAR OBJECT ARRANGEMENT ======

    // 1. CENTRAL GLASS SPHERE (refractive caustics)
    Material glassMat{0.0f, 1.0f, 1000.0f, vec3(1.0f, 1.0f, 1.0f)};
    auto glassSphere = std::make_shared<Sphere>(vec3(middlex, middley, middlez-1.5f), 0.6f, glassMat);
    scene.objects.push_back(glassSphere);

    // 2. MIRRORED SPHERE (specular caustics and reflections)
    Material mirrorMat{0.0f, 1.0f, 2000.0f, vec3(0.95f, 0.95f, 1.0f)};
    auto mirrorSphere = std::make_shared<Sphere>(vec3(middlex+1.0f, middley-0.3f, middlez-1.0f), 0.4f, mirrorMat);
    scene.objects.push_back(mirrorSphere);

    // 3. COLORED METALLIC SPHERE (glossy reflections)
    Material metalMat{0.3f, 0.7f, 300.0f, vec3(0.1f, 0.6f, 0.9f)};
    auto metalSphere = std::make_shared<Sphere>(vec3(middlex-1.0f, middley-0.3f, middlez-1.0f), 0.4f, metalMat);
    scene.objects.push_back(metalSphere);

    // 4. DIAMOND-LIKE GLASS PYRAMID (complex caustics)
    Material diamondMat{0.0f, 1.0f, 1500.0f, vec3(1.0f, 1.0f, 1.0f)};
    // For simplicity, using a small glass sphere as placeholder - in advanced version, implement actual triangles
    auto diamond = std::make_shared<Sphere>(vec3(middlex, middley-1.2f, middlez-0.5f), 0.25f, diamondMat);
    scene.objects.push_back(diamond);

    // 5. GLASS RECTANGLE ON FLOOR (planar refraction)
    Material glassRectMat{0.0f, 1.0f, 1000.0f, vec3(1.0f, 1.0f, 1.0f)};
    auto glassRect = std::make_shared<XZRectangle>(
        middlex-0.4f, middlex+0.4f, middlez-2.0f, middlez-1.0f, down+0.02f, glassRectMat);
    scene.objects.push_back(glassRect);

    // 6. FLOATING DIFFUSE SPHERES (color bleeding demonstration)
    Material redSphereMat{1.0f, 0.1f, 50.0f, vec3(0.9f, 0.1f, 0.1f)};
    Material greenSphereMat{1.0f, 0.1f, 50.0f, vec3(0.1f, 0.9f, 0.1f)};
    Material yellowSphereMat{1.0f, 0.1f, 50.0f, vec3(0.9f, 0.9f, 0.1f)};

    auto redSphere = std::make_shared<Sphere>(vec3(middlex-0.8f, middley+0.8f, middlez-2.2f), 0.25f, redSphereMat);
    auto greenSphere = std::make_shared<Sphere>(vec3(middlex+0.8f, middley+0.8f, middlez-2.2f), 0.25f, greenSphereMat);
    auto yellowSphere = std::make_shared<Sphere>(vec3(middlex, middley+1.0f, middlez-1.8f), 0.2f, yellowSphereMat);

    scene.objects.push_back(redSphere);
    scene.objects.push_back(greenSphere);
    scene.objects.push_back(yellowSphere);

    // 7. MIRRORED CUBE (complex specular interactions)
    Material mirrorCubeMat{0.0f, 1.0f, 2000.0f, vec3(0.9f, 0.9f, 1.0f)};
    // Using a sphere as cube placeholder - implement actual cube in advanced version
    auto mirrorCube = std::make_shared<Sphere>(vec3(middlex-0.6f, middley+0.6f, middlez-0.8f), 0.3f, mirrorCubeMat);
    scene.objects.push_back(mirrorCube);
}
std::atomic scanlinesCompleted(0);
auto renderStartTime = std::chrono::steady_clock::now();
std::mutex coutMutex;


/**
 * Renders a tile of the image using multithreading.
 * Each thread processes a horizontal strip of the image, computing path tracing
 * for each pixel with multiple samples for anti-aliasing and noise reduction.
 *
 * @param image The output image to write pixel values to
 * @param scene The scene to render
 * @param cam The camera defining the viewpoint
 * @param startY Starting Y coordinate for this tile
 * @param endY Ending Y coordinate for this tile (exclusive)
 * @param width Image width in pixels
 * @param height Image height in pixels
 * @param samplesPerPixel Number of Monte Carlo samples per pixel
 */
void renderTile(Image& image, const Scene& scene, const Camera& cam,
                const int startY, const int endY, const int width, const int height, const int samplesPerPixel) {
    for (int j = startY; j < endY; j++) {
        for (int i = 0; i < width; i++) {
            vec3 color(0.0f);

            for (int s = 0; s < samplesPerPixel; s++) {
                float u = (static_cast<float>(i) + Random::nextFloat()) / static_cast<float>(width);
                float v = (static_cast<float>(j) + Random::nextFloat()) / static_cast<float>(height);
                Ray ray = cam.getRay(u, v);
                color += tracePath(ray, scene);
            }

            color /= static_cast<float>(samplesPerPixel);
            color = sqrt(color);
            image.pixels[j * width + i] = color;
        }

        // Update progress - SYNCHRONIZED
        const int completed = ++scanlinesCompleted;

        {
            std::lock_guard<std::mutex> lock(coutMutex);  // Only one thread can print at a time
            auto currentTime = std::chrono::steady_clock::now();
            const auto elapsedSeconds = std::chrono::duration_cast<std::chrono::seconds>(currentTime - renderStartTime).count();

            if (completed > 0) {
                const double secondsPerScanline = static_cast<double>(elapsedSeconds) / completed;
                const int remainingSeconds = static_cast<int>(secondsPerScanline * (height - completed));

                std::cout << "\rProgress: " << completed << "/" << height
                          << " (" << (completed * 100 / height) << "%) | "
                          << "Elapsed: " << elapsedSeconds << "s | "
                          << "ETA: " << remainingSeconds << "s" << std::flush;
            }
        }
    }
}

/**
 * Main rendering application entry point.
 * Sets up the scene, camera, and rendering parameters, then spawns multiple
 * threads to render the image using path tracing with progress tracking.
 *
 * @return Exit status code (0 for success)
 */
int main() {
    Camera cam;
    // cam.pos = vec3(0.0f, 0.5f, 3.0f);        // Elevated viewpoint
    // cam.lookAt = vec3(0.0f, -0.5f, -2.0f);   // Looking at the center of action
    // cam.up = vec3(0.0f, 1.0f, 0.0f);
    // cam.fov = 45.0f;                         // Wider field of view
    // cam.aspectRatio = 1.0f;
    Scene scene;

    //createScene(scene);
    createFlashyScene(scene);

    int width = 400, height = 400;
    int samplesPerPixel = 4096; // Increase for better quality
    Image image(width, height);

    int numThreads = static_cast<int>(std::thread::hardware_concurrency())-2;
    //numThreads = 1; // FOR TESTING
    std::vector<std::thread> threads;
    const int tileHeight = height / numThreads;
    std::cout << "Rendering with " << samplesPerPixel << " samples per pixel on " << numThreads << " threads"<< std::endl;
    for (int t = 0; t < numThreads; t++) {
        int startY = t * tileHeight;
        int endY = (t == numThreads - 1) ? height : startY + tileHeight;
        threads.emplace_back(renderTile, std::ref(image), std::ref(scene),
                            std::ref(cam), startY, endY, width, height, samplesPerPixel);
    }
    for (auto& thread : threads) thread.join();
    std::cout << "\nDone!" << std::endl;
    image.savePPM("monte_carlo_output.ppm");
    return 0;
}