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
        return 4.0f * M_PI * sphere->radius * sphere->radius;
    }
    // rectangles
    if (const auto rect = std::dynamic_pointer_cast<XYRectangle>(obj)) {
        return (rect->x1 - rect->x0) * (rect->y1 - rect->y0);
    }
    if (auto rect = std::dynamic_pointer_cast<XZRectangle>(obj)) {
        return (rect->x1 - rect->x0) * (rect->z1 - rect->z0);
    }
    if (auto rect = std::dynamic_pointer_cast<YZRectangle>(obj)) {
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
        vec3 localPoint = Random::randomInUnitSphere();
        vec3 direction = normalize(localPoint);
        return sphere->center + direction * sphere->radius;
    }
    // rectangles
    if (auto rect = std::dynamic_pointer_cast<XYRectangle>(obj)) {
        float x = rect->x0 + Random::nextFloat() * (rect->x1 - rect->x0);
        float y = rect->y0 + Random::nextFloat() * (rect->y1 - rect->y0);
        return vec3(x, y, rect->k);
    }
    if (auto rect = std::dynamic_pointer_cast<XZRectangle>(obj)) {
        float x = rect->x0 + Random::nextFloat() * (rect->x1 - rect->x0);
        float z = rect->z0 + Random::nextFloat() * (rect->z1 - rect->z0);
        return vec3(x, rect->k, z);
    }
    if (auto rect = std::dynamic_pointer_cast<YZRectangle>(obj)) {
        float y = rect->y0 + Random::nextFloat() * (rect->y1 - rect->y0);
        float z = rect->z0 + Random::nextFloat() * (rect->z1 - rect->z0);
        return vec3(rect->k, y, z);
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

    // Sample random light
    int lightIndex = int(Random::nextFloat() * lights.size());
    auto& lightObj = lights[lightIndex];
    const Material& lightMat = lightObj->getMaterial();

    // Sample point on light
    vec3 lightPoint = pickRandomPointOnObject(lightObj, vec3(0.0f));

    // Get normal at sampled point
    HitRecord lightHitRec;
    Ray dummyRay(lightPoint, vec3(0,1,0));
    lightObj->intersect(dummyRay, lightHitRec);

    // Sample direction (cosine-weighted for area lights)
    vec3 lightDir = Random::randomCosineDirection();
    // Transform to world coordinates using light's normal...

    float area = estimateObjectArea(lightObj);
    float areaPdf = 1.0f / area;
    float lightPdf = 1.0f / lights.size();
    float totalPdf = areaPdf * lightPdf;

    vec3 throughput = lightMat.emissionColor * lightMat.emissionStrength / totalPdf;

    PathVertex vertex(lightPoint, lightHitRec.normal, &lightMat, throughput);
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
    float diffuseProb = vertex.material->diffuse / (vertex.material->diffuse + vertex.material->specular + 1e-5f);

    if (Random::nextFloat() < diffuseProb) {
        // Cosine-weighted hemisphere sampling
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
        // Specular bounce
        newDirection = reflect(-vertex.wi, vertex.normal);
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
        return vertex.material->emissionColor * vertex.material->emissionStrength;
    }

    vec3 brdf(0.0f);

    // Diffuse component
    if (vertex.material->diffuse > 0.0f) {
        brdf += vertex.material->color * vertex.material->diffuse / static_cast<float>(M_PI);
    }

    // Specular component
    if (vertex.material->specular > 0.0f) {
        vec3 perfectReflection = reflect(-vertex.wi, vertex.normal);
        float specularFactor = pow(max(dot(perfectReflection, outgoingDir), 0.0f),
                                 vertex.material->shininess);
        brdf += vec3(1.0f) * vertex.material->specular * specularFactor;
    }

    return brdf;
}

/**
 * Computes direct lighting contribution using Next Event Estimation (NEE).
 * Instead of waiting for paths to randomly hit lights, this function explicitly
 * samples light sources and computes their direct contribution to the current surface point.
 * Implements Monte Carlo integration with importance sampling of light sources.
 *
 * @param hitRecord Information about the surface point being shaded
 * @param scene The scene containing lights and geometry
 * @return The direct lighting contribution from all light sources
 */
vec3 sampleDirectLighting(const HitRecord& hitRec, const Scene& scene) {
    vec3 direct(0.0f);

    // Count actual lights
    std::vector<std::shared_ptr<Geometry>> lights;
    for (auto& obj : scene.objects) {
        if (obj->getMaterial().isEmissive()) {
            lights.push_back(obj);
        }
    }

    if (lights.empty()) return direct; //if no lights, return black

    int numSamples = 1;
    for (int i = 0; i < numSamples; ++i) {
        // Pick a random light
        int lightIndex = int(Random::nextFloat() * lights.size());
        auto& lightObj = lights[lightIndex];
        const Material& lightMat = lightObj->getMaterial();

        // get random point
        vec3 lightPoint = pickRandomPointOnObject(lightObj, hitRec.point);

        // Get light surface normal at sampled point
        HitRecord lightHitRec;
        Ray dummyRay(lightPoint, vec3(0,1,0)); // Direction doesn't matter for normal
        lightObj->intersect(dummyRay, lightHitRec); // lightHitRec returns the normal

        // Compute geometry term G
        float G = computeGeometryTerm(hitRec.point, hitRec.normal,
                                     lightPoint, lightHitRec.normal);

        if (G > 1e-5f) { //if G is not "zero"
            // Visibility test
            vec3 toLight = lightPoint - hitRec.point;
            float lightDistance = length(toLight);
            vec3 lightDir = normalize(toLight);

            Ray shadowRay(hitRec.point + hitRec.normal * 0.001f, lightDir);
            HitRecord shadowRec;

            if (!scene.intersect(shadowRay, shadowRec) || shadowRec.t > lightDistance) {

                // BRDF evaluation
                PathVertex tempVertex(hitRec, vec3(0.0f), vec3(1.0f)); // Create temporary vertex
                vec3 brdf = computeBRDF(tempVertex, lightDir);
                vec3 lightEmission = lightMat.emissionColor * lightMat.emissionStrength;

                // Area PDF for sampling the light surface
                float lightArea = estimateObjectArea(lightObj);
                float areaPdf = 1.0f / lightArea;

                // Light selection PDF
                float lightPdf = 1.0f / lights.size();

                // Full Monte Carlo estimator
                direct += brdf * lightEmission * G * lightArea / (lightPdf * areaPdf);
            }
        }
    }

    return direct / float(numSamples);
}

/**
 * Standard unidirectional path tracing from the eye perspective.
 * Traces rays from the camera, computing direct and indirect lighting at each intersection.
 * This is the classic Monte Carlo path tracing algorithm that will be extended to bidirectional.
 *
 * @param ray The ray to trace through the scene
 * @param scene The scene containing geometry and lights
 * @param depth Current recursion depth for path termination
 * @return The radiance value accumulated along the path
 */
vec3 traceEyeOnly(const Ray& ray, const Scene& scene, int depth = 0) {
    if (depth > MAX_DEPTH) return vec3(0.0f);

    HitRecord hitRec;
    if (!scene.intersect(ray, hitRec)) {
        return vec3(0.01f, 0.01f, 0.02f);
    }

    // Create path vertex
    PathVertex vertex(hitRec, -ray.direction, vec3(1.0f));

    // Emissive contribution
    vec3 result(0.0f);
    if (vertex.isLight) {
        result += vertex.material->emissionColor * vertex.material->emissionStrength;
        if (depth > 0) return result; // Only count emission on direct hits for now
    }

    // Russian Roulette
    float continueProbability = 1.0f;
    if (depth > RR_START) {
        continueProbability = std::max(vertex.material->diffuse, vertex.material->specular);
        continueProbability = std::min(0.95f, continueProbability);
        if (Random::nextFloat() > continueProbability) {
            return result;
        }
    }

    // Direct lighting
    vec3 direct = sampleDirectLighting(hitRec, scene);
    result += direct;

    // Indirect lighting
    vec3 newDirection;
    float pdf;
    sampleScattering(vertex, newDirection, pdf);

    if (pdf > 1e-5f) {
        Ray newRay(vertex.point + vertex.normal * 0.001f, newDirection);
        vec3 incoming = traceEyeOnly(newRay, scene, depth + 1);

        vec3 brdf = computeBRDF(vertex, newDirection);
        float cosTheta = max(dot(vertex.normal, newDirection), 0.0f);
        vec3 indirect = brdf * incoming * cosTheta / pdf;

        if (depth > RR_START) {
            indirect /= continueProbability;
        }

        result += indirect;
    }

    return result;
}
/**
 * Main path tracing entry point that currently delegates to unidirectional tracing.
 * This function serves as an abstraction layer that will be replaced with
 * bidirectional path tracing in the future implementation.
 *
 * @param ray The ray to trace through the scene
 * @param scene The scene containing geometry and lights
 * @return The computed radiance value for this ray
 */
vec3 tracePath(const Ray& ray, const Scene& scene) {
    return traceEyeOnly(ray, scene);
}
/**
 * Constructs the Cornell box scene with walls, lights, and test objects.
 * Creates a classic Cornell box setup with colored walls, area lights, and reflective spheres.
 * This scene is commonly used for testing global illumination algorithms.
 *
 * @param scene The scene object to populate with geometry
 */

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
    Material lightMat{0.0f, 0.0f, 0.0f, vec3(1.0f), vec3(1.0f, 1.0f, 1.0f), 5.0f};
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


    Material sphereLightMat(0.0f, 0.0f, 0.0f, vec3(1.0f),
                           vec3(1.0f, 0.7f, 0.3f), 1.0f); // Orange light
    auto lightSphere = std::make_shared<Sphere>(vec3(left+0.4f, up-0.4f, -1.5f), 0.3f, sphereLightMat);
    scene.objects.push_back(lightSphere);


    auto material1 = Material{1, 1.1, 100, vec3(0,0.6,1)};
    auto sphere1 = std::make_shared<Sphere>(vec3(middlex, middley, middlez-2),0.6,material1);
    scene.objects.push_back(sphere1);
}

std::atomic<int> scanlinesCompleted(0);
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
                int startY, int endY, int width, int height, int samplesPerPixel) {
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
        int completed = ++scanlinesCompleted;

        {
            std::lock_guard<std::mutex> lock(coutMutex);  // Only one thread can print at a time
            auto currentTime = std::chrono::steady_clock::now();
            auto elapsedSeconds = std::chrono::duration_cast<std::chrono::seconds>(currentTime - renderStartTime).count();

            if (completed > 0) {
                double secondsPerScanline = static_cast<double>(elapsedSeconds) / completed;
                int remainingSeconds = static_cast<int>(secondsPerScanline * (height - completed));

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
    Scene scene;

    createFlashyScene(scene);

    int width = 400, height = 400;
    int samplesPerPixel = 4096; // Increase for better quality
    Image image(width, height);

    std::cout << "Rendering with " << samplesPerPixel << " samples per pixel..." << std::endl;
    
    int numThreads = std::thread::hardware_concurrency()-2;
    std::vector<std::thread> threads;
    int tileHeight = height / numThreads;
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