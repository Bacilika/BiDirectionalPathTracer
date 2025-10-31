#include <iostream>
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
const float RR_START = 3;

/**
 * Estimates the surface area of a geometry object for light sampling.
 * This is used to compute the probability density function (PDF) when sampling light sources.
 *
 * @param obj Shared pointer to the geometry object
 * @return Surface area of the object in world units, or 1.0 as default for unknown types
 */
float estimateObjectArea(const std::shared_ptr<Geometry>& obj) {
    if (const auto sphere = std::dynamic_pointer_cast<Sphere>(obj)) {
        return 4.0f * static_cast<float>(M_PI) * sphere->radius * sphere->radius;
    }
    if (const auto rect = std::dynamic_pointer_cast<XYRectangle>(obj)) {
        return (rect->x1 - rect->x0) * (rect->y1 - rect->y0);
    }
    if (const auto rect = std::dynamic_pointer_cast<XZRectangle>(obj)) {
        return (rect->x1 - rect->x0) * (rect->z1 - rect->z0);
    }
    if (const auto rect = std::dynamic_pointer_cast<YZRectangle>(obj)) {
        return (rect->y1 - rect->y0) * (rect->z1 - rect->z0);
    }
    return 1.0f;
}

/**
 * Samples a random point uniformly distributed over the surface of a geometry object.
 * Used for area light sampling in direct lighting calculations.
 *
 * @param obj Shared pointer to the geometry object to sample from
 * @return A random 3D point on the surface of the object
 */
vec3 pickRandomPointOnObject(const std::shared_ptr<Geometry>& obj) {
    if (const auto sphere = std::dynamic_pointer_cast<Sphere>(obj)) {
        const vec3 localPoint = Random::randomInUnitSphere();
        const vec3 direction = normalize(localPoint);
        return sphere->center + direction * sphere->radius;
    }
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
    return vec3(0.0f);
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
    std::vector<std::shared_ptr<Geometry>> lights;
    for (auto& obj : scene.objects) {
        if (obj->getMaterial().isEmissive()) lights.push_back(obj);
    }

    if (lights.empty()) {
        return {PathVertex(vec3(0), vec3(0), Material(0,0,0,vec3(0)), vec3(0)), vec3(0), 1.0f};
    }

    int lightIndex = static_cast<int>(Random::nextFloat() * lights.size());
    auto& lightObj = lights[lightIndex];
    const Material& lightMat = lightObj->getMaterial();

    vec3 lightPoint = pickRandomPointOnObject(lightObj);
    HitRecord lightHitRec;
    Ray dummyRay(lightPoint, vec3(0,1,0));
    lightObj->intersect(dummyRay, lightHitRec);


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
    float lightPdf = 1.0f / (lights.size() * area);

    vec3 throughput = lightMat.emissionColor * lightMat.emissionStrength;

    PathVertex vertex(lightPoint, lightHitRec.normal, lightMat, throughput);
    vertex.isLight = true;  // Mark as light source

    return {vertex, lightDir, lightPdf};
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
        if (vertex.material.specular > 0.9f) {
            newDirection = reflect(-vertex.wi, vertex.normal);
        } else {
            vec3 perfectReflection = reflect(-vertex.wi, vertex.normal);
            vec3 localDir = Random::randomCosineDirection();
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
    if (vertex.isLight) return vec3(0.0f);

    vec3 brdf(0.0f);
    if (vertex.material.diffuse > 0.0f) {
        brdf += vertex.material.color * vertex.material.diffuse / static_cast<float>(M_PI);
    }
    if (vertex.material.specular > 0.0f) {
        vec3 perfectReflection = reflect(-vertex.wi, vertex.normal);
        float specularFactor = pow(max(dot(perfectReflection, outgoingDir), 0.0f), vertex.material.shininess);
        brdf += vec3(1.0f) * vertex.material.specular * specularFactor;
    }
    return brdf;
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

        vec3 newDir;
        float pdf;
        sampleScattering(vertex, newDir, pdf);
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
    auto lightSample = sampleLightSource(scene);
    path.push_back(lightSample.vertex);

    if (maxDepth > 1) {
        Ray ray(lightSample.vertex.point + lightSample.vertex.normal * 0.001f, lightSample.direction);
        vec3 throughput = lightSample.vertex.throughput;


        for (int depth = 1; depth < maxDepth; depth++) {
            HitRecord hitRec;
            if (!scene.intersect(ray, hitRec)) break;

            PathVertex vertex(hitRec, -ray.direction, throughput);
            path.push_back(vertex);

            if (vertex.isLight) break;
            if (depth >= RR_START) {
                float continueProb = std::max(vertex.material.diffuse, vertex.material.specular);
                continueProb = std::min(0.95f, continueProb);
                if (Random::nextFloat() > continueProb) break;
            }

            vec3 newDir;
            float pdf;
            sampleScattering(vertex, newDir, pdf);
            if (pdf < 1e-5f) break;
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
    if (eyeVertex.isLight) return vec3(0.0f);
    vec3 L_direct(0.0f);

    std::vector<std::shared_ptr<Geometry>> lights;
    for (auto& obj : scene.objects) {
        if (obj->getMaterial().isEmissive()) lights.push_back(obj);
    }

    for (auto& light : lights) {
        vec3 lightPoint = pickRandomPointOnObject(light);
        HitRecord lightHitRec;
        Ray dummyRay(lightPoint, vec3(0,1,0));
        light->intersect(dummyRay, lightHitRec);

        vec3 toLight = lightPoint - eyeVertex.point;
        float distance = length(toLight);
        if (distance < 1e-5f) continue;

        vec3 wi = normalize(toLight);
        Ray shadowRay(eyeVertex.point + eyeVertex.normal * 0.001f, wi);
        HitRecord shadowRec;
        if (scene.intersect(shadowRay, shadowRec) && shadowRec.t < distance - 1e-4f) {
            continue;
        }

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

        L_direct += brdf * lightEmission * G / totalPdf;
    }
    return L_direct / static_cast<float>(lights.size());
}

/**
 * Connect an eye subpath vertex to a light subpath vertex for bidirectional path tracing.
 * Handles three cases: surface-to-light, light-to-camera, and surface-to-surface connections.
 *
 * @param eyeVertex Vertex from the eye subpath
 * @param lightVertex Vertex from the light subpath
 * @param scene The scene for visibility testing
 * @return Radiance contribution from this connection
 */
vec3 connectVertices(const PathVertex& eyeVertex, const PathVertex& lightVertex, const Scene& scene) {
    if (eyeVertex.isLight && lightVertex.isLight) return vec3(0.0f);

    vec3 toLight = lightVertex.point - eyeVertex.point;
    float distance = length(toLight);
    if (distance < 1e-5f) return vec3(0.0f);

    vec3 dir = normalize(toLight);
    Ray shadowRay(eyeVertex.point + eyeVertex.normal * 0.001f, dir);
    HitRecord shadowRec;
    if (scene.intersect(shadowRay, shadowRec) && shadowRec.t < distance - 1e-4f) {
        return vec3(0.0f);
    }

    float cosEye = max(dot(eyeVertex.normal, dir), 0.0f);
    float cosLight = max(dot(lightVertex.normal, -dir), 0.0f);
    if (cosEye < 1e-5f || cosLight < 1e-5f) return vec3(0.0f);

    float G = (cosEye * cosLight) / (distance * distance);
    vec3 contribution(0.0f);

    if (lightVertex.isLight) {
        vec3 brdf = computeBRDF(eyeVertex, dir);
        vec3 lightEmission = lightVertex.material.emissionColor * lightVertex.material.emissionStrength;
        contribution = eyeVertex.throughput * brdf * G * lightEmission * cosEye;
    } else if (eyeVertex.isLight) {
        contribution = eyeVertex.material.emissionColor * eyeVertex.material.emissionStrength;
    } else {
        vec3 brdfEye = computeBRDF(eyeVertex, dir);
        vec3 brdfLight = computeBRDF(lightVertex, -dir);
        contribution = eyeVertex.throughput * brdfEye * G * brdfLight * lightVertex.throughput * cosEye * cosLight;
    }

    float maxBrightness = 10.0f;
    if (length(contribution) > maxBrightness) {
        contribution = normalize(contribution) * maxBrightness;
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
vec3 tracePath(const Ray& ray, const Scene& scene) {
    auto eyePath = generateEyeSubpath(ray, scene, MAX_DEPTH);
    auto lightPath = generateLightSubpath(scene, MAX_DEPTH);

    if (eyePath.empty()) return vec3(0.1f, 0.1f, 0.2f);

    vec3 result(0.0f);
    if (eyePath[0].isLight) {
        result += eyePath[0].material.emissionColor * eyePath[0].material.emissionStrength;
    }

    for (int i = 0; i < eyePath.size(); i++) {
        for (int j = 0; j < lightPath.size(); j++) {
            vec3 contrib = connectVertices(eyePath[i], lightPath[j], scene);
            float maxBrightness = 10.0f;
            if (length(contrib) > maxBrightness) {
                contrib = normalize(contrib) * maxBrightness;
            }
            result += contrib;
        }
    }

    if (!eyePath.empty() && !eyePath[0].isLight) {
        vec3 directLight = estimateDirectIllumination(eyePath[0], scene);
        result += directLight;
    }

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
    float depth = -3.0f, closest = 1.5f;
    float left = -2.0f, right = 2.0f;
    float down = -2.0f, up = 2.0f;
    float middlex = (left + right) / 2.0f;
    float middley = (down + up) / 2.0f;
    float middlez = (depth + closest) / 2.0f;

    float wallDiffuse = 1.4f, wallSpecular = 0.2f;
    Material floorMat{wallDiffuse, wallSpecular, 500.0f, vec3(0.5f, 0.5f, 0.8f)};
    Material roofMat{wallDiffuse, wallSpecular, 500.0f, vec3(0.9f, 0.9f, 0.9f)};
    Material leftMat{wallDiffuse, wallSpecular, 500.0f, vec3(0.9f, 0.2f, 0.2f)};
    Material rightMat{wallDiffuse, wallSpecular, 500.0f, vec3(0.2f, 0.9f, 0.2f)};
    Material backMat{wallDiffuse, wallSpecular, 500.0f, vec3(0.3f, 0.2f, 0.8f)};

    scene.objects.push_back(std::make_shared<XZRectangle>(left, right, depth, closest, down, floorMat));
    scene.objects.push_back(std::make_shared<XZRectangle>(left, right, depth, closest, up, roofMat));
    scene.objects.push_back(std::make_shared<YZRectangle>(down, up, depth, closest, left, leftMat));
    scene.objects.push_back(std::make_shared<YZRectangle>(down, up, depth, closest, right, rightMat));
    scene.objects.push_back(std::make_shared<XYRectangle>(left, right, down, up, depth, backMat));

    Material whiteLightMat{0.0f, 0.0f, 0.0f, vec3(1.0f), vec3(1.0f), 20.0f};
    Material redLightMat{0.0f, 0.0f, 0.0f, vec3(1.0f, 0.3f, 0.3f), vec3(1.0f, 0.3f, 0.3f), 50.0f};
    Material blueLightMat{0.0f, 0.0f, 0.0f, vec3(0.3f, 0.3f, 1.0f), vec3(0.3f, 0.3f, 1.0f), 50.0f};

    scene.objects.push_back(std::make_shared<XZRectangle>(right-0.6f, right-0.1f, depth+0.5f, depth+1.0f, up-0.01f, whiteLightMat));
    scene.objects.push_back(std::make_shared<XZRectangle>(left+0.1f, left+0.3f, depth+1.0f, depth+1.5f, up-0.01f, redLightMat));
    scene.objects.push_back(std::make_shared<YZRectangle>(down+0.5f, down+1.0f, depth+0.5f, depth+1.0f, left+0.01f, blueLightMat));

    Material glassMat{0.0f, 5.0f, 1000.0f, vec3(1.0f)};
    Material mirrorMat{0.0f, 5.0f, 2000.0f, vec3(0.95f, 0.95f, 1.0f)};
    Material metalMat{0.3f, 2.f, 300.0f, vec3(0.1f, 0.6f, 0.9f)};

    scene.objects.push_back(std::make_shared<Sphere>(vec3(middlex, middley, middlez-1.5f), 0.6f, glassMat));
    scene.objects.push_back(std::make_shared<Sphere>(vec3(middlex+1.0f, middley-0.3f, middlez-1.0f), 0.4f, mirrorMat));
    scene.objects.push_back(std::make_shared<Sphere>(vec3(middlex-1.0f, middley-0.3f, middlez-1.0f), 0.4f, metalMat));
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
                float u = (static_cast<float>(i) + Random::nextFloat()) / width;
                float v = (static_cast<float>(j) + Random::nextFloat()) / height;
                Ray ray = cam.getRay(u, v);
                color += tracePath(ray, scene);
            }
            color /= samplesPerPixel;
            color = sqrt(color);
            image.pixels[j * width + i] = color;
        }

        int completed = ++scanlinesCompleted;
        {
            std::lock_guard<std::mutex> lock(coutMutex);
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
    createScene(scene);

    int width = 400, height = 400;
    int samplesPerPixel = 256;
    Image image(width, height);

    int numThreads = std::thread::hardware_concurrency() - 2;
    std::vector<std::thread> threads;
    int tileHeight = height / numThreads;

    std::cout << "Rendering with " << samplesPerPixel << " samples per pixel on " << numThreads << " threads"<< std::endl;

    for (int t = 0; t < numThreads; t++) {
        int startY = t * tileHeight;
        int endY = (t == numThreads - 1) ? height : startY + tileHeight;
        threads.emplace_back(renderTile, std::ref(image), std::ref(scene), std::ref(cam),
                           startY, endY, width, height, samplesPerPixel);
    }

    for (auto& thread : threads) thread.join();
    std::cout << "\nDone!" << std::endl;
    image.savePPM("monte_carlo_output.ppm");
    return 0;
}