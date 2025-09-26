

#include "geometry.h"

class Sphere final :public Geometry {
public:
    vec3 center;
    float radius;
    Material material;

    Sphere(const vec3& c, float r, const Material& m)
        : center(c), radius(r), material(m) {}

    bool intersect(const Ray& ray, HitRecord& rec) const override {
        vec3 oc = ray.origin - center;
        float a = dot(ray.direction, ray.direction);
        float b = 2.0f * dot(oc, ray.direction);
        float c = dot(oc, oc) - radius * radius;

        float discriminant = b*b - 4*a*c;
        if (discriminant < 0) return false;

        float t = (-b - sqrt(discriminant)) / (2.0f * a);
        if (t < 0) t = (-b + sqrt(discriminant)) / (2.0f * a);
        if (t < 0) return false;

        rec.t = t;
        rec.point = ray.origin + t * ray.direction;
        rec.normal = normalize(rec.point - center);
        rec.material = material;
        return true;
    }
};

class XYRectangle final : public Geometry {
public:
    float x0, x1, y0, y1, k;
    Material material;

    XYRectangle(float _x0, float _x1,
                float _y0, float _y1,
                float _k, const Material& m)
        : x0(_x0), x1(_x1), y0(_y0), y1(_y1), k(_k), material(m) {}

    bool intersect(const Ray& ray, HitRecord& rec) const override {
        if (fabs(ray.direction.z) < 1e-6) return false;
        float t = (k - ray.origin.z) / ray.direction.z;
        if (t < 0) return false;

        float x = ray.origin.x + t * ray.direction.x;
        float y = ray.origin.y + t * ray.direction.y;
        if (x < x0 || x > x1 || y < y0 || y > y1) return false;

        rec.t = t;
        rec.point = ray.origin + t * ray.direction;
        rec.normal = vec3(0, 0, (ray.direction.z > 0 ? -1 : 1));
        rec.material = material;
        return true;
    }
};

class XZRectangle final : public Geometry {
public:
    float x0, x1, z0, z1, k;
    Material material;

    XZRectangle(float _x0, float _x1,
                float _z0, float _z1,
                float _k, const Material& m)
        : x0(_x0), x1(_x1), z0(_z0), z1(_z1), k(_k), material(m) {}

    bool intersect(const Ray& ray, HitRecord& rec) const override {
        if (fabs(ray.direction.y) < 1e-6) return false;
        float t = (k - ray.origin.y) / ray.direction.y;
        if (t < 0) return false;

        float x = ray.origin.x + t * ray.direction.x;
        float z = ray.origin.z + t * ray.direction.z;
        if (x < x0 || x > x1 || z < z0 || z > z1) return false;

        rec.t = t;
        rec.point = ray.origin + t * ray.direction;
        rec.normal = vec3(0, (ray.direction.y > 0 ? -1 : 1), 0);
        rec.material = material;
        return true;
    }
};

class YZRectangle final : public Geometry {
public:
    float y0, y1, z0, z1, k;
    Material material;

    YZRectangle(float _y0, float _y1,
                float _z0, float _z1,
                float _k, const Material& m)
        : y0(_y0), y1(_y1), z0(_z0), z1(_z1), k(_k), material(m) {}

    bool intersect(const Ray& ray, HitRecord& rec) const override {
        if (fabs(ray.direction.x) < 1e-6) return false;
        float t = (k - ray.origin.x) / ray.direction.x;
        if (t < 0) return false;

        float y = ray.origin.y + t * ray.direction.y;
        float z = ray.origin.z + t * ray.direction.z;
        if (y < y0 || y > y1 || z < z0 || z > z1) return false;

        rec.t = t;
        rec.point = ray.origin + t * ray.direction;
        rec.normal = vec3((ray.direction.x > 0 ? -1 : 1), 0, 0);
        rec.material = material;
        return true;
    }
};