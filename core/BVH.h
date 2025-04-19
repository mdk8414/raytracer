#include <vector>
#include "raytracer.h"

struct Vec3{
    float x,y,z;

    Vec3(float x=0, float y=0, float z=0): x(x), y(y), z(z) {}

    Vec3(Point p) : x(p.x), y(p.y), z(p.z) {}
    Vec3(Dir p) : x(p.x), y(p.y), z(p.z) {}

    Vec3 operator-(Vec3 v) {
        return Vec3(x-v.x, y-v.y, z-v.z);
    };

    Vec3 operator*(Vec3 v) {
        return Vec3(x*v.x, y*v.y, z*v.z);
    };

    Vec3 operator/(Vec3 v) {
        return Vec3(x/v.x, y/v.y, z/v.z);
    };

    float getDim(int d) {
        if(d == 0) {
            return x;
        } else if (d == 1) {
            return y;
        } else {
            return z;
        }
    }
  
};

struct AABB {
    Vec3 min, max;

    AABB(Vec3 mn, Vec3 mx) : min(mn), max(mx) {}
    AABB() : min(Vec3(INFINITY,INFINITY,INFINITY)), max(Vec3(-INFINITY,-INFINITY,-INFINITY)) {}

};

AABB calculateSphereAABB(Sphere s) {

    Vec3 min = Vec3(s.center.x - s.radius, s.center.y - s.radius, s.center.z - s.radius);
    Vec3 max = Vec3(s.center.x + s.radius, s.center.y + s.radius, s.center.z + s.radius);

    return AABB(min, max);
}


struct BVHNode {
    AABB aabb;
    BVHNode* left; 
    BVHNode* right;

    int count;
    int* indices;


    BVHNode() :
        aabb(AABB()), 
        left(NULL), 
        right(NULL), 
        count(-1) {}

    void print() {
        cout << "Count: " << count << endl;
    }
};

