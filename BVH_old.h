#include <vector>
#include "raytracer.h"

struct Vec3{
  float x,y,z;

  Vec3(float x=0, float y=0, float z=0): x(x), y(y), z(z) {}

  Vec3 operator-(Vec3 v) {
      return Vec3(x-v.x, y-v.y, z-v.z);
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
   // float min, max;

    AABB(Vec3 mn, Vec3 mx) : min(mn), max(mx) {}
    AABB() : min(Vec3(INFINITY,INFINITY,INFINITY)), max(Vec3(-INFINITY,-INFINITY,-INFINITY)) {}
    //AABB(float mn, float mx) : min(mn), max(mx) {}
    //AABB() : min(INFINITY), max(-INFINITY) {}
};

AABB calculateSphereAABB(Sphere s) {

    Vec3 min = Vec3(s.center.x - s.radius, s.center.y - s.radius, s.center.z - s.radius);
    Vec3 max = Vec3(s.center.x + s.radius, s.center.y + s.radius, s.center.z + s.radius);

    return AABB(min, max);
}


struct BVHNode {
    AABB aabb;
    // BVHNode* left; 
    // BVHNode* right;
    int left; 
    int right;
    //Sphere* sphere;
    uint first, count;
    std::vector<Sphere> spheres;

    // BVHNode(AABB& b, BVHNode* l, BVHNode* r, uint f, uint c) :
    //     aabb(b), 
    //     left(l), 
    //     right(r), 
    //     first(f),
    //     count(c) {}

    // BVHNode() :
    //     aabb(AABB()), 
    //     left(NULL), 
    //     right(NULL), 
    //     first(-1),
    //     count(-1) {}

    BVHNode(AABB& b, int l, int r, uint f, uint c) :
        aabb(b), 
        left(l), 
        right(r), 
        first(f),
        count(c) {}
    
    BVHNode() :
        aabb(AABB()), 
        left(-1), 
        right(-1), 
        first(-1),
        count(-1) {}

    void print() {
        cout << "Left: " << left << "\nRight: " << right << "\nFirst: " << first << "\nCount: " << count << endl;
    }
};