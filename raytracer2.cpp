#define cimg_use_png

#include "CImg/CImg.h"
#include <stdio.h>
#include <stdlib.h>
#include <sstream>

#include <png.h>
#include "BVH_old.h"


#define HIT_THRESH 0.0001
#define MAX_DEPTH 5

// png values
string name;
int width, height;

cimg_library::CImg<float> image;
cimg_library::CImg<float> texture;

// objects
std::vector<Sphere> spheres;
std::vector<Plane> planes;


std::vector<Light> lights;

Point eye = Point(0,0,0);
Dir forward = Dir(0,0,-1);
Dir right = Dir(1,0,0);
Dir up = Dir(0,1,0);

Dir global_up = Dir(0,1,0);

bool recalc = false;

float EXPOSE = -1;
bool FISHEYE = false;
bool PANORAMA = false;


Ray genRay(int x, int y, Dir f, Dir r, Dir u) {  // pixel (x,y), forward, right, up
    float sx = (2.0*x - width) / fmax(width, height);
    float sy = (height - 2.0*y) / fmax(width, height);

    if(FISHEYE) {
        float sx2 = pow(sx, 2);
        float sy2 = pow(sy, 2);
        if(sx2 + sy2 > 1) {
            return Ray();
        }
        f = sqrt(1 - sx2 - sy2) * f;
    }

    if(PANORAMA) {

        float theta = x * (2.0 * M_PI / width);
        float phi =  -y * (M_PI / height);

        Dir dir = f * (cos(theta) * sin(phi)) + u * cos(phi) + r * (sin(theta) * sin(phi));

        return Ray(eye, dir.normalized());
    }
    //cout << sx << sy << endl;

    Dir dir = f + sx * r + sy * u;

    return Ray(eye, dir.normalized());
}

Intersect raySphereIntersect(Ray ray, Sphere sphere){
    float r2 = pow(sphere.radius, 2);
    bool inside = (sphere.center - ray.origin).magnitudeSqr() < r2;
    float tc = dot((sphere.center - ray.origin), ray.dir) / ray.dir.magnitude();
    if (!inside && tc < 0) return Intersect();
    float d2 = (ray.origin + tc*ray.dir - sphere.center).magnitudeSqr();
    if(!inside && r2 < d2) return Intersect();
    float offset = sqrt(r2 - d2) / ray.dir.magnitude();

    float t = 0;
    if (inside) t = tc + offset;
    else t = tc - offset;

    Point c = t * ray.dir + ray.origin;
    //Dir norm = (c - sphere.center).normalized();
    return Intersect(c, t, sphere);

}

Intersect rayPlaneIntersect(Ray ray, Plane plane){
    float t = dot((plane.point - ray.origin), plane.norm) / dot(ray.dir, plane.norm);
    
    if(t < 0) return Intersect();

    Point p = t * ray.dir + ray.origin;
    return Intersect(p, t, plane);
}

//BVHNode* bvhNode;
std::vector<BVHNode> bvhNode;
//BVHNode bvhNode;
uint rootIdx = 0, nodesUsed = 1;

void initBVHRoot() {
    //BVHNode b[spheres.size() * 2 - 1];
    //bvhNode = b;
    //cout << bvhNode << endl;
    for (int i = 0; i < spheres.size() * 2 - 1; i++) {
        bvhNode.push_back(BVHNode());
    }
    bvhNode.at(rootIdx).spheres = spheres;
   // cout << "Here" << endl;
}

void calculateBounds(uint nodeIdx) {
    BVHNode node = bvhNode.at(nodeIdx);
    // cout << "Here 111" << endl;
    // node.aabb is constructor so AABB min and max should already be initialized to INF and -INF
    for (int i = 0; i < node.count; i++) {
        // cout << "Here 114: " << node.first + i << endl;
        Sphere sphere = spheres[node.first + i];
        AABB aabb = calculateSphereAABB(sphere);

        node.aabb.max = Vec3(fmax(aabb.max.x, node.aabb.max.x),fmax(aabb.max.y, node.aabb.max.y),fmax(aabb.max.z, node.aabb.max.z));
        node.aabb.min = Vec3(fmin(aabb.min.x, node.aabb.min.x),fmin(aabb.min.y, node.aabb.min.y),fmin(aabb.min.z, node.aabb.min.z));

    } 
    //bvhNode.at(nodeIdx) = node; 
    // cout << "Here 121" << endl;
}

void subdivide(uint nodeIdx) {
    BVHNode node = bvhNode.at(nodeIdx);
    // cout << "Here 127" << endl;
    if (node.count <= 2) return;

    // cout << "Here 128" << endl;

    Vec3 range = node.aabb.max - node.aabb.min;
    int axis = 0;
    if (range.y > range.x) axis = 1;
    if (range.z > range.getDim(axis)) axis = 2;

    float split = node.aabb.min.getDim(axis) + range.getDim(axis) * 0.5f;
   // cout << "Split: " << split << endl;
   // cout << "Axis: " << axis << endl;

    // sort objects
    int i = node.first;
    int j = i + node.count - 1;
    while (i <= j) {
        if (spheres[i].center.getDim(axis) < split) i++;
        else {
          //  cout << "Swapping...\n" << string(spheres[i].center) << endl;
          //  cout << string(spheres[j].center) << endl;
            Sphere temp = spheres[i];
            spheres[i] = spheres[j];
            //cout << "j Before: " << j << endl;
            spheres[j] = temp;

            //cout << "j After: " << j << endl;
            j--;
        }
    }

    for (int i = node.first; i < node.first + node.count; i++) {
        if (spheres[i].center.getDim(axis) < split) i++;
        else {
          //  cout << "Swapping...\n" << string(spheres[i].center) << endl;
          //  cout << string(spheres[j].center) << endl;
            Sphere temp = spheres[i];
            spheres[i] = spheres[j];
            //cout << "j Before: " << j << endl;
            spheres[j] = temp;

            //cout << "j After: " << j << endl;
            j--;
        }
    }

    // cout << "Here 152" << endl;

    // split into child nodes
    int lCount = i - node.first;
    bvhNode.at(nodeIdx) = node; 
   // cout << "Here 164" << endl;
    if(lCount == 0 || lCount == node.count) return;
  //  cout << "Here 166" << endl;
    int lIdx = nodesUsed++;
    int rIdx = nodesUsed++;
    bvhNode.at(lIdx).first = node.first;
    bvhNode.at(lIdx).count = lCount;
    bvhNode.at(rIdx).first = i;
    bvhNode.at(rIdx).count = node.count - lCount;
    node.left = lIdx;
    // cout << "Left: " << node.left << endl;
    node.count = 0;
    // cout << "Here 166" << endl;
    bvhNode.at(nodeIdx) = node; 
    calculateBounds(lIdx);
    calculateBounds(rIdx);

    subdivide(lIdx);
    subdivide(rIdx);

}

void buildBVH() {
    // cout << "Here 168" << endl;
   // BVHNode& root = bvhNode.at(rootIdx);
    BVHNode root;
   // root.print();
    root.left = 0;
    root.right = 0;
    root.first = 0;
    root.count = spheres.size();
   // root.print();
    bvhNode.at(rootIdx) = root;
    calculateBounds(rootIdx);
    subdivide(rootIdx);
}


bool rayAABBIntersect( Ray ray, Vec3 min, Vec3 max ) {
    // Vec3 origin = Vec3(ray.origin.x, ray.origin.y, ray.origin.z);
    // Vec3 rayDir = Vec3(ray.dir.x, ray.dir.y, ray.dir.z);
    // Vec3 t1 = (min - origin) / rayDir;
    // Vec3 t2 = (max - origin) / rayDir;

    float tx1 = (min.x - ray.origin.x) / ray.dir.x;
    float tx2 = (max.x - ray.origin.x) / ray.dir.x;
    float tmin = fmin( tx1, tx2 );
    float tmax = fmax( tx1, tx2 );
    float ty1 = (min.y - ray.origin.y) / ray.dir.y;
    float ty2 = (max.y - ray.origin.y) / ray.dir.y;
    tmin = fmax( tmin, fmin( ty1, ty2 ) );
    tmax = fmin( tmax, fmax( ty1, ty2 ) );
    float tz1 = (min.z - ray.origin.z) / ray.dir.z;
    float tz2 = (max.z - ray.origin.z) / ray.dir.z;
    tmin = fmax( tmin, fmin( tz1, tz2 ) );
    tmax = fmin( tmax, fmax( tz1, tz2 ) );
    return tmax >= tmin && tmin < INFINITY && tmax > 0;
}

Intersect rayBVHIntersect( Ray ray, const uint nodeIdx ) {
    BVHNode node = bvhNode.at(nodeIdx);
   // cout << "Here 216: " << endl;
    
    if (!rayAABBIntersect( ray, node.aabb.min, node.aabb.max )) return Intersect();
   // node.print();
   // cout << "Here 218" << endl;
  //  cout << "Node: " << nodeIdx << endl;
    if (node.count > 0) {
       // cout << "Leaf node!" << endl;
        float min_t = INFINITY;
        Intersect nearest = Intersect();
        
        for (uint i = 0; i < node.count; i++ ) {
            //cout << string(spheres[node.first + i]) << endl;
            // cout << "Here 223: " << node.first + i << endl;
            Intersect intersect = raySphereIntersect( ray, spheres[node.first + i] );
            // cout << "Here 225" << endl;
            if(intersect.dist > 0 && intersect.dist < min_t) {
                min_t = intersect.dist;
                nearest = intersect;
            }
        }
        // cout << "Nearest: " << string(nearest) << endl;
        return nearest;
    }
    else {
      //  cout << "Branch node!" << endl;
      //  cout << "Left node index: " << node.left << endl;
        Intersect leftIntersect = Intersect();
        Intersect rightIntersect = Intersect();

        leftIntersect = rayBVHIntersect( ray, node.left );
        rightIntersect = rayBVHIntersect( ray, node.left + 1 );

        if (leftIntersect.dist > 0 && rightIntersect.dist <= 0) {
            return leftIntersect;
        } else if (leftIntersect.dist <= 0 && rightIntersect.dist > 0) {
            return rightIntersect;
        } else if (leftIntersect.dist > 0 && rightIntersect.dist > 0) {
            if(leftIntersect.dist < rightIntersect.dist) return leftIntersect;
            else return rightIntersect;
        }

        return Intersect();
    }
    return Intersect();
}



float sRGBToLinear(float col) {

    if (col <= 0.04045f) {
        return col / 12.92f;
    } else {
        return pow((col + 0.055f)/1.055f, 2.4f);
    }

    return col;
}

float linearTosRGB(float col) {

    if (col <= 0.0031308f) {
        return 12.92f * col;
    } else {
        return 1.055f*pow(col, 1/2.4f) - 0.055f;
    }

}

void parseObjects(char *f) {
    // Read file and store objects
    std::ifstream file(f);
    string data;

    Color currColor;
    // cout << "Current " << string(currColor) << endl;

    if(file.is_open()) {
        string str;
        while(getline(file, str)) {
            if(str.empty() || str == "") {
                continue;
            }
            std::istringstream line(str);
            line >> data;
            
            if(data.compare("png") == 0) {
                line >> width >> height >> name;
                image.assign(width, height, 1, 4);
            }
            else if(data.compare("color") == 0) {
                float r, g, b;
                while (!line.eof()) {
                    line >> r >> g >> b;
                    currColor = Color(r,g,b);
                }
                // cout << "Current " << string(currColor) << endl;
            } else if(data.compare("sun") == 0) {
                float x, y, z;
                while (!line.eof()) {
                    line >> x >> y >> z;
                    lights.push_back(Light(currColor, Dir(x,y,z).normalized()));
                }
               // cout << "Current " << string(sun) << endl;
            } else if(data.compare("sphere") == 0) {
                float x, y, z, r;
                Sphere sphere;
                while (!line.eof()) {
                    line >> x >> y >> z >> r;
                    Point center(x,y,z);
                    sphere = Sphere(center, r, currColor);
                    spheres.push_back(sphere);
                }
               // cout << string(sphere) << endl;
            } else if(data.compare("plane") == 0) {
                float a, b, c, d;
                Plane plane;
                while (!line.eof()) {
                    line >> a >> b >> c >> d;
                    plane = Plane(a,b,c,d);
                    planes.push_back(plane);
                }
                // cout << string(plane) << endl;
            }
            else if(data.compare("eye") == 0) {
                float x, y, z;
                line >> x >> y >> z;
                eye = Point(x,y,z);
            } 
            else if(data.compare("forward") == 0) {
                float x, y, z;
                line >> x >> y >> z;
                forward = Dir(x,y,z);

                recalc = true;
            }
            else if(data.compare("up") == 0) {
                float x, y, z;
                line >> x >> y >> z;
                up = Dir(x,y,z).normalized();

                //calc_right = true;
            }
            else if(data.compare("expose") == 0) line >> EXPOSE;
            else if(data.compare("fisheye") == 0) FISHEYE = true;
            else if(data.compare("panorama") == 0) PANORAMA = true;

        }
    } else {
        cout << "Couldn't open file\n";
        file.close();
    }

    file.close();
}

void printStats(Ray ray, Intersect intersect, Light sun, float dotty, Color linear, Color sRGB) {
    cout << string(ray) << endl;
    cout << string(intersect) << endl;
    cout << "Normal: " << string(intersect.norm) << endl;
    cout << "Sun " << string(sun.dir) << endl;
    cout << "Dot: " << dotty << endl;
    cout << "Linear " << string(linear) << endl;
    cout << "sRGB " << string(sRGB) << endl;

}

bool isShadow(Point hit, Dir lightdir) {
    Ray shadowRay = Ray(hit, lightdir);
    bool isShadow = false;
    for(int j = 0; j < spheres.size(); j++) {
        Intersect intersect = raySphereIntersect(shadowRay, spheres[j]);
        if(intersect.dist > HIT_THRESH) {
            return true;
        }
    }
    return false;
}

void setPixelColor(float x, float y, Ray ray, Intersect nearest) {
    float r = 0, g = 0, b = 0;
    //Color c;

    for (int i = 0; i < lights.size(); i++) {
        Light light = lights[i];
        float dotty = dot(nearest.norm, light.dir);
        //cout << "Dot: " << dotty << endl;
        //color = nearest.sphere.col * light.color 
        
        if(!isShadow(nearest.point, light.dir)) {
            r += nearest.sphere.col.r * light.color.r * dotty; 
            g += nearest.sphere.col.g * light.color.g * dotty;
            b += nearest.sphere.col.b * light.color.b * dotty;
        }
    }

    if(EXPOSE > 0) {
        r = 1 - exp(-EXPOSE * r);
        g = 1 - exp(-EXPOSE * g);
        b = 1 - exp(-EXPOSE * b);
    }

    r = fclamp(r, 0, 1);
    g = fclamp(g, 0, 1);
    b = fclamp(b, 0, 1);

    r = linearTosRGB(r);
    g = linearTosRGB(g);
    b = linearTosRGB(b);

    // if(x == 55 && y == 45) {
    //     printStats(ray, nearest, lights[0], dot(nearest.norm, lights[0].dir), Color(sRGBToLinear(r),sRGBToLinear(g),sRGBToLinear(b)), Color(r*255,g*255,b*255));
    // }

    image(x, y, 0, 0) = r  * 255.f;   //r
    image(x, y, 0, 1) = g  * 255.f;    //g
    image(x, y, 0, 2) = b  * 255.f;   //b

    image(x, y, 0, 3) = 255.f;  //a 
}

int main(int argc, char *argv[]){
    // Read file and store objects
    parseObjects(argv[1]);
    initBVHRoot();
    buildBVH();

    if (recalc) {
        right = cross(global_up, forward).normalized();
        up = cross(right, forward).normalized();
    }
    
    for(int x = 0; x < width; x++) {
        for(int y = 0; y < height; y++) {
            Ray ray = genRay(x, y, forward, right, up);
            if(ray.dir.magnitude() == 0) continue;
          //  cout << "Light " << string(ray) << endl;
           // float min_t = INFINITY;
           // Intersect nearest = Intersect();
            // for(int i = 0; i < spheres.size(); i++) {
            //     Intersect intersect = raySphereIntersect(ray, spheres[i]);
            //     if(intersect.dist > 0 && intersect.dist < min_t) {
            //         min_t = intersect.dist;
            //         nearest = intersect;
            //     }
            // }
            
            Intersect nearest = rayBVHIntersect(ray, rootIdx);

            // for(int i = 0; i < planes.size(); i++) {
            //     Intersect intersect = rayPlaneIntersect(ray, planes[i]);
            //     if(intersect.dist > 0 && intersect.dist < min_t) {
            //         min_t = intersect.dist;
            //         nearest = intersect;
            //     }
            // }
            //cout << "Nearest " << string(nearest) << endl;
            if(nearest.dist > 0) {  // intersection found
                setPixelColor(x, y, ray, nearest);
            }
        }
    }
    image.save_png(name.c_str());
} 

