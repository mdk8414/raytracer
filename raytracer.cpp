#define cimg_use_png

#include "CImg/CImg.h"
#include <stdio.h>
#include <stdlib.h>
#include <sstream>

#include <png.h>
#include "BVH.h"


#define HIT_THRESH 0.0001

// png values
string name;
int width, height;

cimg_library::CImg<float> image;
cimg_library::CImg<float> texture;

// objects
std::vector<Sphere> spheres;
std::vector<Plane> planes;
std::vector<Triangle> triangles;
std::vector<Vertex> vertices;

std::vector<int> triIndices;
std::vector<Mat> triMats;


std::vector<Light> lights;

Point eye = Point(0,0,0);
Dir forward = Dir(0,0,-1);
Dir right = Dir(1,0,0);
Dir up = Dir(0,1,0);

Dir global_up = Dir(0,1,0);

bool recalc = false;

bool useBVH = true;

float EXPOSE = -1;
bool FISHEYE = false;
bool PANORAMA = false;
int MAX_DEPTH = -1;


// DEBUG tools
auto start = std::chrono::high_resolution_clock::now();
long long avgBvhDuration = 0;



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

    Dir dir = f + sx * r + sy * u;

    return Ray(eye, dir.normalized());
}


BVHNode bvhRoot;
uint rootIdx = 0, nodesUsed = 1;

void initBVHRoot() {

    bvhRoot.indices = new int[spheres.size()];
    for(int i = 0; i < spheres.size(); i++)
        bvhRoot.indices[i] = i; 
}

void calculateBounds(BVHNode* node) {

    // node.aabb is constructor so AABB min and max should already be initialized to INF and -INF
    for (int i = 0; i < node->count; i++) {
        Sphere sphere = spheres[node->indices[i]];
        AABB aabb = calculateSphereAABB(sphere);

        node->aabb.max = Vec3(fmax(aabb.max.x, node->aabb.max.x),fmax(aabb.max.y, node->aabb.max.y),fmax(aabb.max.z, node->aabb.max.z));
        node->aabb.min = Vec3(fmin(aabb.min.x, node->aabb.min.x),fmin(aabb.min.y, node->aabb.min.y),fmin(aabb.min.z, node->aabb.min.z));

    } 

}

void subdivide(BVHNode* node, int left) {

    if (node->count <= 2) return;

    Vec3 range = node->aabb.max - node->aabb.min;
    int axis = 0;
    if (range.y > range.x) axis = 1;
    if (range.z > range.getDim(axis)) axis = 2;

    float split = node->aabb.min.getDim(axis) + range.getDim(axis) * 0.5f;

    int* leftSpheres = new int[node->count];
    int* rightSpheres = new int[node->count];
    
    int lIndex = 0;
    int rIndex = 0;

    for (int i = 0; i < node->count; i++) {
        Sphere sphere = spheres[node->indices[i]];
        if (sphere.center.getDim(axis) < split) {
            leftSpheres[lIndex] = node->indices[i];
            lIndex++;
        } else {
            rightSpheres[rIndex] = node->indices[i];
            rIndex++;
        }
    }

   if (lIndex == 0 || rIndex == 0) {
        return;
    }

    node->left = new BVHNode();
    node->right = new BVHNode();
    
    BVHNode* leftNode = node->left;
    BVHNode* rightNode = node->right;

    leftNode->indices = leftSpheres;
    rightNode->indices = rightSpheres;

    leftNode->count = lIndex;
    rightNode->count = rIndex;

    delete[] node->indices;
   
    node->count = 0;


    calculateBounds(leftNode);
    calculateBounds(rightNode);

    subdivide(leftNode, 1);
    subdivide(rightNode, 0);

}

void buildBVH() {
    bvhRoot.count = spheres.size();
    calculateBounds(&bvhRoot);
    subdivide(&bvhRoot,1);
}

int node_count = 0;
void printBVH(BVHNode* node, int depth=0) {
    node_count++;
    cout << depth << endl;
    node->print();
    if(node->count > 0) return;
    cout << "Left ";
    printBVH(node->left, depth+1);
    cout << "Right ";
    printBVH(node->right, depth+1);
    cout << "Total nodes: " << node_count << endl;
}


bool rayAABBIntersect( Point rayOrigin, Dir rayInvDir, Vec3 min, Vec3 max ) {

    float tx1 = (min.x - rayOrigin.x) * rayInvDir.x;
    float tx2 = (max.x - rayOrigin.x) * rayInvDir.x;
    float tmin = fmin( tx1, tx2 );
    float tmax = fmax( tx1, tx2 );
    float ty1 = (min.y - rayOrigin.y) * rayInvDir.y;
    float ty2 = (max.y - rayOrigin.y) * rayInvDir.y;
    tmin = fmax( tmin, fmin( ty1, ty2 ) );
    tmax = fmin( tmax, fmax( ty1, ty2 ) );
    float tz1 = (min.z - rayOrigin.z) * rayInvDir.z;
    float tz2 = (max.z - rayOrigin.z) * rayInvDir.z;
    tmin = fmax( tmin, fmin( tz1, tz2 ) );
    tmax = fmin( tmax, fmax( tz1, tz2 ) );

    return tmax > 0 && tmax > tmin && tmin < INFINITY;
}


int iters = 0;

SphereIntersect raySphereIntersect(Point rayOrigin, Dir rayDir, Sphere sphere){
    float r2 = pow(sphere.radius, 2);
    bool inside = (sphere.center - rayOrigin).magnitudeSqr() < r2;
    float t = 0;
    if(!inside) {
        float tc = dot((sphere.center - rayOrigin), rayDir) / rayDir.magnitude();
        if (tc < 0) return SphereIntersect();
        float d2 = (rayOrigin + tc*rayDir - sphere.center).magnitudeSqr();
        if(r2 < d2) return SphereIntersect();
        float offset = sqrt(r2 - d2) / rayDir.magnitude();
        t = tc - offset;
    } else {
        float tc = dot((sphere.center - rayOrigin), rayDir) / rayDir.magnitude();
        float d2 = (rayOrigin + tc*rayDir - sphere.center).magnitudeSqr();
        float offset = sqrt(r2 - d2) / rayDir.magnitude();
        t = tc + offset;
    }


    return SphereIntersect(t * rayDir + rayOrigin, t, sphere);
}

bool raySphereShadowIntersect(Point rayOrigin, Dir rayDir, Sphere sphere, Point hit, Point lightPos){
    float r2 = pow(sphere.radius, 2);
    bool inside = (sphere.center - rayOrigin).magnitudeSqr() < r2;
    float t = 0;
    if(!inside) {
        float tc = dot((sphere.center - rayOrigin), rayDir) / rayDir.magnitude();
        if (tc < 0) return false;
        float d2 = (rayOrigin + tc*rayDir - sphere.center).magnitudeSqr();
        if(r2 < d2) return false;
        float offset = sqrt(r2 - d2) / rayDir.magnitude();
        t = tc - offset;
    } else {
        float tc = dot((sphere.center - rayOrigin), rayDir) / rayDir.magnitude();
        float d2 = (rayOrigin + tc*rayDir - sphere.center).magnitudeSqr();
        float offset = sqrt(r2 - d2) / rayDir.magnitude();
        t = tc + offset;
    }

    if(t > HIT_THRESH && t < (lightPos - hit).magnitude()) return true;

    return false;

}

SphereIntersect rayBVHIntersect( BVHNode* node, Point rayOrigin, Dir rayDir, Dir rayInvDir ) {

    if (!rayAABBIntersect( rayOrigin, rayInvDir, node->aabb.min, node->aabb.max )) return SphereIntersect();

    if (node->count > 0) {
        float min_t = INFINITY;
        SphereIntersect nearest;
        
        for (int i = 0; i < node->count; i++ ) {
          //  iters++;

            SphereIntersect intersect = raySphereIntersect( rayOrigin, rayDir, spheres[node->indices[i]] );
            //intersect.mat = spheres[node->indices[i]].mat;

            if(intersect.dist < min_t) {
                min_t = intersect.dist;
                nearest = intersect;
            }
        }

        return nearest;
    }

    SphereIntersect leftIntersect;
    SphereIntersect rightIntersect;

    if(node->left != NULL && node->left->count >= 0) {
      //  iters++;
        leftIntersect = rayBVHIntersect(node->left, rayOrigin, rayDir, rayInvDir);
    }

    if(node->right != NULL && node->right->count >= 0) {
      //  iters++;
        rightIntersect = rayBVHIntersect(node->right, rayOrigin, rayDir, rayInvDir);
    } 

    
    if(leftIntersect.dist < rightIntersect.dist) return leftIntersect;

    return rightIntersect;

}

bool shadowRayBVHIntersect( BVHNode* node, Point rayOrigin, Dir rayDir, Dir rayInvDir, Point hit, Point lightPos ) {

    if (!rayAABBIntersect( rayOrigin, rayInvDir, node->aabb.min, node->aabb.max )) return false;

    if (node->count > 0) {
        for (int i = 0; i < node->count; i++ ) {

            if (raySphereShadowIntersect(rayOrigin, rayDir, spheres[node->indices[i]], hit, lightPos)) 
                return true;
        }
        return false;
    }

    bool leftIntersect = false;
    bool rightIntersect = false;

    if(node->left != NULL && node->left->count >= 0) {
        leftIntersect = shadowRayBVHIntersect(node->left, rayOrigin, rayDir, rayInvDir, hit, lightPos);
    }

    if (leftIntersect) return true;

    if(node->right != NULL && node->right->count >= 0) {
        rightIntersect = shadowRayBVHIntersect(node->right, rayOrigin, rayDir, rayInvDir, hit, lightPos);
    }

    return rightIntersect;

}

Intersect rayPlaneIntersect(Ray ray, Plane plane){
    float t = dot((plane.point - ray.origin), plane.norm) / dot(ray.dir, plane.norm);
    
    if(t < 0) return Intersect();

    Point p = t * ray.dir + ray.origin;
    return PlaneIntersect(p, t, plane);
}

TriangleIntersect rayTriangleIntersect(Ray ray, Triangle triangle) {

    Point v1 = triangle.v1.point;
    Point v2 = triangle.v2.point;
    Point v3 = triangle.v3.point;

    Dir e1 = v2 - v1;
    Dir e2 = v3 - v1;

    Dir pvec = cross(ray.dir, e2);
    float det = dot(e1, pvec);

    // parallel check
    if (std::fabs(det) < HIT_THRESH) return TriangleIntersect();

    float inv = 1.f / det;
    Dir s = ray.origin - v1;
    float u = inv * dot(s, pvec);

    // outside triangle check
    if (u < 0.f || u > 1.f) return TriangleIntersect();

    Dir q = cross(s, e1);
    float v = inv * dot(ray.dir, q);

    // outside triangle check
    if (v < 0.f || u + v > 1.f) return TriangleIntersect();

    float t = inv * dot(e2, q);

    if(t > HIT_THRESH) {
        Dir triNorm = cross(e1, e2).normalized();
        if (dot(ray.dir, triNorm) > 0) triNorm = -1 * triNorm;
        return TriangleIntersect(ray.origin + t*ray.dir, t, triangle, triNorm);
    }
    
    return TriangleIntersect();
}

Intersect findNearestSphere(Ray ray) {
    Intersect nearest;
    float min_t = INFINITY;
    for(int i = 0; i < spheres.size(); i++) {
        Intersect intersect = raySphereIntersect(ray.origin, ray.dir, spheres[i]);

        if(intersect.dist < min_t) {
            min_t = intersect.dist;
            nearest = intersect;
        }
    }
    return nearest;
}

Intersect findNearest(Ray ray) {
                        
    Intersect nearest = rayBVHIntersect(&bvhRoot, ray.origin, ray.dir, ray.invDir);

    float min_t = nearest.dist;  

    for(int i = 0; i < planes.size(); i++) {
        Intersect intersect = rayPlaneIntersect(ray, planes[i]);
        if(intersect.dist < min_t) {
            min_t = intersect.dist;
            nearest = intersect;
        }
    }

    for(int i = 0; i < triangles.size(); i++) {
        Intersect intersect = rayTriangleIntersect(ray, triangles[i]);
        if(intersect.dist < min_t) {
            min_t = intersect.dist;
            nearest = intersect;
        }
    }

    return nearest;
}

std::vector<Triangle> initTriangles() {
    std::vector<Triangle> triangles;
    for(int i = 0; i < int(triIndices.size()) - 2; i += 3) {
        int i1 = triIndices[i];
        int i2 = triIndices[i+1];
        int i3 = triIndices[i+2];
        if(i1 >= 0) i1--;
        else i1 = vertices.size() + i1;
        if(i2 >= 0) i2--;
        else i2 = vertices.size() + i2;
        if(i3 >= 0) i3--;
        else i3 = vertices.size() + i3;

        Triangle triangle = Triangle(vertices[i1], vertices[i2], vertices[i3]);
       // triangle.print();
        triangles.push_back(triangle);
    }

    for (int i = 0; i < triMats.size(); i++) {
        triangles.at(i).mat = triMats.at(i);
    }

    return triangles;
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

    Mat currMat;


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
                    currMat.color = Color(r,g,b);
                }
            } else if(data.compare("shininess") == 0) {
                if (MAX_DEPTH == -1) MAX_DEPTH = 4;

                float sr, sg, sb=-1;
                line >> sr;
                while (!line.eof()) {
                    line >> sg >> sb;
                }

                if (sb == -1) currMat.shine = Color(sr,sr,sr);
                else currMat.shine = Color(sr,sg,sb);
            } else if(data.compare("transparency") == 0) {
                if (MAX_DEPTH == -1) MAX_DEPTH = 4;

                float tr, tg, tb=-1;
                line >> tr;
                while (!line.eof()) {
                    line >> tg >> tb;
                }

                if (tb == -1) currMat.trans = Color(tr,tr,tr);
                else currMat.trans = Color(tr,tg,tb);
            } else if(data.compare("sun") == 0) {
                float x, y, z;
                while (!line.eof()) {
                    line >> x >> y >> z;
                    lights.push_back(Light(currMat.color, Dir(x,y,z).normalized()));
                }
            } else if(data.compare("bulb") == 0) {
                float x, y, z;
                while (!line.eof()) {
                    line >> x >> y >> z;
                    lights.push_back(Light(currMat.color, Dir(), Point(x,y,z)));
                }
            } else if(data.compare("sphere") == 0) {
                float x, y, z, r;
                while (!line.eof()) {
                    line >> x >> y >> z >> r;
                    Point center(x,y,z);
                    spheres.push_back(Sphere(center, r, currMat));
                }
            } else if(data.compare("plane") == 0) {
                float a, b, c, d;
                Plane plane;
                while (!line.eof()) {
                    line >> a >> b >> c >> d;
                    plane = Plane(a,b,c,d);
                    planes.push_back(plane);
                }
            } else if(data.compare("eye") == 0) {
                float x, y, z;
                line >> x >> y >> z;
                eye = Point(x,y,z);
            } else if(data.compare("forward") == 0) {
                float x, y, z;
                line >> x >> y >> z;
                forward = Dir(x,y,z);

                recalc = true;
            } else if(data.compare("up") == 0) {
                float x, y, z;
                line >> x >> y >> z;
                up = Dir(x,y,z).normalized();

            } else if(data.compare("xyz") == 0) {
                float x, y, z;
                line >> x >> y >> z;
                vertices.push_back(Vertex(Point(x,y,z)));
            } else if(data.compare("tri") == 0) {
                int i1, i2, i3;
                line >> i1 >> i2 >> i3;
                
                triIndices.push_back(i1);
                triIndices.push_back(i2);
                triIndices.push_back(i3);

                triMats.push_back(currMat);
            }
            else if(data.compare("expose") == 0) line >> EXPOSE;
            else if(data.compare("fisheye") == 0) FISHEYE = true;
            else if(data.compare("panorama") == 0) PANORAMA = true;
            else if(data.compare("bounces") == 0) line >> MAX_DEPTH;
            else if(data.compare("ior") == 0) line >> currMat.ior;

        }
    } else {
        cout << "Couldn't open file\n";
        file.close();
    }

    file.close();
}

// void printStats(Ray ray, Intersect intersect, Light sun, float dotty, Color linear, Color sRGB) {
//     cout << string(ray) << endl;
//     cout << string(intersect) << endl;
//     cout << "Normal: " << string(intersect.norm) << endl;
//     cout << "Sun " << string(sun.dir) << endl;
//     cout << "Dot: " << dotty << endl;
//     cout << "Linear " << string(linear) << endl;
//     cout << "sRGB " << string(sRGB) << endl;

// }

void printLights() {
    for (int i = 0; i < lights.size(); i++) {
        lights[i].print();
    }
}

bool isShadow(Point hit, Dir lightdir, Point lightPos) {
    Ray shadowRay = Ray(hit + 0.0001*lightdir, lightdir);
    bool isShadow = false;

    Intersect intersect;
    isShadow = shadowRayBVHIntersect(&bvhRoot, shadowRay.origin, shadowRay.dir, shadowRay.invDir, hit, lightPos);

    // else {
    //     for(int j = 0; j < spheres.size(); j++) {

    //         Intersect intersect = raySphereIntersect(shadowRay.origin, shadowRay.dir, spheres[j]);
    //         //intersect.mat = spheres[j].mat;
    //         if(intersect.dist > HIT_THRESH && intersect.dist < (lightPos - hit).magnitude()) {
    //             return true;
    //         }
    //     }
    // }

    for(int j = 0; j < triangles.size(); j++) {
        Intersect intersect = rayTriangleIntersect(shadowRay, triangles[j]);
        if(intersect.dist > HIT_THRESH && intersect.dist < (lightPos - hit).magnitude()) {
            return true;
        }
    }

    return isShadow;
}

float sign(float x){return (x > 0) ? 1 : (x < 0) ? -1 : 0; }

Dir refractRay(Dir rayDir, Dir reflectedRayDir, Dir norm, float nu) {
    Dir n = norm;
    float a = dot(rayDir, n);
    float index;
    if(a < 0) {
        index = 1.f / nu;
    } else {
        n = -1.f*n;
        index = nu;
    }

    float root = 1.f - index*index + pow(dot(n, rayDir), 2)*index*index;

    if(root < 0){
        return reflectedRayDir.normalized();
    } 
    
    Dir u = (sign(dot(n, rayDir)) * sqrt(root) - dot(n, rayDir)*index)*n + index*rayDir;

    return u.normalized();

}


Color calculateColor(Intersect nearest, Ray ray, int depth) {
    Color newColor = Color(0,0,0);

    if(nearest.dist < INFINITY) {

        for (int i = 0; i < lights.size(); i++) {
            float dropoff = 1;
            Light light = lights[i];

            if(lights[i].dir.magnitude() == 0.f) {
                Dir lightDir = light.pos - nearest.point;
                dropoff = 1.f / lightDir.magnitudeSqr();
                light.dir = lightDir.normalized();
            }
            
            if(!isShadow(nearest.point, light.dir, light.pos)) {
                float dotty = fclamp(dot(nearest.norm, light.dir), 0, 1);
                
                // diffuse

               newColor.r += (1.f - nearest.mat.shine.r) * (1.f - nearest.mat.trans.r) * nearest.mat.color.r * lights[i].color.r * dotty * dropoff; 
               newColor.g += (1.f - nearest.mat.shine.g) * (1.f - nearest.mat.trans.g) * nearest.mat.color.g * lights[i].color.g * dotty * dropoff;
               newColor.b += (1.f - nearest.mat.shine.b) * (1.f - nearest.mat.trans.b) * nearest.mat.color.b * lights[i].color.b * dotty * dropoff;

            }
        }


        if (depth <= MAX_DEPTH) {
            int newDepth = depth+1;

            // SHININESS
            Dir reflectedRayDir = ray.dir - 2*dot(nearest.norm, ray.dir) * nearest.norm;
            Point reflectedRayOrigin = nearest.point + 0.0001*reflectedRayDir;
            Ray reflectedRay = Ray(reflectedRayOrigin, reflectedRayDir.normalized());
            Intersect nearestReflect = findNearest(reflectedRay);


            newColor =  newColor + nearest.mat.shine * calculateColor(nearestReflect, reflectedRay, newDepth);

            // TRANSPARENCY

            Dir refractedRayDir = refractRay(ray.dir, reflectedRayDir, nearest.norm, nearest.mat.ior);
            Point refractedRayOrigin = nearest.point + 0.0001*refractedRayDir;
            Ray refractedRay = Ray(refractedRayOrigin, refractedRayDir.normalized());
            Intersect nearestRefract = findNearest(refractedRay);

            newColor =  newColor + (Color() - nearest.mat.shine) * nearest.mat.trans * calculateColor(nearestRefract, refractedRay, newDepth);

        }
    }

    return newColor;
}

void setPixelColor(float x, float y, Ray ray, Intersect nearest) {

    Color color = calculateColor(nearest, ray, 0);

    float r = color.r;
    float g = color.g;
    float b = color.b;

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

    start = std::chrono::high_resolution_clock::now();

    initBVHRoot();
    buildBVH();

    if (recalc) {
        right = cross(global_up, forward).normalized();
        up = cross(right, forward).normalized();
    }

    triangles = initTriangles();

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    cout << "Setup Phase: " << duration.count() / 1000.f / 1000.f << " seconds." << endl;
    
    for(int x = 0; x < width; x++) {
        for(int y = 0; y < height; y++) {
            Ray primaryRay = genRay(x, y, forward, right, up);
           // if(primaryRay.dir.magnitude() == 0) continue;

            Intersect nearest = findNearest(primaryRay);

            if(nearest.dist < INFINITY) {  // intersection found
                setPixelColor(x, y, primaryRay, nearest);
            }
        }
    }

    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    cout << "Program execution time: " << duration.count() / 1000.f / 1000.f << " seconds." << endl;
    //cout << "# Iterations: " << iters << endl;
    // cout << "Average Ray-BVH duration: " << avgBvhDuration / float(width*height) << endl;
    image.save_png(name.c_str());
} 

