#define cimg_use_png

#include "CImg/CImg.h"
#include <stdio.h>
#include <stdlib.h>
#include <sstream>

#include <png.h>
#include "BVH.h"


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
std::vector<Triangle> triangles;
std::vector<Vertex> vertices;

std::vector<int> triIndices;
std::vector<Color> triColors;


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


// DEBUG tools
auto start = std::chrono::high_resolution_clock::now();


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

//BVHNode* bvhNode;
//std::vector<BVHNode> bvhNode;
BVHNode bvhRoot;
uint rootIdx = 0, nodesUsed = 1;

void initBVHRoot() {
    //BVHNode b[spheres.size() * 2 - 1];
    //bvhNode = b;
    //cout << bvhNode << endl;
    bvhRoot.spheres = spheres;
}

void calculateBounds(BVHNode* node) {

    // node.aabb is constructor so AABB min and max should already be initialized to INF and -INF
    for (int i = 0; i < node->spheres.size(); i++) {
       //  cout << "Here 114" << endl;
        Sphere sphere = node->spheres.at(i);
        AABB aabb = calculateSphereAABB(sphere);

        node->aabb.max = Vec3(fmax(aabb.max.x, node->aabb.max.x),fmax(aabb.max.y, node->aabb.max.y),fmax(aabb.max.z, node->aabb.max.z));
        node->aabb.min = Vec3(fmin(aabb.min.x, node->aabb.min.x),fmin(aabb.min.y, node->aabb.min.y),fmin(aabb.min.z, node->aabb.min.z));

    } 

}

void subdivide(BVHNode* node, int left) {
   // if(left) cout << "Left Side!" << endl;
   // else cout << "Right Side!" << endl;
   // cout << "Count: " << node->count << endl;
    if (node->count <= 4) return;

    Vec3 range = node->aabb.max - node->aabb.min;
    int axis = 0;
    if (range.y > range.x) axis = 1;
    if (range.z > range.getDim(axis)) axis = 2;

    float split = node->aabb.min.getDim(axis) + range.getDim(axis) * 0.5f;


    std::vector<Sphere> leftSpheres;
    std::vector<Sphere> rightSpheres;
    
    node->left = new BVHNode();
    node->right = new BVHNode();
    
    for (int i = 0; i < node->spheres.size(); i++) {
        Sphere sphere = node->spheres.at(i);
        if (sphere.center.getDim(axis) < split) {
            leftSpheres.push_back(sphere);
        } else {
            rightSpheres.push_back(sphere);
        }
    }

   if (leftSpheres.size() == 0 || rightSpheres.size() == 0) {
        // Adjust the condition based on your requirements
        return;
    }

    
    BVHNode* leftNode = node->left;
    BVHNode* rightNode = node->right;

    leftNode->spheres = leftSpheres;
    rightNode->spheres = rightSpheres;

    leftNode->count = leftSpheres.size();
    rightNode->count = rightSpheres.size();

    node->spheres.clear();
    node->count = 0;

    calculateBounds(leftNode);
    calculateBounds(rightNode);

   // cout << "Subdividing left... " << endl;
    subdivide(leftNode, 1);
   // cout << "Subdividing right... " << endl;
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
   // node->print();
    if(node->count > 0) return;
    cout << "Left ";
    printBVH(node->left, depth+1);
    cout << "Right ";
    printBVH(node->right, depth+1);
    cout << "Total nodes: " << node_count << endl;
}


bool rayAABBIntersect( Ray ray, Vec3 min, Vec3 max ) {

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

    return tmax > 0 && tmax >= tmin && tmin < INFINITY;
}

int iters = 0;

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

Intersect rayBVHIntersect( BVHNode* node, Ray ray ) {
    
    if (!rayAABBIntersect( ray, node->aabb.min, node->aabb.max )) return Intersect();

    if (node->count > 0) {
       // cout << "Leaf node count: " << node->count << endl;
        float min_t = INFINITY;
        Intersect nearest = Intersect();
        
        for (uint i = 0; i < node->count; i++ ) {
            iters++;

            Intersect intersect = raySphereIntersect( ray, node->spheres[i] );

            if(intersect.dist > 0 && intersect.dist < min_t) {
                min_t = intersect.dist;
                nearest = intersect;

            }
        }
       // cout << "Nearest: " << string(nearest) << endl;
        // auto stop = std::chrono::high_resolution_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        // cout << "Found nearest in: " << duration.count() << endl;
        return nearest;
    }

    //  cout << "Branch node count: " << node->count << endl;
    Intersect leftIntersect = Intersect();
    Intersect rightIntersect = Intersect();

   // BVHNode* tempNode = node->left;
   // tempNode->print();

    if(node->left != NULL && node->left->count >= 0) {
        // cout << "Left" << endl;
      //  iters++;
        leftIntersect = rayBVHIntersect(node->left, ray);
    }
    if(node->right != NULL && node->right->count >= 0) {
        // cout << "Right" << endl;
      //  iters++;
        rightIntersect = rayBVHIntersect(node->right, ray);
    }

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

Intersect rayPlaneIntersect(Ray ray, Plane plane){
    float t = dot((plane.point - ray.origin), plane.norm) / dot(ray.dir, plane.norm);
    
    if(t < 0) return Intersect();

    Point p = t * ray.dir + ray.origin;
    return Intersect(p, t, plane);
}

Intersect rayTriangleIntersect(Ray ray, Triangle triangle) {

    Point v1 = triangle.v1.point;
    Point v2 = triangle.v2.point;
    Point v3 = triangle.v3.point;

    Dir e1 = v2 - v1;
    Dir e2 = v3 - v1;

    Dir pvec = cross(ray.dir, e2);
    float det = dot(e1, pvec);

    // parallel check
    if (std::fabs(det) < HIT_THRESH) return Intersect();

    float inv = 1.f / det;
    Dir s = ray.origin - v1;
    float u = inv * dot(s, pvec);

    // outside triangle check
    if (u < 0.f || u > 1.f) return Intersect();

    Dir q = cross(s, e1);
    float v = inv * dot(ray.dir, q);

    // outside triangle check
    if (v < 0.f || u + v > 1.f) return Intersect();

    float t = inv * dot(e2, q);

    if(t > HIT_THRESH) {
        //Color triColor = u*triangle.v1.color + v*triangle.v2.color + (1.f-u-v)*triangle.v3.color;
        Dir triNorm = cross(e1, e2).normalized();
        if (dot(ray.dir, triNorm) > 0) triNorm = -1 * triNorm;
        return Intersect(ray.origin + t*ray.dir, t, triangle, triangle.color, triNorm);
    }
    
    return Intersect();
}

Intersect findNearestSphere(Ray ray) {
    Intersect nearest;
    float min_t = INFINITY;
    // Intersect nearest = Intersect();
    for(int i = 0; i < spheres.size(); i++) {
        iters++;
        Intersect intersect = raySphereIntersect(ray, spheres[i]);
        if(intersect.dist > 0 && intersect.dist < min_t) {
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
        triangle.print();
        triangles.push_back(triangle);
    }

    for (int i = 0; i < triColors.size(); i++) {
        triangles.at(i).color = triColors.at(i);
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
            } else if(data.compare("bulb") == 0) {
                float x, y, z;
                while (!line.eof()) {
                    line >> x >> y >> z;
                    lights.push_back(Light(currColor, Dir(), Point(x,y,z)));
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

                triColors.push_back(currColor);
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

void printLights() {
    for (int i = 0; i < lights.size(); i++) {
        lights[i].print();
    }
}

float shadowRayBVHIntersect( BVHNode* node, Ray ray ) {

    if (!rayAABBIntersect( ray, node->aabb.min, node->aabb.max )) return false;

    if (node->count > 0) {
        for (int i = 0; i < node->count; i++ ) {
            Intersect intersect = raySphereIntersect( ray, node->spheres[i] );
            if (intersect.dist > HIT_THRESH) return intersect.dist;
            // if(intersect.dist > HIT_THRESH && intersect.dist < (lightPos - hit).magnitude()) {
            //     return true;
            // }
        }
    }

    float leftIntersect = -1;
    float rightIntersect = -1;

    if(node->left != NULL && node->left->count >= 0) {
        leftIntersect = shadowRayBVHIntersect(node->left, ray);
    }
    if(node->right != NULL && node->right->count >= 0) {
        rightIntersect = shadowRayBVHIntersect(node->right, ray);
    }

    if(leftIntersect > 0) return leftIntersect;
    if(rightIntersect > 0) return rightIntersect;

    return -1;

}

bool isShadow(Point hit, Dir lightdir, Point lightPos) {
    Ray shadowRay = Ray(hit, lightdir);
    bool isShadow = false;

    Intersect intersect;
    if (useBVH) {
        float shadowDist = shadowRayBVHIntersect(&bvhRoot, shadowRay);
        if (shadowDist > HIT_THRESH && shadowDist < (lightPos - hit).magnitude()) return true;
    }
    else {
        for(int j = 0; j < spheres.size(); j++) {

            Intersect intersect = raySphereIntersect(shadowRay, spheres[j]);
            if(intersect.dist > HIT_THRESH && intersect.dist < (lightPos - hit).magnitude()) {
                return true;
            }
        }
    }

    for(int j = 0; j < triangles.size(); j++) {
        Intersect intersect = rayTriangleIntersect(shadowRay, triangles[j]);
        if(intersect.dist > HIT_THRESH && intersect.dist < (lightPos - hit).magnitude()) {
            return true;
        }
    }
    return false;
}

Color calculateColor(Intersect nearest) {
    float r = 0, g = 0, b = 0;
    for (int i = 0; i < lights.size(); i++) {
        float dropoff = 1;
        Light light = lights[i];

        if(lights[i].dir.magnitude() == 0.f) {
            Dir lightDir = light.pos - nearest.point;
            dropoff = lightDir.magnitudeSqr();
            light.dir = lightDir.normalized();
        }
        
        if(!isShadow(nearest.point, light.dir, light.pos)) {
            float dotty = fclamp(dot(nearest.norm, light.dir), 0, 1);
            

            r += nearest.color.r * light.color.r * dotty / dropoff; 
            g += nearest.color.g * light.color.g * dotty / dropoff;
            b += nearest.color.b * light.color.b * dotty / dropoff;
        }
    }

    return Color(r,g,b);
}

void setPixelColor(float x, float y, Intersect nearest) {
    //float r = 0, g = 0, b = 0;
    //Color c;
    Color color = calculateColor(nearest);

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
   // printBVH(&bvhRoot);
    // auto stop = std::chrono::high_resolution_clock::now();

    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    // cout << "Built BVH in: " << duration.count() << endl;

   // printLights();

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
            Ray ray = genRay(x, y, forward, right, up);
            if(ray.dir.magnitude() == 0) continue;
          //  cout << "Light " << string(ray) << endl;
            Intersect nearest;
            
           // start = std::chrono::high_resolution_clock::now();
            if (useBVH) nearest = rayBVHIntersect(&bvhRoot, ray);
            else nearest = findNearestSphere(ray);

            float min_t = nearest.dist;  
            if(min_t < 0) min_t = INFINITY;    

            for(int i = 0; i < planes.size(); i++) {
                Intersect intersect = rayPlaneIntersect(ray, planes[i]);
                if(intersect.dist > 0 && intersect.dist < min_t) {
                    min_t = intersect.dist;
                    nearest = intersect;
                }
            }

            for(int i = 0; i < triangles.size(); i++) {
                Intersect intersect = rayTriangleIntersect(ray, triangles[i]);
                if(intersect.dist > 0 && intersect.dist < min_t) {
                    min_t = intersect.dist;
                    nearest = intersect;
                }
            }

            //cout << "Nearest " << string(nearest) << endl;
            if(nearest.dist > 0) {  // intersection found
                //cout << "Nearest: " << string(nearest) << endl;
                setPixelColor(x, y, nearest);
            }
        }
    }

    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    cout << "Program execution time: " << duration.count() / 1000.f / 1000.f << " seconds." << endl;
    cout << "# Iterations: " << iters << endl;
    image.save_png(name.c_str());
} 

