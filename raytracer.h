#include <iostream>
#include <fstream>
#include <cmath>

using std::cout;
using std::string;
using std::max;
using std::endl;

struct Color;
struct Sphere;
struct Point;
struct Dir;

struct Point{
  float x,y,z;

  Point(float x=0, float y=0, float z=0): x(x), y(y), z(z) {}

  Point scale(float f){
    return Point(x*f,y*f,z*f);
  }

  Dir operator-(Point rhs);
  Point operator+(Dir rhs);
  Point operator-(Dir rhs);
  Point operator+(Point rhs);

  Point normalized(){
    return Point(x,y,z); //Points are already normalized
  }

  float distTo(Point p){
    return sqrt((x-p.x)*(x-p.x) + (y-p.y)*(y-p.y) + (z-p.z)*(z-p.z));
  }

  float distToSqr(Point p){
    return (x-p.x)*(x-p.x) + (y-p.y)*(y-p.y) + (z-p.z)*(z-p.z);
  }

  float getDim(int d) {
      if(d == 0) {
          return x;
      } else if (d == 1) {
          return y;
      } else {
          return z;
      }
  }

  operator std::string() const {
    char result[1000];
    sprintf(result, "Point: (%.8f, %.8f, %.8f)",x,y,z);
    return result;
  }

  void print(const char* title=""){
    printf("%s - %s\n",title, std::string(*this).c_str());
  }
  
};

struct Dir{
    float x,y,z;

    Dir(float x=0, float y=0, float z=0): x(x), y(y), z(z) {}


    Point operator+(Point rhs);
    Dir operator+(Dir rhs);
    Dir operator-(Dir rhs);

    float magnitude(){
        return sqrt(x*x+y*y+z*z);
    }

    float magnitudeSqr(){
        return x*x+y*y+z*z;
    }

    Dir normalized(){
        float mag = magnitude();
        return Dir(x/mag, y/mag, z/mag);
    }

    operator std::string() const {
        char result[1000];
        sprintf(result, "Dir: (%.8f, %.8f, %.8f)", x, y, z);
        return result;
    }

    void print(const char* title=""){
        printf("%s - %s\n",title, std::string(*this).c_str());
    }
};

struct Ray{
  Point origin;
  Dir dir;
  Dir invDir;

  Ray(Point o, Dir d) : origin(o), dir(d), invDir(Dir(1.f / d.x, 1.f / d.y, 1.f / d.z)) {}
  Ray() : origin(Point()), dir(Dir()) {}

  operator std::string() const {
    char result[1000];
    sprintf(result, "Ray:\n\tOrigin: (%.8f, %.8f, %.8f) \n\tDirection: (%.8f, %.8f, %.8f)", origin.x, origin.y, origin.z, dir.x, dir.y, dir.z);
    return result;
  }
};

struct Color{
  float r,g,b;

  Color(float r=1.f, float g=1.f, float b=1.f) : r(r), g(g), b(b) {}

  operator std::string() const {
    char result[1000];
    sprintf(result, "Color: (%.8f, %.8f, %.8f)",r,g,b);
    return result;
  }
};

struct Mat{
  Color color;
  Color shine;
  Color trans;
  float ior;

  Mat(Color c=Color(), Color s=Color(0,0,0), Color t=Color(0,0,0), float i=1.458) : color(c), shine(s), trans(t), ior(i) {}

};

struct Sphere{
  Point center;
  float radius;
  Mat mat;

  Sphere(Point p=Point(), float r=-1, Mat m=Mat()) : center(p), radius(r), mat(m) {}

  operator std::string() const {
    char result[1000];
    sprintf(result, "Sphere:\n\tCenter: (%.8f, %.8f, %.8f) \n\tRadius: %.8f \n\tColor: (%.8f, %.8f, %.8f)",center.x,center.y,center.z, radius, mat.color.r, mat.color.g, mat.color.b);
    return result;
  }
};

struct Plane{
  float a, b, c, d;
  Dir norm;
  Point point;
  Mat mat;


  Plane(float a=0, float b=0, float c=0, float d=0, Mat m=Mat()) : 
    a(a), 
    b(b), 
    c(c),
    d(d),
    norm(Dir(a,b,c).normalized()),
    point(getPoint()),
    mat(m) {}
  
  Point getPoint() {
    float m = -d / sqrt(pow(a,2) + pow(b,2) + pow(c,2));
    return Point(m*a,m*b,m*c) ;
  }
  

  operator std::string() const {
    char result[1000];
    sprintf(result, "Plane:\n\tEquation: %.4fx + %.4fy + %.4fz + %.4f\n\tPoint: (%.4f, %.4f, %.4f)",a,b,c,d, point.x, point.y, point.z);
    return result;
  }
};

struct Vertex{
  Point point;

  Vertex(Point p=Point()) : 
    point(p) {}
  
  operator std::string() const {
    char result[1000];
    sprintf(result, "Vertex:\n\t(%.4f, %.4f, %.4f)", point.x, point.y, point.z);
    return result;
  }

};

struct Triangle{
  Vertex v1,v2,v3;
  Mat mat;

  Triangle(Vertex va=Vertex(), Vertex vb=Vertex(), Vertex vc=Vertex(), Mat m=Mat()) : 
    v1(va), 
    v2(vb), 
    v3(vc),
    mat(m) {}

  void print() {
    cout << "Triangle: " << endl;
    cout << "\t" << string(v1) << endl;
    cout << "\t" << string(v2) << endl;
    cout << "\t" << string(v3) << endl;
  }

};

struct Intersect{
    Point point;
    float dist;
    Dir norm;
    Mat mat;

    Intersect(Point p=Point(), float d=INFINITY, Dir n=Dir(), Mat m=Mat()) :
      point(p),
      dist(d),
      norm(n),
      mat(m) {}

};

struct SphereIntersect : Intersect{

    Sphere sphere;

    SphereIntersect(Point p=Point(), float d=INFINITY, Sphere s=Sphere()) : 
        sphere(s),
        Intersect(p, d, (p - s.center).normalized(), s.mat ) {}

    operator std::string() const {
        char result[1000];
        sprintf(result, "Intersect:\n\tPoint: (%.8f, %.8f, %.8f)\n\tDistance: %.8f",point.x,point.y,point.z, dist);
        return result;
    }
};

struct TriangleIntersect : Intersect{

    Triangle triangle;

    TriangleIntersect(Point p=Point(), float d=INFINITY, Triangle tr=Triangle(), Dir n=Dir()) :  
        triangle(tr),
        Intersect(p, d, n, tr.mat ) {}


    operator std::string() const {
        char result[1000];
        sprintf(result, "Intersect:\n\tPoint: (%.8f, %.8f, %.8f)\n\tDistance: %.8f",point.x,point.y,point.z, dist);
        return result;
    }
};

struct PlaneIntersect : Intersect{

    Plane plane;


    PlaneIntersect(Point p=Point(), float d=INFINITY, Plane pl=Plane()) : 
        plane(pl),
        Intersect(p, d, pl.norm) {}

    operator std::string() const {
        char result[1000];
        sprintf(result, "Intersect:\n\tPoint: (%.8f, %.8f, %.8f)\n\tDistance: %.8f",point.x,point.y,point.z, dist);
        return result;
    }
};

struct Light {
  Color color;
  Dir dir;
  Point pos;

  Light(Color c=Color(), Dir d=Dir(), Point p=Point(INFINITY, INFINITY, INFINITY)) : color(c), dir(d), pos(p) {}

  void print() {
    cout << "Light:" << endl; 
    cout << "\t" << string(color) << endl;
    cout << "\t" << string(dir) << endl;
    cout << "\t" << string(pos) << endl;
  }

};

inline float dot(Dir a, Dir b){
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

inline Dir cross(Dir a, Dir b){
  return Dir(a.y*b.z - b.y*a.z, a.z*b.x - a.x*b.z, a.x*b.y - b.x*a.y);
}


//Operations on Points:
inline Point Point::operator+(Point rhs){
  return Point(x+rhs.x,y+rhs.y,z+rhs.z);
}

inline Dir Point::operator-(Point rhs){
  return Dir(x-rhs.x,y-rhs.y,z-rhs.z);
}

inline Point Point::operator+(Dir rhs){
  return Point(x+rhs.x,y+rhs.y,z+rhs.z);
}

inline Point Point::operator-(Dir rhs){
  return Point(x-rhs.x,y-rhs.y,z-rhs.z);
}

inline Point operator*(Point p, float f){
  return Point(p.x*f,p.y*f,p.z*f);
}

inline Point operator*(float f, Point p){
  return Point(p.x*f,p.y*f,p.z*f);
}

inline Point operator/(Point p, float f){
  return Point(p.x/f,p.y/f,p.z/f);
}

inline bool operator==(Point p1, Point p2){
  return (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z);
}


//Operations on Directions:
inline Point Dir::operator+(Point rhs){
  return Point(x+rhs.x,y+rhs.y,z+rhs.z);
}

inline Dir Dir::operator+(Dir rhs){
  return Dir(x+rhs.x,y+rhs.y,z+rhs.z);
}

inline Dir Dir::operator-(Dir rhs){
  return Dir(x-rhs.x,y-rhs.y,z-rhs.z);
}

inline Dir operator*(Dir d, float f){
  return Dir(d.x*f,d.y*f,d.z*f);
}

inline Dir operator*(float f,Dir d){
  return Dir(d.x*f,d.y*f,d.z*f);
}


//Operation on Colors:
inline Color operator+(Color c1, Color c2){
  return Color(c1.r+c2.r, c1.g+c2.g, c1.b+c2.b);
}

inline Color operator-(Color c1, Color c2){
  return Color(c1.r-c2.r, c1.g-c2.g, c1.b-c2.b);
}

inline Color operator*(Color c1, Color c2){
  return Color(c1.r*c2.r, c1.g*c2.g, c1.b*c2.b);
}

inline Color operator*(Color d, float f){
  return Color(d.r*f,d.g*f,d.b*f);
}

inline Color operator*(float f,Color d){
  return Color(d.r*f,d.g*f,d.b*f);
}

inline Color operator/(Color d, float f){
  return Color(d.r/f,d.g/f,d.b/f);
}

float fclamp(float v, float l, float h){ return (v < l) ? l : (v > h) ? h : v; }