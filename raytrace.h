///////////////////////////////////////////////////////////////////////
// A framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#include <vector>

#include "acceleration.h"

const float PI = 3.14159f;
const float Radians = PI/180.0f;    // Convert degrees to radians

constexpr float epsilon = 0.0001f;

////////////////////////////////////////////////////////////////////////
// Material: encapsulates a BRDF and communication with a shader.
////////////////////////////////////////////////////////////////////////
class Material
{
 public:
    vec3 Kd, Ks;
    float alpha;
    unsigned int texid;

    virtual bool isLight() { return false; }

    Material()  : Kd(vec3(1.0, 0.5, 0.0)), Ks(vec3(1,1,1)), alpha(1.0), texid(0) {}
    Material(const vec3 d, const vec3 s, const float a) 
        : Kd(d), Ks(s), alpha(a), texid(0) {}
    Material(Material& o) { Kd=o.Kd;  Ks=o.Ks;  alpha=o.alpha;  texid=o.texid; }

    //virtual void apply(const unsigned int program);
};

////////////////////////////////////////////////////////////////////////
// Data structures for storing meshes -- mostly used for model files
// read in via ASSIMP.
//
// A MeshData holds two lists (stl::vector) one for vertices
// (VertexData: consisting of point, normal, texture, and tangent
// vectors), and one for triangles (ivec3: consisting of three
// indices into the vertex array).
    
class VertexData
{
 public:
    vec3 pnt;
    vec3 nrm;
    vec2 tex;
    vec3 tan;
    VertexData(const vec3& p, const vec3& n, const vec2& t, const vec3& a) 
        : pnt(p), nrm(n), tex(t), tan(a) 
    {}
};

struct MeshData
{
    std::vector<VertexData> vertices;
    std::vector<ivec3> triangles;
    Material *mat;
};

class Shape;

////////////////////////////////////////////////////////////////////////
// Light: encapsulates a light and communiction with a shader.
////////////////////////////////////////////////////////////////////////
class Light: public Material
{
public:
    Light(const vec3 e) : Material() { Kd = e; }
    virtual bool isLight() { return true; }

    Shape* shape = nullptr;
};

////////////////////////////////////////////////////////////////////////
// Shape
////////////////////////////////////////////////////////////////////////
enum SHAPE_TYPE
{
    SHAPE_SPHERE,
    SHAPE_CYLINDER,
    SHAPE_BOX,
    SHAPE_TRIANGLE,
};

class Slab
{
public:
    vec3 N;
    float d1, d2;
};

class Shape
{
public:
    Shape(Material* material, SHAPE_TYPE shapetype) : mat(material), type(shapetype) {}
    Material* mat;
    SHAPE_TYPE type;

    virtual SimpleBox boundingbox() = 0;
};

class Sphere : public Shape
{
public:
    Sphere(Material* material, const vec3& center, const float radius) : Shape(material, SHAPE_SPHERE), C(center), r(radius) {}
    vec3 C;
    float r;

    virtual SimpleBox boundingbox() override;
};

class Cylinder : public Shape
{
public:
    Cylinder(Material* material, const vec3& center, const vec3& axis, float radius) : Shape(material, SHAPE_CYLINDER), B(center), A(axis), r(radius) {}
    vec3 B;
    vec3 A;
    float r;

    virtual SimpleBox boundingbox() override;
};

class Box : public Shape
{
public:
    Box(Material* material, const vec3& corner, const vec3& diagonal) : Shape(material, SHAPE_BOX), C(corner), d(diagonal) {}
    vec3 C;
    vec3 d;

    virtual SimpleBox boundingbox() override;
};

class Triangle : public Shape
{
public:
    Triangle(Material* material, vec3 v0, vec3 v1, vec3 v2) : Shape(material, SHAPE_TRIANGLE), V0(v0), V1(v1), V2(v2) {}
    Triangle(Material* material, vec3 v0, vec3 v1, vec3 v2, vec3 n0, vec3 n1, vec3 n2) : 
        Shape(material, SHAPE_TRIANGLE), V0(v0), V1(v1), V2(v2), N0(n0), N1(n1), N2(n2) {}
    Triangle(Material* material, vec3 v0, vec3 v1, vec3 v2, vec3 n0, vec3 n1, vec3 n2, vec2 t0, vec2 t1, vec2 t2) : 
        Shape(material, SHAPE_TRIANGLE), V0(v0), V1(v1), V2(v2), N0(n0), N1(n1), N2(n2), T0(t0), T1(t1), T2(t2) {}

    vec3 V0;
    vec3 V1;
    vec3 V2;

    std::optional<vec3> N0;
    std::optional<vec3> N1;
    std::optional<vec3> N2;

    std::optional<vec2> T0;
    std::optional<vec2> T1;
    std::optional<vec2> T2;

    virtual SimpleBox boundingbox() override;
};

////////////////////////////////////////////////////////////////////////////////
// Scene
class Scene {
public:
    int width, height;
    vec3 eye;
    quat orient;
    float ry;

    vec3 ambient;

    std::vector<Light*> lights;
    std::vector<Shape*> vectorOfShapes;

    Material* currentMat;

    Scene();
    void Finit();

    // The scene reader-parser will call the Command method with the
    // contents of each line in the scene file.
    void Command(const std::vector<std::string>& strings,
                 const std::vector<float>& f);

    // To read a model file into the scene via ASSIMP, call ReadAssimpFile.  
    void ReadAssimpFile(const std::string& path, const mat4& M);

    // Once ReadAssimpFile parses the information from the model file,
    // it will call:
    void triangleMesh(MeshData* mesh);

    // The main program will call the TraceImage method to generate
    // and return the image.  This is the Ray Tracer!
    void TraceImage(Color* image, const int pass);

    Color TracePath(const Ray& ray, AccelerationBvh& bvh);

    Intersection SampleLight();
    float PdfLight(Intersection Q);
};

float GeometryFactor(Intersection A, Intersection B);
vec3 SampleLobe(vec3 A, float c, float phi);
Intersection SampleSphere(vec3 C, float R, Shape* obj);

vec3 EvalRadiance(Intersection Q);

vec3 SampleBrdf(vec3 N);
float PdfBrdf(vec3 N, vec3 wi);
vec3 EvalScattering(vec3 N, vec3 wi, vec3 Kd);