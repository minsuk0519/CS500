///////////////////////////////////////////////////////////////////////
// A framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#include <vector>

const float PI = 3.14159f;
const float Radians = PI/180.0f;    // Convert degrees to radians

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

////////////////////////////////////////////////////////////////////////
// Light: encapsulates a light and communiction with a shader.
////////////////////////////////////////////////////////////////////////
class Light: public Material
{
public:

    Light(const vec3 e) : Material() { Kd = e; }
    virtual bool isLight() { return true; }
    //virtual void apply(const unsigned int program);
};

////////////////////////////////////////////////////////////////////////
// Shape
////////////////////////////////////////////////////////////////////////
class Shape
{
public:
    Shape(Material* material) : mat(material) {}
    Material* mat;
};

class Sphere : public Shape
{
public:
    Sphere(Material* material, const vec3 centerPos, const float r) : Shape(material), center(centerPos), radius(r) {}
    vec3 center;
    float radius;
};

class Cylinder : public Shape
{
public:
    Cylinder(Material* material, const vec3 centerPos, const vec3 axisVector, float r) : Shape(material), center(centerPos), axis(axisVector), radius(r) {}
    vec3 center;
    vec3 axis;
    float radius;
};

class Box : public Shape
{
public:
    Box(Material* material, const vec3 centerPos, const vec3 diagonal) : Shape(material), center(centerPos), diag(diagonal) {}
    vec3 center;
    vec3 diag;
};

class Triangle : public Shape
{
public:

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

    void sphere(const vec3 center, const float radius, Material* mat);
};
