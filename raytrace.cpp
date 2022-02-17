//////////////////////////////////////////////////////////////////////
// Provides the framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#include <vector>

#ifdef _WIN32
    // Includes for Windows
    #include <windows.h>
    #include <cstdlib>
    #include <limits>
    #include <crtdbg.h>
#else
    // Includes for Linux
#endif

#include "geom.h"
#include "raytrace.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
std::random_device device;
std::mt19937_64 RNGen(device());
std::uniform_real_distribution<> myrandom(0.0, 1.0);
std::uniform_int_distribution<> myrandom_INT(0, 1);

// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].

#define TIME_MEASURE
#ifdef TIME_MEASURE
    #include <chrono>
#endif

constexpr float RussianRoulette = 0.8f;

constexpr int PATH_PASS = 16;

Scene::Scene() {}

void Scene::Finit()
{
}

void Scene::triangleMesh(MeshData* mesh) 
{ 
    for (auto& triangle : mesh->triangles)
    {
        vec3 V0 = mesh->vertices[triangle.x].pnt;
        vec3 V1 = mesh->vertices[triangle.y].pnt;
        vec3 V2 = mesh->vertices[triangle.z].pnt;

        vec3 N0 = mesh->vertices[triangle.x].nrm;
        vec3 N1 = mesh->vertices[triangle.y].nrm;
        vec3 N2 = mesh->vertices[triangle.z].nrm;

        vec2 T0 = mesh->vertices[triangle.x].tex;
        vec2 T1 = mesh->vertices[triangle.y].tex;
        vec2 T2 = mesh->vertices[triangle.z].tex;
        
        vectorOfShapes.push_back(new Triangle(mesh->mat, V0, V1, V2, N0, N1, N2, T0, T1, T2));
    }
}

quat Orientation(int i, 
                        const std::vector<std::string>& strings,
                        const std::vector<float>& f)
{
    quat q(1,0,0,0); // Unit quaternion
    while (i<strings.size()) {
        std::string c = strings[i++];
        if (c == "x")  
            q *= angleAxis(f[i++]*Radians, Xaxis());
        else if (c == "y")  
            q *= angleAxis(f[i++]*Radians, Yaxis());
        else if (c == "z")  
            q *= angleAxis(f[i++]*Radians, Zaxis());
        else if (c == "q")  {
            q *= quat(f[i+0], f[i+1], f[i+2], f[i+3]);
            i+=4; }
        else if (c == "a")  {
            q *= angleAxis(f[i+0]*Radians, normalize(vec3(f[i+1], f[i+2], f[i+3])));
            i+=4; } }
    return q;
}

void Scene::Command(const std::vector<std::string>& strings,
                    const std::vector<float>& f)
{
    if (strings.size() == 0) return;
    std::string c = strings[0];
    
    if (c == "screen") 
    {
        // syntax: screen width height
        width = int(f[1]);
        height = int(f[2]); 
    }
    else if (c == "camera") 
    {
        // syntax: camera x y z   ry   <orientation spec>
        // Eye position (x,y,z),  view orientation (qw qx qy qz),  frustum height ratio ry
        eye = vec3(f[1], f[2], f[3]);
        orient = Orientation(5, strings, f);
        ry = f[4];
    }
    else if (c == "ambient") 
    {
        // syntax: ambient r g b
        // Sets the ambient color.  Note: This parameter is temporary.
        // It will be ignored once your raytracer becomes capable of
        // accurately *calculating* the true ambient light.
        ambient = vec3(f[1], f[2], f[3]); 
    }
    else if (c == "brdf")  
    {
        // syntax: brdf  r g b   r g b  alpha
        // later:  brdf  r g b   r g b  alpha  r g b ior
        // First rgb is Diffuse reflection, second is specular reflection.
        // third is beer's law transmission followed by index of refraction.
        // Creates a Material instance to be picked up by successive shapes
        currentMat = new Material(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7]); 
    }
    else if (c == "light") 
    {
        // syntax: light  r g b   
        // The rgb is the emission of the light
        // Creates a Material instance to be picked up by successive shapes
        currentMat = new Light(vec3(f[1], f[2], f[3])); 
    }
    else if (c == "sphere") 
    {
        // syntax: sphere x y z   r
        // Creates a Shape instance for a sphere defined by a center and radius
        //realtime->sphere(vec3(f[1], f[2], f[3]), f[4], currentMat);
        Shape* shape = new Sphere(currentMat, vec3(f[1], f[2], f[3]), f[4]);
        vectorOfShapes.push_back(shape);
        if (currentMat->isLight())
        {
            Light* light = dynamic_cast<Light*>(currentMat);
            light->shape = shape;
            lights.push_back(light);
        }
    }
    else if (c == "box") 
    {
        // syntax: box bx by bz   dx dy dz
        // Creates a Shape instance for a box defined by a corner point and diagonal vector
        Shape* shape = new Box(currentMat, vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]));
        vectorOfShapes.push_back(shape);
        if (currentMat->isLight())
        {
            Light* light = dynamic_cast<Light*>(currentMat);
            light->shape = shape;
            lights.push_back(light);
        }
    }
    else if (c == "cylinder") {
        // syntax: cylinder bx by bz   ax ay az  r
        // Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
        Shape* shape = new Cylinder(currentMat, vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7]);
        vectorOfShapes.push_back(shape);
        if (currentMat->isLight())
        {
            Light* light = dynamic_cast<Light*>(currentMat);
            light->shape = shape;
            lights.push_back(light);
        }
    }
    else if (c == "mesh") {
        // syntax: mesh   filename   tx ty tz   s   <orientation>
        // Creates many Shape instances (one per triangle) by reading
        // model(s) from filename. All triangles are rotated by a
        // quaternion (qw qx qy qz), uniformly scaled by s, and
        // translated by (tx ty tz) .
        mat4 modelTr = translate(vec3(f[2],f[3],f[4]))
                          *scale(vec3(f[5],f[5],f[5]))
                          *toMat4(Orientation(6,strings,f));
        ReadAssimpFile(strings[1], modelTr);  }

    
    else {
        fprintf(stderr, "\n*********************************************\n");
        fprintf(stderr, "* Unknown command: %s\n", c.c_str());
        fprintf(stderr, "*********************************************\n\n");
    }
}

void Scene::TraceImage(Color* image, const int pass)
{
    float rx = ry * width / (float)(height);
    vec3 X = rx * transformVector(orient, Xaxis());
    vec3 Y = ry * transformVector(orient, Yaxis());
    vec3 Z = transformVector(orient, Zaxis());

    AccelerationBvh bvh(vectorOfShapes);

#ifdef TIME_MEASURE
    auto before = std::chrono::high_resolution_clock::now();
    fprintf(stderr, "Trace Image start!\n");
#endif // TIME_MEASURE

    for (int pass = 0; pass < PATH_PASS; ++pass)
    {
#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
        for (int y = 0; y < height; y++) {
            fprintf(stderr, "Rendering %4d\r", y);
            for (int x = 0; x < width; x++) {
                float dx = 2 * (x + myrandom(RNGen)) / width - 1;
                float dy = 2 * (y + myrandom(RNGen)) / height - 1;
                Ray ray(eye, normalize(dx * X + dy * Y - Z));

                Color c =TracePath(ray, bvh);
                image[y * width + x] += c;
            }
        }
    }

#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            image[y * width + x] /= (float)PATH_PASS;
        }
    }
    fprintf(stderr, "\n");

#ifdef TIME_MEASURE
    auto after = std::chrono::high_resolution_clock::now();
    float result = (float)(std::chrono::duration<double, std::ratio<1, 1>>(after -before).count());

    fprintf(stderr, "Trace Image time passed : %4f\n", result);
#endif // TIME_MEASURE

}

Color Scene::TracePath(const Ray& ray, AccelerationBvh& bvh)
{
    vec3 C = vec3(0, 0, 0);
    vec3 W = vec3(1, 1, 1);

    Intersection P;

    P = bvh.intersect(ray);
    vec3 N = P.N;

    if (P.t == INFINITY || P.t <= 0) return C;
    if (P.object->mat->isLight()) return EvalRadiance(P);

    while (myrandom(RNGen) < RussianRoulette)
    {
        {
            Intersection L = SampleLight();
            float p = PdfLight(L) / GeometryFactor(P, L);
            vec3 wi = normalize(L.P - P.P);
            Ray shadowray = Ray(P.P, wi);
            Intersection I = bvh.intersect(shadowray);
            if (p > 0 && I.t != INFINITY && I.t > 0)
            {
                if (I.P == L.P)
                {
                    vec3 f = EvalScattering(N, wi, I.object->mat->Kd);
                    C += 0.5f * W * (f / p) * EvalRadiance(L);
                }
            }
        }

        {
            N = P.N;

            vec3 wi = SampleBrdf(N);

            Ray newray(P.P, wi);

            Intersection Q = bvh.intersect(newray);

            if (Q.t == INFINITY || Q.t <= 0) return C;

            vec3 f = EvalScattering(N, wi, P.object->mat->Kd);
            float p = PdfBrdf(N, wi) * RussianRoulette;

            if (p < 0.000001f) break;

            W *= f / p;

            if (Q.object->mat->isLight())
            {
                C += 0.5f * W * EvalRadiance(Q);
                break;
            }

            P = Q;
        }
    }

    return C;
}

SimpleBox Sphere::boundingbox()
{
    vec3 diag = vec3(r, r, r);

    SimpleBox boundingbox(C - diag);
    boundingbox.extend(C + diag);

    return boundingbox;
}

SimpleBox Cylinder::boundingbox()
{
    vec3 rrr = vec3(r, r, r);

    SimpleBox boundingbox(B + rrr);
    boundingbox.extend(B - rrr);
    boundingbox.extend(B + A + rrr);
    boundingbox.extend(B + A - rrr);

    return boundingbox;
}

SimpleBox Box::boundingbox()
{
    SimpleBox boundingbox(C);
    boundingbox.extend(C + d);

    return boundingbox;
}

SimpleBox Triangle::boundingbox()
{
    SimpleBox bounding_box(V0);
    bounding_box.extend(V1);
    bounding_box.extend(V2);

    return bounding_box;
}

float GeometryFactor(Intersection A, Intersection B)
{
    vec3 D = A.P - B.P;
    float DdotD = dot(D, D);
    return abs((dot(A.N, D) * dot(B.N, D)) / (DdotD * DdotD));
}

vec3 SampleLobe(vec3 A, float c, float phi)
{
    float s = sqrt(1 - c * c);

    vec3 K = vec3(s * cos(phi), s * sin(phi), c);

    if (abs(A.z - 1) < 0.001) return K;
    if (abs(A.z + 1) < 0.001) return vec3(K.x, -K.y, -K.z);

    vec3 B = normalize(vec3(-A.y, A.x, 0));
    vec3 C = cross(A, B);

    return K.x * B + K.y * C + K.z * A;
}

Intersection SampleSphere(vec3 C, float R, Shape* obj)
{
    float x1 = myrandom(RNGen);
    float x2 = myrandom(RNGen);

    float z = 2 * x1 - 1;
    float r = sqrt(1 - z * z);

    float a = 2 * PI * x2;

    Intersection result;

    result.N = vec3(r * cos(a), r * sin(a), z);
    result.P = C + R * result.N;
    result.object = obj;

    return result;
}

Intersection Scene::SampleLight()
{
    //only for sphere light
    unsigned int numberoflights = lights.size();
    int N = myrandom_INT(RNGen) * (numberoflights - 1);
    Light* light = lights[N];
    if (light->shape->type != SHAPE_SPHERE) fprintf(stderr, "Error! light is not sphere!\n");
    Sphere* lightsphere = dynamic_cast<Sphere*>(light->shape);
    return SampleSphere(lightsphere->C, lightsphere->r, lightsphere);
}

float Scene::PdfLight(Intersection Q)
{
    float r = dynamic_cast<Sphere*>(Q.object)->r;
    float areaoflight = r * r * 4.0 * PI;
    return 1.0f / (lights.size() * areaoflight);
}

vec3 SampleBrdf(vec3 N)
{
    float c = myrandom(RNGen);
    float phi = myrandom(RNGen);

    return SampleLobe(N, sqrt(c), 2 * PI * phi);
}

vec3 EvalRadiance(Intersection Q)
{
    return Q.object->mat->Kd;
}

float PdfBrdf(vec3 N, vec3 wi)
{
    return abs(dot(N, wi)) / PI;
}

vec3 EvalScattering(vec3 N, vec3 wi, vec3 Kd)
{
    return abs(dot(N, wi)) * Kd / PI;
}
