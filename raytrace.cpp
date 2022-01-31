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
// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].

#define TIME_MEASURE 1
#ifdef TIME_MEASURE
    #include <chrono>
#endif

#define BASECOLORDISPLAY 1
#define TVALUEDISPLAY 2
#define NORMALDISPLAY 3
#define LIGHTDISPLAY 4
#define POSITIONDISPLAY 5
#define TEXDISPLAY 6

constexpr int DISPLAY_MODE = BASECOLORDISPLAY;

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
            light->center = vec3(f[1], f[2], f[3]);
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
            light->center = vec3(f[1], f[2], f[3]) + 0.5f * vec3(f[4], f[5], f[6]);
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
            light->center = vec3(f[1], f[2], f[3]) + 0.5f * vec3(f[4], f[5], f[6]);
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

#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
    for (int y=0;  y<height;  y++) {
        fprintf(stderr, "Rendering %4d\r", y);
        for (int x=0;  x<width;  x++) {
            float dx = 2 * (x + 0.5f) / width - 1;
            float dy = 2 * (y + 0.5f) / height - 1;
            Ray ray(eye, normalize(dx * X + dy * Y - Z));

            Intersection front, current;
            front.t = INFINITY;

            front = bvh.intersect(ray);
            
            Color color = Color(0, 0, 0);

            if (front.t != INFINITY && front.t > 0)
            {
                if (DISPLAY_MODE == BASECOLORDISPLAY)
                {
                    color = front.object->mat->Kd;
                }
                else if (DISPLAY_MODE == TVALUEDISPLAY)
                {
                    color = Color((front.t - 5) / 4.0f);
                }
                else if (DISPLAY_MODE == NORMALDISPLAY)
                {
                    color = front.N;

                    //prevent the underflow for hdr format image
                    color.x = std::max(color.x, 0.0f);
                    color.y = std::max(color.y, 0.0f);
                    color.z = std::max(color.z, 0.0f);
                }
                else if (DISPLAY_MODE == POSITIONDISPLAY)
                {
                    color = (front.P + vec3(0, 0, 1)) / 2.0f;

                    //prevent the underflow for hdr format image
                    color.x = std::max(color.x, 0.0f);
                    color.y = std::max(color.y, 0.0f);
                    color.z = std::max(color.z, 0.0f);
                }
                else if (DISPLAY_MODE == LIGHTDISPLAY)
                {
                    for (auto light : lights)
                    {
                        vec3 L = light->center;
                        color += front.object->mat->Kd * dot(front.N, L);
                    }
                }
                else if (DISPLAY_MODE == TEXDISPLAY)
                {
                    color = vec3(front.UV.x, front.UV.y, 0.0);
                }
            }

            image[y*width + x] = color;
        }
    }
    fprintf(stderr, "\n");

#ifdef TIME_MEASURE
    auto after = std::chrono::high_resolution_clock::now();
    float result = (float)(std::chrono::duration<double, std::ratio<1, 1>>(after -before).count());

    fprintf(stderr, "Trace Image time passed : %4f\n", result);
#endif // TIME_MEASURE

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
