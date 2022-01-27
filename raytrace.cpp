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
#include "acceleration.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
std::random_device device;
std::mt19937_64 RNGen(device());
std::uniform_real_distribution<> myrandom(0.0, 1.0);
// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].

Scene::Scene() {}

void Scene::Finit()
{
}

void Scene::triangleMesh(MeshData* mesh) 
{ 
    //triangleMesh(mesh);
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
        eye = vec3(f[0], f[1], f[2]);
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
        vectorOfShapes.push_back(new Sphere(currentMat, vec3(f[1], f[2], f[3]), f[4]));
        if (currentMat->isLight()) lights.push_back(dynamic_cast<Light*>(currentMat));
    }
    else if (c == "box") 
    {
        // syntax: box bx by bz   dx dy dz
        // Creates a Shape instance for a box defined by a corner point and diagonal vector
        vectorOfShapes.push_back(new Box(currentMat, vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6])));
        if (currentMat->isLight()) lights.push_back(dynamic_cast<Light*>(currentMat));
    }
    else if (c == "cylinder") {
        // syntax: cylinder bx by bz   ax ay az  r
        // Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
        vectorOfShapes.push_back(new Cylinder(currentMat, vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7]));
        if (currentMat->isLight()) lights.push_back(dynamic_cast<Light*>(currentMat));
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
#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
    for (int y=0;  y<height;  y++) {

        fprintf(stderr, "Rendering %4d\r", y);
        for (int x=0;  x<width;  x++) {
            Ray ray();
            Color color;
            image[y*width + x] = color;
        }
    }
    fprintf(stderr, "\n");
}
