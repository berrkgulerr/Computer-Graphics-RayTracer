#include <iostream>
#include "parser.h"
#include "ppm.h"
#include <cmath>

using namespace parser;
using namespace std;

Vec3f Vec3f::add (const Vec3f& b)
{
    Vec3f result;
    result.x = x+b.x;
    result.y = y+b.y;
    result.z = z+b.z;
    return result;
}

Vec3f Vec3f::substract (const Vec3f& b)
{
    Vec3f result;
    result.x = x - b.x;
    result.y = y - b.y;
    result.z = z - b.z;
    return result;
}

Vec3f Vec3f::multS (double s)
{
    Vec3f result;
    result.x = x*s;
    result.y = y*s;
    result.z = z*s;
    return result;
}

Vec3f Vec3f::multC (const Vec3f& b)
{
    Vec3f result;
    result.x = y*b.z - z*b.y;
    result.y = z*b.x - x*b.z;
    result.z = x*b.y - y*b.x;
    return result;
}

double Vec3f::dot (const Vec3f& b)
{
    return (x*b.x+y*b.y+z*b.z);
}

double Vec3f::length ()
{
    return sqrtf(x*x + y*y + z*z);
}

Vec3f Vec3f::normalize ()
{
    Vec3f result;
    double length = this->length();
    result.x = x/length;
    result.y = y/length;
    result.z = z/length;
    return result;
}

typedef struct 
{
    Vec3f origin, direction;
    bool isShadow;
}Ray;

typedef struct
{
    int R,G,B;
}Color;


double dtrmnnt(const Vec3f &a, const Vec3f &b, const Vec3f &c)
{
	double det =  a.x*(b.y*c.z - c.y*b.z) + a.y*(c.x*b.z - b.x*c.z) + a.z*(b.x*c.y - b.y*c.x);
    return det;
}


double intersectSphere(const Ray &r, const Sphere &s, vector<Vec3f> &vertices)
{
    double A,B,C;
    double delta;
    Vec3f c = vertices[s.center_vertex_id -1];
    double t,t1,t2;

    C = (r.origin.x-c.x)*(r.origin.x-c.x)+(r.origin.y-c.y)*(r.origin.y-c.y)+(r.origin.z-c.z)*(r.origin.z-c.z)-s.radius*s.radius;
	B = 2*r.direction.x*(r.origin.x-c.x)+2*r.direction.y*(r.origin.y-c.y)+2*r.direction.z*(r.origin.z-c.z);
    A = r.direction.x*r.direction.x+r.direction.y*r.direction.y+r.direction.z*r.direction.z;
    delta = B*B-4*A*C;

    if (delta<0) return -1;
	else if (delta==0)
	{
		t = -B / (2*A);
	}
	else
	{
		delta = sqrtf(delta);
		A = 2*A;
		t1 = (-B + delta) / A;
		t2 = (-B - delta) / A;
		if (t1<t2) t=t1; else t=t2;
	}
	return t;
}

double intersectTriangle(const Ray &r, const Triangle &triangle, vector<Vec3f> &vertices)
{
    Vec3f a = vertices[triangle.indices.v0_id-1];
    Vec3f b = vertices[triangle.indices.v1_id-1];
    Vec3f c = vertices[triangle.indices.v2_id-1];

    Vec3f a_o = a.substract(r.origin);
    Vec3f a_c = a.substract(c);
    Vec3f a_b = a.substract(b);
    /* !!!!! IF CASELER İLE DAHA EFFICIENT HALE GETİRİLEBİLİR !!!! */
    double detA = dtrmnnt(a_b, a_c, r.direction);
    if(detA == 0) return -1;
    double beta = dtrmnnt(a_o, a_c, r.direction)/detA;
    double gamma = dtrmnnt(a_b,a_o,r.direction)/detA;
    double t = dtrmnnt(a_b, a_c, a_o)/detA;

    if(t>0 && gamma>=0 && beta>=0 && (beta+gamma)<=1){
        return t;
    }
    else return -1;

}

double intersectMesh(const Ray &r, const Mesh &mesh, vector<Vec3f> &vertices, Vec3f &normal)
{
    double t_min = 99999;
    int triangle_size = mesh.faces.size();
    Triangle myTriangle;
    for(int i=0 ; i<triangle_size ; i++){
        Triangle triangle;
        triangle.material_id =  mesh.material_id;
        triangle.indices = mesh.faces[i];
        double temp = intersectTriangle(r, triangle, vertices);
        if(temp != -1 && temp<t_min)
        {
            t_min = temp;
            myTriangle = triangle;
        }
    }

    if(t_min == 99999) return -1;
    else
    {       
        Vec3f a = vertices[myTriangle.indices.v0_id-1];
        Vec3f b = vertices[myTriangle.indices.v1_id-1];
        Vec3f c = vertices[myTriangle.indices.v2_id-1];
        normal = b.substract(a).multC(c.substract(a)).normalize();
        return t_min;
    }
}

Ray generateRay(int i, int j,const Camera &currentCamera, Vec3f q, Vec3f u, Vec3f v)
{
    Ray ray;

    double left = currentCamera.near_plane.x;
    double right = currentCamera.near_plane.y;
    double bottom = currentCamera.near_plane.z;
    double top = currentCamera.near_plane.w;
    int nx = currentCamera.image_width;
    int ny = currentCamera.image_height;
    
	double su = (j+0.5)*(right - left)/nx;
	double sv = (i+0.5)*(top - bottom)/ny;

    Vec3f s = q.add(u.multS(su).add(v.multS(-sv)));
    ray.origin = currentCamera.position;
    ray.direction = s.substract(ray.origin).normalize();
    ray.isShadow  = false;
    return ray;
}

Color computeColor(const Scene &scene, const Camera &currentCamera, const Ray &ray, int &maxDepth, int &primary)
{
    Color pixelRGB;
    if(maxDepth < 0){
        pixelRGB.R = pixelRGB.G = pixelRGB.B = 0;
        return pixelRGB;
    }
    Sphere currSphere;
    Triangle currTriangle;
    Mesh currMesh;
    Vec3f normal, intersectionPoint;
    Ray myray = ray;
    Scene myscene = scene;
    Camera mycamera = currentCamera;
    vector<Vec3f> my_vertices = myscene.vertex_data;
    int material_id;
    double t;
    double t_min = 99999;
    int numOfPLights = scene.point_lights.size();
    int numOfSpheres = scene.spheres.size();
    int numOfTriangles = scene.triangles.size();
    int numOfMeshes = scene.meshes.size();
    for (int i = 0; i < numOfSpheres; i++)
    {
        currSphere = scene.spheres[i];
        t = intersectSphere(myray,currSphere,my_vertices);
        if (t!=-1 && t<t_min && t>=0)
        {
            t_min=t;
            intersectionPoint=myray.origin.add(myray.direction.multS(t_min));
            normal=intersectionPoint.substract(my_vertices[currSphere.center_vertex_id-1]).normalize();
            material_id = currSphere.material_id;
        }
    }

    for (int i = 0; i < numOfTriangles; i++)
    {
        currTriangle = scene.triangles[i];
        t = intersectTriangle(myray,currTriangle,my_vertices);
        if(t!=-1 && t<t_min && t>=0)
        {
            t_min = t;
            Vec3f a = my_vertices[currTriangle.indices.v0_id-1];
            Vec3f b = my_vertices[currTriangle.indices.v1_id-1];
            Vec3f c = my_vertices[currTriangle.indices.v2_id-1];
            intersectionPoint=myray.origin.add(myray.direction.multS(t_min));
            normal = b.substract(a).multC(c.substract(a)).normalize();
            material_id = currTriangle.material_id;
        }
    }

    for (int i=0 ; i<numOfMeshes ; i++)
    {
        Vec3f tempnormal;
        currMesh = scene.meshes[i];
        t = intersectMesh(myray, currMesh, my_vertices, tempnormal);
        if(t!=-1 && t<t_min && t>=0)
        {
            t_min = t;
            intersectionPoint=myray.origin.add(myray.direction.multS(t_min));
            normal = tempnormal;
            material_id = currMesh.material_id;
        }

    }
    
    if(t_min != 99999)
    {
        Material mymaterial = myscene.materials[material_id-1];
        pixelRGB.R = round(mymaterial.ambient.x * myscene.ambient_light.x);
        pixelRGB.G = round(mymaterial.ambient.y * myscene.ambient_light.y);
        pixelRGB.B = round(mymaterial.ambient.z * myscene.ambient_light.z);
    
        for(int i=0 ; i<numOfPLights ; i++)
        {
            PointLight currPLight = myscene.point_lights[i];
            Vec3f wi = currPLight.position.substract(intersectionPoint);
            wi = wi.normalize();
            Vec3f intersectionPointwithEpsilon = intersectionPoint.add(normal.multS(myscene.shadow_ray_epsilon)); 
            Ray shadowRay = {intersectionPointwithEpsilon, wi, false};

            double tPLight = currPLight.position.substract(shadowRay.origin).x / shadowRay.direction.x;
            
            for(int i=0 ; i<numOfSpheres ; i++)
            {
                currSphere = myscene.spheres[i];
                t = intersectSphere(shadowRay,currSphere,my_vertices);
                if (t!=-1 && t>myscene.shadow_ray_epsilon && t<tPLight)
                {
                    shadowRay.isShadow = true;
                    break;
                }
            }
            
            if(shadowRay.isShadow == false)
            {
                for(int i=0; i<numOfTriangles;i++)
                {
                    currTriangle = myscene.triangles[i];
                    t = intersectTriangle(shadowRay, currTriangle, my_vertices);           
                    if (t!=-1 && t>myscene.shadow_ray_epsilon &&t<tPLight)
                    {
                        shadowRay.isShadow = true;
                        break;
                    }
                }
            }
            if(shadowRay.isShadow == false)
            {
                for(int i=0; i<numOfMeshes; i++)
                {
                    currMesh = myscene.meshes[i];
                    Vec3f tempNormal;
                    t = intersectMesh(shadowRay, currMesh, my_vertices, tempNormal);
                    if (t!=-1 && t>myscene.shadow_ray_epsilon &&t<tPLight)
                    {
                        shadowRay.isShadow = true;
                        break;
                    }
                }
            }
            
            ///// DIFFUSE HESAPLAMA/////
            double costeta=normal.dot(wi);
            if (costeta<0) costeta=0;
            double d = currPLight.position.substract(intersectionPoint).length();
            Vec3f intensityOverDistance;
            intensityOverDistance.x = currPLight.intensity.x / (d*d);
            intensityOverDistance.y = currPLight.intensity.y / (d*d);
            intensityOverDistance.z = currPLight.intensity.z / (d*d);
            if(!shadowRay.isShadow)
            {
                pixelRGB.R += round(mymaterial.diffuse.x * costeta * intensityOverDistance.x);
                pixelRGB.G += round(mymaterial.diffuse.y * costeta * intensityOverDistance.y);
                pixelRGB.B += round(mymaterial.diffuse.z * costeta * intensityOverDistance.z);

            }


            ///// SPECULAR HESAPLAMA/////
            Vec3f halfVector = wi.substract(myray.direction).normalize();
            costeta = normal.dot(halfVector);
            if (costeta < 0) costeta = 0;
            if(costeta>0 && !shadowRay.isShadow)
            {
                Vec3f specRefCof = mymaterial.specular;
                double phongExp = mymaterial.phong_exponent;
                pixelRGB.R += round(specRefCof.x * pow(costeta, phongExp) * intensityOverDistance.x);
                pixelRGB.G += round(specRefCof.y * pow(costeta, phongExp) * intensityOverDistance.y);
                pixelRGB.B += round(specRefCof.z * pow(costeta, phongExp) * intensityOverDistance.z);
            }
        }        

        if (mymaterial.is_mirror)
        {
            double wi = -2 * myray.direction.dot(normal);
            Vec3f reflectionRayDirection = normal.multS(wi).add(myray.direction).normalize();
            Ray reflectionRay = { intersectionPoint.add(normal.multS(myscene.shadow_ray_epsilon)), reflectionRayDirection, false};
            int reflect=0;
            int newDepth = maxDepth-1;
            Color newcol = computeColor(myscene,mycamera,reflectionRay, newDepth, reflect);
            pixelRGB.R += round(newcol.R * mymaterial.mirror.x);
            pixelRGB.G += round(newcol.G * mymaterial.mirror.y);
            pixelRGB.B += round(newcol.B * mymaterial.mirror.z);
        }

    }
    else if(primary) 
    {
        pixelRGB.R = myscene.background_color.x;
        pixelRGB.G = myscene.background_color.y;
        pixelRGB.B = myscene.background_color.z;
    }
    else pixelRGB.B = pixelRGB.G = pixelRGB.R = 0;
    
    return pixelRGB;
}

int main(int argc, char* argv[])
{
    Scene scene;
    scene.loadFromXml(argv[1]);
    vector<Vec3f> vertices = scene.vertex_data;
    int maxRecDepth = scene.max_recursion_depth;
    int numberOfCameras = scene.cameras.size();

    for(int cameraNumber = 0; cameraNumber < numberOfCameras; cameraNumber++)
        {
            Camera currentCamera = scene.cameras[cameraNumber];
            int width = currentCamera.image_width;
            int height = currentCamera.image_height;
            int numberOfLights = scene.point_lights.size();

            unsigned char* image = new unsigned char [width * height * 3];
            int pixelNumber = 0;


            //////////////////////////////////////////////////////////////////
            Vec3f v = currentCamera.up;
            Vec3f gaze = currentCamera.gaze;
            Vec3f u = gaze.multC(v).normalize();
            gaze.normalize();
            v.normalize();

            double distance = currentCamera.near_distance;

            Vec3f m = currentCamera.position.add(gaze.multS(distance));
            Vec3f q = m.add(u.multS(currentCamera.near_plane.x).add(v.multS(currentCamera.near_plane.w)));
            //////////////////////////////////////////////////////////////////

            for(int i = 0; i < height; i++)
            {
                for(int j = 0; j < width; j++)
                {
                    /*********GENERATE RAYS*********/
                    int primary = 1;
                    Ray ray = generateRay(i, j, currentCamera, q, u, v);

                    Color pixelRGB = computeColor(scene, currentCamera, ray, maxRecDepth, primary);

                    if(pixelRGB.R > 255) image[pixelNumber] = 255;
                    else image[pixelNumber] = round(pixelRGB.R);

                    if(pixelRGB.G > 255) image[pixelNumber + 1] = 255;
                    else image[pixelNumber + 1] = round(pixelRGB.G);

                    if(pixelRGB.B > 255) image[pixelNumber + 2] = 255;
                    else image[pixelNumber + 2] = round(pixelRGB.B);

                    pixelNumber += 3;
                }
            }
            write_ppm(currentCamera.image_name.c_str(), image, width, height);
        }
    }