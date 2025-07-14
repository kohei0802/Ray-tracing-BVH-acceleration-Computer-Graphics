//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_INTERSECTION_H
#define RAYTRACING_INTERSECTION_H
#include "Vector.hpp"
#include "Material.hpp"
class Object;
class Sphere;

struct Intersection
{
    Intersection(){
        happened=false;
        coords=Vector3f();
        normal=Vector3f();
        distance= std::numeric_limits<double>::max();
        obj =nullptr;
        m=nullptr;
    }
    // there was a hit
    bool happened;
    // where the intersection
    Vector3f coords;
    // normal of the point on surface
    Vector3f normal;
    // t
    double distance;
    // object being hit
    Object* obj;
    // material of hit object
    Material* m;
};
#endif //RAYTRACING_INTERSECTION_H
