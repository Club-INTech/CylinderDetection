//
// Created by serandour on 24/02/2020.
//

#ifndef CYLINDERDETECTION_CAMERALOCATION_H
#define CYLINDERDETECTION_CAMERALOCATION_H

#include <iostream>
#include <chrono>
#include <ctime>

struct float3
{
    float x, y, z;

    float3 operator+(const float3 pos){
        return {x+pos.x, y+pos.y, z+pos.z};
    }

    float3 operator*(float t)
    {
        return { x * t, y * t, z * t };
    }

    float3 operator-(float t)
    {
        return { x - t, y - t, z - t };
    }

    void operator*=(float t)
    {
        x = x * t;
        y = y * t;
        z = z * t;
    }

    void operator=(float3 other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
    }

    void add(float t1, float t2, float t3)
    {
        x += t1;
        y += t2;
        z += t3;
    }
};

void initRotationEstimator();
void estimateCameraPositionRotation(rs2::frame&);
void get_rotation(float3*);
void get_position(float3*);
void calc_position(float3);
#endif //CYLINDERDETECTION_CAMERALOCATION_H
