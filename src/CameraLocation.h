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

    float3 operator*(const float dt){
        return {x*dt, y*dt, z*dt};
    }

    float3 operator+(const float3 pos){
        return {x+pos.x, y+pos.y, z+pos.z};
    }
};


void initRotationEstimator();
void estimateCameraPositionRotation(rs2::frame&);
float3 get_rotation();
float3 get_position();
void calc_position(float3);
#endif //CYLINDERDETECTION_CAMERALOCATION_H
