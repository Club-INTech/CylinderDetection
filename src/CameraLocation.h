//
// Created by serandour on 24/02/2020.
//

#ifndef CYLINDERDETECTION_CAMERALOCATION_H
#define CYLINDERDETECTION_CAMERALOCATION_H

struct float3
{
    float x, y, z;
};

void initRotationEstimator();
float3 get_rotation();
#endif //CYLINDERDETECTION_CAMERALOCATION_H
