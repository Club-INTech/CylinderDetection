//
// Created by jglrxavpok on 02/03/2020.
//

#ifndef CYLINDERDETECTION_AABB_H
#define CYLINDERDETECTION_AABB_H

#include <cmath>

class AABB {
private:
    float minX;
    float minY;
    float minZ;
    float maxX;
    float maxY;
    float maxZ;

public:
    AABB(float minX, float minY, float minZ, float maxX, float maxY, float maxZ);

    ~AABB() = default;

    bool isPointIn(float x, float y, float z);
    bool intersects(const AABB& other);
};

static AABB INFINITE_BB(-INFINITY, -INFINITY, -INFINITY, INFINITY, INFINITY, INFINITY);


#endif //CYLINDERDETECTION_AABB_H
