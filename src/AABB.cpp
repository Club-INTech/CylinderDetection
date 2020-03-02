//
// Created by jglrxavpok on 02/03/2020.
//

#include "AABB.h"

AABB::AABB(float minX, float minY, float minZ, float maxX, float maxY, float maxZ): minX(minX), minY(minY), minZ(minZ),
maxX(maxX), maxY(maxY), maxZ(maxZ) {

}

bool AABB::isPointIn(float x, float y, float z) const {
    return x >= minX && x <= maxX && y >= minY && y <= maxY && z >= minZ && z <= maxZ;
}

bool AABB::intersects(const AABB &other) const {

    return !(minX > other.maxX || maxX < other.minX
            || minY > other.maxY || maxY < other.minY
            || minZ > other.maxZ || maxZ < other.minZ
    );
}
