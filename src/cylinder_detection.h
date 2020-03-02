//
// Created by jglrxavpok on 24/02/2020.
//

#ifndef CYLINDERDETECTION_CYLINDER_DETECTION_H
#define CYLINDERDETECTION_CYLINDER_DETECTION_H

#include <vector>
#include <librealsense2/rs.hpp>
#include <shape-fitting/include/cylinder_fitting_hough.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include "constants.h"
#include "AABB.h"

struct Cylinder {
    float x;
    float y;
    float z;
    float radius;
};

class CylinderDetection {
private:
    CylinderFittingHough* hough;

public:
    CylinderDetection();
    ~CylinderDetection() = default;

    std::vector<Cylinder>* findCylinders(rs2::points& points, const AABB& searchArea = INFINITE_BB);
};
#endif //CYLINDERDETECTION_CYLINDER_DETECTION_H
