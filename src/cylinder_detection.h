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

struct Cylinder {
    float x;
    float y;
    float z;
    float radius;
};

class CylinderDetection {
private:
    OrientationAccumulatorSpace* space = new OrientationAccumulatorSpace(900, 10);
    CylinderFittingHough* hough = new CylinderFittingHough(*space);

public:
    CylinderDetection() = default;
    ~CylinderDetection() = default;

    std::vector<Cylinder> findCylinders(rs2::points& points);
};
#endif //CYLINDERDETECTION_CYLINDER_DETECTION_H
