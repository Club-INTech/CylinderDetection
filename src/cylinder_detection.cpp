//
// Created by jglrxavpok on 24/02/2020.
//

#include "cylinder_detection.h"

CylinderDetection::CylinderDetection() {

}

std::vector<Cylinder>* CylinderDetection::findCylinders(rs2::frameset& frames, const AABB& searchArea) {
    std::vector<Cylinder>* cylinders = new std::vector<Cylinder>;
    return cylinders;
}