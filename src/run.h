//
// Created by jglrxavpok on 30/03/2020.
//

#ifndef CYLINDERDETECTION_RUN_H
#define CYLINDERDETECTION_RUN_H

#include <librealsense2/rs.hpp>
#include <mutex>
#include <cstring>
#include "opengl_helper.h"
#include "CameraLocation.h"
#include "cylinder_detection.h"

/// Permet de lancer le programme, avec playback ou live
/// Mettre 'filename' à NULL ou nullptr lance le mode live
void run(const char* filename);
#endif //CYLINDERDETECTION_RUN_H
