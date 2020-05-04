//
// Created by jglrxavpok on 24/02/2020.
//

#ifndef CYLINDERDETECTION_CYLINDER_DETECTION_H
#define CYLINDERDETECTION_CYLINDER_DETECTION_H

#include <vector>
#include <librealsense2/rs.hpp>
#include <mutex>  // For std::unique_lock
#include <shared_mutex>
#include "constants.h"
#include "AABB.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

enum CylinderColor {
    Red,
    Green
};

struct Cylinder {
    float x;
    float y;
    float z;
    float radius;
    CylinderColor color;
};

constexpr int QUEUE_CAPACITY = 5;

class CylinderDetection {
private:
    rs2::frame_queue video_queue{QUEUE_CAPACITY};
    rs2::frame_queue depth_queue{QUEUE_CAPACITY};
    rs2::colorizer falseColors;
    std::vector<Cylinder> shapes;
    mutable std::shared_mutex shapesMutex; // RWLock pour l'ajout des cylindres et la lecture dans 'shapes'

public:
    explicit CylinderDetection();
    ~CylinderDetection() = default;

    /// Détecte les cylindres dans l'image
    void detect();

    /// Met à jour l'information de couleur à utiliser
    void newVideoFrame(rs2::video_frame& frame);

    /// Met à jour l'information de profondeur à utiliser. Utilise une version recolorisée de la carte de profondeur
    void newDepthFrame(rs2::depth_frame& frame);

    Mat extract(Mat channels[3], int index);

    void createPacket(std::string& packet);
};
#endif //CYLINDERDETECTION_CYLINDER_DETECTION_H
