#include <iostream>
#include <librealsense2/rs.hpp>
#include "opengl_helper.h"
#include "CameraLocation.h"
#include "cylinder_detection.h"

int main() {
    printf("Initializing realsense2\n");
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::colorizer falseColors;
    cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR);
    cfg.enable_stream(rs2_stream::RS2_STREAM_DEPTH);
    cfg.enable_stream(rs2_stream::RS2_STREAM_GYRO,RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(rs2_stream::RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);

    printf("Initializing GLFW\n");
    vis::window* glFrame = vis::createWindow("Hello world", 640, 480);

    initRotationEstimator();

    printf("Starting acquisition\n");
    auto profile = pipe.start(cfg, [&](rs2::frame frame)
    {
        estimateCameraPositionRotation(frame);

        auto video = frame.as<rs2::video_frame>();
        auto depth = frame.as<rs2::depth_frame>();

        if(video) {
            vis::uploadVideoFrame(video);
        }
        if(depth) {
            rs2::video_frame colorized = falseColors.colorize(depth);
            vis::uploadDepthFrame(colorized);
        }

        float xRotation = get_rotation().x;
        float yRotation = get_rotation().y;
        float zRotation = get_rotation().z;
        char s[300];
        sprintf(s,"x = %f, y = %f, z = %f\n", xRotation, yRotation, zRotation);
        printf("%s", s);
    });

    CylinderDetection* detector = new CylinderDetection();

    while(vis::keepOpen(glFrame)) {
        vis::render(glFrame);
    }

    vis::cleanup(glFrame);

    return 0;

}
