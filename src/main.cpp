#include <iostream>
#include <librealsense2/rs.hpp>
#include <zconf.h>
//#include "cylinder_fitting.h"
#include "opengl_helper.h"
#include "CameraLocation.h"
#include "cylinder_detection.h"


int main() {
    printf("Initializing realsense2\n");
    rs2::pointcloud pc = rs2::pointcloud();
    rs2::points points;
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR);
    cfg.enable_stream(rs2_stream::RS2_STREAM_DEPTH);
    cfg.enable_stream(rs2_stream::RS2_STREAM_GYRO);
    cfg.enable_stream(rs2_stream::RS2_STREAM_ACCEL);
    //pipe.start(cfg);

    printf("Initializing GLFW\n");
    //vis::window* glFrame = vis::createWindow("Hello world", 640, 480);

    initRotationEstimator(pipe);
    while(true){
        sleep(1.5);
        float xRotation = get_rotation().x;
        float yRotation = get_rotation().y;
        float zRotation = get_rotation().z;
        char s[300];
        sprintf(s,"x = %f, y = %f, z = %f\n", xRotation, yRotation, zRotation);
        printf("%s", s);
    }

/*

    CylinderDetection* detector = new CylinderDetection();

    while(vis::keepOpen(glFrame)) {
        // from https://github.com/IntelRealSense/librealsense/tree/master/examples/pointcloud
        auto data = pipe.wait_for_frames(); // Wait for next set of frames from the camera

        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        auto depth = frames.get_depth_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);

        detector->findCylinders(points);

        auto color = frames.get_color_frame();

        // Tell pointcloud object to map to this color frame
        pc.map_to(color);

        // Upload the color frame to OpenGL
        vis::uploadColorFrame(color);

        vis::setupMatrices();
        vis::renderCylinder(0.0f, 0.0f, 10.0f, // position
                0.0f, 1.0f, 0.0f, // normal
                2.0f, // height
                0.25f, // radius
                0.0f, 1.0f, 0.0f); // color
        vis::renderPointCloud(points);
        vis::endFrame(glFrame);
    }

    vis::cleanup(glFrame);
    */
    return 0;

}
