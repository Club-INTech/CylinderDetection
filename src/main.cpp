#include <iostream>
#include <librealsense2/rs.hpp>
//#include "cylinder_fitting.h"
#include "opengl_helper.h"


int main() {
    printf("Initializing realsense2\n");
    rs2::pointcloud pc = rs2::pointcloud();
    rs2::points points;
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR);
    cfg.enable_stream(rs2_stream::RS2_STREAM_DEPTH);
    pipe.start(cfg);

    printf("Initializing GLFW\n");
    vis::window* glFrame = vis::createWindow("Hello world", 640, 480);

    while(vis::keepOpen(glFrame)) {
        // from https://github.com/IntelRealSense/librealsense/tree/master/examples/pointcloud
        auto data = pipe.wait_for_frames(); // Wait for next set of frames from the camera

        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        auto depth = frames.get_depth_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);

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
    return 0;
}
