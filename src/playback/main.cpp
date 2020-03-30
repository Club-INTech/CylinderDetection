//
// Rejoue un fichier sauvegardé.
//
#include <librealsense2/rs.hpp>
#include <mutex>
#include <cstring>
#include "../opengl_helper.h"
#include "../CameraLocation.h"
#include "../cylinder_detection.h"

int main() {
    printf("Playback file:\n");
    char filename[1024];
    fgets(filename, 1024, stdin);

    strtok(filename, "\n"); // retrait du retour à la ligne à la fin du nom de fichier

    printf("Reading from file %s\n", filename);
    printf("Initializing realsense2\n");
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::colorizer falseColors;

    cfg.enable_device_from_file(filename);
    cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR, RS2_FORMAT_RGB8);
    cfg.enable_stream(rs2_stream::RS2_STREAM_DEPTH, RS2_FORMAT_Z16);
    cfg.enable_stream(rs2_stream::RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(rs2_stream::RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);

    printf("Initializing GLFW\n");
    vis::window* glFrame = vis::createWindow("Hello world", 640, 480);

    initRotationEstimator();

    printf("Starting acquisition\n");

    auto profile = pipe.start(cfg);

    printf("Starting rendering\n");
    CylinderDetection* detector = new CylinderDetection();

    while(vis::keepOpen(glFrame)) {
        auto frames = pipe.wait_for_frames();

        for(auto frame : frames) {
            estimateCameraPositionRotation(frame);

            auto video = frame.as<rs2::video_frame>();
            if(video && frame.get_profile().stream_type() == RS2_STREAM_COLOR) {
                vis::uploadVideoFrame(video);
            }
            auto depth = frame.as<rs2::depth_frame>();
            if(depth && frame.get_profile().stream_type() == RS2_STREAM_DEPTH) {
                rs2::video_frame colorized = falseColors.colorize(depth);
                vis::uploadDepthFrame(colorized);
            }
        }

        float xRotation = get_rotation().x;
        float yRotation = get_rotation().y;
        float zRotation = get_rotation().z;
        char s[300];
        sprintf(s,"x = %f, y = %f, z = %f\n", xRotation, yRotation, zRotation);
        printf("%s", s);

        vis::render(glFrame);
    }

    vis::cleanup(glFrame);
    pipe.stop();


    return 0;

}