//
// Permet d'enregistrer les données de la caméra vers un fichier
//
#include <librealsense2/rs.hpp>
#include <mutex>
#include <cstring>
#include "../opengl_helper.h"
#include "../CameraLocation.h"
#include "../cylinder_detection.h"

int main() {
    printf("Initializing realsense2\n");
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::colorizer falseColors;

    char filename[100];
    time_t t = time(0);
    struct tm * now = localtime( & t );
    strftime(filename, 80, "%Y-%m-%d-%H-%M-%S", now);

    strcat(filename, ".bin");
    cfg.enable_record_to_file(filename);
    cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR, RS2_FORMAT_RGB8);
    cfg.enable_stream(rs2_stream::RS2_STREAM_DEPTH, RS2_FORMAT_Z16);
    cfg.enable_stream(rs2_stream::RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(rs2_stream::RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);

    printf("Initializing GLFW\n");
    vis::window* glFrame = vis::createWindow("Hello world", 640, 480);

    printf("Starting acquisition\n");

    auto profile = pipe.start(cfg);

    printf("Starting rendering\n");
    CylinderDetection* detector = new CylinderDetection();

    // juste pour visualiser ce qu'on fait
    while(vis::keepOpen(glFrame)) {
        auto frames = pipe.wait_for_frames();

        for(auto frame : frames) {
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

        vis::render(glFrame);
    }

    vis::cleanup(glFrame);
    pipe.stop();
    printf("File written to %s\n", filename);


    return 0;

}