//
// Created by jglrxavpok on 30/03/2020.
//

#include "run.h"

void run(const char* filename) {
    printf("Initializing realsense2\n");
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::colorizer falseColors;
    falseColors.set_option(RS2_OPTION_COLOR_SCHEME, 2.0f); // White to Black

    if(filename) {
        cfg.enable_device_from_file(filename);
    }
    cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR, RS2_FORMAT_RGB8);
    cfg.enable_stream(rs2_stream::RS2_STREAM_DEPTH, RS2_FORMAT_Z16);
    cfg.enable_stream(rs2_stream::RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(rs2_stream::RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    printf("Initializing GLFW\n");
    vis::window* glFrame = vis::createWindow("Hello world", 640, 480);

    initRotationEstimator();

    printf("Starting acquisition\n");

    Client highLevel(address, port);
    highLevel.connect();

    auto profile = pipe.start(cfg);

    printf("Starting rendering\n");
    CylinderDetection* detector = new CylinderDetection();

    rs2::align align_to_color(RS2_STREAM_COLOR);

    std::string incomingMessage;
    std::string outcomingMessage;

    while(vis::keepOpen(glFrame)) {
        if(highLevel.receive(incomingMessage, false)) {
            // TODO: respond to incoming messages
            std::cout << "Received: " << incomingMessage << std::endl;
        }

        auto frames = pipe.wait_for_frames();

        frames = align_to_color.process(frames);

        for(auto frame : frames) {
            estimateCameraPositionRotation(frame);

            auto video = frame.as<rs2::video_frame>();
            if(video && frame.get_profile().stream_type() == RS2_STREAM_COLOR) {
                vis::uploadVideoFrame(video);

                detector->newVideoFrame(video);
            }
            auto depth = frame.as<rs2::depth_frame>();
            if(depth && frame.get_profile().stream_type() == RS2_STREAM_DEPTH) {
                rs2::video_frame colorized = falseColors.colorize(depth);
                vis::uploadDepthFrame(colorized);

                detector->newDepthFrame(depth);
            }
        }

        detector->detect();

        if(highLevel) { // si on a une connexion
            detector->createPacket(outcomingMessage);
            highLevel.send(outcomingMessage);
        }

        float3 rotation{};
        get_rotation(&rotation);
        float xRotation = rotation.x;
        float yRotation = rotation.y;
        float zRotation = rotation.z;
        char s[300];
        sprintf(s,"x = %f, y = %f, z = %f\n", xRotation, yRotation, zRotation);
     //   printf("%s", s);

        vis::render(glFrame);
    }

    vis::cleanup(glFrame);
    pipe.stop();

}