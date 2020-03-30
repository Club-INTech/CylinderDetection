//
// Created by jglrxavpok on 24/02/2020.
//

#include "cylinder_detection.h"

CylinderDetection::CylinderDetection() {
    namedWindow("Test OpenCV - Color", WINDOW_AUTOSIZE);
    namedWindow("Test OpenCV - Depth", WINDOW_AUTOSIZE);
}

void CylinderDetection::detect() {
    rs2::frame frame;
    rs2::video_frame video{frame};
    if(this->video_queue.poll_for_frame(&video)) {
        Mat color(Size(video.get_width(), video.get_height()), CV_8UC3, (void*)video.get_data(), Mat::AUTO_STEP);
        Mat bgrColor;
        cvtColor(color, bgrColor, COLOR_RGB2BGR); // conversion realsense->OpenCV

        Mat channels[3];
        split(bgrColor, channels); // séparation en canaux

        int channel = 2;

        float thresholdValue = 0.4f;
        Mat onlyChannel;
        if(channel == 2) {
            onlyChannel = channels[channel] - (channels[1] + channels[0]) / 2; // on soustrait les composantes bleu et vert, pour ne récupérer que ce qui est très rouge
        } else if(channel == 1) {
            onlyChannel = channels[channel] - (channels[2] + channels[0]) / 2; // on soustrait les composantes bleu et rouge, pour ne récupérer que ce qui est très vert
            thresholdValue = 0.2f;
        } else {
            fprintf(stderr, "Canal non valide: %d (seuls 1 et 2 sont supportés)\n", channel);
            exit(EXIT_FAILURE);
        }

        Mat brightChannel;
        threshold(onlyChannel, brightChannel, thresholdValue * 255, 255, THRESH_TOZERO);

        imshow("Test OpenCV - Color", brightChannel);

        rs2::video_frame depth{frame};
        if(this->depth_queue.poll_for_frame(&depth)) {
            Mat color(Size(depth.get_width(), depth.get_height()), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
            Mat bgrColor;
            cvtColor(color, bgrColor, COLOR_RGB2BGR); // conversion realsense->OpenCV

            Mat resized;
            resize(bgrColor, resized, Size(video.get_width(), video.get_height())); // agrandit l'image pour être à la même taille que le masque (nécessaire pour copyTo)

            Mat regionOfInterest;
            copyTo(resized, regionOfInterest, brightChannel); // on utilise les pixels potentiels de gobelets pour récupérer la profondeur qui nous intéresse
            imshow("Test OpenCV - Depth", regionOfInterest);
            waitKey(1);
        }

        waitKey(1);
    }

}

void CylinderDetection::newDepthFrame(rs2::video_frame& frame) {
    this->depth_queue.enqueue(frame);
}

void CylinderDetection::newVideoFrame(rs2::video_frame& frame) {
    this->video_queue.enqueue(frame);
}