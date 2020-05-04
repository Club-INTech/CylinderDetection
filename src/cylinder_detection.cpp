//
// Created by jglrxavpok on 24/02/2020.
//

#include "cylinder_detection.h"

CylinderDetection::CylinderDetection() {
    namedWindow("Test OpenCV - Color", WINDOW_AUTOSIZE);
    namedWindow("Test OpenCV - Depth", WINDOW_AUTOSIZE);

    this->falseColors.set_option(RS2_OPTION_COLOR_SCHEME, 2.0f); // White to Black
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

        Mat redChannel = extract(channels, 2);
        Mat greenChannel = extract(channels, 1);

        Mat cups = redChannel + greenChannel;

        imshow("Test OpenCV - Color", cups);

        rs2::depth_frame depth{frame};
        if(this->depth_queue.poll_for_frame(&depth)) {
            rs2::video_frame colorizedDepth = falseColors.process(depth);
            Mat color(Size(colorizedDepth.get_width(), colorizedDepth.get_height()), CV_8UC3, (void*)colorizedDepth.get_data(), Mat::AUTO_STEP);
            Mat depthImage;
            cvtColor(color, depthImage, COLOR_RGB2BGR); // conversion realsense->OpenCV

            Mat resized;
            resize(depthImage, resized, Size(video.get_width(), video.get_height())); // agrandit l'image pour être à la même taille que le masque (nécessaire pour copyTo)

            std::unique_lock lock(shapesMutex);
            shapes.clear();
            for(int regionIndex = 0; regionIndex < 2;regionIndex++) {

                Mat regionOfInterest;
                Mat channel = regionIndex == 0 ? redChannel : greenChannel;
                CylinderColor cylinderColor = regionIndex == 0 ? CylinderColor::Red : CylinderColor::Green;
                resized.copyTo(regionOfInterest, channel); // on utilise les pixels potentiels de gobelets pour récupérer la profondeur qui nous intéresse

                /*
                // Application de K-Means pour séparer les différents objets
                Mat samples;
                regionOfInterest.convertTo(samples, CV_32F);
                samples = samples.reshape(1, samples.total());
                int maxClusters = 5; // TODO: configurable or dynamic?

                Mat centers;
                Mat labels;
                kmeans(samples, maxClusters, labels, TermCriteria(TermCriteria::EPS+TermCriteria::COUNT, 5, 2.0), 1, KMEANS_PP_CENTERS, centers);

                // https://answers.opencv.org/question/182006/opencv-c-k-means-color-clustering/
                centers = centers.reshape(3, centers.rows); // convert to row of Vec3f
                Mat data = samples.reshape(3, samples.rows);

                Vec3f* pixelsVector = data.ptr<Vec3f>();
                for (size_t i=0; i<data.rows; i++) {
                    int center_id = labels.at<int>(i);
                    pixelsVector[i] = centers.at<Vec3f>(center_id);
                }

                // back to 2d, and uchar:
                Mat ocv = data.reshape(3, regionOfInterest.rows);
                ocv.convertTo(ocv, CV_8U);
    */

                /*    Mat extractedCups[maxClusters];
                    for (int clusterID = 0; clusterID < maxClusters; ++clusterID) { // initialisation des différents plans
                        Mat image = Mat(Size(regionOfInterest.cols, regionOfInterest.rows), regionOfInterest.type());
                        image.convertTo(extractedCups[clusterID], CV_8UC3);
                    }

                    for (size_t i=0; i<data.rows; i++) {
                        int centerID = labels.at<int>(i);
                        Mat correspondingImage = extractedCups[centerID];
                        Vec3f pixel = data.at<Vec3f>(i);
                        correspondingImage.at<Vec3b>(i) = Vec3b(pixel[0]*255, pixel[1]*255, pixel[2]*255);
                    }*/

                // recherche des contours
                std::vector<std::vector<Point> > contours;
                std::vector<Vec4i> hierarchy;
                Mat canny_output;
                int thresh = 200;
                Canny(channel, canny_output, thresh, thresh*2, 5 );
                /// Find contours
                findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

                for( int i = 0; i< contours.size(); i++ )
                {
                    std::vector<Point> contour = contours[i];
                    double area = contourArea(contour);
                    Moments contourMoments = moments(contour);
                    // https://docs.opencv.org/2.4/doc/tutorials/imgproc/shapedescriptors/moments/moments.html
                    Point2f center = Point2f(contourMoments.m10/contourMoments.m00, contourMoments.m01/contourMoments.m00);
                    Scalar color = Scalar( 0,255*regionIndex,255*(1-regionIndex)); // BGR
                    drawContours( depthImage, contours, i, color, 2, 8, hierarchy, 0, Point() );
                    circle(depthImage, center, 4, color, -1, 8 ,0);

                    if(center.x > 0.0f && center.x < depth.get_width() && center.y > 0.0f && center.y < depth.get_height()) {
                        double dist = depth.get_distance(center.x, center.y);
                        char distance[50];
                        sprintf(distance, "%fm", dist);
                        putText(depthImage, distance, center, FONT_HERSHEY_SIMPLEX, 1.0, color);

                        // TODO: calculer ces valeurs correctement
                        float cylinderCenterX = center.x;
                        float cylinderCenterY = center.y;
                        float radius = 1.0f;
                        Cylinder cylinder { .x = cylinderCenterX, .y = cylinderCenterY, .z=(float)dist, .radius=radius, .color=cylinderColor };
                        shapes.push_back(cylinder);
                    }
                }

            }
            lock.unlock(); // obligé de unlock ici car il peut y avoir des calculs après qui n'ont rien à voir

            imshow("Segmentation", depthImage);
        }
        waitKey(1);
    }


}

void CylinderDetection::newDepthFrame(rs2::depth_frame& frame) {
    this->depth_queue.enqueue(frame);
}

void CylinderDetection::newVideoFrame(rs2::video_frame& frame) {
    this->video_queue.enqueue(frame);
}

Mat CylinderDetection::extract(Mat *channels, int index) {
    float thresholdValue = 0.4f;
    Mat onlyChannel;
    if(index == 2) {
        onlyChannel = channels[index] - (channels[1] + channels[0]) / 2; // on soustrait les composantes bleu et vert, pour ne récupérer que ce qui est très rouge
    } else if(index == 1) {
        onlyChannel = channels[index] - (channels[2] + channels[0]) / 2; // on soustrait les composantes bleu et rouge, pour ne récupérer que ce qui est très vert
        thresholdValue = 0.2f;
    } else {
        fprintf(stderr, "Canal non valide: %d (seuls 1 et 2 sont supportés)\n", index);
        exit(EXIT_FAILURE);
    }

    Mat brightChannel;
    threshold(onlyChannel, brightChannel, thresholdValue * 255, 255, THRESH_BINARY);

    Mat result;
    Mat element = getStructuringElement(MORPH_RECT, Size(30,30)); // TODO: configurable
    erode(brightChannel, result, element);
    return result;
}

void CylinderDetection::createPacket(std::string& packet) {
    packet.clear();

    // TODO: trouver un moyen propre d'envoyer l'orientation
    packet += "!§"; // channel
    packet += "0 0 0 ";

    // on évite de lire n'importe quoi
    std::shared_lock lock(shapesMutex);
    packet += std::to_string(shapes.size());
    packet += " ";
    for(const auto& shape : shapes) {
        packet += "cylinder ";
        switch(shape.color) {
            case Green:
                packet += "green ";
                break;

            case Red:
                packet += "red ";
                break;
        }

        // TODO: calcul de la hauteur
        packet += "1 ";
        packet += std::to_string(shape.x);
        packet += " ";
        packet += std::to_string(shape.y);
        packet += " ";
        packet += std::to_string(shape.z);
        packet += " ";

        // TODO: calcul de la position sur la table
        packet += std::to_string(shape.x);
        packet += " ";
        packet += std::to_string(shape.y);
        packet += " ";
        packet += std::to_string(shape.z);
        packet += " ";

    }

    packet += "\n";
}
