//
// Created by jglrxavpok on 24/02/2020.
//

#include "cylinder_detection.h"

std::vector<Cylinder> CylinderDetection::findCylinders(rs2::points& points) {
    // convert RealSensense2 points to PCL clouds
    pcl::PointCloud<PointT>::Ptr pointCloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<pcl::Normal>::Ptr estimatedNormals(new pcl::PointCloud<pcl::Normal>());

    const int undersampling = 100;
    const rs2::vertex* vertices = points.get_vertices();
    for (int pointIndex = 0; pointIndex < points.size(); ++pointIndex) {
        if(pointIndex % undersampling != 0) {
            continue;
        }
        const rs2::vertex vertex = vertices[pointIndex];

        PointT* point = new PointT(vertex.x, vertex.y, vertex.z);
        pointCloud.get()->push_back(*point);
    }
    printf("%li vertices from camera\n", points.size());
    printf("Undersampled to %li vertices\n", points.size()/undersampling);

    static pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    static pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setRadiusSearch (0.1);
    ne.setSearchMethod (tree);

    ne.setInputCloud(pointCloud);
    ne.compute(*estimatedNormals);

    printf("normals estimated\n");

    FittingData data = this->hough->fit(pointCloud, estimatedNormals);
    //printf("Found %li cylinder points\n", data.inliers->size());

    return {};
}