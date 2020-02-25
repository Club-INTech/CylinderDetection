//
// Created by jglrxavpok on 24/02/2020.
//

#include "cylinder_detection.h"

CylinderDetection::CylinderDetection() {
    std::vector<double> weights;
    std::vector<Eigen::Matrix<double, 3 ,1> > means;
    std::vector<Eigen::Matrix<double, 3 ,1> > std_devs;
    weights.push_back(1.0);
    Eigen::Matrix<double, 3 ,1> mean_eigen(0,0,0);
    means.push_back(mean_eigen);
    Eigen::Matrix<double, 3 ,1> std_dev_eigen(0.5,0.5,0.5);
    std_devs.push_back(std_dev_eigen);

    GaussianMixtureModel* gmm = new GaussianMixtureModel(weights,means,std_devs);
    GaussianSphere* gaussian_sphere = new GaussianSphere(*gmm, 100);
    this->hough = new CylinderFittingHough(*gaussian_sphere);
}

std::vector<Cylinder>* CylinderDetection::findCylinders(rs2::points& points) {
    // convert RealSensense2 points to PCL clouds
    pcl::PointCloud<PointT>::Ptr pointCloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<pcl::Normal>::Ptr estimatedNormals(new pcl::PointCloud<pcl::Normal>());

    const rs2::vertex* vertices = points.get_vertices();
    for (int pointIndex = 0; pointIndex < points.size(); ++pointIndex) {
        if(pointIndex % UNDERSAMPLING != 0) {
            continue;
        }
        const rs2::vertex vertex = vertices[pointIndex];
        if(vertex.z) {
            if(ABS(vertex.z) < 0.5f) { // not too far
                PointT* point = new PointT(vertex.x, vertex.y, vertex.z);
                pointCloud.get()->push_back(*point);
            }
        }
    }
    printf("%li vertices from camera\n", points.size());
    printf("Undersampled to %li vertices\n", pointCloud.get()->size());

    static pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    static pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setKSearch(6);
    ne.setSearchMethod (tree);

    ne.setInputCloud(pointCloud);
    ne.compute(*estimatedNormals);

    printf("normals estimated\n");

    FittingData data = this->hough->fit(pointCloud, estimatedNormals);
    printf("Found %li cylinder points\n", data.inliers->size());

    std::vector<Cylinder>* cylinders = new std::vector<Cylinder>;

    for(int i = 0; i < data.inliers->size(); i++) {
        PointT& point = data.inliers->at(i);
        cylinders->push_back({point.x, point.y, point.z, 0.05f}); // TODO: correct radius
    }
    return cylinders;
}