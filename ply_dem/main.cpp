#include <iostream>
#include <string>
#include <thread>
#include <cmath>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <opencv2/opencv.hpp>
#include <opencv2/flann.hpp>
#include <opencv2/core/eigen.hpp>

#include <Eigen/Geometry>

#include "mosaicing_tools.h"

int main(int, char**){
    std::string plyPath = "/home/vanja/Desktop/Tests/HA720p5mmRTAB.ply";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if(pcl::io::loadPLYFile<pcl::PointXYZRGB>(plyPath, *cloud) == -1) {
        cerr << "ERROR: Unable to load PLY file." << endl;
        return -1;
    }

    MosaicingTools::filterCloud(cloud, cloud, 50, 1.5, 1);

    auto mosaic = MosaicingTools::generateMosaic(cloud, 0.005, 16);

    cv::namedWindow("OFFLINE MOSAIC", cv::WINDOW_GUI_EXPANDED);
    cv::resizeWindow("OFFLINE MOSAIC", 1280, 720);

    cv::imwrite("../mosaic.png", mosaic);

    return 0;
}
