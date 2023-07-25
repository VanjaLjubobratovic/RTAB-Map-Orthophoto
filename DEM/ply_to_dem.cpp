#include <iostream>
#include <string>

#include <pcl-1.12/pcl/common/common.h>
#include <pcl-1.12/pcl/point_cloud.h>
#include <pcl-1.12/pcl/point_types.h>
#include <pcl-1.12/pcl/io/ply_io.h>
#include <pcl-1.12/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.12/pcl/filters/statistical_outlier_removal.h>

#include <opencv4/opencv2/opencv.hpp>

using namespace std;

pcl::PointXYZRGB calculateCentroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    //--- CALCULATING CENTROID ---
    pcl::PointXYZRGB centroid(0, 0, 0, 0, 0, 0);

    for (const auto point : cloud->points) {
        centroid.x += point.x;
        centroid.y += point.y;
        centroid.z += point.z;
    }

    centroid.x /= cloud->size();
    centroid.y /= cloud->size();
    centroid.z /= cloud->size();

    return centroid;
}

void filtrationViz(pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered, pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw) {
    //--- POINTCLOUD VISUALIZATION ---
    pcl::visualization::PCLVisualizer::Ptr viewer_filtered(new pcl::visualization::PCLVisualizer("Point Cloud FILTER"));
    pcl::visualization::PCLVisualizer::Ptr viewer_raw(new pcl::visualization::PCLVisualizer("Point Cloud RAW"));

    viewer_filtered->addPointCloud<pcl::PointXYZRGB>(filtered, "cloud_filtered");
    viewer_raw->addPointCloud<pcl::PointXYZRGB>(raw, "cloud_raw");
    
    viewer_filtered->setBackgroundColor(0.0, 0.0, 0.0);
    viewer_raw->setBackgroundColor(0.0, 0.0, 0.0);

    auto centroid = calculateCentroid(filtered);

    viewer_filtered->setCameraPosition(centroid.x, centroid.y, centroid.z + 10.0,
                              centroid.x, centroid.y, centroid.z,
                              0, -1, 0);

    viewer_raw->setCameraPosition(centroid.x, centroid.y, centroid.z + 10.0,
                              centroid.x, centroid.y, centroid.z,
                              0, -1, 0);

    viewer_filtered->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_filtered");
    viewer_filtered->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "cloud_filtered");
    viewer_filtered->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "cloud_filtered"); // green

    viewer_raw->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_raw");
    viewer_raw->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "cloud_raw");
    viewer_raw->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud_raw"); //red

    while(!viewer_filtered->wasStopped() || !viewer_raw->wasStopped()) {
        viewer_filtered->spin();
        viewer_raw->spin();
    }

}

int main() {
    string plyPath = "/home/vanja/Desktop/CLOUD/room_test/cloud_nofilter.ply";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if(pcl::io::loadPLYFile<pcl::PointXYZRGB>(plyPath, *cloud) == -1) {
        cerr << "ERROR: Unable to load PLY file." << endl;
        return -1;
    }

    //--- FILTERING OUTLIERS ---
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50); // Number of neighbors to use for mean distance estimation
    sor.setStddevMulThresh(0.3); // Standard deviation multiplier for distance thresholding

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    sor.filter(*cloud_filtered);


    //--- VISUALIZATION OF POINTCLOUDS ---
    //filtrationViz(cloud_filtered, cloud);

    //--- GENERATING DEM ---
    double grid_resolution = 0.008;
    float idw_power = 2.0;

    pcl::PointXYZRGB min, max;
    pcl::getMinMax3D(*cloud_filtered, min, max);

    int rows = ceil((max.y - min.y) / grid_resolution);
    int cols = ceil((max.x - min.x) / grid_resolution);

    cv::Mat heightmap(rows, cols, CV_32FC1, cv::Scalar::all(numeric_limits<float>::quiet_NaN()));

    for(const auto& point : cloud_filtered->points) {
        int col = floor((point.x - min.x) / grid_resolution);
        int row = floor((point.y - min.y) / grid_resolution);

        if(col >= 0 && col < cols && row >= 0 && row < rows) {
            if(isnan(heightmap.at<float>(row, col)) || point.z > heightmap.at<float>(row, col)) {
                heightmap.at<float>(row, col) = point.z;
            }
        }
    }

    cv::flip(heightmap, heightmap, 1); //Flip along the Y-axis
    cv::normalize(heightmap, heightmap, 0, 255, cv::NORM_MINMAX, CV_8UC1); //fixes the issue of a lot of points "missing"
    cv::imshow("DEM", heightmap);
    cv::waitKey(0);

    return 0;
}