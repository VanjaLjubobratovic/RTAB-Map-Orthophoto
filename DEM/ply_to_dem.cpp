#include <iostream>
#include <string>

#include <pcl-1.12/pcl/point_cloud.h>
#include <pcl-1.12/pcl/point_types.h>
#include <pcl-1.12/pcl/io/ply_io.h>
#include <pcl-1.12/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.12/pcl/filters/statistical_outlier_removal.h>

using namespace std;

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
    sor.setStddevMulThresh(1.0); // Standard deviation multiplier for distance thresholding

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    sor.filter(*cloud_filtered);
    

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
    

    //--- POINTCLOUD VISUALIZATION ---
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));

    viewer->addPointCloud<pcl::PointXYZRGB>(cloud_filtered, "cloud_filtered");
    
    viewer->setBackgroundColor(0.0, 0.0, 0.0);

    viewer->setCameraPosition(centroid.x, centroid.y, centroid.z + 10.0,
                              centroid.x, centroid.y, centroid.z,
                              0, -1, 0);

    while(!viewer->wasStopped()) {
        viewer->spin();
    }

    return 0;
}