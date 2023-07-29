#include <iostream>
#include <string>
#include <thread>
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>

#include <opencv2/opencv.hpp>
#include <opencv2/flann.hpp>


cv::Mat extractDataPoints(const cv::Mat& dem) {
    cv::Mat points;

    for (int i = 0; i < dem.rows; i++) {
        for (int j = 0; j < dem.cols; j++) {
            if(!std::isnan(dem.at<float>(i, j))) {
                points.push_back(cv::Point2f(i, j));
            }
        }
    }

    points = points.reshape(1);
    points.convertTo(points, CV_32F);

    return points;
}

cv::flann::Index buildKDTree(const cv::Mat& dataPoints) {
    return cv::flann::Index(dataPoints, cv::flann::KDTreeIndexParams());
}

pcl::PointXYZRGB calculateCentroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    //---CALCULATING CENTROID---
    pcl::PointXYZRGB centroid(0, 0, 0, 0, 0, 0);

    for(const auto point : cloud->points) {
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
    //---POINTCLOUD VISUALIZATION---
    auto centroid = calculateCentroid(filtered);

    auto viewer(new pcl::visualization::PCLVisualizer("Point cloud filter vs raw"));
    viewer->initCameraParameters();

    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("STATISTICAL FILTER", 10, 10, "filtered_text", v1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(filtered);
    viewer->addPointCloud<pcl::PointXYZRGB>(filtered, rgb, "filtered_cloud", v1);

    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0, 0, 0, v2);
    viewer->addText("UNFILTERED", 10, 10, "raw_text", v2);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> single_color(raw);
    viewer->addPointCloud<pcl::PointXYZRGB>(raw, single_color, "raw_cloud", v2);

    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "filtered_cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "raw_cloud");
    viewer->addCoordinateSystem (1.0);

    viewer->setCameraPosition(centroid.x, centroid.y, centroid.z + 10,
                              centroid.x, centroid.y, centroid.z,
                              0, -1, 0);

    while(!viewer->wasStopped()) {
        viewer->spin();
    }
}

void bilinearInterpolation(cv::Mat& dem) {
    cv::Mat interpolatedDem = dem.clone();

    for(int i = 0; i < dem.rows; ++i) {
        for(int j = 0; j < dem.cols; ++j) {
            if(std::isnan(dem.at<float>(i, j))) {
                float sum = 0.0f;
                int count = 0;

                for(int x = i-1; x <= i+1; ++x) {
                    for(int y = j-1; y <= j+1; ++y) {
                        if(!std::isnan(dem.at<float>(x, y))) {
                            sum += dem.at<float>(x, y);
                            count++;
                        }
                    }
                }

                if(count > 0) {
                    interpolatedDem.at<float>(i, j) = sum / count;
                }
            }
        }
    }

    dem = interpolatedDem;
}

void nearestNeighbourInterpolation(cv::Mat& dem, cv::Mat& dataPoints, cv::flann::Index& kdTree) {
    cv::Mat interpolated = dem.clone();

    for(int i = 0; i < dem.rows; i++) {
        for(int j = 0; j < dem.cols; j++) {
            if(std::isnan(dem.at<float>(i, j))) {
                std::vector<float> queryData = {(float) i, (float) j};
                int querynum = 1;
                std::vector<int> indices(querynum);
                std::vector<float> dists(querynum);

                //kdTree.knnSearch(queryData, indices, dists, querynum, cv::flann::SearchParams());
                kdTree.radiusSearch(queryData, indices, dists, 10, querynum, cv::flann::SearchParams()); //radius is in "pixels"

                //cout << dists[0] << endl;

                int index = indices[0];

                //I know that I is supposed to represent y axis, leave me alone
                int nearest_i = dataPoints.at<cv::Point2f>(index).x;
                int nearest_j = dataPoints.at<cv::Point2f>(index).y;

                interpolated.at<float>(i, j) = dem.at<float>(nearest_i, nearest_j);
            }
        }
    }

    dem = interpolated;
}

int main(int, char**){
    //std::string plyPath = "/home/vanja/Desktop/CLOUD/room_test/cloud_nofilter.ply";
    std::string plyPath = "/home/vanja/Desktop/Robotika Projekt/stereo_outdoor.ply";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if(pcl::io::loadPLYFile<pcl::PointXYZRGB>(plyPath, *cloud) == -1) {
        cerr << "ERROR: Unable to load PLY file." << endl;
        return -1;
    }

    //---FILTERING OUTLIERS---
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50); // Number of neighbors to use for mean distance estimation
    sor.setStddevMulThresh(0.3); // Standard deviation multiplier for distance thresholding

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    sor.filter(*cloud_filtered);


    //---VISUALIZATION OF POINTCLOUDS---
    /*
        Call this at the end of your code, because it will
        block execution of code after it and closing
        the visualizer causes a segmentation fault!
    */
    //filtrationViz(cloud_filtered, cloud);


    //---GENERATING DEM---
    cout << "Starting heightmap generation" << endl;
    double grid_resolution = 0.05;

    pcl::PointXYZRGB min, max;
    pcl::getMinMax3D(*cloud_filtered, min, max);

    int rows = ceil((max.y - min.y) / grid_resolution);
    int cols = ceil((max.x - min.x) / grid_resolution);

    cv::Mat heightmap(rows, cols, CV_32F, cv::Scalar::all(std::numeric_limits<float>::quiet_NaN()));


    for(const auto& point : cloud_filtered->points){
        int col = ((max.x - point.x) / grid_resolution);
        int row = ((max.y - point.y) / grid_resolution);

        if(col >= 0 && col < cols && row >= 0 && row < rows) {
            if(isnan(heightmap.at<float>(row, col)) || point.z > heightmap.at<float>(row, col)){
                heightmap.at<float>(row, col) = point.z;
            }
        }
    }

    cout << "Done generating heightmap" << endl;
    
    //cv::flip(heightmap, heightmap, 0); //Flip along X-axis (row and col calculation flips image)
   
    //---INTERPOLATION---
    cout << "Starting interpolation" << endl;
    cv::Mat dataPoints = extractDataPoints(heightmap); //We build kd-tree only with non-NaN points
    cv::flann::Index kdTree = buildKDTree(dataPoints); //kd-tree build
    nearestNeighbourInterpolation(heightmap, dataPoints, kdTree); // interpolation
    cout << "Done interpolating" << endl;


    //---SHOW FINAL RESULT---
    cv::normalize(heightmap, heightmap, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::imshow("Digital Elevation Model", heightmap);
    cv::waitKey(0);

    

    //filtrationViz(cloud_filtered, cloud);
    return 0;
}
