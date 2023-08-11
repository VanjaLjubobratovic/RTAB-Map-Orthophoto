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

struct CameraPose {
    Eigen::Isometry3d pose;
    int id;

    CameraPose(Eigen::Isometry3d pose, int id) 
        : pose(pose), id(id){}

};

template <typename T>
bool isNaN(const T& value) {
    return std::isnan(value);
}

template <>
bool isNaN<cv::Vec3f>(const cv::Vec3f& value) {
    return std::isnan(value[0]) || std::isnan(value[1]) || std::isnan(value[2]);
}

template <typename T>
cv::Mat extractDataPoints(const cv::Mat& dem) {
    cv::Mat points;

    for (int i = 0; i < dem.rows; i++) {
        for (int j = 0; j < dem.cols; j++) {
            if(!isNaN(dem.at<T>(i, j))) {
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
     //---VISUALIZATION OF POINTCLOUDS---
    /*
        Call this at the end of your code, because it will
        block execution of code after it and closing
        the visualizer causes a segmentation fault for some reason!
    */

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

    /*viewer->setCameraPosition(centroid.x, centroid.y, centroid.z + 10,
                              centroid.x, centroid.y, centroid.z,
                              0, -1, 0);*/

    while(!viewer->wasStopped()) {
        viewer->spin();
    }
}

template <typename T>
void nearestNeighborInterpolation(cv::Mat& dem, cv::Mat& dataPoints, cv::flann::Index& kdTree, float searchRadius) {
    cv::Mat interpolated = dem.clone();

    for(int i = 0; i < dem.rows; i++) {
        for(int j = 0; j < dem.cols; j++) {
            if(isNaN(dem.at<T>(i, j))) {
                std::vector<float> queryData = {(float) i, (float) j};
                std::vector<int> indices;
                std::vector<float> dists;

                kdTree.radiusSearch(queryData, indices, dists, searchRadius, 1, cv::flann::SearchParams()); //radius is in "pixels"

                if(indices[0] == 0 && dists[0] == 0.0)
                    continue; //No neighbour found

                //I know that I is supposed to represent y axis, leave me alone
                int nearest_i = dataPoints.at<cv::Point2f>(indices[0]).x;
                int nearest_j = dataPoints.at<cv::Point2f>(indices[0]).y;

                interpolated.at<T>(i, j) = dem.at<T>(nearest_i, nearest_j);
            }
        }
    }

    dem = interpolated;
}

void knnInterpolation(cv::Mat& dem, cv::Mat& dataPoints, cv::flann::Index& kdTree, float searchRadius, int nNeighbors, float p) {
    cv::Mat interpolated = dem.clone();

    for(int i = 0; i < dem.rows; i++) {
        for(int j = 0; j < dem.cols; j++) {
            if(std::isnan(dem.at<float>(i, j))){
                std::vector<float> queryData = {(float) i, (float) j};
                std::vector<float> dists;
                std::vector<int> indices;

                kdTree.radiusSearch(queryData, indices, dists, searchRadius, nNeighbors, cv::flann::SearchParams());

                if(indices[0] == 0 && dists[0] == 0.0)
                    continue; //No neighbours found*/

                float numerator = 0;
                float denominator = 0;

                for(int k = 0; k < indices.size(); k++) {
                    int n_i = dataPoints.at<cv::Point2f>(indices[k]).x;
                    int n_j = dataPoints.at<cv::Point2f>(indices[k]).y;

                    float wi = 1.0 / pow(dists[k], p);
                    numerator += wi * dem.at<float>(n_i, n_j);
                    denominator += wi;
                }

                interpolated.at<float>(i, j) = numerator / denominator;
            }
        }
    }

    dem = interpolated;
}

std::vector<CameraPose> readPoses(const std::string& path) {
    std::vector<CameraPose> cameraPoses;

    std::ifstream file(path);
    if(!file.is_open()) {
        std::cerr << "Failed to open poses file " << path << std::endl;
        return cameraPoses; 
    }

    //Skip first line with format definition
    std::string line;
    std::getline(file, line);

    double x, y, z, qx, qy, qz, qw;
    double timestamp;
    int id;
    while(file >> timestamp >> x >> y >> z >> qx >> qy >> qz >> qw >> id) {

        Eigen::Isometry3d pose;

        Eigen::Vector3d translation(x, y, z);
        Eigen::Quaternion rotation(qw, qx, qy, qz);

        pose = Eigen::Isometry3d::Identity();
        pose.translation() = translation;
        pose.linear() = rotation.toRotationMatrix();

        cameraPoses.push_back(CameraPose(pose, id));
    }

    file.close();
    return cameraPoses;
}

cv::Mat warpImage(const cv::Mat& image, const CameraPose& cameraPose, const cv::Mat& dem) {
    //---INVERSE CAMERA TRANSFORM---
    cv::Mat warpedImage;
    cv::Mat transformMatrix(3, 3, CV_64F);

    auto eigenMat = cameraPose.pose.matrix();

    for(int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            transformMatrix.at<float>(i, j) = eigenMat(i, j);
        }
    }

    cv::warpPerspective(image, warpedImage, transformMatrix, image.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

    return warpedImage;
}

std::vector<cv::Mat> readImages(const std::string& path, std::vector<CameraPose>& poses) {
    std::vector<cv::Mat> images;

    for(auto pose : poses) {
        std::string pathToImg = path + std::to_string(pose.id) + ".jpg";

        cv::Mat image = cv::imread(pathToImg, cv::IMREAD_COLOR);

        if(image.empty()) {
            std::cerr << "Error reading image with ID: " << pose.id << "\n";
            std::cerr << pathToImg << std::endl;
        }

        images.push_back(image);
    }

    std::cout << "Done reading images" << std::endl;

    return images;
}

cv::Mat generateColorizedDem(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double grid_resolution) {
    /* 
        This method generates the mosaic simply 
        by projecting the pointcloud with its color 
        information vertically to a ground plane
    */

    pcl::PointXYZRGB min, max;
    pcl::getMinMax3D(*cloud, min, max);

    int rows = ceil((max.y - min.y) / grid_resolution);
    int cols = ceil((max.x - min.x) / grid_resolution);

    cv::Mat heightmap(rows, cols, CV_32FC3, cv::Scalar::all(std::numeric_limits<float>::quiet_NaN()));

    for(const auto& point : cloud->points){
        int col = ((max.x - point.x) / grid_resolution);
        int row = ((max.y - point.y) / grid_resolution);

        if(col >= 0 && col < cols && row >= 0 && row < rows) {
            if(isnan(heightmap.at<cv::Vec3f>(row, col)[0]) || point.z > heightmap.at<cv::Vec3f>(row, col)[0]){
                //heightmap.at<float>(row, col) = point.z;
                heightmap.at<cv::Vec3f>(row, col) = cv::Vec3f(point.b, point.g, point.r); //Yeah, opencv is weird, it's BGR
            }
        }
    }

    return heightmap;
}

int main(int, char**){
    //std::string plyPath = "/home/vanja/Desktop/cloudExportTest/cloud9.ply";
    std::string plyPath = "/home/vanja/Desktop/CLOUD/livingroom/livingroom.ply";
    std::string posesPath = "/home/vanja/Desktop/cloudExportTest/poses.txt";
    std::string imagesPath = "/home/vanja/Desktop/cloudExportTest/rgb/";
    //std::string plyPath = "/home/vanja/Desktop/CLOUD/rgbd-scenes-v2/pc/09.ply";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if(pcl::io::loadPLYFile<pcl::PointXYZRGB>(plyPath, *cloud) == -1) {
        cerr << "ERROR: Unable to load PLY file." << endl;
        return -1;
    }

    //This is cs correction for rgbd-scenes-v2 dataset
    /*for (int i = 0; i < cloud->size(); i++) {
        //Swapping y and z axis and changing z axis direction
        float tmp = cloud->points[i].y;
        cloud->points[i].y = cloud->points[i].z;
        cloud->points[i].z = -tmp;
    }*/

    //Approximately correcting the tilt in dataset
    float angle_rad = 25.0f * M_PI / 180.0;
    Eigen::Affine3f rotation_matrix = Eigen::Affine3f::Identity();
    rotation_matrix.rotate(Eigen::AngleAxisf(angle_rad, Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud(*cloud, *cloud, rotation_matrix);

    //---FILTERING OUTLIERS---
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50); // Number of neighbors to use for mean distance estimation
    sor.setStddevMulThresh(0.3); // Standard deviation multiplier for distance thresholding
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    sor.filter(*cloud_filtered);


    //---GENERATING DEM---
    cout << "Starting heightmap generation" << endl;
    double grid_resolution = 0.005;

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
    
    //---INTERPOLATION---
    cout << "Starting interpolation" << endl;
    cv::Mat dataPoints = extractDataPoints<float>(heightmap); //We build kd-tree only with non-NaN points
    cv::flann::Index kdTree = buildKDTree(dataPoints); //kd-tree build
    //nearestNeighbourInterpolation<float>(heightmap, dataPoints, kdTree, 10); // interpolation
    //knnInterpolation(heightmap, dataPoints, kdTree, 10, 5, 4.0);
    cout << "Done interpolating" << endl;
    cv::normalize(heightmap, heightmap, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::imwrite("../outputDem.jpg", heightmap);
    cout << "DEM saved to file!" << endl;


    //---GENERATING MOSAIC METHOD 1---
    auto mosaic = generateColorizedDem(cloud_filtered, 0.005);
    auto mosaicDataPoints = extractDataPoints<cv::Vec3f>(mosaic);
    auto mosaicKdTree = buildKDTree(mosaicDataPoints);
    nearestNeighborInterpolation<cv::Vec3f>(mosaic, mosaicDataPoints, mosaicKdTree, 10);
    cv::normalize(mosaic, mosaic, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::imwrite("../colorizedDem.jpg", mosaic);
    

    //---LOADING CAMERA POSES---
    std::vector<CameraPose> cameraPoses = readPoses(posesPath);

    //---LOADING IMAGES---
    std::vector<cv::Mat> rgbImages = readImages(imagesPath, cameraPoses);

    // std::ofstream outputFile("../points");
    // if (!outputFile.is_open()) {
    //     std::cerr << "Error: Failed to open output file." << std::endl;
    //     return 1;  // Return an error code
    // }
    //
    // //---GENERATING MOSAIC---
    // cv::Mat orthomosaic(heightmap.rows, heightmap.cols, CV_32FC3, cv::Scalar(0, 0, 0));
    // for(int i = 0; i < orthomosaic.rows; i++) {
    //     for(int j = 0; j < orthomosaic.cols; j++) {
    //         if(std::isnan(heightmap.at<float>(i,j)))
    //             continue;
    //
    //         pcl::PointXYZRGB Xz(i, j, heightmap.at<float>(i,j)); //Project mosaic point to DEM
    //         /*auto pose = cameraPoses[0].pose.translation();
    //
    //         pcl::PointXYZ Pc((float)pose.x(), (float)pose.y(), (float)pose.z()); //Camera point in map frame
    //
    //         pcl::PointXYZ Pc_dem; //Camera point translation to DEM frame
    //         Pc_dem.x = (max.x - Pc.x) / grid_resolution;
    //         Pc_dem.y = (max.y - Pc.y) / grid_resolution;
    //         Pc_dem.z = Pc.z;*/
    //
    //         /*  Image space -> camera space matrix
    //             K = | fx   0    cx |
    //                 | 0    fy   cy |
    //                 | 0    0    1  |
    //         */
    //         double fx = 424.82;
    //         double fy = 424.82;
    //         double cx = 421.071;
    //         double cy = 242.692;
    //         cv::Mat intrinsicMatrix = (cv::Mat_<double>(3, 3) <<
    //             fx, 0, cx,
    //             0, fy, cy, 
    //             0, 0, 1);
    //         Eigen::Matrix4d intrinsicMatrixEigen;
    //         Eigen::Matrix4d calibrationEigen = Eigen::Matrix4d::Identity();
    //         cv::cv2eigen(intrinsicMatrix, intrinsicMatrixEigen);
    //
    //         Eigen::Matrix4d rotation = cameraPoses[0].pose.linear().matrix();
    //         Eigen::Matrix4d translation = -cameraPoses[0].pose.translation().matrix();
    //
    //         calibrationEigen.block<3, 1>(0, 3) = translation;
    //
    //         std::cout << calibrationEigen << "\n-----" << std::endl;
    //
    //         auto cameraMatrix = intrinsicMatrixEigen * rotation * calibrationEigen;
    //      
    //
    //         Eigen::Vector4d demEigen(Xz.x, Xz.y, Xz.z, 1.0); //Converting pcl::PointXYZRGB to Eigen
    //         demEigen[0] = max.x - demEigen[0] * grid_resolution;
    //         demEigen[1] = max.y - demEigen[1] * grid_resolution; //to world space
    //         //Eigen::Vector3d cameraEigen(Pc_dem.x, Pc_dem.y, Pc_dem.z);
    //         //Eigen::Vector3d demToCamera = cameraEigen - demEigen; //Line from DEM surface to camera point
    //         //demToCamera.normalize();
    //
    //         Eigen::Vector4d imagePoint = cameraMatrix * demEigen;
    //         outputFile << imagePoint << "\n-------" << std::endl;
    //
    //         //std::cout << Pc << " || " << max << "\n------------" << std::endl;
    //         //std::cout << Pc_dem << std::endl;
    //
    //         //Plane equation: Ax + By + Cz + D = 0
    //         /*Eigen::Vector3d principalPoint = cameraPoses[0].pose * Eigen::Vector3d(0, 0, fx); //In map frame; (x, y, z)
    //         principalPoint.x() = (max.x - principalPoint.x()) / grid_resolution;
    //         principalPoint.y() = (max.y - principalPoint.y()) / grid_resolution; //DEM frame
    //         Eigen::Vector3d imgPlaneNormal = cameraPoses[0].pose * Eigen::Vector3d(0, 0, 1); //In map frame; (A, B, C)
    //         imgPlaneNormal.x() = (max.x - imgPlaneNormal.x()) / grid_resolution;
    //         imgPlaneNormal.y() = (max.y - imgPlaneNormal.y()) / grid_resolution; //DEM frame
    //         double D = -imgPlaneNormal.dot(principalPoint);
    //
    //         double t = (principalPoint - demEigen).dot(imgPlaneNormal) / demToCamera.dot(imgPlaneNormal);
    //         Eigen::Vector3d intersectionPoint = demEigen + t * demToCamera; //In DEM frame
    //
    //         std::cout << principalPoint << "\n----------" << std::endl;
    //       
    //         //DEM -> map -> camera -> image
    //         Eigen::Vector3d intersectionTransformed = intersectionPoint;
    //         intersectionTransformed.x() = max.x - intersectionTransformed.x() * grid_resolution;
    //         intersectionTransformed.y() = max.y - intersectionTransformed.y() * grid_resolution; //DEM -> map 
    //         intersectionTransformed = cameraPoses[0].pose.inverse() * intersectionPoint; //map -> camera
    //        
    //         //cout << intersectionTransformed.x() << " " << intersectionTransformed.y() << " " << intersectionTransformed.z() << endl;*/
    //     }
    // }
    //
    // outputFile.close();

    return 0;
}
