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

//----------------------------------------------------------------------------------
template <typename T>
void nnInterpolationThread(cv::Mat& input, cv::Mat& output, cv::Mat& dataPoints, cv::flann::Index& kdTree, float searchRadius, int startRow, int endRow) {
    for(int i = startRow; i < endRow; i++) {
        for(int j = 0; j < input.cols; j++) {
            if(isNaN(input.at<T>(i, j))) {
                std::vector<float> queryData = {(float) i, (float) j};
                std::vector<int> indices;
                std::vector<float> dists;

                kdTree.radiusSearch(queryData, indices, dists, searchRadius, 1, cv::flann::SearchParams()); //radius is in "pixels"

                if(indices[0] == 0 && dists[0] == 0.0)
                    continue; //No neighbour found

                int nearest_i = dataPoints.at<cv::Point2f>(indices[0]).x;
                int nearest_j = dataPoints.at<cv::Point2f>(indices[0]).y;

                output.at<T>(i, j) = input.at<T>(nearest_i, nearest_j);
            }
        }
    }

    std::cout << "NN thread with START " << startRow << " and END " << endRow << " finished!" << std::endl;
}

template <typename T>
void nnInterpolation(cv::Mat& dem, cv::Mat& dataPoints, cv::flann::Index& kdTree, float searchRadius, int numThreads = 1) {
    std::cout << "Starting NN threads" << std::endl;
    cv::Mat interpolated = dem.clone();
    
    std::vector<std::thread> threads;
    int rowsPerThread = dem.rows / numThreads;

    for(int threadId = 0; threadId < numThreads; threadId++) {
        int startRow = threadId * rowsPerThread;
        int endRow = (threadId == numThreads - 1) ? dem.rows : (startRow + rowsPerThread);

        threads.emplace_back(nnInterpolationThread<cv::Vec3f>, std::ref(interpolated), std::ref(interpolated),
                            std::ref(dataPoints), std::ref(kdTree), searchRadius, startRow, endRow);
    }

    for(auto& thread : threads) {
        thread.join();
    }

    dem = interpolated;
}
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
void knnInterpolationThread(const cv::Mat& input, cv::Mat& output, const cv::Mat& dataPoints, cv::flann::Index& kdTree, float searchRadius, int nNeighbors, float p, int startRow, int endRow) {

    for(int i = startRow; i < endRow; i++) {
        for(int j = 0; j < input.cols; j++) {
            if(isNaN(input.at<cv::Vec3f>(i, j))){
                std::vector<float> queryData = {(float) i, (float) j};
                std::vector<float> dists;
                std::vector<int> indices;

                kdTree.radiusSearch(queryData, indices, dists, searchRadius, nNeighbors, cv::flann::SearchParams());

                if(indices[0] == 0 && dists[0] == 0.0)
                    continue; //No neighbours found*/

                cv::Vec3f numerator = cv::Vec3f::all(0.0f);
                cv::Vec3f denominator = cv::Vec3f::all(0.0f);

                for(int k = 0; k < indices.size(); k++) {
                    if(dists[k] == 0) 
                        continue;

                    int n_i = dataPoints.at<cv::Point2f>(indices[k]).x;
                    int n_j = dataPoints.at<cv::Point2f>(indices[k]).y;

                    cv::Vec3f u_i = input.at<cv::Vec3f>(n_i, n_j);
                    float wi = 1.0 / pow(dists[k], p);
                    numerator += wi * u_i;
                    denominator += cv::Vec3f::all(wi);
                }

                cv::Vec3f result;
                cv::divide(numerator, denominator, result);

                output.at<cv::Vec3f>(i, j) = result;
            }
        }
    }

    std::cout << "KNN thread with START " << startRow << " and END " << endRow << " finished!" << std::endl;
}

void knnInterpolation(cv::Mat& dem, cv::Mat& dataPoints, cv::flann::Index& kdTree, float searchRadius, int nNeighbors, float p, int numThreads = 1) {
    std::cout << "Starting KNN threads" << std::endl;
    cv::Mat interpolated = dem.clone();
    
    std::vector<std::thread> threads;
    int rowsPerThread = dem.rows / numThreads;

    for(int threadId = 0; threadId < numThreads; threadId++) {
        int startRow = threadId * rowsPerThread;
        int endRow = (threadId == numThreads - 1) ? dem.rows : (startRow + rowsPerThread);

        threads.emplace_back(knnInterpolationThread, std::ref(interpolated), std::ref(interpolated),
                            std::ref(dataPoints), std::ref(kdTree), searchRadius, nNeighbors, p, startRow, endRow);
    }

    for(auto& thread : threads) {
        thread.join();
    }

    dem = interpolated;
}
//----------------------------------------------------------------------------------

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

//----------------------------------------------------------------------------------
struct Voxel {
    int datapoints = 0;
    int height = 0; 
    cv::Vec3f pointBGR{cv::Vec3f::all(std::numeric_limits<float>::quiet_NaN())};

    void addPoint(pcl::PointXYZRGB* point, int heightIndex) {
        if(height > heightIndex) {
            return;
        } else if (height < heightIndex) {
            datapoints = 1;
            height = heightIndex;
            pointBGR = cv::Vec3f(point->b, point->g, point->r);
            return;
        }

        if(!datapoints) {
            pointBGR = cv::Vec3f(point->b, point->g, point->r);
            height = heightIndex;
        } else {
            pointBGR *= datapoints;
            pointBGR += cv::Vec3f(point->b, point->g, point->r);
        }

        datapoints++;
        pointBGR /= datapoints;
    }
};

cv::Mat voxelizeCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double grid_resolution) {
    pcl::PointXYZRGB min, max;
    pcl::getMinMax3D(*cloud, min, max);

    int xSize = ceil((max.x - min.x) / grid_resolution);
    int ySize = ceil((max.y - min.y) / grid_resolution);
    int zSize = ceil((max.z - min.z) / grid_resolution);

    //Generating a raster which acts as a top layer of a voxelized space
    std::vector<std::vector<Voxel>> voxelized(ySize, std::vector<Voxel>(xSize));

    for(auto point : cloud -> points) {
        int x = ((max.x - point.x) / grid_resolution);
        int y = ((max.y - point.y) / grid_resolution);
        int z = ((point.z - min.z) / grid_resolution); //Bottom of the volume cube has height of 0 for simplicity

        voxelized[y][x].addPoint(&point, z);
    }

    //Generating colorized raster
    cv::Mat raster(ySize, xSize, CV_32FC3, cv::Scalar::all(std::numeric_limits<float>::quiet_NaN()));
    for(int i = 0; i < ySize; i++) {
        for(int j = 0; j < xSize; j++) {
            raster.at<cv::Vec3f>(i, j) = voxelized[i][j].pointBGR;
        }
    }

    return raster;
}
//----------------------------------------------------------------------------------


//----------------------------------------------------------------------------------
cv::Mat gaussSmooth(cv::Mat* raster, int kernelSize, float sigma) {
    auto result = raster->clone();
    cv::Mat kernel = cv::getGaussianKernel(kernelSize, sigma, CV_32F);
    cv::filter2D(result, result, -1, kernel); //-1 -> output is the same data type as input

    return result;
}
//----------------------------------------------------------------------------------

int main(int, char**){
    std::string plyPath = "/home/vanja/Desktop/CLOUD/livingroom4/cloud.ply";
    std::string posesPath = "/home/vanja/Desktop/cloudExportTest/poses.txt";
    std::string imagesPath = "/home/vanja/Desktop/cloudExportTest/rgb/";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if(pcl::io::loadPLYFile<pcl::PointXYZRGB>(plyPath, *cloud) == -1) {
        cerr << "ERROR: Unable to load PLY file." << endl;
        return -1;
    }

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
    cv::normalize(heightmap, heightmap, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::imwrite("../outputDem.jpg", heightmap);
    cout << "DEM saved to file!" << endl;


    //---GENERATING MOSAIC METHOD 1---
    std::cout << "Generating mosaic" << std::endl;
    //auto mosaic = generateColorizedDem(cloud_filtered, 0.005);
    auto mosaic = voxelizeCloud(cloud_filtered, grid_resolution);
    auto mosaicNN = mosaic.clone();
    auto mosaicKNN = mosaic.clone();

    std::cout << "Generating k-d trees" << std::endl;
    auto mosaicDataPoints = extractDataPoints<cv::Vec3f>(mosaic);
    auto mosaicKdTree = buildKDTree(mosaicDataPoints);

    std::cout << "Interpolating" << std::endl;
    nnInterpolation<cv::Vec3f>(mosaicNN, mosaicDataPoints, mosaicKdTree, 10, 8);
    knnInterpolation(mosaicKNN, mosaicDataPoints, mosaicKdTree, 10, 20, 2.0, 8);

    cv::normalize(mosaic, mosaic, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::normalize(mosaicNN, mosaicNN, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::normalize(mosaicKNN, mosaicKNN, 0, 255, cv::NORM_MINMAX, CV_8U);

    /*std::cout << "Smoothing" << std::endl;
    mosaicNN = gaussSmooth(&mosaicNN, 5, 0.5);
    mosaicKNN = gaussSmooth(&mosaicKNN, 5, 0.5);*/

    cv::imwrite("../colorizedDem.jpg", mosaic);
    cv::imwrite("../colorizedDemNN.jpg", mosaicNN);
    cv::imwrite("../colorizedDemKNN.jpg", mosaicKNN);
    std::cout << "Images saved!" << std::endl;

    //filtrationViz(cloud_filtered, cloud);

    //---LOADING CAMERA POSES---
    //std::vector<CameraPose> cameraPoses = readPoses(posesPath);

    //---LOADING IMAGES---
    //std::vector<cv::Mat> rgbImages = readImages(imagesPath, cameraPoses);

    return 0;
}
