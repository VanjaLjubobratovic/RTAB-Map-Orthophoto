#include "mosaicing_tools.h"

template <typename T>
bool MosaicingTools::isNaN(const T& value) {
    return std::isnan(value);
}

template <>
bool MosaicingTools::isNaN<cv::Vec3f>(const cv::Vec3f& value) {
    return std::isnan(value[0]) || std::isnan(value[1]) || std::isnan(value[2]);
}

template <typename T>
cv::Mat MosaicingTools::extractDataPoints(const cv::Mat& dem) {
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

cv::flann::Index MosaicingTools::buildKDTree(const cv::Mat& dataPoints) {
    return cv::flann::Index(dataPoints, cv::flann::KDTreeIndexParams());
}

pcl::PointXYZRGB MosaicingTools::calculateCentroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
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

void MosaicingTools::filterCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, int nNeighbors = 50, float stdDevMulThresh = 0.3) {
    //---FILTERING OUTLIERS---
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(input);
    sor.setMeanK(nNeighbors); // Number of neighbors to use for mean distance estimation
    sor.setStddevMulThresh(stdDevMulThresh); // Standard deviation multiplier for distance thresholding
    sor.filter(*output);
}

template <typename T>
void MosaicingTools::nnInterpolationThread(cv::Mat& input, cv::Mat& output, cv::Mat& dataPoints, cv::flann::Index& kdTree, float searchRadius, int startRow, int endRow) {
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
void MosaicingTools::nnInterpolation(cv::Mat& dem, cv::Mat& dataPoints, cv::flann::Index& kdTree, float searchRadius, int numThreads = 1) {
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



void MosaicingTools::knnInterpolationThread(const cv::Mat& input, cv::Mat& output, const cv::Mat& dataPoints, cv::flann::Index& kdTree, float searchRadius, int nNeighbors, float p, int startRow, int endRow) {

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

void MosaicingTools::knnInterpolation(cv::Mat& dem, cv::Mat& dataPoints, cv::flann::Index& kdTree, float searchRadius, int nNeighbors, float p, int numThreads = 1) {
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



void MosaicingTools::Voxel::addPoint(pcl::PointXYZRGB* point, int heightIndex) {
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

cv::Mat MosaicingTools::generateMosaic(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double grid_resolution) {
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



cv::Mat MosaicingTools::gaussSmooth(cv::Mat* raster, int kernelSize, float sigma) {
    auto result = raster->clone();
    cv::Mat kernel = cv::getGaussianKernel(kernelSize, sigma, CV_32F);
    cv::filter2D(result, result, -1, kernel); //-1 -> output is the same data type as input

    return result;
}
