#include "mosaicing_tools.h"

std::mutex MosaicingTools::mosaicingLock;
std::mutex MosaicingTools::minMaxLock;
MosaicingTools::VoxelRaster MosaicingTools::voxelRaster;

int MosaicingTools::VoxelRaster::rows() {
    return raster.size();
}

int MosaicingTools::VoxelRaster::cols() {
    return (rows() > 0) ? raster[0].size() : 0;
}

template <typename T>
bool MosaicingTools::isNaN(const T& value) {
    return std::isnan(value);
}

template <>
bool MosaicingTools::isNaN<cv::Vec3f>(const cv::Vec3f& value) {
    return std::isnan(value[0]) || std::isnan(value[1]) || std::isnan(value[2]);
}

void MosaicingTools::minMaxThread(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointXYZRGB& min, pcl::PointXYZRGB& max, int startP, int endP) {
    pcl::PointXYZRGB lMin, lMax;
    lMin = cloud->points[startP];
    lMax = cloud->points[startP];

    for(int i = startP; i < endP; i++) {
        auto point = cloud->points[i];
        if(point.x < lMin.x)
            lMin.x = point.x;
        else if(point.x > lMax.x)
            lMax.x = point.x;
        
        if(point.y < lMin.y)
            lMin.y = point.y;
        else if(point.y > lMax.y)
            lMax.y = point.y;
        
        if(point.z < lMin.z)
            lMin.z = point.z;
        else if(point.z > lMax.z)
            lMax.z = point.z;
    }

    minMaxLock.lock();
    if(lMin.x < min.x)
        min.x = lMin.x;
    if(lMin.y < min.y)
        min.y = lMin.y;
    if(lMin.z < min.z)
        min.z = lMin.z;
    
    if(lMax.x > max.x)
        max.x = lMax.x;
    if(lMax.y > max.y)
        max.y = lMax.y;
    if(lMax.z > max.z)
        max.z = lMax.z;
    minMaxLock.unlock();

    //std::cout << "MINMAX thread with START " << startP << " and END " << endP << " finished!" << std::endl;
}

void MosaicingTools::fasterGetMinMax3D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointXYZRGB& min, pcl::PointXYZRGB& max, int numThreads) {
    min = cloud->points[0];
    max = cloud->points[0];

    std::vector<std::thread> threads;
    int pointsPerThread = cloud->points.size() / numThreads;

    for(int threadId = 0; threadId < numThreads; threadId++) {
        int startP = threadId * pointsPerThread;
        int endP = (threadId == numThreads - 1) ? (cloud->points.size() - 1) : (startP + pointsPerThread);

        threads.emplace_back(MosaicingTools::minMaxThread, std::ref(cloud), std::ref(min), std::ref(max),
                            startP, endP);
    }

    for(auto& thread : threads) {
        thread.join();
    }
}

cv::Mat MosaicingTools::extractDataPoints(const cv::Mat& dem) {
    cv::Mat points;

    for (int i = 0; i < dem.rows; i++) {
        for (int j = 0; j < dem.cols; j++) {
            if(!isNaN(dem.at<cv::Vec3f>(i, j))) {
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

void MosaicingTools::filterCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, int nNeighbors, float stdDevMulThresh) {
    //---FILTERING OUTLIERS---
    pcl::StopWatch watch;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(input);
    sor.setMeanK(nNeighbors); // Number of neighbors to use for mean distance estimation
    sor.setStddevMulThresh(stdDevMulThresh); // Standard deviation multiplier for distance thresholding
    sor.filter(*output);
    std::cout << "Stat. filtering cloud ended after: " << watch.getTimeSeconds() << "s" << std::endl;
}

void MosaicingTools::radiusFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, double radius, int minNeighbors) {
    pcl::StopWatch watch;

    // Create the radius outlier removal filter
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    outrem.setInputCloud(input);
    outrem.setRadiusSearch(radius);  // Set the radius within which points are considered neighbors
    outrem.setMinNeighborsInRadius(minNeighbors);  // Minimum number of neighbors required for a point to be considered an inlier

    // Apply the filter to remove outliers
    outrem.filter(*output);

    std::cout << "Radius filter ended after: " << watch.getTimeSeconds() << "s" << std::endl;
}

void MosaicingTools::statDistanceFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, pcl::PointXYZRGB& referencePoint, float stDevMultiplier) {
	pcl::StopWatch watch;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr resultCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	// Initialize variables for mean and standard deviation
	double S = 0.0;
	double M = 0.0;
	double mean = 0.0;
	int k = 1;

	//Wellford's algorithm for calculating mean and standard deviation
	for(auto point : input->points) {
		double dx = point.x - referencePoint.x;
		double dy = point.y - referencePoint.y;
		double dist = std::sqrt(dx * dx + dy * dy);

		mean += dist;

		double tmpM = M;
		M += (dist - tmpM) / k;
		S += (dist - tmpM) * (dist - M);
		k++;
	}

	double stDev = std::sqrt(S / (k-1));
	mean /= (k-1);

	//Max allowed distance in the XY plane
	double Y = mean + stDevMultiplier * stDev;

	//Filter the cloud
	for(auto point : input->points) {
		double dx = point.x - referencePoint.x;
		double dy = point.y - referencePoint.y;
		double dist = std::sqrt(dx * dx + dy * dy);

		if(dist <= Y) {
			resultCloud->push_back(point);
		}
	}

	output = resultCloud;
	std::cout << "Statistical distance filter ended after: " << watch.getTimeSeconds() << "s" << std::endl;
}

void MosaicingTools::nnInterpolationThread(cv::Mat& input, cv::Mat& output, cv::Mat& dataPoints, cv::flann::Index& kdTree, float searchRadius, int startRow, int endRow) {
    for(int i = startRow; i < endRow; i++) {
        for(int j = 0; j < input.cols; j++) {
            if(isNaN(input.at<cv::Vec3f>(i, j))) {
                std::vector<float> queryData = {(float) i, (float) j};
                std::vector<int> indices;
                std::vector<float> dists;

                kdTree.radiusSearch(queryData, indices, dists, searchRadius, 1, cv::flann::SearchParams()); //radius is in "pixels"

                if(indices[0] == 0 && dists[0] == 0.0)
                    continue; //No neighbour found

                int nearest_i = dataPoints.at<cv::Point2f>(indices[0]).x;
                int nearest_j = dataPoints.at<cv::Point2f>(indices[0]).y;

                output.at<cv::Vec3f>(i, j) = input.at<cv::Vec3f>(nearest_i, nearest_j);
            }
        }
    }

    //std::cout << "NN thread with START " << startRow << " and END " << endRow << " finished!" << std::endl;
}

void MosaicingTools::nnInterpolation(cv::Mat& dem, cv::Mat& dataPoints, cv::flann::Index& kdTree, float searchRadius, int numThreads) {
    //std::cout << "Starting NN threads" << std::endl;
    pcl::StopWatch watch;

    cv::Mat interpolated = dem.clone();
    
    std::vector<std::thread> threads;
    int rowsPerThread = dem.rows / numThreads;

    for(int threadId = 0; threadId < numThreads; threadId++) {
        int startRow = threadId * rowsPerThread;
        int endRow = (threadId == numThreads - 1) ? dem.rows : (startRow + rowsPerThread);

        threads.emplace_back(MosaicingTools::nnInterpolationThread, std::ref(interpolated), std::ref(interpolated),
                            std::ref(dataPoints), std::ref(kdTree), searchRadius, startRow, endRow);
    }

    for(auto& thread : threads) {
        thread.join();
    }

    //std::cout << "NN interpolation ended after: " << watch.getTimeSeconds() << "s" << std::endl;

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

                for(std::vector<int>::size_type k = 0; k < indices.size(); k++) {
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

    //std::cout << "KNN thread with START " << startRow << " and END " << endRow << " finished!" << std::endl;
}

void MosaicingTools::knnInterpolation(cv::Mat& dem, cv::Mat& dataPoints, cv::flann::Index& kdTree, float searchRadius, int nNeighbors, float p, int numThreads) {
    //std::cout << "Starting KNN threads" << std::endl;
    pcl::StopWatch watch;
    cv::Mat interpolated = dem.clone();
    
    std::vector<std::thread> threads;
    int rowsPerThread = dem.rows / numThreads;

    for(int threadId = 0; threadId < numThreads; threadId++) {
        int startRow = threadId * rowsPerThread;
        int endRow = (threadId == numThreads - 1) ? dem.rows : (startRow + rowsPerThread);

        threads.emplace_back(MosaicingTools::knnInterpolationThread, std::ref(interpolated), std::ref(interpolated),
                            std::ref(dataPoints), std::ref(kdTree), searchRadius, nNeighbors, p, startRow, endRow);
    }

    for(auto& thread : threads) {
        thread.join();
    }

    //std::cout << "KNN interpolation ended after: " << watch.getTimeSeconds() << std::endl;
    dem = interpolated;
}

void MosaicingTools::interpolate(cv::Mat& mosaic, cv::Mat& interpolated, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string method, int numThreads, double grid_resolution) {
    pcl::StopWatch watch;
    pcl::PointXYZRGB min, max;
    fasterGetMinMax3D(cloud, min, max, numThreads);


    //Resizing interpolated mosaic
    int xSize = mosaic.cols;
    int ySize = mosaic.rows;

    if(xSize != interpolated.cols || ySize != interpolated.rows) {
        cv::Mat resizedRaster(ySize, xSize, CV_32FC3, cv::Scalar(0));
        cv::Rect dstRect((int)voxelRaster.lastResize.x, (int)voxelRaster.lastResize.y, interpolated.cols, interpolated.rows);

        //Tends to happen sometimes because of rounding errors somewhere
        if(dstRect.width + dstRect.x > resizedRaster.cols)
            dstRect.width -= (dstRect.width + dstRect.x - resizedRaster.cols);
        if(dstRect.height + dstRect.y > resizedRaster.rows)
            dstRect.height -= (dstRect.height + dstRect.y - resizedRaster.rows);

        interpolated.copyTo(resizedRaster(dstRect));
        interpolated = resizedRaster;
    }

    //Calculating coordinates of new tile
    int xMax = ceil((voxelRaster.max.x - min.x) / grid_resolution);
    int yMax = ceil((voxelRaster.max.y - min.y) / grid_resolution);
    int xMin = ceil((voxelRaster.max.x - max.x) / grid_resolution);
    int yMin = ceil((voxelRaster.max.y - max.y) / grid_resolution);

    //Rect takes starting coordinates and then width and height
    cv::Rect tileRect(xMin, yMin, xMax - xMin, yMax - yMin);
    if(tileRect.width + xMin > mosaic.cols)
        tileRect.width -= (tileRect.width + xMin - mosaic.cols);
    if(tileRect.height + yMin > mosaic.rows)
        tileRect.height -= (tileRect.height + yMin - mosaic.rows);

    auto tile = mosaic(tileRect);

    auto dataPoints = MosaicingTools::extractDataPoints(tile);
    auto tileKdTree = MosaicingTools::buildKDTree(dataPoints);

    if(method == "NN") {
        nnInterpolation(tile, dataPoints, tileKdTree, 10, numThreads);
    } else if(method == "KNN") {
        knnInterpolation(tile, dataPoints, tileKdTree, 10, 20, 2.0, numThreads);
    }

    tile.copyTo(interpolated(tileRect));
    std::cout << "Interpolation finished after: " << watch.getTimeSeconds() << "s" << std::endl;
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

void MosaicingTools::voxelizationThread(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<std::vector<Voxel>>& voxelized, int startP, int endP, pcl::PointXYZRGB min, pcl::PointXYZRGB max, double grid_resolution) {
    for(int i = startP; i < endP; i++) {
        auto point = cloud->points[i];

        int x = ((max.x - point.x) / grid_resolution);
        int y = ((max.y - point.y) / grid_resolution);
        int z = ((point.z - min.z) / (grid_resolution * 2)); //Bottom of the volume cube has height of 0 for simplicity

        mosaicingLock.lock();
        voxelized[y][x].addPoint(&point, z);
        mosaicingLock.unlock();
    }
    //std::cout << "VOXELIZATION thread with START " << startP << " and END " << endP << " finished!" << std::endl;
}

void MosaicingTools::rasterizationThread(std::vector<std::vector<Voxel>>& voxelized, cv::Mat& raster, int startRow, int endRow) {
    for(int i = startRow; i < endRow; i++) {
        for(int j = 0; j < raster.cols; j++) {
            raster.at<cv::Vec3f>(i, j) = voxelized[i][j].pointBGR;
        }
    }
    //std::cout << "RASTER thread with START " << startRow << " and END " << endRow << " finished!" << std::endl;
}

void MosaicingTools::resizeRaster(pcl::PointXYZRGB min, pcl::PointXYZRGB max, double grid_size) {
    pcl::StopWatch watch;
    int moveX = 0;
    int moveY = 0;

    if(!voxelRaster.initialized) {
        voxelRaster.min = min;
        voxelRaster.max = max;

        int xSize = ceil((voxelRaster.max.x - voxelRaster.min.x) / grid_size);
        int ySize = ceil((voxelRaster.max.y - voxelRaster.min.y) / grid_size);

        voxelRaster.raster = std::vector<std::vector<Voxel>>(ySize, std::vector<Voxel>(xSize));
        voxelRaster.initialized = true;
        return;
    }

    //The raster is made so that (0,0) corresponds to max coordinates (for some reason)
    if(max.x > voxelRaster.max.x) {
        moveX = (max.x - voxelRaster.max.x) / grid_size;
    }
    if(max.y > voxelRaster.max.y) {
        moveY = (max.y - voxelRaster.max.y) / grid_size;
    }

    voxelRaster.lastResize.x = moveX;
    voxelRaster.lastResize.y = moveY;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->points.push_back(min);
    cloud->points.push_back(max);
    cloud->points.push_back(voxelRaster.min);
    cloud->points.push_back(voxelRaster.max);

    pcl::getMinMax3D(*cloud, voxelRaster.min, voxelRaster.max); //Calculating new min and max

    int xSize = ceil((voxelRaster.max.x - voxelRaster.min.x) / grid_size);
    int ySize = ceil((voxelRaster.max.y - voxelRaster.min.y) / grid_size);

    if(xSize != voxelRaster.cols() || ySize != voxelRaster.rows()) {
        std::vector<std::vector<Voxel>> resizedRaster(ySize, std::vector<Voxel>(xSize));
        for(int i = 0; i < voxelRaster.rows(); i++) {
            for(int j = 0; j < voxelRaster.cols(); j++) {
                int dstX = moveX + j;
                int dstY = moveY + i;
                if(dstY >= 0 && dstY < ySize && dstX >= 0 && dstX < xSize)
                    resizedRaster[dstY][dstX] = voxelRaster.raster[i][j]; 
            }
        }

        voxelRaster.raster = resizedRaster;
    }

    std::cout << "Finished resizing raster after: " << watch.getTimeSeconds() << "s" << std::endl;
}

cv::Mat MosaicingTools::generateMosaic(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double grid_resolution, int numThreads) {
    pcl::PointXYZRGB min, max;
    fasterGetMinMax3D(cloud, min, max, 8);

    //Generating a raster which acts as a top layer of a voxelized space
    std::cout << "Voxelizing..." << std::endl;

    resizeRaster(min, max, grid_resolution);

    pcl::StopWatch watch; //timing execution
    std::vector<std::thread> threads;
    int pointsPerThread = cloud->points.size() / numThreads;

    for(int threadId = 0; threadId < numThreads; threadId++) {
        int startP = threadId * pointsPerThread;
        int endP = (threadId == numThreads - 1) ? (cloud->points.size() - 1) : (startP + pointsPerThread);

        threads.emplace_back(MosaicingTools::voxelizationThread, std::ref(cloud), std::ref(voxelRaster.raster),
                            startP, endP, voxelRaster.min, voxelRaster.max, grid_resolution);
    }

    for(auto& thread : threads) {
        thread.join();
    }

    threads.clear();
    std::cout << "Voxelization ended after: " << watch.getTimeSeconds() << "s" << std::endl;

    //Generating colorized raster
    std::cout << "Projecting to raster..." << std::endl;
    watch.reset();

    cv::Mat raster(voxelRaster.rows(), voxelRaster.cols(), CV_32FC3, cv::Scalar::all(std::numeric_limits<float>::quiet_NaN()));
    int rowsPerThread = raster.rows / numThreads;

    for(int threadId = 0; threadId < numThreads; threadId++) {
        int startRow = threadId * rowsPerThread;
        int endRow = (threadId == numThreads - 1) ? raster.rows : (startRow + rowsPerThread);

        threads.emplace_back(MosaicingTools::rasterizationThread, std::ref(voxelRaster.raster), std::ref(raster),
                            startRow, endRow);
    }

    for(auto& thread : threads) {
        thread.join();
    }

    std::cout << "Rasterization ended after: " << watch.getTimeSeconds() << "s" << std::endl;

    return raster;
}

cv::Mat MosaicingTools::gaussSmooth(cv::Mat* raster, int kernelSize, float sigma) {
    auto result = raster->clone();
    cv::Mat kernel = cv::getGaussianKernel(kernelSize, sigma, CV_32F);
    cv::filter2D(result, result, -1, kernel); //-1 -> output is the same data type as input

    return result;
}
