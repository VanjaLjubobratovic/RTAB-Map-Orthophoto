#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <opencv2/opencv.hpp>
#include <opencv2/flann.hpp>
#include <opencv2/core/eigen.hpp>

#include <Eigen/Geometry>

/**
 * @file mosaicing_tools.h
 * @brief Utility class with custom methods to support generating 
 * orthophoto mosaics from point clouds.
 * 
 * @author Vanja Ljubobratović
 * @date 25.09.2023.
*/

class MosaicingTools {
public:
    template <typename T>
    static bool isNaN(const T& value);

    template <typename T>
    static cv::Mat extractDataPoints(const cv::Mat& raster);

    static cv::flann::Index buildKDTree(const cv::Mat& dataPoints);

    static pcl::PointXYZRGB calculateCentroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    static void filterCloud(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output,
        int nNeighbors,
        float stdDevMulThresh);

    //TODO: make this a bit more generalized
    static void filtrationViz(pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered, pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw);

    template <typename T>
    static void nnInterpolation(cv::Mat& dem, cv::Mat& dataPoints, cv::flann::Index& kdTree, float searchRadius, int numThreads = 1);
    static void knnInterpolation(cv::Mat& dem, cv::Mat& dataPoints, cv::flann::Index& kdTree, float searchRadius, int nNeighbors, float p, int numThreads = 1);

    static cv::Mat generateMosaic(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double grid_resolution=0.05);

    static cv::Mat gaussSmooth(cv::Mat* raster, int kernelSize, float sigma);

private:
    MosaicingTools();

    struct Voxel{
        int datapoints = 0;
        int height = 0; 
        cv::Vec3f pointBGR{cv::Vec3f::all(std::numeric_limits<float>::quiet_NaN())};

        void addPoint(pcl::PointXYZRGB* point, int heightIndex);
    };

    template <typename T>
    void nnInterpolationThread(cv::Mat& input, cv::Mat& output, cv::Mat& dataPoints, cv::flann::Index& kdTree, float searchRadius, int startRow, int endRow);
    void knnInterpolationThread(const cv::Mat& input, cv::Mat& output, const cv::Mat& dataPoints, cv::flann::Index& kdTree, float searchRadius, int nNeighbors, float p, int startRow, int endRow);
};

template <>
bool MosaicingTools::isNaN<cv::Vec3f>(const cv::Vec3f& value);
