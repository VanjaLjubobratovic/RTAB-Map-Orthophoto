#include "mosaicing_tools.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <thread>
#include <mutex>

std::mutex callbackLock;

class RTABMapPointCloudSubscriber : public rclcpp::Node {
public:
    RTABMapPointCloudSubscriber() : Node("rtabmap_pointcloud_subscriber") {
        point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/rtabmap/cloud_map", 10,
            std::bind(&RTABMapPointCloudSubscriber::pointCloudCallback, this, std::placeholders::_1)
        );
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if(!callbackLock.try_lock())
            return;

        std::cout << "Callback number: " << callbacks << std::endl;
        // Convert the ROS point cloud message to a PCL point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);

        // Process the point cloud data here
        // You can perform various PCL operations on pcl_cloud
        callbacks++;

        if(callbacks == 1) {
            callbacks = 0;
            std::cout << "Generating mosaic..." << std::endl;

            //---FILTERING OUTLIERS---
            MosaicingTools::filterCloud(cloud, cloud, 50, 0.3);

            auto mosaic = MosaicingTools::generateMosaic(cloud, 0.005);
            auto mosaicNN = mosaic.clone();
            auto mosaicKNN = mosaic.clone();

            auto dataPoints = MosaicingTools::extractDataPoints(mosaic);
            auto mosaicKdTree = MosaicingTools::buildKDTree(dataPoints);
            
            MosaicingTools::nnInterpolation(mosaicNN, dataPoints, mosaicKdTree, 10, 8);
            MosaicingTools::knnInterpolation(mosaicKNN, dataPoints, mosaicKdTree, 10, 20, 2.0, 8);

            cv::imwrite("../colorizedDem.jpg", mosaic);
            cv::imwrite("../colorizedDemNN.jpg", mosaicNN);
            cv::imwrite("../colorizedDemKNN.jpg", mosaicKNN);

            std::cout << "Done generating!" << std::endl;
        }

        std::cout << "Thread sleeping for 10s..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        std::cout << "Thread awake, unlocking..." << std::endl;
        callbackLock.unlock();
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    int callbacks = 0;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RTABMapPointCloudSubscriber>());
    rclcpp::shutdown();
    return 0;
}
