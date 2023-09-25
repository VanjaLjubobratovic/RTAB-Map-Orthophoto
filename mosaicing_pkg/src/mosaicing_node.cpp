#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "mosaicing_tools.h"


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
        std::cout << "Callback number: " << callbacks << std::endl;
        // Convert the ROS point cloud message to a PCL point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *tmp);

        // Process the point cloud data here
        // You can perform various PCL operations on pcl_cloud
        callbacks++;

        if(callbacks == 20) {
            auto visualizer(new pcl::visualization::PCLVisualizer("Point cloud filter vs raw"));
            visualizer->addPointCloud<pcl::PointXYZRGB>(tmp, "cloud");
            visualizer->spin();
        }
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
