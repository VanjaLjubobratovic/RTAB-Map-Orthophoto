#include "mosaicing_tools.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <thread>
#include <mutex>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap_conversions/MsgConversion.h>
#include <rtabmap_msgs/srv/get_map.hpp>

#define CLOUD_DECIMATION 2
#define CLOUD_MAX_DEPTH 0.0
#define CLOUD_MIN_DEPTH 0.0
#define CLOUD_VOXEL_SIZE 0.01
#define NUM_POSES_TO_MOSAIC 5


std::mutex callbackLock;

class RTABMapPointCloudSubscriber : public rclcpp::Node {
public:
    RTABMapPointCloudSubscriber() : Node("rtabmap_pointcloud_subscriber") {
        point_cloud_subscription_ = this->create_subscription<rtabmap_msgs::msg::MapData>(
            "/rtabmap/mapData", 10,
            std::bind(&RTABMapPointCloudSubscriber::processMapData, this, std::placeholders::_1)
        );
    }

private:
    void mosaicer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
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

        auto img = cv::imread("../colorizedDem.jpg");
        cv::imshow("LIVE MOSAIC", img);
        cv::waitKey(50);

        std::cout << "Done generating!" << std::endl;
    }

    void processMapData(const rtabmap_msgs::msg::MapData map) {
        std::cout << "Caught message, processing..." << std::endl;
        std::map<int, rtabmap::Transform> poses;
        for(unsigned int i = 0; i < map.graph.poses_id.size() && i < map.graph.poses.size(); ++i) {
            poses.insert(std::make_pair(map.graph.poses_id[i], rtabmap_conversions::transformFromPoseMsg(map.graph.poses[i])));
        }

        //Add new clouds...
        bool fromDepth = true;
        std::set<int> nodeDataReceived;
        for(unsigned int i = 0; i < map.nodes.size() && i < map.nodes.size(); ++i) {

            //Always refresh cloud if there is data
            rtabmap::Signature s = rtabmap_conversions::nodeDataFromROS(map.nodes[i]);
            
            std::cout << "Checking massive IF #" << i << "..." << std::endl;
            if((fromDepth && !s.sensorData().imageCompressed().empty() && !s.sensorData().depthOrRightCompressed().empty() &&
                (s.sensorData().cameraModels().size() || s.sensorData().stereoCameraModels().size())) || 
                (!fromDepth && !s.sensorData().laserScanCompressed().isEmpty())) {
                
                std::cout << "Passed IF..." << std::endl;
                cv::Mat image, depth;
                rtabmap::LaserScan scan;

                s.sensorData().uncompressData(fromDepth?&image:0, fromDepth?&depth:0, !fromDepth?&scan:0);

                sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg(new sensor_msgs::msg::PointCloud2);

                if(fromDepth && !s.sensorData().imageRaw().empty() && !s.sensorData().depthOrRightRaw().empty()) {
                    std::cout << "Passed IF for unpacking data..." << std::endl;
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
                    pcl::IndicesPtr validIndices(new std::vector<int>);

                    cloud = rtabmap::util3d::cloudRGBFromSensorData(
                                    s.sensorData(),
                                    CLOUD_DECIMATION,
                                    CLOUD_MAX_DEPTH,
                                    CLOUD_MIN_DEPTH,
                                    validIndices.get());
                    
                    if(!cloud->empty()) {
                        if(CLOUD_VOXEL_SIZE) {
                            cloud = rtabmap::util3d::voxelize(cloud, validIndices, CLOUD_VOXEL_SIZE);
                        }

                        //Floor height and ceiling height filter filter
                        /*
                            IMPLEMENTATION HERE IF NECESSARY
                        */
                       
                       std::vector<int> index;
                       pcl::removeNaNFromPointCloud(*cloud, *cloud, index);
                       *cloud = *rtabmap::util3d::transformPointCloud(cloud, s.getPose());

                       callbackLock.lock();
                       *pcl_cloud += *cloud;
                       if(poses.size() % NUM_POSES_TO_MOSAIC == 0)
                            mosaicer(pcl_cloud);
                       callbackLock.unlock();
                    }
                }                
            }
        }
    }

    rclcpp::Subscription<rtabmap_msgs::msg::MapData>::SharedPtr point_cloud_subscription_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RTABMapPointCloudSubscriber>());
    rclcpp::shutdown();
    return 0;
}
