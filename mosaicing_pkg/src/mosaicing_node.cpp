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

// #define CLOUD_DECIMATION 2
// #define CLOUD_MAX_DEPTH 0.0
// #define CLOUD_MIN_DEPTH 0.0
// #define CLOUD_VOXEL_SIZE 0.01
// #define ACCUMULATED_CLOUDS 5

std::mutex cloudLock;
std::condition_variable dataReadyCondition;

class RTABMapPointCloudSubscriber : public rclcpp::Node {
public:
    RTABMapPointCloudSubscriber(const rclcpp::NodeOptions& options) : Node("rtabmap_pointcloud_subscriber", options) {
        point_cloud_subscription_ = this->create_subscription<rtabmap_msgs::msg::MapData>(
            "/rtabmap/mapData", 10,
            std::bind(&RTABMapPointCloudSubscriber::processMapData, this, std::placeholders::_1)
        );

        declare_parameter<int>("cloud_decimation", 2);
        declare_parameter<double>("cloud_min_depth", 0.0);
        declare_parameter<double>("cloud_max_depth", 0.0);
        declare_parameter<double>("cloud_voxel_size", 0.01);
        declare_parameter<int>("cloud_buffer_size", 5);
        declare_parameter<bool>("interpolate", false);
        declare_parameter<bool>("show_live", true);
    }

public:
    void dataProcessingThread() {
        while(true) {
            {
                std::cout << "Mosaicing thread; DATA TO PROCESS: " << cloudsToProcess.size() << std::endl;
                std::unique_lock<std::mutex> lock(cloudLock);
                dataReadyCondition.wait(lock, [this]{ return (cloudsToProcess.size() >= (std::size_t)get_parameter("cloud_buffer_size").as_int());});
                for(std::size_t i = 0; i < cloudsToProcess.size(); i++) {
                    *pcl_cloud += *cloudsToProcess.front();
                    cloudsToProcess.pop();
                }

                mosaicer(pcl_cloud, get_parameter("interpolate").as_bool(), get_parameter("show_live").as_bool());
            }
        }
    }

private:
    void mosaicer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, bool interpolate, bool showLive) {
        std::cout << "Generating mosaic..." << std::endl;

        auto mosaic = MosaicingTools::generateMosaic(cloud, 0.005, 8);
        cv::imwrite("../mosaic.jpg", mosaic);

        if(interpolate) {
            auto mosaicNN = mosaic.clone();
            auto mosaicKNN = mosaic.clone();

            auto dataPoints = MosaicingTools::extractDataPoints(mosaic);
            auto mosaicKdTree = MosaicingTools::buildKDTree(dataPoints);
            
            MosaicingTools::nnInterpolation(mosaicNN, dataPoints, mosaicKdTree, 10, 4);
            MosaicingTools::knnInterpolation(mosaicKNN, dataPoints, mosaicKdTree, 10, 20, 2.0, 4);

            cv::imwrite("../mosaicNN.jpg", mosaicNN);
            cv::imwrite("../mosaicKNN.jpg", mosaicKNN);
        }


        if(showLive) {
            auto img = cv::imread("../mosaic.jpg");
            cv::imshow("LIVE MOSAIC", img);
            cv::waitKey(50);
        }

        std::cout << "Done generating!" << std::endl;
        pcl_cloud->clear(); //clearing processed data
    }

    //Adjusted official RTAB-Map rviz plugin code
    void processMapData(const rtabmap_msgs::msg::MapData map) {
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
            
            if((fromDepth && !s.sensorData().imageCompressed().empty() && !s.sensorData().depthOrRightCompressed().empty() &&
                (s.sensorData().cameraModels().size() || s.sensorData().stereoCameraModels().size())) || 
                (!fromDepth && !s.sensorData().laserScanCompressed().isEmpty())) {
                
                cv::Mat image, depth;
                rtabmap::LaserScan scan;

                s.sensorData().uncompressData(fromDepth?&image:0, fromDepth?&depth:0, !fromDepth?&scan:0);

                sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg(new sensor_msgs::msg::PointCloud2);

                if(fromDepth && !s.sensorData().imageRaw().empty() && !s.sensorData().depthOrRightRaw().empty()) {
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
                    pcl::IndicesPtr validIndices(new std::vector<int>);

                    cloud = rtabmap::util3d::cloudRGBFromSensorData(
                                    s.sensorData(),
                                    get_parameter("cloud_decimation").as_int(),
                                    get_parameter("cloud_max_depth").as_double(),
                                    get_parameter("cloud_min_depth").as_double(),
                                    validIndices.get());
                    
                    if(!cloud->empty()) {
                        if(get_parameter("cloud_voxel_size").as_double()) {
                            cloud = rtabmap::util3d::voxelize(cloud, validIndices, get_parameter("cloud_voxel_size").as_double());
                        }
                        //Floor height and ceiling height filter filter
                        /*
                            IMPLEMENTATION HERE IF NECESSARY
                        */
                       
                       std::vector<int> index;
                       pcl::removeNaNFromPointCloud(*cloud, *cloud, index);

                       //Filtering outliers > X meters away from camera pose and with statistical filter
                       auto pose = poses.rbegin()->second;
                       pcl::PointXYZRGB referencePoint = pcl::PointXYZRGB(pose.x(), pose.y(), pose.z());

                     	if(!cloud->empty()) {
									*cloud = *rtabmap::util3d::transformPointCloud(cloud, pose);
								} else {
									std::cerr << "Filtering removed all points, skipping..." << std::endl;
								}

								//Poses are given in "map" frame so filtering goes after transforming the cloud to "map" frame
								MosaicingTools::thresholdFilter(cloud, cloud, referencePoint, get_parameter("cloud_max_depth").as_double());

                                //Painfully slow and removes way too many points
								//MosaicingTools::filterCloud(cloud, cloud, 50, 0.3);

                                MosaicingTools::statDistanceFilter(cloud, cloud, referencePoint, 2);

                        //adding clouds to processing queue
                       {
                        std::lock_guard<std::mutex> lock(cloudLock);
                        cloudsToProcess.push(cloud);
                        dataReadyCondition.notify_one();
                       }
                    }
                }                
            }
        }
    }

    rclcpp::Subscription<rtabmap_msgs::msg::MapData>::SharedPtr point_cloud_subscription_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    std::queue<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudsToProcess;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<RTABMapPointCloudSubscriber>(options);
    std::thread mosaicingThread(&RTABMapPointCloudSubscriber::dataProcessingThread, node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
