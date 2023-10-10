"""Launch realsense2_camera node."""
import copy
from launch import LaunchDescription, LaunchContext
import launch_ros.actions
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import sys
import os
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
#sys.path.append(os.path.expanduser("~/ros2_ws/src/realsense_ros/realsense2_camera"))
sys.path.append(get_package_prefix('realsense2_camera'))
import rs_launch

local_parameters = [{'name': 'camera_name1', 'default': 'camera1', 'description': 'camera1 unique name'},
                    {'name': 'camera_name2', 'default': 'camera2', 'description': 'camera2 unique name'},
                    {'name': 'camera_namespace1', 'default': 'camera1', 'description': 'camera1 namespace'},
                    {'name': 'camera_namespace2', 'default': 'camera2', 'description': 'camera2 namespace'},
                    {'name': 'fixed_frame', 'default': 'base_link', 'description': ''},
                    {'name': 'camera_distance', 'default': '1.0', 'description': 'distance between cameras in meters'}
                    ]

config_rviz = os.path.join(
            get_package_share_directory('mosaicing_pkg'), 'launch', 'config', 'slam_D455x2_config.rviz')

def set_configurable_parameters(local_params):
    return dict([(param['original_name'], LaunchConfiguration(param['name'])) for param in local_params])

def duplicate_params(general_params, posix):
    local_params = copy.deepcopy(general_params)
    for param in local_params:
        param['original_name'] = param['name']
        param['name'] += posix
    return local_params

def launch_rtab_nodes(context : LaunchContext):
    rviz_node = launch_ros.actions.Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=[["-d"], [config_rviz]]
    )

    rgbd_sync1_node = launch_ros.actions.Node(
        package='rtabmap_sync', executable='rgbd_sync', name='rgbd_sync1', output="screen",
        parameters=[{
            "approx_sync": False
            }],
        remappings=[
            ("rgb/image", '/camera1/color/image_raw'),
            ("depth/image", '/camera1/aligned_depth_to_color/image_raw'),
            ("rgb/camera_info", '/camera1/color/camera_info'),
            ("rgbd_image", 'rgbd_image')],
        namespace='realsense_camera1'
    )
    rgbd_sync2_node = launch_ros.actions.Node(
        package='rtabmap_sync', executable='rgbd_sync', name='rgbd_sync2', output="screen",
        parameters=[{
            "approx_sync": False
            }],
        remappings=[
            ("rgb/image", '/camera2/color/image_raw'),
            ("depth/image", '/camera2/aligned_depth_to_color/image_raw'),
            ("rgb/camera_info", '/camera2/color/camera_info'),
            ("rgbd_image", 'rgbd_image')],
        namespace='realsense_camera2'
    )
            
    # RGB-D odometry
    rgbd_odometry_node = launch_ros.actions.Node(
        package='rtabmap_odom', executable='rgbd_odometry', output="screen",
        parameters=[{
            "frame_id": 'base_link',
            "odom_frame_id": 'odom',
            "publish_tf": True,
            "approx_sync": True,
            "subscribe_rgbd": True,
            }],
        remappings=[
            ("rgb/image", '/camera1/color/image_raw'),
            ("depth/image", '/camera1/aligned_depth_to_color/image_raw'),
            ("rgb/camera_info", '/camera1/color/camera_info'),
            ("rgbd_image", '/realsense_camera1/rgbd_image'),
            ("odom", 'odom')],
        arguments=["--delete_db_on_start", ''],
        prefix='',
        namespace='rtabmap'
    )

    # SLAM 
    slam_node = launch_ros.actions.Node(
        package='rtabmap_slam', executable='rtabmap', output="screen",
        parameters=[{
            "rgbd_cameras":2,
            "queue_size": 300,
            "subscribe_depth": True,
            "subscribe_rgbd": True,
            "subscribe_rgb": True,
            "subscribe_odom_info": True,
            "frame_id": 'base_link',
            "map_frame_id": 'map',
            "publish_tf": True,
            "database_path": '~/.ros/rtabmap.db',
            "approx_sync": True,
            "Mem/IncrementalMemory": "true",
            "Mem/InitWMWithAllNodes": "true"
        }],
        remappings=[
            ("rgbd_image0", '/realsense_camera1/rgbd_image'),
            ("rgbd_image1", '/realsense_camera2/rgbd_image'),
            ("odom", 'odom')],
        arguments=["--delete_db_on_start"],
        prefix='',
        namespace='rtabmap'
    )

    voxelcloud1_node = launch_ros.actions.Node(
        package='rtabmap_util', executable='point_cloud_xyzrgb', name='point_cloud_xyzrgb1', output='screen',
        parameters=[{
            "approx_sync": True,
        }],
        remappings=[
            ('rgb/image', '/camera1/color/image_raw'),
            ('depth/image', '/camera1/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera1/color/camera_info'),
            ('rgbd_image', 'rgbd_image'),
            ('cloud', 'voxel_cloud1')]
    )

    voxelcloud2_node = launch_ros.actions.Node(
        package='rtabmap_util', executable='point_cloud_xyzrgb', name='point_cloud_xyzrgb2', output='screen',
        parameters=[{
            "approx_sync": True,
        }],
        remappings=[
            ('rgb/image', '/camera2/color/image_raw'),
            ('depth/image', '/camera2/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera2/color/camera_info'),
            ('rgbd_image', 'rgbd_image'),
            ('cloud', 'voxel_cloud2')]
    )

    return [
        rviz_node,
        rgbd_sync1_node,
        rgbd_sync2_node,
        rgbd_odometry_node,
        slam_node,
        voxelcloud1_node,
        voxelcloud2_node
    ]


def launch_camera_static_transform_publishers(context : LaunchContext):
    cam1_tf = launch_ros.actions.Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        arguments = ["0", "0", "0", "0", "0", "0",
                    context.launch_configurations['fixed_frame'],
                    context.launch_configurations['camera_name1'] + "_link"]
        )

    cam2_tf = launch_ros.actions.Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        arguments = ["0", context.launch_configurations['camera_distance'], "0", "0", "0", "0",
                    context.launch_configurations['fixed_frame'],
                    context.launch_configurations['camera_name2'] + "_link"]
        )
    return [cam1_tf, cam2_tf]

def launch_camera2_static_transform_publisher(context : LaunchContext):
    
    return [node]

def generate_launch_description():
    params1 = duplicate_params(rs_launch.configurable_parameters, '1')
    params2 = duplicate_params(rs_launch.configurable_parameters, '2')
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params1) +
        rs_launch.declare_configurable_parameters(params2) +
        [
        OpaqueFunction(function=rs_launch.launch_setup,
                       kwargs = {'params'           : set_configurable_parameters(params1),
                                 'param_name_suffix': '1'}),
        OpaqueFunction(function=rs_launch.launch_setup,
                       kwargs = {'params'           : set_configurable_parameters(params2),
                                 'param_name_suffix': '2'}),
        OpaqueFunction(function=launch_camera_static_transform_publishers),
        OpaqueFunction(function=launch_rtab_nodes)
    ])