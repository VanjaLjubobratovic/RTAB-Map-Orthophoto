from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments for each parameter
        DeclareLaunchArgument('cloud_decimation', default_value='2',
                              description='Decimation factor for point cloud data'),
        DeclareLaunchArgument('cloud_min_depth', default_value='0.0',
                              description='Minimum depth for point cloud data'),
        DeclareLaunchArgument('cloud_max_depth', default_value='0.0',
                              description='Maximum depth for point cloud data'),
        DeclareLaunchArgument('cloud_voxel_size', default_value='0.01',
                              description='Voxel size for point cloud data'),
        DeclareLaunchArgument('cloud_buffer_size', default_value='5',
                              description='Buffer size for point cloud data'),
        DeclareLaunchArgument('interpolate', default_value='false',
                              description='Save additional interpolated mosaics'),
        DeclareLaunchArgument('show_live', default_value='true',
                              description='Show live mosaic generation'),

        # LogInfo to display the parameters
        LogInfo(
            msg="Launching mosaicing_node with the following parameters:\n"
                "cloud_decimation: {0}, cloud_min_depth: {1}, cloud_max_depth: {2}, "
                "cloud_voxel_size: {3}, cloud_buffer_size: {4}, interpolate: {5}, show_live: {6}"
                .format(
                    LaunchConfiguration('cloud_decimation'), 
                    LaunchConfiguration('cloud_min_depth'),
                    LaunchConfiguration('cloud_max_depth'), 
                    LaunchConfiguration('cloud_voxel_size'),
                    LaunchConfiguration('cloud_buffer_size'), 
                    LaunchConfiguration('interpolate'),
                    LaunchConfiguration('show_live')
                )
        ),

        # Launch the mosaicing_node with the specified parameters
        Node(
            package='mosaicing_pkg',  # Replace with your package name
            executable='mosaicing_node',  # Replace with your node name
            name='mosaicing_node',
            namespace='',
            output='screen',
            parameters=[
                {'cloud_decimation': LaunchConfiguration('cloud_decimation')},
                {'cloud_min_depth': LaunchConfiguration('cloud_min_depth')},
                {'cloud_max_depth': LaunchConfiguration('cloud_max_depth')},
                {'cloud_voxel_size': LaunchConfiguration('cloud_voxel_size')},
                {'cloud_buffer_size': LaunchConfiguration('cloud_buffer_size')},
                {'interpolate': LaunchConfiguration('interpolate')},
                {'show_live': LaunchConfiguration('show_live')},
            ],
            remappings=[],  # Add remappings if necessary
            arguments=[]    # Add additional arguments if necessary
        ),
    ])
