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
        DeclareLaunchArgument('interpolate', default_value='false',
                              description='Save additional interpolated mosaics'),
        DeclareLaunchArgument('interpolation_method', default_value='NN',
                              description='Nearest neighbors or k-nearest neighbors interpolation'),
        DeclareLaunchArgument('show_live', default_value='true',
                              description='Show live mosaic generation'),
        DeclareLaunchArgument('num_threads', default_value='1',
                              description='Number of threads used for mosaicing'),
        DeclareLaunchArgument('grid_resolution', default_value='0.005',
                              description='Resolution of the mosaic'),
        DeclareLaunchArgument('sor_filter_enable', default_value='true',
                              description='Enable statistic outlier removal'),
        DeclareLaunchArgument('sor_neighbors', default_value='50',
                              description=''),
        DeclareLaunchArgument('sor_stdev_mul', default_value='1.0',
                              description=''),
        DeclareLaunchArgument('dist_filter_enable', default_value='true',
                              description='Enable custom distance thresholding'),
        DeclareLaunchArgument('dist_stdev_mul', default_value='1.0',
                              description=''),

        # LogInfo to display the parameters
        LogInfo(
            msg="Launching mosaicing_node with the following parameters:\n"
                "cloud_decimation: {0}, cloud_min_depth: {1}, cloud_max_depth: {2}, "
                "cloud_voxel_size: {3}, interpolate: {4}, show_live: {5}"
                .format(
                    LaunchConfiguration('cloud_decimation'), 
                    LaunchConfiguration('cloud_min_depth'),
                    LaunchConfiguration('cloud_max_depth'), 
                    LaunchConfiguration('cloud_voxel_size'),
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
                {'interpolate': LaunchConfiguration('interpolate')},
                {'interpolation_method': LaunchConfiguration('interpolation_method')},
                {'show_live': LaunchConfiguration('show_live')},
                {'num_threads': LaunchConfiguration('num_threads')},
                {'grid_resolution': LaunchConfiguration('grid_resolution')},
                {'sor_filter_enable': LaunchConfiguration('sor_filter_enable')},
                {'sor_neighbors': LaunchConfiguration('sor_neighbors')},
                {'sor_stdev_mul': LaunchConfiguration('sor_stdev_mul')},
                {'dist_filter_enable': LaunchConfiguration('dist_filter_enable')},
                {'dist_stdev_mul': LaunchConfiguration('dist_stdev_mul')},
            ],
            remappings=[],  # Add remappings if necessary
            arguments=[]    # Add additional arguments if necessary
        ),
    ])
