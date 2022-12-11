
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
   
    return LaunchDescription([

        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),

        # Launch arguments
        DeclareLaunchArgument('gui', default_value='true', description='Launch GUI.'),
        DeclareLaunchArgument('approx_sync', default_value='true', description=''),
        DeclareLaunchArgument('pnp', default_value='true', description=''),
        DeclareLaunchArgument('object_prefix', default_value='object', description='TF prefix of objects.'),
        DeclareLaunchArgument('objects_path', default_value='~/objects', description='Directory containing objects to load on initialization.'),
        DeclareLaunchArgument('settings_path', default_value='~/.ros/find_object_2d.ini', description='Config file.'),

        DeclareLaunchArgument('rgb_topic', default_value='camera/rgb/image_rect_color', description='Image topic.'),
        DeclareLaunchArgument('depth_topic', default_value='camera/depth_registered/image_raw', description='Registered depth topic.'),
        DeclareLaunchArgument('camera_info_topic', default_value='camera/rgb/camera_info', description='Camera info topic.'),

        # Nodes to launch
        Node(
            package='find_object_2d', executable='find_object_2d', output='screen',
            parameters=[{
              'subscribe_depth':True,
              'gui':LaunchConfiguration('gui'),
              'approx_sync':LaunchConfiguration('approx_sync'),
              'pnp':LaunchConfiguration('pnp'),
              'object_prefix':LaunchConfiguration('object_prefix'),
              'objects_path':LaunchConfiguration('objects_path'),
              'settings_path':LaunchConfiguration('settings_path')
            }],
            remappings=[
                ('rgb/image_rect_color', LaunchConfiguration('rgb_topic')),
                ('depth_registered/image_raw', LaunchConfiguration('depth_topic')),
                ('depth_registered/camera_info', LaunchConfiguration('camera_info_topic'))]), 
    ])
