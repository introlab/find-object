
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
        DeclareLaunchArgument('image_topic', default_value='image', description='Image topic to subscribe to.'),
        DeclareLaunchArgument('objects_path', default_value='~/objects', description='Directory containing objects to load on initialization.'),
        DeclareLaunchArgument('settings_path', default_value='~/.ros/find_object_2d.ini', description='Config file.'),

        # Nodes to launch
        Node(
            package='find_object_2d', executable='find_object_2d', output='screen',
            parameters=[{
              'gui':LaunchConfiguration('gui'),
              'objects_path':LaunchConfiguration('objects_path'),
              'settings_path':LaunchConfiguration('settings_path')
            }],
            remappings=[
                ('image', LaunchConfiguration('image_topic'))]), 
    ])
