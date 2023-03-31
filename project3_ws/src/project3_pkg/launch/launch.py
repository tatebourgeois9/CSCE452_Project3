import glob
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    input_bag_arg = DeclareLaunchArgument('input_bag')
    output_bag_arg = DeclareLaunchArgument('output_bag')
    
    # Create Node for your project
    laser_scan_to_pointcloud_node = Node(package='project3',
                                         executable='laser_scan_to_pointcloud')
    
    # ExecuteProcess to play the input bag file
    play_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('input_bag')]
    )
    
    # ExecuteProcess to record the output bag file
    record_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', LaunchConfiguration('output_bag')]
    )
    
    # Event handler to shutdown the system when the playback of the input bag file has completed
    event_handler = OnProcessExit(target_action=play_bag, on_exit=[EmitEvent(event=Shutdown())])
    terminate_at_end = RegisterEventHandler(event_handler)
    
    # Create a LaunchDescription with all the components
    ld = LaunchDescription([
        input_bag_arg,
        output_bag_arg,
        laser_scan_2_pointcloud_node,
        play_bag,
        record_bag,
        terminate_at_end
    ])
    
    return ld



