import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()
    
    package_name='rbot_description'      

    
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(package_name), 'launch', 'map.launch.py'))
    )    
    diff_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(package_name), 'launch', 'robot.launch.py'))
    )
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(package_name), 'launch', 'rplidar.launch.py'))
    )   
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(package_name), 'launch', 'twist_mux.launch.py'))
    )
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(package_name), 'launch', 'ekf.launch.py'))
    )   
   
    
    
    
    actions = [
        rviz,
        #ekf,
        #rplidar,
        diff_controller, 
        twist_mux,
    ]
    
    return LaunchDescription(actions)