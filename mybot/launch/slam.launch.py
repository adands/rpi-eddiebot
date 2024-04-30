import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import  FindPackageShare

def generate_launch_description():
  
  ## launch file directory
  eddiebot_dir = "/home/adan/ros2_ws/src/fortest/launch/"
  lidar_pkg_dir = "/home/adan/ros2_ws/src/sllidar_ros2/launch/"
  robot_launch_dir = "/home/adan/ros2_ws/src/fortest/launch"
  pkg_share = FindPackageShare(package="mybot").find("mybot")

  ## include launch file
  #    motor controller, lidar and joint_state_publisher
  eddiebot =   IncludeLaunchDescription(
  PythonLaunchDescriptionSource([eddiebot_dir,"eddiebot.launch.py"])
  )
  lidar_launch =   IncludeLaunchDescription(
  PythonLaunchDescriptionSource([lidar_pkg_dir,"sllidar_a1_launch.py"])
  )
  
  joint_state = IncludeLaunchDescription(
  PythonLaunchDescriptionSource([robot_launch_dir, '/eddiebot_state_publisher.launch.py'])
  )    
  #    rviz2
  rviz = IncludeLaunchDescription(
  PythonLaunchDescriptionSource([eddiebot_dir,"rviz2.launch.py"])
  )
  #    slam_toolbox
  slam_toolbox = IncludeLaunchDescription(
  PythonLaunchDescriptionSource(["/opt/ros/humble/share/slam_toolbox/launch/","online_async_launch.py"])
  )
  #    na2_bringup navigation_launch.py
  navigation = IncludeLaunchDescription(
  PythonLaunchDescriptionSource(["/opt/ros/humble/share/nav2_bringup/launch/","navigation_launch.py"])
  )
  
  ##  node
  
  robot_localization = Node(
  package="robot_localization",
  executable="ekf_node",
  name="ekf_filter_node",
  output="screen",
  parameters=["/home/adan/ros2_ws/src/mybot/config/ekf.yaml"],
  arguments=["--ros-args","--param","use_sim_time:=false"]
  )
  
  motor_node = Node(
  package="fortest",
  executable="motor",
  )
  

 
 
  return LaunchDescription([

  
  eddiebot,
  navigation,
  slam_toolbox,
  robot_localization,
  rviz
  
  ])
