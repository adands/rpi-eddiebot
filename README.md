# rpi-eddiebot
use Raspberry pi 4 to control robot

Hardware：
Raspberry pi 
Pololu TB67H420FTG Dual/Single Motor Driver Carrier 
DENSO Robotics R730556-7030
Adafruit ICM-20948 9-DoF IMU
RPLidar A1
Parallax Quadrature Encoder

Software：

Ubuntu OS 

ROS2 Humble

sllidar_ros2

navigation2

slam_toolbox

robot_localization

Pin sets：

  left_wheel：
    DENSO Robotics R730556-7030 162000-2231 R
    rpi：
      PIN_B_PO：5
      PIN_B_NE：6
      PWM_B：13
      encoder_left：24
    motor driver：
      A+： red → blue
      A-： black → yellow

  right_wheel：
    DENSO Robotics R730556-7030 162000-2241 L
    rpi：
      PIN_A_PO：26
      PIN_A_NE：16
      PWM_A：12
      encoder_right：17
    motor driver：
      B+： red → yellow
      B-： black → blue
