#!/usr/bin/python3

#import
import rclpy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist, Pose ,Point ,Vector3 ,Quaternion, TransformStamped, Transform
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from rclpy.node import Node
from math import *
from tf2_ros import TransformBroadcaster
import sys
import time
from tf_transformations import quaternion_from_euler


#functions
class motor_control(Node):

  def __init__(self):
    super().__init__("motor_node")
    self.sub = self.create_subscription(Twist,"cmd_vel",self.motor_direction,10)
    self.pub = self.create_publisher(Odometry,"odom_diff",10)
    self.timer1 = self.create_timer(0.1,self.speed_control)
    self.timer_odom = self.create_timer(1/30,self.odom_count)
    self.timer_pub = self.create_timer(1/200,self.publish_odom)
    #self.tf_broadcaster = TransformBroadcaster(self)


    # variable and pins setup
    self.dc_value_r = 0
    self.dc_value_l = 0
    self.x = 0
    self.z = 0
    self.pos_x = 0
    self.pos_y = 0
    self.del_th = 0
    self.status = 0
    self.pre_status = 0

    # for PID
    self.kp = 75
    self.ki = 5
    self.kd = 100

    self.error_r =0
    self.del_error_r = 0
    self.error_plus_r = 0
    self.error_l =0
    self.del_error_l = 0
    self.error_plus_l = 0

    # speed counting
    self.prev_time_odom = time.time()
    self.prev_time_left = time.time()    # previous time
    self.prev_time_right = time.time()    # previous time
    self.speed_Right = 0.0          # speed of right wheel
    self.speed_Left = 0.0           # speed of left wheel

    self.PWM_A = 12
    self.PIN_A_PO = 26
    self.PIN_A_NE = 16

    self.PWM_B = 13
    self.PIN_B_PO = 5
    self.PIN_B_NE = 6

    self.encoder_right = 17
    self.encoder_left = 24

    #GPIO setup
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(self.PWM_A, GPIO.OUT)
    GPIO.setup(self.PWM_B, GPIO.OUT)

    GPIO.setup(self.PIN_A_PO,GPIO.OUT)
    GPIO.setup(self.PIN_A_NE,GPIO.OUT)
    GPIO.setup(self.PIN_B_PO,GPIO.OUT)
    GPIO.setup(self.PIN_B_NE,GPIO.OUT)

    self.pwm_a = GPIO.PWM(self.PWM_A, 10000)
    self.pwm_b = GPIO.PWM(self.PWM_B, 10000)


    GPIO.setup(self.encoder_right,GPIO.IN,pull_up_down = GPIO.PUD_UP)
    GPIO.setup(self.encoder_left,GPIO.IN,pull_up_down = GPIO.PUD_UP)

    # encoder interrupt
    GPIO.add_event_detect(self.encoder_left, GPIO.RISING, callback=self.encoder_read_left)
    GPIO.add_event_detect(self.encoder_right,GPIO.RISING,  callback=self.encoder_read_right)

    # pwm start
    self.pwm_a.start(0)
    self.pwm_b.start(0)

    # stanby
    GPIO.output(self.PIN_A_NE,GPIO.LOW)
    GPIO.output(self.PIN_A_PO,GPIO.LOW)
    GPIO.output(self.PIN_B_NE,GPIO.LOW)
    GPIO.output(self.PIN_B_PO,GPIO.LOW)

  # encoder interrupt callback functions
  def encoder_read_left(self,channel):
    self.count_speed_left()
  def encoder_read_right(self,channel):
    self.count_speed_right()

  def stop(self):
    GPIO.output(self.PIN_A_PO,GPIO.LOW)
    GPIO.output(self.PIN_A_NE,GPIO.LOW)
    GPIO.output(self.PIN_B_PO,GPIO.LOW)
    GPIO.output(self.PIN_B_NE,GPIO.LOW)
    self.error_r =0
    self.del_error_r = 0
    self.error_plus_r = 0
    self.error_l =0
    self.del_error_l = 0
    self.error_plus_l = 0


  # motor direction function
  def motor_direction(self,msg):
    self.x = msg.linear.x
    self.z = msg.angular.z

    if self.x > 0 :
      GPIO.output(self.PIN_A_PO,GPIO.HIGH)
      GPIO.output(self.PIN_A_NE,GPIO.LOW)
      GPIO.output(self.PIN_B_PO,GPIO.HIGH)
      GPIO.output(self.PIN_B_NE,GPIO.LOW)
      self.status = 1
    elif self.x < 0:
      GPIO.output(self.PIN_A_PO,GPIO.LOW)
      GPIO.output(self.PIN_A_NE,GPIO.HIGH)
      GPIO.output(self.PIN_B_PO,GPIO.LOW)
      GPIO.output(self.PIN_B_NE,GPIO.HIGH)
      self.status = -1
    else:
      if self.z > 0 :
        GPIO.output(self.PIN_A_PO,GPIO.HIGH)
        GPIO.output(self.PIN_A_NE,GPIO.LOW)
        GPIO.output(self.PIN_B_PO,GPIO.LOW)
        GPIO.output(self.PIN_B_NE,GPIO.HIGH)
        self.status = 2
      elif self.z< 0:
        GPIO.output(self.PIN_A_PO,GPIO.LOW)
        GPIO.output(self.PIN_A_NE,GPIO.HIGH)
        GPIO.output(self.PIN_B_PO,GPIO.HIGH)
        GPIO.output(self.PIN_B_NE,GPIO.LOW)
        self.status = -2
      else:
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)
        self.dc_value_r = 0
        self.dc_value_l = 0
        self.stop()
        self.speed_Left = 0
        self.speed_Right = 0
    if self.pre_status != self.status:
      print(self.status)
      self.stop()

    self.pre_status = self.status

  # speed counting function
  def count_speed_left(self):
    current_time = time.time()
    time_elapsed = current_time - self.prev_time_left


    # my robot's wheel turn 1 round will get 36 pulse from encoder. my robot's perimeter is 0.1524*pi
    # so  the speed will be ((pulse/36)*perimeter)/time_duration
    if time_elapsed >0:
      self.speed_Left = (0.8*abs(self.speed_Left)) + (0.2*(1 / time_elapsed) * ((0.1524*pi)/36))

      # change the speed to negative when the wheel go backward
      if self.x < 0 :
        self.speed_Left  *= -1
      elif self.x == 0:
        if self.z > 0:
          self.speed_Left *= -1
    self.prev_time_left = current_time


  def count_speed_right(self):
    current_time = time.time()
    time_elapsed = current_time - self.prev_time_right

    # my robot's wheel turn 1 round will get 36 pulse from encoder. my robot's perimeter is 0.1524*pi
    # so  the speed will be ((pulse/36)*perimeter)/time_duration
    if time_elapsed >0:
      self.speed_Right = (0.8* abs(self.speed_Right)) + (0.2*(1 / time_elapsed) * ((0.1524*pi)/36))

      # change the speed to negative when the wheel go backward
      if self.x < 0 :
        self.speed_Right *= -1
      elif self.x == 0:
        if self.z < 0:
          self.speed_Right *= -1

    self.prev_time_right = current_time



  # speed control function
  def speed_control(self):
    print("左輪PWM: ",self.dc_value_l,"右輪PWM: ",self.dc_value_r)
    print(f"左輪速度:{self.speed_Left}\t右輪速度:{self.speed_Right}")

    # linear(m/s) = (r + l)/ 2
    # angular(radian/s) = (r - l)/wheel_separation  .
    # wheel separation = 0.39 m
    # if z > 0, the robot will turn left and the left wheel need to go reverse so the speed will be negative
    r = (2*self.x + 0.39*self.z)/2
    l = (2*self.x - 0.39*self.z)/2

    self.error_l = abs(l) - abs(self.speed_Left)
    self.error_r = abs(r) - abs(self.speed_Right)

    if abs(self.speed_Left) != abs(l):
      self.dc_value_l += self.MIN_MAX(int(self.error_l * self.kp + self.error_plus_l * self.ki+ (self.error_l-self.del_error_l)/0.1*self.kd),self.dc_value_l)
      self.pwm_b.ChangeDutyCycle(self.dc_value_l)

    if abs(self.speed_Right) != abs(r):

      self.dc_value_r += self.MIN_MAX(int(self.error_r * self.kp + self.error_plus_r * self.ki+ (self.error_r-self.del_error_r)/0.1*self.kd),self.dc_value_r)
      self.pwm_a.ChangeDutyCycle(self.dc_value_r)

    self.error_plus_l += self.error_l
    self.del_error_l = self.error_l
    self.error_plus_r += self.error_r
    self.del_error_r = self.error_r

  def MIN_MAX(self,value,duty):

      if duty + value < 100 and duty + value >0:
          return value
      else:
          return 0


  # count odometry
  def odom_count(self):

    duration = 1/30
    linear = (self.speed_Right + self.speed_Left)/2
    angular = (self.speed_Right -self.speed_Left)/0.39
    if duration > 0:
      self.del_th += angular*duration
      self.pos_x += linear*duration * cos(self.del_th)
      self.pos_y += linear*duration * sin(self.del_th)

  # publish odometry
  def publish_odom(self):
    trans = TransformStamped()
    t = self.get_clock().now()
    odom = Odometry()

    # header
    odom.header.stamp = t.to_msg()
    odom.header.frame_id = "odom"
    trans.header.stamp = t.to_msg()
    trans.header.frame_id = "odom"

    # child_frame_id
    odom.child_frame_id = "base_footprint"
    trans.child_frame_id = "base_footprint"

    # position
    pos = odom.pose.pose.position
    pos.x = self.pos_x ; pos.y = self.pos_y
    trans.transform.translation.x = self.pos_x
    trans.transform.translation.y = self.pos_y

    orie = odom.pose.pose.orientation
    x,y,z,w = quaternion_from_euler(0,0,self.del_th)
    orie.x = x ; orie.y = y ; orie.z = z ; orie.w = w
    trans_ro = trans.transform.rotation
    trans_ro.x = x ; trans_ro.y = y ; trans_ro.z = z ; trans_ro.w = w

    # twist
    odom.twist.twist.linear.x = (self.speed_Right + self.speed_Left)/2
    odom.twist.twist.angular.z = (self.speed_Right -self.speed_Left)/0.39

    self.pub.publish(odom)
    #self.tf_broadcaster.sendTransform(trans)




# main code
def main(args=None):
  try:
    rclpy.init(args = args)
    node = motor_control()
    rclpy.spin(node)
    rclpy.shutdown()
  finally:
    GPIO.cleanup()

