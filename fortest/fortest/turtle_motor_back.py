  #!/usr/bin/python3

#import

import rclpy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist, Pose ,Point ,Vector3 ,Quaternion
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from rclpy.node import Node
import math
import sys
import time
from tf_transformations import quaternion_from_euler


#functions
class motor_control(Node):
  
  def __init__(self):
    print("starting ...")
    super().__init__("motor_node")
    self.sub = self.create_subscription(
      Twist,"cmd_vel",self.motor_direction,10)
    self.pub = self.create_publisher(Odometry,"odom",10)
    self.timer1 = self.create_timer(0.1,self.count_speed)
    self.timer_odom = self.create_timer(1/30,self.odom_count)
    self.timer_pub = self.create_timer(1/30,self.publish_odom)

    # variable and pins setup
    self.dc_value_r = 0
    self.dc_value_l = 0
    self.x = 0
    self.z = 0
    self.pos_x = 0
    self.pos_y = 0
    self.del_th = 0

    self.prev_time_odom = time.time()
    self.prev_time = time.time()    # previous time 
    self.prev_right_count = 0       # previous right wheel pulse
    self.prev_left_count = 0        # previous left wheel pulse
    self.encoder_count_left = 0     # current left wheel pulse
    self.encoder_count_right = 0    # current left wheel pulse
    self.speed_Right = 0.0          # speed of right wheel
    self.speed_Left = 0.0           # speed of left wheel

    self.PWM_A = 12
    self.PIN_A_PO = 26
    self.PIN_A_NE = 16
    # self.A_EN = 23
    
    self.PWM_B = 13
    self.PIN_B_PO = 5
    self.PIN_B_NE = 6
    # self.B_EN = 22

    self.encoder_right = 17
    self.encoder_left = 24
    
    #GPIO setup
    GPIO.setmode(GPIO.BCM)
    
    GPIO.setup(self.PWM_A, GPIO.OUT)
    GPIO.setup(self.PWM_B, GPIO.OUT)

    # GPIO.setup(self.A_EN, GPIO.OUT) 
    # GPIO.setup(self.B_EN, GPIO.OUT)

    GPIO.setup(self.PIN_A_PO,GPIO.OUT)
    GPIO.setup(self.PIN_A_NE,GPIO.OUT)
    GPIO.setup(self.PIN_B_PO,GPIO.OUT)
    GPIO.setup(self.PIN_B_NE,GPIO.OUT)
    
    self.pwm_a = GPIO.PWM(self.PWM_A, 10000)  
    self.pwm_b = GPIO.PWM(self.PWM_B, 10000)  
    
    # GPIO.output(self.B_EN,GPIO.HIGH)
    # GPIO.output(self.A_EN,GPIO.HIGH)
    
    GPIO.setup(self.encoder_right,GPIO.IN,pull_up_down = GPIO.PUD_UP)
    GPIO.setup(self.encoder_left,GPIO.IN,pull_up_down = GPIO.PUD_UP)
    GPIO.add_event_detect(self.encoder_right,GPIO.RISING,  callback=self.encoder_read_right)
    GPIO.add_event_detect(self.encoder_left, GPIO.RISING, callback=self.encoder_read_left)

    # pwm start
    self.pwm_a.start(0)
    self.pwm_b.start(0)
    # stanby
    GPIO.output(self.PIN_A_NE,GPIO.LOW)
    GPIO.output(self.PIN_A_PO,GPIO.LOW)
    GPIO.output(self.PIN_B_NE,GPIO.LOW)
    GPIO.output(self.PIN_B_PO,GPIO.LOW)
    
  # callback functions
  def encoder_read_left(self,channel):
    self.encoder_count_left +=1
  def encoder_read_right(self,channel):
    self.encoder_count_right +=1 

  # motor direction function
  def motor_direction(self,msg):
    self.x = msg.linear.x
    self.z = msg.angular.z
    
    if self.x > 0 :
      GPIO.output(self.PIN_A_PO,GPIO.HIGH)
      GPIO.output(self.PIN_A_NE,GPIO.LOW)
      GPIO.output(self.PIN_B_PO,GPIO.HIGH)
      GPIO.output(self.PIN_B_NE,GPIO.LOW)
    elif self.x < 0:
      GPIO.output(self.PIN_A_PO,GPIO.LOW)
      GPIO.output(self.PIN_A_NE,GPIO.HIGH)
      GPIO.output(self.PIN_B_PO,GPIO.LOW)
      GPIO.output(self.PIN_B_NE,GPIO.HIGH)
    else:
      if self.z > 0 :
        GPIO.output(self.PIN_A_PO,GPIO.HIGH)
        GPIO.output(self.PIN_A_NE,GPIO.LOW)
        GPIO.output(self.PIN_B_PO,GPIO.LOW)
        GPIO.output(self.PIN_B_NE,GPIO.HIGH)
      elif self.z< 0:    
        GPIO.output(self.PIN_A_PO,GPIO.LOW)
        GPIO.output(self.PIN_A_NE,GPIO.HIGH)
        GPIO.output(self.PIN_B_PO,GPIO.HIGH)
        GPIO.output(self.PIN_B_NE,GPIO.LOW)
      else:
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_a.ChangeDutyCycle(0)
        GPIO.output(self.PIN_A_PO,GPIO.LOW)
        GPIO.output(self.PIN_A_NE,GPIO.LOW)
        GPIO.output(self.PIN_B_PO,GPIO.LOW)
        GPIO.output(self.PIN_B_NE,GPIO.LOW)
 
  # speed function    
  def speed_control(self):
    print("左輪PWM: ",self.dc_value_l,"右輪PWM: ",self.dc_value_r)
    
    # linear(m/s) = (r + l)/ 2
    # angular(radian/s) = (r - l)/wheel_separation  .
    # wheel separation = 0.39 m
    # if z > 0, the robot will turn left and the left wheel need to go reverse so the speed will be negative

    # need pid controller    
    r = (2*self.x + 0.39*self.z)/2
    l = (2*self.x - 0.39*self.z)/2

    if  abs(self.speed_Right) < abs(r)  and self.dc_value_r <100:
      self.dc_value_r += 1
      self.pwm_a.ChangeDutyCycle(self.dc_value_r)
    elif abs(self.speed_Right) > abs(r) and self.dc_value_r > 0:
      self.dc_value_r -= 1
      self.pwm_a.ChangeDutyCycle(self.dc_value_r)       
    
    if  abs(self.speed_Left) < abs(l)  and self.dc_value_l <100:
      self.dc_value_l += 1
      self.pwm_b.ChangeDutyCycle(self.dc_value_l)
    elif abs(self.speed_Left) > abs(l) and self.dc_value_l > 0:
      self.dc_value_l -= 1
      self.pwm_b.ChangeDutyCycle(self.dc_value_l)       

  # speed counting function
  def count_speed(self):
    current_time = time.time()
    time_elapsed = current_time - self.prev_time
    pulse_change_right = self.encoder_count_right - self.prev_right_count
    pulse_change_left = self.encoder_count_left - self.prev_left_count 

    if time_elapsed >0:
      self.speed_Right = (pulse_change_right / time_elapsed) * ((0.1524*math.pi)/36)
      self.speed_Left = (pulse_change_left / time_elapsed) * ((0.1524*math.pi)/36)
      if self.x < 0 :
        self.speed_Right * -1 ; self.speed_Left * -1
      elif self.x == 0:
        if self.z > 0:
          self.speed_Left * -1
        elif self.z < 0:
          self.speed_Right *-1
          
    print(f'左輪速度:{self.speed_Left} \t 右輪速度:{self.speed_Right}')
    self.speed_control()
    self.prev_time = current_time
    self.prev_right_count = self.encoder_count_right
    self.prev_left_count = self.encoder_count_left

  

  # count odometry
  def odom_count(self):
    
    duration = 1/30
    linear = (self.speed_Right + self.speed_Right)/2
    angular = (self.speed_Right -self.speed_Left)/0.39
    if duration > 0:
      self.del_th += angular/duration
      self.pos_x += linear*duration * math.cos(self.del_th)
      self.pos_y += linear*duration * math.sin(self.del_th)


  def publish_odom(self):
    t = self.get_clock().now()
    odom = Odometry()

    # header
    odom.header.stamp = t.to_msg()
    odom.header.frame_id = "odom"

    # child_frame_id
    odom.child_frame_id = "base_footprint"

    # position
    pos = odom.pose.pose.position
    pos.x = self.pos_x ; pos.y = self.pos_y
    orie = odom.pose.pose.orientation 
    x,y,z,w = quaternion_from_euler(0,0,self.del_th)
    orie.x = x ; orie.y = y ; orie.z = z ; orie.w = w
    # twist
    odom.twist.twist.linear.x = (self.speed_Right + self.speed_Right)/2
    odom.twist.twist.angular.z = (self.speed_Right -self.speed_Left)/0.39

    self.pub.publish(odom)
    


# main code
def main(args=None):
  try:
    rclpy.init(args = args)
    node = motor_control()
    rclpy.spin(node)
    rclpy.shutdown()
  finally:
    GPIO.cleanup()

