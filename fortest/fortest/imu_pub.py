#!usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3 ,Quaternion
import board
import adafruit_icm20x
from tf_transformations import quaternion_from_euler

class imu_pub(Node):

    def __init__(self):
        super().__init__("imu_pub")
        print("start imu....",flush=True)
        self.pub = self.create_publisher(Imu,"imu",10)
        self.i2c = board.I2C()
        self.icm = adafruit_icm20x.ICM20948(self.i2c)
        self.timer = self.create_timer(1/200,self.imu_publish)
        #self.timer2 = self.create_timer(1,self.imu_print)
        self.del_th_x = 0
        self.del_th_y = 0
        self.del_th_z = 0
        
    def imu_publish(self):
        imu_msg=Imu()
        t = self.get_clock().now()    
        acc = self.icm.acceleration
        ang = self.icm.gyro
        duration = 1/200
        
        
        imu_msg.header.stamp = t.to_msg()
        imu_msg.header.frame_id = "imu_link"
        
        self.del_th_z += ang[2]*duration 
        self.del_th_x += ang[0]*duration
        self.del_th_y += ang[1]*duration
        x,y,z,w = quaternion_from_euler(self.del_th_x,self.del_th_y,self.del_th_z)
        
        imu_msg.orientation.x = x
        imu_msg.orientation.y = y
        imu_msg.orientation.z = z
        imu_msg.orientation.w = w
        
        imu_msg.angular_velocity.x = ang[0]
        imu_msg.angular_velocity.y = ang[1]
        imu_msg.angular_velocity.z = ang[2]
        
        imu_msg.linear_acceleration.x = acc[0]
        imu_msg.linear_acceleration.y = acc[1]
        imu_msg.linear_acceleration.z = acc[2]
        
        self.pub.publish(imu_msg)
    
    def imu_print(self):
        #print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (self.icm.acceleration))
        print("Gyro X:%.2f, Y: %.2f, Z: %.2f rads/s" % (self.icm.gyro))
        #print("Magnetometer X:%.2f, Y: %.2f, Z: %.2f uT" % (self.icm.magnetic))
        print("")
        
        
def main(args=None):
    try:
        rclpy.init(args=args)
        node = imu_pub()
        rclpy.spin(node)
        rclpy.shutdown()
    finally:
        pass