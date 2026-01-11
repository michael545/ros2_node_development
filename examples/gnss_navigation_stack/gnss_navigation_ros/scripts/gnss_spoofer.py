#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
import math
import time

class GnssSpoofer(Node):
    def __init__(self):
        super().__init__('gnss_spoofer')
        self.publisher_fix = self.create_publisher(NavSatFix, '/gnss/fix', 10)
        self.publisher_imu = self.create_publisher(Imu, '/gnss/imu', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz
        self.t = 0.0
        
        # Simulation parameters for a circular path
        self.center_lat = 45.0
        self.center_lon = 10.0
        self.radius = 0.0001 # approx 10m
        self.speed = 0.5 

        self.get_logger().info('GNSS Spoofer started. Publishing to /gnss/fix and /gnss/imu')

    def timer_callback(self):
        msg_fix = NavSatFix()
        msg_fix.header.stamp = self.get_clock().now().to_msg()
        msg_fix.header.frame_id = "gnss_link"
        
        # Circular motion
        # lat = y, lon = x (roughly)
        angle = self.speed * self.t
        
        # Simple flat earth approximation for storage
        d_lat = self.radius * math.sin(angle)
        d_lon = self.radius * math.cos(angle)
        
        msg_fix.latitude = self.center_lat + d_lat
        msg_fix.longitude = self.center_lon + d_lon
        msg_fix.altitude = 100.0
        
        # Add some noise
        # msg_fix.latitude += random.gauss(0, 0.000001)
        
        msg_fix.status.status = 0 # Fix
        self.publisher_fix.publish(msg_fix)

        msg_imu = Imu()
        msg_imu.header.stamp = self.get_clock().now().to_msg()
        msg_imu.header.frame_id = "base_link"
        
        # Orientation (Yaw)
        # Tangent to the circle + pi/2 (since north is 0, but wait, usually East is 0 in ENU ...)
        # Let's just output a rotating yaw
        yaw = angle + (math.pi / 2)
        
        # Quaternion from yaw
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = 1.0 # cos(0)
        sp = 0.0
        cr = 1.0
        sr = 0.0
        
        msg_imu.orientation.w = cy * cp * cr + sy * sp * sr
        msg_imu.orientation.x = cy * cp * sr - sy * sp * cr
        msg_imu.orientation.y = sy * cp * sr + cy * sp * cr
        msg_imu.orientation.z = sy * cp * cr - cy * sp * sr
        
        self.publisher_imu.publish(msg_imu)
        
        self.t += 0.1

def main(args=None):
    rclpy.init(args=args)
    node = GnssSpoofer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
