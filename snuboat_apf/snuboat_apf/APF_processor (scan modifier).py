import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

import math

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


        
class PUBnSUB(Node):

    def __init__(self):
        super().__init__('pubnsub')
    	
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        
        
        self.sub = self.create_subscription(LaserScan, '/scan', self.listener_callback, qos_profile)        
        self.pub_scan = self.create_publisher(LaserScan, '/kaboat/scan_mod', qos_profile)

        
        
        
    def listener_callback(self, msg):
        #self.get_logger().info('Scan info imported')
    
        
        
        msg_mod = LaserScan()
        msg_mod.header = msg.header
        msg_mod.angle_min =  msg.angle_min
        msg_mod.angle_max =  msg.angle_max
        msg_mod.angle_increment = msg.angle_increment*4
        msg_mod.time_increment = msg.time_increment
        msg_mod.scan_time = msg.scan_time
        msg_mod.range_min = msg.range_min
        msg_mod.range_max = msg.range_max
        msg_mod.ranges = [ msg.ranges[r] for r in range(0,497)]
        



        
        
        self.pub_scan.publish(msg_mod)
        
           


def main(args=None):
    rclpy.init(args=args)

    pubnsub = PUBnSUB()

    rclpy.spin(pubnsub)
    
    pubnsub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

