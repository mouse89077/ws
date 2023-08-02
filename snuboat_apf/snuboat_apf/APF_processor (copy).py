import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

import math
import numpy as np

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped


covariance=1
prohibited_range=0.2


class APFProcessor(Node):

    def __init__(self):
    
        super().__init__('apf_processor')
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile) 
            
                   
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            qos_profile)



    def lidar_callback(self, scan):
        
        lidar_array = np.array(scan.ranges)
        
        lidar_array = np.reciprocal(lidar_array, out=np.zeros_like(lidar_array), where=lidar_array!=0) / len(scan.ranges)
        
        
        x=0.0
        y=0.0
        
        
        
        for i in range(0,len(lidar_array)):
            x += (lidar_array[i]*math.sin(i*math.pi/ (len(lidar_array)/2) ))
            y += (lidar_array[i]*math.cos(i*math.pi/ (len(lidar_array)/2) ))       
        
        
        
        
        
        self.get_logger().info('I heard: "%f,  %f,  %f"' % (  len(lidar_array), x, y  )  )

	

    def pose_callback(self, pose):
        
        x = pose.pose.pose.position.x
        y = pose.pose.pose.position.y
        z = pose.pose.pose.position.z
        qx = pose.pose.pose.orientation.x
        qy = pose.pose.pose.orientation.y
        qz = pose.pose.pose.orientation.z
        qw = pose.pose.pose.orientation.w
        #self.get_logger().info('I heard: "%f,  %f,  %f"' % (  x, y, z  )  )









def main(args=None):

    rclpy.init(args=args)

    apf_processor = APFProcessor()

    rclpy.spin(apf_processor)

    apf_processor.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
