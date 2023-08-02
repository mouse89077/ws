import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

import math
import numpy as np

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped



#current_heading_angle = 0.0



apf = np.array([0.0,0.0]) # x, y

pos = np.array([0.0,0.0,0.0]) # x, y, z

quat = np.array([0.0,0.0,0.0,0.0]) # x, y, z, w

destination = np.array([0.0,0.0]) # x, y

data = PoseStamped()


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
            PoseStamped,
            '/current_pose',
            self.pose_callback,
            qos_profile) 
            
                   
        self.subscription = self.create_subscription(
            Float32,
            '/destination/x',
            self.destination_x_callback,
            qos_profile) 
            
                   
        self.subscription = self.create_subscription(
            Float32,
            '/destination/y',
            self.destination_y_callback,
            qos_profile)
            
            
        self.publisher_ = self.create_publisher(PoseStamped, 'desired_heading', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        





#########################################################
####  import & process 'lidar scan data'


    def lidar_callback(self, scan):
        
        lidar_array = np.array(scan.ranges)
        
        lidar_array = np.reciprocal(lidar_array, out=np.zeros_like(lidar_array), where=lidar_array!=0) / len(scan.ranges)
        
        
        lidar_array = lidar_array*5  #covariance
        
        
        x=0.0
        y=0.0
        
        
        for i in range(0,len(lidar_array)-1):
            if (lidar_array[i]<2.5):
                x += (lidar_array[i]*math.sin(i*math.pi/ (len(lidar_array)/2) ))
                y += (lidar_array[i]*math.cos(i*math.pi/ (len(lidar_array)/2) ))       
        
        
        apf[0] = x
        apf[1] = y
        
        
        #self.get_logger().info('I heard: "%f,  %f,  %f"' % (  len(lidar_array), apf_x, apf_y  )  )

        #self.get_logger().info('I heard: "%f,  %f,  %f"' % (  pos_x, pos_y, 0.0  )  )

#########################################################
####  import 'pose data' from SLAM


    def pose_callback(self, pose):
        
        pos[0] = pose.pose.position.x
        pos[1] = pose.pose.position.y
        pos[2] = pose.pose.position.z
        
        quat[0] = pose.pose.orientation.x
        quat[1] = pose.pose.orientation.y
        quat[2] = pose.pose.orientation.z
        quat[3] = pose.pose.orientation.w
        
        data.header = pose.header
        
        '''
        current_heading_angle = np.arctan2(2*(quat_w*quat_z + quat_x*quat_y), 1 - 2*(quat_y**2 + quat_z**2))
        current_heading_angle = np.degrees(yaw)
        self.get_logger().info('I heard: "%f,  %f,  %f"' % (  yaw, pos_x, pos_y  )  )
        '''
        #self.get_logger().info('I heard:')
        


#########################################################
####  import 'destination info'


    def destination_x_callback(self, msg):
        destination[0] = msg.data
        
        #self.get_logger().info('dest_x  %f' % destination_x)

    def destination_y_callback(self, msg):
        destination[1] = msg.data
        
        #self.get_logger().info('dest_y  %f' % destination_y)



#########################################################
####  process and publish 


    def timer_callback(self):
        
        covariance = 5
        
        # 목표지점이랑 현재위치 빼서 목표방향구하기
        desired_x = destination[0] - pos[0]
        desired_y = destination[1] - pos[1]
        
        # 방향 벡터 크기 1로 만들기 
        magnitude = np.sqrt(desired_x**2 + desired_y**2)
        if(magnitude!=0):
            desired_x = desired_x/magnitude
            desired_y = desired_y/magnitude
        
        # 벡터에 상수 곱하기
        desired_x *= covariance
        desired_y *= covariance
        
        # 방향벡터에 apf 계산결과 더하기
        desired_x += apf[0]
        desired_y += apf[1]
        
        # 방향 벡터 크기 1로 만들기 
        magnitude = np.sqrt(desired_x**2 + desired_y**2)
        if(magnitude!=0):
            desired_x = desired_x/magnitude
            desired_y = desired_y/magnitude
        
        
        
        
        
        data.pose.position.x = pos[0]
        data.pose.position.y = pos[1]
        data.pose.position.z = pos[2]
        data.pose.orientation.x = desired_x
        data.pose.orientation.y = desired_y
        data.pose.orientation.z = 0.0
        data.pose.orientation.w = 0.0	
        
        self.publisher_.publish(data)
        self.get_logger().info('Publishing: %f , %f' % (desired_x, desired_y) )



#########################################################




def main(args=None):

    rclpy.init(args=args)

    apf_processor = APFProcessor()

    rclpy.spin(apf_processor)

    apf_processor.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
