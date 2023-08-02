import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

import math
import numpy as np


from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped


current_yaw = 0.0
current_pos = np.array([0.0,0.0])
destination = np.array([0.0,0.0,0.0])
desired_yaw = 0.0
error_yaw = 0.0


class DynamicsProcessor(Node):

    def __init__(self):
        super().__init__('dynamics_processor')
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        
        
        self.publisher = self.create_publisher(Int32, 'pwm', 10)
        
        self.subscription = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.pose_callback,
            10)
        
        self.subscription = self.create_subscription(
            PoseStamped,
            'desired_heading',
            self.desired_heading_callback,
            10)
        
        self.subscription = self.create_subscription(
            Float32,
            '/destination/x',
            self.destination_x_callback,
            10)
        
        self.subscription = self.create_subscription(
            Float32,
            '/destination/y',
            self.destination_y_callback,
            10)
        
        self.subscription = self.create_subscription(
            Float32,
            'destination/arrival_radius',
            self.destination_arrival_radius_callback,
            10)
            
            
        self.publisher_ = self.create_publisher(Int32, 'pwm', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        
        
        
            
#########################################################
####  import 'pose data' from SLAM

    def pose_callback(self, pose):
        global current_yaw
        global desired_yaw
        global error_yaw
        current_pos[0] = pose.pose.position.x
        current_pos[1] = pose.pose.position.y
        x = pose.pose.orientation.x
        y = pose.pose.orientation.y
        z = pose.pose.orientation.z
        w = pose.pose.orientation.w
        rotation_matrix = np.array([[1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
                                [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
                                [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]])
                                
        yaw_radians = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        current_yaw = np.degrees(yaw_radians)
        
        
        #self.get_logger().info('current_yaw: "%f"' % current_yaw)
        #self.get_logger().info('desired_yaw: "%f"' % desired_yaw)
        

#########################################################
####  import 'desired heading'        

    def desired_heading_callback(self, desired_heading):
        global current_yaw
        global desired_yaw
        global error_yaw
        x = desired_heading.pose.orientation.x
        y = desired_heading.pose.orientation.y
        yaw_radians = np.arctan2(y, x)
        desired_yaw = np.degrees(yaw_radians)
        


#########################################################
####  import 'destination info'


    def destination_x_callback(self, msg):
        destination[0] = msg.data
        
        #self.get_logger().info('dest_x  %f' % destination_x)

    def destination_y_callback(self, msg):
        destination[1] = msg.data
        
        #self.get_logger().info('dest_y  %f' % destination_y)

    def destination_arrival_radius_callback(self, msg):
        destination[2] = msg.data
        
        #self.get_logger().info('dest_y  %f' % destination_y)



       


#########################################################
####  process and publish 


    def timer_callback(self):
            
        error_yaw = desired_yaw - current_yaw
        if error_yaw < -180:
            error_yaw += 360
        
        x=(current_pos[0] - destination[0])
        y=(current_pos[1] - destination[1])
        distance = math.sqrt(x**2 + y**2)
        


        
        if (distance<=destination[2]):    # destination[2] : arrival radius
            thruster_pwm = 1500
            servo_pwm = 1500
            
        elif ( error_yaw<=10 and error_yaw>= -10 ):
            thruster_pwm = 1580
            servo_pwm = 1500
            
            
        elif ( error_yaw<=40 and error_yaw>= 10 ):
            thruster_pwm = 1580
            servo_pwm = 1600
        elif ( error_yaw<= -10 and error_yaw>= -40 ):
            thruster_pwm = 1580
            servo_pwm = 1400
            
            
        elif ( error_yaw<=80 and error_yaw>= 40 ):
            thruster_pwm = 1580
            servo_pwm = 1700
        elif ( error_yaw<= -40 and error_yaw>= -80 ):
            thruster_pwm = 1580
            servo_pwm = 1300
            
            
        elif ( error_yaw>= 80 ):
            thruster_pwm = 1580
            servo_pwm = 1800
        elif ( error_yaw<= -80 ):
            thruster_pwm = 1580
            servo_pwm = 1200
            
            
        
        
        
        data=Int32()
        data.data= thruster_pwm*10000 + servo_pwm + 100000000
        self.publisher_.publish(data)
        self.get_logger().info('Publishing: %d' % data.data )






def main(args=None):

    rclpy.init(args=args)

    dynamics_processor = DynamicsProcessor()

    rclpy.spin(dynamics_processor)

    dynamics_processor.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
