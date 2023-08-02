import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

import math
import numpy as np



waypoint_sequence = [ 'waypoint_1', 'waypoint_2', 'waypoint_3' ]
waypoint_data = {

    'waypoint_1': {
        'x': 3.0,
        'y': 0.0,
        'time': 3
    },

    'waypoint_2': {
        'x': 0.0,
        'y': 0.0,
        'time': 3
    },

    'waypoint_3': {
        'x': 3.0,
        'y': 0.0,
        'time': 3
    },



}

timer_count = 0
waypoint_count = 0
arrival_radius = 0.75

x_data = Float32()
y_data = Float32()
arrival_radius_data = Float32()

pos=np.array([0.0,0.0]) # x, y
pos_imported = 0



class WaypointPublisher(Node):

    def __init__(self):
        super().__init__('waypoint_publisher')
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        
        
        self.publisher_x = self.create_publisher(Float32, '/destination/x', 10)
        self.publisher_y = self.create_publisher(Float32, '/destination/y', 10)
        self.publisher_arrival_radius = self.create_publisher(Float32, '/destination/arrival_radius', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) 
            
                   
        self.subscription = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.pose_callback,
            qos_profile)



    def timer_callback(self):
        
        global waypoint_count
        global timer_count
        global arrival_radius
        
        current_waypoint = waypoint_sequence[waypoint_count]
        
        x_data.data = waypoint_data[current_waypoint]['x']
        y_data.data = waypoint_data[current_waypoint]['y']
        arrival_radius_data.data = arrival_radius
        
        self.publisher_x.publish(x_data)
        self.publisher_y.publish(y_data)
        self.publisher_arrival_radius.publish(arrival_radius_data)
        
        x=(pos[0] - waypoint_data[current_waypoint]['x'])
        y=(pos[1] - waypoint_data[current_waypoint]['y'])
        distance = math.sqrt(x**2 + y**2)
        
        
        
        if (distance <= arrival_radius):
            timer_count+=1
        else:
            timer_count=0
            
            
        if (timer_count > 10 * waypoint_data[current_waypoint]['time'] and waypoint_count < len(waypoint_data)-1 ):
            waypoint_count+=1
            timer_count=0
        
        
        self.get_logger().info('waypoint = %d , timer = %d , distance = %f' % (waypoint_count, timer_count, distance ) )




    def pose_callback(self, pose):
        #global pos_imported
        #pos_imported=1
        pos[0] = pose.pose.position.x
        pos[1] = pose.pose.position.y



def main(args=None):

    rclpy.init(args=args)

    waypoint_publisher = WaypointPublisher()

    rclpy.spin(waypoint_publisher)

    waypoint_publisher.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()

