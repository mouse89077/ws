# KABOAT
import rclpy
import os
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float64, Float64MultiArray, Int32MultiArray
from sensor_msgs.msg import LaserScan
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler

class Lidar_Converter(Node):
    def __init__(self):
        super().__init__('lidar_converter')
        
        self.dt = 0.1
        self.polar_scan = np.empty((0,2),float)
        self.cart_scan = np.empty((0,2),float) # origin: boat
        self.scan_labels = np.empty((0, 2), int)

        self.lidar_scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_scan_callback, qos_profile_sensor_data)
        
        self.obs_labels_pub = self.create_publisher(Int32MultiArray, '/obs/labels', 1)
        self.obs_r_pub = self.create_publisher(Float64MultiArray, '/obs/r', 1)
        self.obs_phi_pub = self.create_publisher(Float64MultiArray, '/obs/phi', 1)
        self.obs_x_pub = self.create_publisher(Float64MultiArray, '/obs/x', 1)
        self.obs_y_pub = self.create_publisher(Float64MultiArray, '/obs/y', 1)
        
        #LIDAR range limit
        self.range_max = 5.0

        self.obstacles_timer = self.create_timer(self.dt, self.pub_obstacles)

        self.lidar_scan_received = False
        
    def wait_for_topics(self):
        self.timer = self.create_timer(1.0, self.check_topic_status)
        
    def check_topic_status(self):
        if not self.lidar_scan_received:
            self.get_logger().info('Topic lidar_scan not received')
        if not self.lidar_scan_received:
            self.get_logger().info('Waiting for topics to be published')
        else:
            self.get_logger().info('All topics received')
    
    # scan msg => save in polar_pub
    def lidar_scan_callback(self, msg):
        self.lidar_scan_received = True
        phi = msg.angle_min
        self.polar_scan = np.empty((0,2),float)
        self.cart_scan = np.empty((0,2),float)
        self.scan_labels = np.empty((0, 2), int)
        for r in msg.ranges:
            if msg.range_min <= r <= self.range_max:
                polar_point = np.array([[r, phi]])  # Convert to a NumPy array with shape (1, 2)
                cart_point = np.array([[r * np.cos(phi), r * np.sin(phi)]])  # Convert to a NumPy array with shape (1, 2)
                self.polar_scan = np.append(self.polar_scan, polar_point, axis=0)  # Append the NumPy arrays
                self.cart_scan = np.append(self.cart_scan, cart_point, axis=0)
            phi += msg.angle_increment
            
        if len(self.cart_scan) > 50:
            points = np.array(self.cart_scan)
            scaler = StandardScaler()
            points_scaled = scaler.fit_transform(points)
            dbscan = DBSCAN(eps=0.2, min_samples=50)  # Adjust the parameters as per your data
            dbscan.fit(points_scaled)
            self.scan_labels = dbscan.labels_
        # print("bef", len(self.scan_labels), len(self.polar_scan), len(self.cart_scan))
        
    # publish obstacle sinfo
    def pub_obstacles(self):
        if self.lidar_scan_received == True:
            if len(self.scan_labels) != 0:
                # print("aft1", len(self.scan_labels))
                indices_to_delete = [i 
                                    for i, label in enumerate(self.scan_labels) 
                                    if label == -1]
                self.scan_labels = np.delete(self.scan_labels, indices_to_delete)
                self.polar_scan = np.delete(self.polar_scan, indices_to_delete, axis=0)
                self.cart_scan = np.delete(self.cart_scan, indices_to_delete, axis=0)

                # print("label num", max(self.scan_labels) - min(self.scan_labels) + 1)
                # print(self.scan_labels)
                # print("polar scan",self.polar_scan)
                
                self.scan_labels = np.array(self.scan_labels)
                self.polar_scan = np.array(self.polar_scan)  # Convert back to NumPy array
                self.cart_scan = np.array(self.cart_scan)  # Convert back to NumPy array
                
                # print("aft2", len(self.scan_labels))

            obs_labels = Int32MultiArray()
            obs_labels.data = self.scan_labels.tolist()
            obs_r = Float64MultiArray()
            obs_r.data = self.polar_scan[:, 0].tolist()
            obs_phi = Float64MultiArray()
            obs_phi.data = self.polar_scan[:, 1].tolist()
            obs_x = Float64MultiArray()
            obs_x.data = self.cart_scan[:, 0].tolist()
            obs_y = Float64MultiArray()
            obs_y.data = self.cart_scan[:, 1].tolist()
            
            # print("pub", len(obs_labels.data), len(obs_r.data), len(obs_phi.data), len(obs_x.data), len(obs_y.data))
            
            if len(obs_labels.data) - len(obs_r.data) != 0:
                print("ERROR_r")
            if len(obs_labels.data) - len(obs_phi.data) != 0:
                print("ERROR_phi")
            if len(obs_labels.data) - len(obs_x.data) != 0:
                print("ERROR_x")
            if len(obs_labels.data) - len(obs_y.data) != 0:
                print("ERROR_y")
            if not(len(obs_labels.data) - len(obs_r.data) == 0 \
                and len(obs_labels.data) - len(obs_phi.data) == 0\
                and len(obs_labels.data) - len(obs_x.data) == 0 \
                and len(obs_labels.data) - len(obs_y.data) == 0) :
                return
            
            
            self.obs_labels_pub.publish(obs_labels)
            self.obs_r_pub.publish(obs_r)
            self.obs_phi_pub.publish(obs_phi)
            self.obs_x_pub.publish(obs_x)
            self.obs_y_pub.publish(obs_y)
            # obs_safe_phi = Float64MultiArray()
            # obs_safe_phi.data = 
            # self.obs_safe_phi_pub.publish(obs_safe_phi)
            # print(len(self.scan_labels), len(obs_r.data), len(obs_x.data))
            
            # print(np.shape(self.polar_scan[:, 1]))
        else:
            return
        
def main(args=None):
    rclpy.init(args=args)
    lidar_converter = Lidar_Converter()
    lidar_converter.wait_for_topics()
    rclpy.spin(lidar_converter)
    lidar_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
