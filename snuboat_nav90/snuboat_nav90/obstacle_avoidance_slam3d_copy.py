import rclpy
import os
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, TwistWithCovarianceStamped, PoseStamped
from microstrain_inertial_msgs.msg import FilterHeading
from std_msgs.msg import Bool, Int8, Int32, Float32, Float64, String, Float64MultiArray, Int32MultiArray, Int8MultiArray
from nav_msgs.msg import Odometry
import numpy as np
import math
import tf2_ros

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance")

        # Constants
        self.L = 0.9
        self.B = 0.3
        self.dt = 0.1
        self.cur_wp_idx = 0
        self.safe_radius = 1.5
        self.ref_safe_radius = 1.5
        self.goal_tol = 1.3
        self.wp_stay_time = 30
        self.inf_length = self.B * 1.3
        self.ref_inf_length = self.B * 1.3
        self.dock_wp_idx = 10
        self.dock_wp_set_x = [-1, 0, 1]
        self.dock_wp_set_y = [-1, 0, 1]
        self.odom_wp_x_set = [5.0, 5.0, 1.0, 1.0]
        self.odom_wp_y_set = [6.0, 1.0, 6.0, 1.0]

        # Other variables
        self.obs_labels = []
        self.obs_r = []
        self.obs_phi = []
        self.obs_x = []
        self.obs_y = []
        self.current_pos = []
        self.current_orientation = []
        self.odom_pos = []
        self.odom_orientation = []
        self.odom_twist = []
        self.odom_twist_ang = []
        self.ref_heading = 0.0
        self.wp_reach_check = False
        self.wp_time_cnt = 0
        self.wp_clear = False
        self.docking_state = False
        self.detect_state = False
        self.wp_state = False
        self.ref_spd = 0.8
        self.safe_heading = []
        self.heading_cost = []
        self.des_heading = np.zeros(10)
        self.heading = np.zeros(10)
        self.des_spd = np.ones(10)
        self.err_heading = np.zeros(10)
        self.inflate_obs_phi = np.deg2rad(20)

        # Environment variables
        os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} [{name}] [{severity}] {message}'

        # Subscribers
        self.obs_labels_sub = self.create_subscription(Int32MultiArray, "/obs/labels", self.obs_labels_callback, 1)
        self.obs_r_sub = self.create_subscription(Float64MultiArray, "/obs/r", self.obs_r_callback, 1)
        self.obs_phi_sub = self.create_subscription(Float64MultiArray, "/obs/phi", self.obs_phi_callback, 1)
        self.obs_x_sub = self.create_subscription(Float64MultiArray, "/obs/x", self.obs_x_callback, 1)
        self.obs_y_sub = self.create_subscription(Float64MultiArray, "/obs/y", self.obs_y_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 1)
        self.current_pos_sub = self.create_subscription(PoseStamped, "/current_pose", self.current_pose_callback, 1)
        self.shape_detect_sub = self.create_subscription(Int8MultiArray, "/detect_state", self.shape_detect_callback, 1)

        # Publishers
        self.des_heading_pub = self.create_publisher(Float64, "/des_heading", 1)
        self.des_spd_pub = self.create_publisher(Float64, "/des_spd", 1)
        self.cur_wp_idx_pub = self.create_publisher(Int8, "/wp_idx", 1)
        self.wp_set_x_pub = self.create_publisher(Float64MultiArray, "wp_set/x", 1)
        self.wp_set_y_pub = self.create_publisher(Float64MultiArray, "wp_set/y", 1)
        self.wp_check_pub = self.create_publisher(Bool, "/wp_check", 1)
        self.wp_clear_pub = self.create_publisher(Bool, "/wp_clear", 1)
        self.err_heading_pub = self.create_publisher(Float64, "/err_heading", 1)
        self.ref_heading_pub = self.create_publisher(Float64, "/ref_heading", 1)
        self.safe_heading_pub = self.create_publisher(Float64MultiArray, "/safe_heading", 1)
        self.des_pub = self.create_timer(self.dt, self.pub_des)
        self.err_x_pub = self.create_publisher(Float64, "/err_x", 1)
        self.err_y_pub = self.create_publisher(Float64, "/err_y", 1)
        self.err_heading_next_pub = self.create_publisher(Float64, "/err_heading_next", 1)
        self.docking_state_pub = self.create_publisher(Bool, "/docking_state", 1)

        # Initialize flags
        self.obs_labels_received = False
        self.obs_r_received = False
        self.obs_phi_received = False
        self.obs_x_received = False
        self.obs_y_received = False
        self.current_pose_received = False

    def wait_for_topics(self):
        self.timer = self.create_timer(1.0, self.check_topic_status)

    def check_topic_status(self):
        if not self.current_pose_received:
            self.get_logger().info("No topic current_pose_received")
        if not self.obs_labels_received:
            self.get_logger().info("No topic obs_labels_received")
        if not self.obs_r_received:
            self.get_logger().info("No topic obs_r_received")
        if not self.obs_phi_received:
            self.get_logger().info("No topic obs_phi_received")
        if not self.obs_x_received:
            self.get_logger().info("No topic obs_x_received")
        if not self.obs_y_received:
            self.get_logger().info("No topic obs_y_received")

        if (
            self.current_pose_received
            and self.obs_labels_received
            and self.obs_r_received
            and self.obs_phi_received
            and self.obs_x_received
            and self.obs_y_received
        ):
            self.get_logger().info("All topics received")
        else:
            self.get_logger().info("Waiting for topics to be published")

    def obs_labels_callback(self, msg):
        self.obs_labels_received = True
        self.obs_labels = np.array(msg.data)

    def obs_r_callback(self, msg):
        self.obs_r_received = True
        self.obs_r = np.array(msg.data)

    def obs_phi_callback(self, msg):
        self.obs_phi_received = True
        self.obs_phi = np.array(msg.data)

    def obs_x_callback(self, msg):
        self.obs_x_received = True
        self.obs_x = np.array(msg.data)

    def obs_y_callback(self, msg):
        self.obs_y_received = True
        self.obs_y = np.array(msg.data)

    def current_pose_callback(self, msg):
        self.current_pose_received = True
        self.current_pos = [msg.pose.position.x, msg.pose.position.y]
        self.current_orientation = [msg.pose.orientation.x, msg.pose.orientation.y,
                                    msg.pose.orientation.z, msg.pose.orientation.w]

    def odom_callback(self, msg):
        self.odom_received = True
        self.odom_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.odom_orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                 msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.odom_twist = [msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                           msg.twist.twist.linear.z]
        self.odom_twist_ang = [msg.twist.twist.angular.x, msg.twist.twist.angular.y,
                               msg.twist.twist.angular.z]

    def shape_detect_callback(self, msg):
        if msg.data[0] == 1:
            self.get_logger().info("Changing waypoint to docking point")
            if msg.data[1] == -1:
                self.get_logger().info("Left")
            elif msg.data[1] == 0:
                self.get_logger().info("Middle")
            elif msg.data[1] == 1:
                self.get_logger().info("Right")
            

    def pub_des(self):
        if self.current_pose_received:
            if len(self.obs_labels) == len(self.obs_r) and len(self.obs_labels) == len(self.obs_phi) \
                    and len(self.obs_labels) == len(self.obs_x) and len(self.obs_labels) == len(self.obs_y):
                self.cal_des()
            else:
                return
        else:
            return

        cur_ort = self.current_orientation
        euler = quaternion_to_euler(cur_ort)
        self.heading = np.append(self.heading, euler[2])
        self.heading = self.heading[1:]

        wp_set_x = Float64MultiArray()
        wp_set_x.data = self.odom_wp_x_set
        self.wp_set_x_pub.publish(wp_set_x)

        wp_set_y = Float64MultiArray()
        wp_set_y.data = self.odom_wp_y_set
        self.wp_set_y_pub.publish(wp_set_y)

        temp_err = self.des_heading[-1] - self.heading[-1]

        if temp_err > np.pi:
            temp_err -= 2 * np.pi
        elif temp_err < -np.pi:
            temp_err += 2 * np.pi

        self.err_heading = np.append(self.err_heading, temp_err)
        self.err_heading = self.err_heading[1:]

        cur_pos = self.current_pos

        err_g = np.array([cur_pos[0] - self.odom_wp_x_set[self.cur_wp_idx],
                        cur_pos[1] - self.odom_wp_y_set[self.cur_wp_idx],
                        self.err_heading[-1]]).transpose()

        Rt = np.array([[np.cos(self.heading[-1]), np.sin(self.heading[-1]), 0],
                    [-np.sin(self.heading[-1]), np.cos(self.heading[-1]), 0],
                    [0, 0, 1]])

        err_b = np.dot(np.linalg.inv(Rt), err_g)
        temp_err_x = Float64()
        temp_err_x.data = float(err_b[0])
        temp_err_y = Float64()
        temp_err_y.data = float(err_b[1])

        self.err_x_pub.publish(temp_err_x)
        self.err_y_pub.publish(temp_err_y)
        
        print("cur_wp_idx", self.cur_wp_idx, \
              "des_heading", np.rad2deg(self.des_heading[-1]), \
              "heading", np.rad2deg(self.heading[-1]), \
              "err_heading", np.ceil(np.rad2deg(self.err_heading[-1])), \
              "des_spd", self.des_spd[-1],\
              "wp_check", self.wp_reach_check)
        ref_heading = Float64()
        ref_heading.data= self.ref_heading
        self.ref_heading_pub.publish(ref_heading)

        des_heading = Float64()
        des_heading.data = self.des_heading[-1]
        self.des_heading_pub.publish(des_heading)

        err_heading_temp = Float64()
        err_heading_temp.data = self.err_heading[-1]
        self.err_heading_pub.publish(err_heading_temp)
            
        des_spd = Float64()
        des_spd.data = self.des_spd[-1]
        self.des_spd_pub.publish(des_spd)

        cur_wp_idx_ = Int8()
        cur_wp_idx_.data = self.cur_wp_idx
        self.cur_wp_idx_pub.publish(cur_wp_idx_)

        wp_check = Bool()
        wp_check.data = self.wp_reach_check
        self.wp_check_pub.publish(wp_check)

        if self.cur_wp_idx + 1 != len(self.odom_wp_x_set):
            err_heading_next = Float64()
            err_heading_next_ = np.arctan2(self.odom_wp_y_set[self.cur_wp_idx+1] \
                                            - cur_pos[1], self.odom_wp_x_set[self.cur_wp_idx+1] - cur_pos[0]) - self.heading[-1]
            if err_heading_next_ > np.pi:
                err_heading_next_ -= 2*np.pi
            elif err_heading_next_ < -np.pi:
                err_heading_next_ += 2*np.pi

            err_heading_next.data = err_heading_next_
            self.err_heading_next_pub.publish(err_heading_next)

        # publish docking state
        if self.cur_wp_idx == self.dock_wp_idx:
            self.docking_state = True
        else:
            self.docking_state = False

        docking_state = Bool()
        docking_state.data = self.docking_state
        self.docking_state_pub.publish(docking_state)

    def cal_des(self):
        # if(self.odom_received==True):
        # if(self.robot_pose_received==True):
        if self.current_pose_received == True:
            # cur_pos = self.odom_pos
            # cur_pos = self.robot_pos
            cur_pos = self.current_pos
            self.print_dist = np.linalg.norm([cur_pos[0] - self.odom_wp_x_set[self.cur_wp_idx], 
                                              cur_pos[1] - self.odom_wp_y_set[self.cur_wp_idx]])
            self.wp_reach_check = bool(
                self.print_dist < self.goal_tol
            )


            #DP later...
            #waypoint reach
            if self.wp_reach_check == True:
                # Waypoint mission clear check
                if self.cur_wp_idx+1 == len(self.odom_wp_x_set):
                    self.get_logger().info("Waypoint Mission Clear")
                    self.wp_clear = True
                else:
                    self.wp_clear = False

                #waypoint mission clear
                if self.wp_clear ==True:
                    print("end")
                
                # on going mission
                else:
                    #first reach in waypoint
                    if self.wp_state == False:
                        self.wp_state = True
                        # self.cur_wp_idx += 1
                        self.des_spd = np.append(self.des_spd, 0)
                        self.des_spd = self.des_spd[1:]
                        des_heading = np.arctan2(self.odom_wp_y_set[self.cur_wp_idx+1] - cur_pos[1], self.odom_wp_x_set[self.cur_wp_idx+1] - cur_pos[0])

                        self.des_heading = np.append(self.des_heading, des_heading)
                        self.des_heading = self.des_heading[1:]
                        # self.wp_time_cnt += 1
                        print("reached")
                        
                    else:  # self.wp_state = True
                        #running while in waypoint
                        if self.wp_time_cnt < self.wp_stay_time:
                            self.wp_time_cnt += 1
                            self.wp_state = True
                            self.des_spd = np.append(self.des_spd, 0)
                            self.des_spd = self.des_spd[1:]
                            des_heading = np.arctan2(self.odom_wp_y_set[self.cur_wp_idx+1] - cur_pos[1], self.odom_wp_x_set[self.cur_wp_idx+1] - cur_pos[0])

                            self.des_heading = np.append(self.des_heading, des_heading)
                            self.des_heading = self.des_heading[1:]

                        #exit waypoint
                        else:  # self.wp_time_cnt > self.wp_stay_time
                            self.wp_time_cnt = 0
                            self.wp_state = False
                            self.des_spd = np.append(self.des_spd, self.ref_spd)
                            self.des_spd = self.des_spd[1:]
                            des_heading = np.arctan2(self.odom_wp_y_set[self.cur_wp_idx+1] - cur_pos[1], self.odom_wp_x_set[self.cur_wp_idx+1] - cur_pos[0])

                            self.des_heading = np.append(self.des_heading, des_heading)
                            self.des_heading = self.des_heading[1:]
                            self.get_logger().info("Changing waypoint ...")
                            self.cur_wp_idx += 1
                        
            #waypoing going
            else:  # wp_reach_check == False:
                self.wp_state = False
                self.ref_heading = np.arctan2(self.odom_wp_y_set[self.cur_wp_idx] - cur_pos[1], self.odom_wp_x_set[self.cur_wp_idx] - cur_pos[0])
                print("this is ref heading")
                print(np.rad2deg(self.ref_heading))
                #### calculate des_heading and des_spd
                if len(self.obs_labels) != 0: # if there are scanned obstacles
                    self.danger_r = []
                    self.danger_phi = []
                    self.danger_x = []
                    self.danger_y = []
                    self.safe_phi = np.linspace(-np.pi, np.pi, 360).transpose()
                    # print(np.shape(self.safe_phi))
                    self.safe_heading = []

                    # safe_phi
                    idx_array = np.where(np.diff(self.obs_labels) != 0)[0] + 1
                    # idx_array = np.where(np.diff(self.obs_labels) != 0)[0]
                    idx_array =np.append(0,idx_array)
                    # print(len(idx_array))
                    # print(idx_array)
                    # print("this is r fro obs")
                    # print(self.obs_r)
                    #cal safe_heading
                    for i, idx in enumerate(idx_array):
                        #loop until last obs
                        if idx != idx_array[-1]:
                            # print(len(self.obs_r[idx:idx_array[i+1]-1]))
                            # delete noise obstable, setting point num
                            if len(self.obs_r[idx:idx_array[i+1]-1]) > 10:
                                # print(1)
                                # print(self.obs_r[idx:idx_array[i+1]-1])
                                print("this is minimum value")
                                print(np.min(self.obs_r[idx:idx_array[i+1]-1]))

                                print("this is obs_phi")
                                print(np.rad2deg(self.obs_phi[idx:idx_array[i+1]-1]))
                                if np.min(self.obs_r[idx:idx_array[i+1]-1]) < self.safe_radius:
                                    # define start_phi
                                    r = self.obs_r[idx]
                                    inflate_obs_phi = np.arccos((2*r*r - (self.B * 1.2)**2)/(2*r*r))
                                    del r

                                    if self.obs_phi[idx] - inflate_obs_phi > -np.pi:
                                        start_phi = self.obs_phi[idx] - inflate_obs_phi
                                    else:
                                        start_phi = -np.pi

                                    del inflate_obs_phi

                                    # define end_phi
                                    r = self.obs_r[idx_array[i+1]-1]
                                    inflate_obs_phi = np.arccos((2*r*r - (self.B * 1.2)**2)/(2*r*r)) # TODO: adjust parameters depending on dimension
                                    del r

                                    if self.obs_phi[idx_array[i+1]-1] + inflate_obs_phi < np.pi:
                                        end_phi = self.obs_phi[idx_array[i+1]-1] + inflate_obs_phi
                                    else:
                                        end_phi = np.pi

                                    del inflate_obs_phi

                                    print("start_phi, end_phi")
                                    print(np.rad2deg(start_phi), np.rad2deg(end_phi))
                                    temp_safe_phi1 = self.safe_phi
                                    safe_idx1 = np.array(temp_safe_phi1 < start_phi)
                                    temp_safe_phi1 = temp_safe_phi1[safe_idx1]
                                    temp_safe_phi1 = np.reshape(temp_safe_phi1, (1, -1))
                                    print("start~")
                                    print(np.rad2deg(temp_safe_phi1))
                                    temp_safe_phi2 = self.safe_phi
                                    safe_idx2 = np.array(temp_safe_phi2 > end_phi)
                                    temp_safe_phi2= temp_safe_phi2[safe_idx2]
                                    temp_safe_phi2 = np.reshape(temp_safe_phi2, (1, -1))
                                    print("~end")
                                    print(np.rad2deg(temp_safe_phi2))
                                    self.safe_phi = np.append(temp_safe_phi1, temp_safe_phi2)
                                    print("this is safe_phi")
                                    print(np.rad2deg(self.safe_phi))
                                    # return 0
                            
                        else:
                            if len(self.obs_r[idx:]) > 10:
                                # print(2)
                                if np.min(self.obs_r[idx:]) < self.safe_radius:
                                    # print(2)
                                    # define start_phi
                                    r = self.obs_r[idx]
                                    inflate_obs_phi = np.arccos((2*r*r - (self.inf_length)**2)/(2*r*r))
                                    del r
                                    print("this is inflate obs phi")
                                    print(inflate_obs_phi)
                                    print("this is last obs phi")
                                    print(np.rad2deg(self.obs_phi[idx:]))
                                    if self.obs_phi[idx] - inflate_obs_phi > -np.pi:
                                        start_phi = self.obs_phi[idx] - inflate_obs_phi
                                    else:
                                        start_phi = -np.pi

                                    del inflate_obs_phi

                                    # define end_phi
                                    r = self.obs_r[-1]
                                    inflate_obs_phi = np.arccos((2*r*r - (self.inf_length)**2)/(2*r*r))
                                    del r

                                    if self.obs_phi[-1] + self.inflate_obs_phi < np.pi:
                                        end_phi = self.obs_phi[-1] + self.inflate_obs_phi
                                    else:
                                        end_phi = np.pi

                                    del inflate_obs_phi

                                    print(np.rad2deg(start_phi), np.rad2deg(end_phi))
                                    temp_safe_phi1 = self.safe_phi
                                    safe_idx1 = np.array(temp_safe_phi1 < start_phi)
                                    temp_safe_phi1 = temp_safe_phi1[safe_idx1]
                                    temp_safe_phi1 = np.reshape(temp_safe_phi1, (1, -1))
                                    print("this is last obs ~start")
                                    print(np.rad2deg(temp_safe_phi1))
                                    temp_safe_phi2 = self.safe_phi
                                    safe_idx2 = np.array(temp_safe_phi2 >end_phi)
                                    temp_safe_phi2= temp_safe_phi2[safe_idx2]
                                    temp_safe_phi2 = np.reshape(temp_safe_phi2, (1, -1))
                                    print("this is last obs end~")
                                    print(np.rad2deg(temp_safe_phi2))
                                    self.safe_phi = np.append(temp_safe_phi1, temp_safe_phi2)
                                    
                    print("safe_phi")
                    print(np.rad2deg(self.safe_phi))
                    # self.safe_heading = self.safe_phi
                    self.safe_heading = self.safe_phi + self.heading[-1]
                    print("safe heading")
                    print(self.safe_heading)
                    minus_idx = self.safe_heading > np.pi
                    self.safe_heading[minus_idx] -= 2*np.pi
                    plus_idx = self.safe_heading < -np.pi
                    self.safe_heading[plus_idx] += 2*np.pi
                    print("deg safe heading")
                    print(np.rad2deg(self.safe_heading))
                    ##safe phi

                    future_cost = self.safe_heading - self.ref_heading
                    future_minus_idx = future_cost > np.pi
                    future_cost[future_minus_idx] -= 2*np.pi
                    future_plus_idx = future_cost < -np.pi
                    future_cost[future_plus_idx] += 2*np.pi

                    past_cost = self.safe_heading - self.des_heading[-1]
                    past_minus_idx = past_cost > np.pi
                    past_cost[past_minus_idx] -= 2*np.pi
                    past_plus_idx = past_cost < -np.pi
                    past_cost[past_plus_idx] += 2*np.pi

                    # print(future_cost)
                    
                    # past_cost=0

                    #ds heading
                    # self.heading_cost = abs(future_cost) + 0.2 * abs(past_cost)
                    self.heading_cost = abs(future_cost)
                    # print(self.heading_cost)
                    # print(np.rad2deg(self.heading_cost))

                    # calculate des_heading
                    self.des_spd = np.append(self.des_spd, self.ref_spd)
                    self.des_spd = self.des_spd[1:]
                    temp_safe = Float64MultiArray()
                    temp_safe.data = (self.safe_heading).tolist()
                    self.safe_heading_pub.publish(temp_safe)
                    print("this is safe heading")
                    print(self.safe_heading)
                    if len(self.safe_heading) != 0:
                        des_heading_idx = np.argmin(self.heading_cost)
                        des_heading = self.safe_heading[des_heading_idx]
                        
                        self.des_heading = np.append(self.des_heading, des_heading)
                        self.des_heading = self.des_heading[1:]
                        # if self.safe_radius < self.ref_safe_radius:
                        #     self.safe_radius += 0.1 # increase safe_radius until it reaches self.ref_safe_radius


                        # # Roll back safe values 
                        if self.inf_length < self.ref_inf_length:
                            self.inf_length += 0.01
                        if self.safe_radius < self.ref_safe_radius:
                            self.safe_radius += 0.01
                        
                    else: # no safe heading
                        #Edit inf length until 0.5 B
                        
                        # if self.inf_length > 0.5 * self.B:
                        #     self.inf_length -= 0.01

                        # # inf length not enough
                        # # Edit safe radius
                        # else: 
                        #     if self.safe_radius > 0.5:
                        #         self.safe_radius -= 0.01
                        #     else:
                        #         self.get_logger().info('No Answer GG')
                        # self.get_logger().info('No safe heading')
                        # self.get_logger().info("reduce inf length : " + str(self.inf_length))
                        # self.get_logger().info("reduce safe radius : " + str(self.safe_radius))
                        return
                    
                    # self.get_logger().info('Inf Length: ' + str(self.inf_length))
                    # self.get_logger().info('Safe radius: ' + str(self.safe_radius))

                else: # if there are no scanned obstacles
                    self.des_spd = np.append(self.des_spd, self.des_spd[-1])
                    self.des_spd = self.des_spd[1:]
                    # self.des_heading = np.append(self.des_heading, self.des_heading[-1])
                    # self.des_heading = self.des_heading[1:]
                    self.des_heading = np.append(self.des_heading, self.ref_heading)
                    self.des_heading = self.des_heading[1:]
            
            wp_clear = Bool()
            wp_clear.data = self.wp_clear
            #wp clear index publish
            self.wp_clear_pub.publish(wp_clear)
        else:
            print("not received yet")


        


def quaternion_to_euler(q):
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw).

    :param q: A tuple or list representing the quaternion (w, x, y, z).
    :return: A tuple representing the Euler angles (roll, pitch, yaw) in radians.
    """
    # w, x, y, z = q

    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = Obstacle_Avoidance()
    obstacle_avoidance.wait_for_topics()
    rclpy.spin(obstacle_avoidance)
    obstacle_avoidance.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()