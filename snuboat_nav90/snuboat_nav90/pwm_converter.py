# X-CORPS
import rclpy
import os
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Point, TwistWithCovarianceStamped
from std_msgs.msg import String, Float64, Int32, Bool, Int64, Float64, Float64MultiArray
from microstrain_inertial_msgs.msg import FilterHeading
import pymap3d as pm
import numpy as np

class PWMConverter(Node):

    def __init__(self):
        super().__init__('pwm_converter')

        default_params = {
            'rudder_lim' : [-50, 50], # 50
            # rudder pwm 500~2500 270 deg
            'rudder_pwm_lim' : [int(1500-50*(1000/135)), int(1500+50*(1000/135))], # 1000: left, 2000: right
            'thrust_pwm_lim' : [1300, 1700], # 1500: stop
            'dt' : 0.1, 
            'Kp_thrust' : 2.5,
            'Kd_thrust' : 0.5,
            'Kp_rudder' : 2.0, # 2.0
            'Kd_rudder' : 0.5, # 0.1
            'Kp_self_rot' : 0.5, # 0.5
            'Kd_self_rot' : 0.1,
            'Kp_DP_x' : 2.0, 
            'Kd_DP_x' : 1.0, 
            'Kp_DP_y' : 2.0, 
            'Kd_DP_y' : 1.0,
            'Kp_DP_hdg' : 2.0,
            'Kd_DP_hdg' : 1.0,
            'lx' : 0.35,
            'ly' : 0.07,
            'rudder_rate' : 150,
            'ref_thrust' : 1580,
        }
        #params setting
        self.rudder_lim = self.declare_parameter("rudder_lim", default_params['rudder_lim']).value
        self.rudder_pwm_lim = self.declare_parameter("rudder_pwm_lim", default_params['rudder_pwm_lim']).value
        self.thrust_pwm_lim = self.declare_parameter("thrust_pwm_lim", default_params['thrust_pwm_lim']).value
        self.dt = self.declare_parameter("dt", default_params['dt']).value
        self.Kp_thrust = self.declare_parameter("Kp_thrust", default_params['Kp_thrust']).value
        self.Kd_thrust = self.declare_parameter("Kd_thrust", default_params['Kd_thrust']).value
        self.Kp_rudder = self.declare_parameter("Kp_rudder", default_params['Kp_rudder']).value
        self.Kd_rudder = self.declare_parameter("Kd_rudder", default_params['Kd_rudder']).value
        self.Kp_self_rot = self.declare_parameter("Kp_self_rot", default_params['Kp_self_rot']).value
        self.Kd_self_rot = self.declare_parameter("Kd_self_rot", default_params['Kd_self_rot']).value
        self.Kp_DP_x = self.declare_parameter("Kp_DP_x", default_params['Kp_DP_x']).value
        self.Kd_DP_x = self.declare_parameter("Kd_DP_x", default_params['Kd_DP_x']).value
        self.Kp_DP_y = self.declare_parameter("Kp_DP_y", default_params['Kp_DP_y']).value
        self.Kd_DP_y = self.declare_parameter("Kd_DP_y", default_params['Kd_DP_y']).value
        self.Kp_DP_hdg = self.declare_parameter("Kp_DP_hdg", default_params['Kp_DP_hdg']).value
        self.Kd_DP_hdg = self.declare_parameter("Kd_DP_hdg", default_params['Kd_DP_hdg']).value
        self.lx = self.declare_parameter("lx", default_params['lx']).value
        self.ly = self.declare_parameter("ly", default_params['ly']).value
        self.rudder_rate = self.declare_parameter("rudder_rate", default_params['rudder_rate']).value
        self.ref_thrust = self.declare_parameter("ref_thrust", default_params['ref_thrust']).value
        
        #initialize          
        self.des_heading = np.zeros(10) # [rad]
        self.des_spd = np.zeros(10)
        self.heading = np.zeros(10) # [rad]
        self.spd = np.zeros(10)
        
        self.err_heading = np.zeros(10)
        self.err_spd = np.zeros(10)
        self.err_heading_next = np.zeros(10)
        self.thrust_pwm_L = 1500 * np.ones(10)
        self.thrust_pwm_R = 1500 * np.ones(10)
        self.rudder_pwm_L = 1500 * np.ones(10)
        self.rudder_pwm_R = 1500 * np.ones(10)

        self.err_x = np.zeros(10)
        self.err_y = np.zeros(10)
        self.safe_heading = []

        self.r_pwm = 1500
        
        self.wp_check_sub = self.create_subscription(Bool, '/wp_check', self.wp_check_callback, 1)

        self.err_heading_sub = self.create_subscription(Float64, '/err_heading', self.err_heading_callback, 1)
        self.err_heading_next_sub = self.create_subscription(Float64, '/err_heading_next', self.err_heading_next_callback, 1)
        self.err_x_sub = self.create_subscription(Float64, '/err_x', self.err_x_callback, 1)
        self.err_y_sub = self.create_subscription(Float64, '/err_y', self.err_y_callback, 1)
        # self.OS_des_spd_sub =self.create_subscription(Float64, '/des_spd', self.OS_des_spd_callback, 1)
        # self.OS_heading_sub = self.create_subscription(FilterHeading, '/nav/heading', self.OS_heading_callback, 1)
        self.safe_heading_sub = self.create_subscription(Float64MultiArray, '/safe_heading', self.safe_heading_callback, 1)
        self.wp_clear_sub = self.create_subscription(Bool, '/wp_clear', self.wp_clear_callback, 1)
        self.docking_state_sub = self.create_subscription(Bool, "/docking_state", self.docking_state_callback, 1)
        self.detect_state_sub = self.create_subscription(Bool, "/detect_state", self.detect_state_callback, 1)

        self.pwm_pub =self.create_publisher(Int32, '/pwm', 1)
        
        # self.pwm_timer = self.create_timer(self.dt, self.pub_pwm)
        self.pwm_timer = self.create_timer(self.dt, self.pub_pwm)

        #waypoint check
        self.wp_check = False
        #waypoint finish index
        self.wp_clear = False
        self.docking_state = False
        
        #detect state
        self.detect_state = False

        self.des_heading_received = False
        self.des_spd_received = False
        self.heading_received = False
        self.spd_received = False
        self.wp_check_received = False
        self.err_heading_received = False
        self.err_x_received = False
        self.err_y_received = False
        self.err_heading_next_received = False

        self.safe_heading_received = False

        self.wp_idx = 0
        self.wp_set_x = []
        self.wp_set_y = []

    def wait_for_topics(self):
        self.timer = self.create_timer(1.0, self.check_topic_status)

    def check_topic_status(self):
        if not self.des_heading_received:
            self.get_logger().info('No topic des_heading_received')
        if not self.des_spd_received:
            self.get_logger().info('No topic des_spd_received')
        if not self.heading_received:
            self.get_logger().info('No topic heading_received')
        if not self.spd_received:
            self.get_logger().info('No topic spd_received')
        if self.des_heading_received and self.des_spd_received and self.heading_received and self.spd_received:
            self.get_logger().info('All topics received')
        else:
            self.get_logger().info('Waiting for topics to be published...')

    #deg transform
    def wp_clear_callback(self,msg):
        self.wp_clear_received = True
        self.wp_clear = msg.data


    def err_heading_callback(self, msg):
        self.err_heading_received = True
        self.err_heading = np.append(self.err_heading, np.rad2deg(msg.data))
        self.err_heading = self.err_heading[1:]

    def err_heading_next_callback(self, msg):
        self.err_heading_next_received = True
        temp = msg.data
        if temp > np.pi:
            temp -= np.pi * 2
        elif temp < -np.pi:
            temp += np.pi * 2
        self.err_heading_next = np.append(self.err_heading_next, np.rad2deg(temp))
        self.err_heading_next = self.err_heading_next[1:]

    def err_x_callback(self, msg):
        self.err_x_received = True
        self.err_x = np.append(self.err_x, msg.data)
        self.err_x = self.err_x[1:]

    def err_y_callback(self, msg):
        self.err_y_received = True
        self.err_y = np.append(self.err_y, msg.data)
        self.err_y = self.err_y[1:]

    def safe_heading_callback(self, msg):
        self.safe_heading_received = True
        self.safe_heading = np.array(msg.data)

    def wp_check_callback(self, msg):
        self.wp_check_received = True
        self.wp_check = msg.data

    def detect_state_callback(self,msg):
        self.detect_state = msg.data

    def docking_state_callback(self, msg):
        self.docking_state = msg.data

    def pub_pwm(self):
        def get_pwm_str(value):
            return str(int((value - 1000) / 10)).zfill(2)

        if self.docking_state:
            if self.detect_state:
                if self.wp_check:
                    # DP algorithm later....
                    # self.allocate_thrust()
                    self.cal_stop()
                else:
                    if len(self.safe_heading) != 0:
                        if abs(self.err_heading[-1]) < 90.0:
                            self.cal_thrust_pwm()  # operate using servo
                        else:
                            self.cal_self_rotate()  # stop and self_rotate until abs(err_heading) < 100
                            self.get_logger().info("error heading over 90, self_rotating...")
                    else:
                        self.cal_stop()
            else:
                self.cal_self_rotate()
        else:
            if self.wp_check:
                if self.wp_clear:
                    self.cal_stop()
                else:
                    self.cal_self_rotate()
            else:
                if len(self.safe_heading) != 0:
                    if abs(self.err_heading[-1]) < 90.0:
                        self.cal_thrust_pwm()  # operate using servo
                    else:
                        self.cal_self_rotate()  # stop and self_rotate until abs(err_heading) < 100
                        self.get_logger().info("error heading over 90, self_rotating...")
                else:
                    # self.cal_stop()
                    self.cal_thrust_pwm()

        print("this is thrust pwm str")
        print(self.thrust_pwm_L[-1], self.thrust_pwm_R[-1], self.rudder_pwm_L[-1], self.rudder_pwm_R[-1])

        # Calculate and publish PWM values
        thrust_pwm_str_L = get_pwm_str(self.thrust_pwm_L[-1])
        thrust_pwm_str_R = get_pwm_str(self.thrust_pwm_R[-1])
        servo_pwm_str_L = get_pwm_str(self.rudder_pwm_L[-1])
        servo_pwm_str_R = get_pwm_str(self.rudder_pwm_R[-1])

        pwm_value = int(thrust_pwm_str_L + thrust_pwm_str_R + servo_pwm_str_L + servo_pwm_str_R)

        pwm = Int32()
        pwm.data = pwm_value
        self.pwm_pub.publish(pwm)

        print("err heading:", self.err_heading[-1])
        print("pwm:", pwm_value)

        
    # functions that return something
    def cal_stop(self):
        self.thrust_pwm_L = np.append(self.thrust_pwm_L, 1500)
        self.thrust_pwm_L = self.thrust_pwm_L[1:]
        self.thrust_pwm_R = np.append(self.thrust_pwm_R, 1500)
        self.thrust_pwm_R = self.thrust_pwm_R[1:]

        self.rudder_pwm_L = np.append(self.rudder_pwm_L, 1500)
        self.rudder_pwm_L = self.rudder_pwm_L[1:]
        self.rudder_pwm_R = np.append(self.rudder_pwm_R, 1500)
        self.rudder_pwm_R = self.rudder_pwm_R[1:]

    def cal_thrust_pwm(self):
        if abs(self.err_heading[-1]) > 60:
            thrust_pwm_L = int(self.ref_thrust - np.sign(self.err_heading[-1]) * 40)
            thrust_pwm_R = int(self.ref_thrust + np.sign(self.err_heading[-1]) * 40)
        elif abs(self.err_heading[-1]) > 40:
            thrust_pwm_L = int(self.ref_thrust - np.sign(self.err_heading[-1]) * 30)
            thrust_pwm_R = int(self.ref_thrust + np.sign(self.err_heading[-1]) * 30)
        elif abs(self.err_heading[-1]) > 20:
            thrust_pwm_L = int(self.ref_thrust - np.sign(self.err_heading[-1]) * 20)
            thrust_pwm_R = int(self.ref_thrust + np.sign(self.err_heading[-1]) * 20)
        else:
            thrust_pwm_L = int(self.ref_thrust)
            thrust_pwm_R = int(self.ref_thrust)

        if thrust_pwm_L > self.thrust_pwm_lim[1]:
            thrust_pwm_L = self.thrust_pwm_lim[1]
        elif thrust_pwm_L < self.thrust_pwm_lim[0]:
            thrust_pwm_L = self.thrust_pwm_lim[0]
        
        if thrust_pwm_R > self.thrust_pwm_lim[1]:
            thrust_pwm_R = self.thrust_pwm_lim[1]
        elif thrust_pwm_R < self.thrust_pwm_lim[0]:
            thrust_pwm_R = self.thrust_pwm_lim[0]

        self.thrust_pwm_L = np.append(self.thrust_pwm_L, thrust_pwm_L)
        self.thrust_pwm_L = self.thrust_pwm_L[1:]
        self.thrust_pwm_R = np.append(self.thrust_pwm_R, thrust_pwm_R)
        self.thrust_pwm_R = self.thrust_pwm_R[1:]

        # fixed thrust pwm
        # thrust_pwm_L = self.ref_thrust
        # thrust_pwm_R = self.ref_thrust
            
        # self.thrust_pwm_L = np.append(self.thrust_pwm_L, thrust_pwm_L)
        # self.thrust_pwm_L = self.thrust_pwm_L[1:]
        # self.thrust_pwm_R = np.append(self.thrust_pwm_R, thrust_pwm_R)
        # self.thrust_pwm_R = self.thrust_pwm_R[1:]

        # self.rudder_pwm_L = np.append(self.rudder_pwm_L, 1500)
        # self.rudder_pwm_L = self.rudder_pwm_L[1:]
        # self.rudder_pwm_R = np.append(self.rudder_pwm_R, 1500)
        # self.rudder_pwm_R = self.rudder_pwm_R[1:]

        # rudder angle
        # Rollback

        # rudder= (self.Kp_rudder * self.err_heading[-1]) + \
        #            self.Kd_rudder * (self.err_heading[-1] - self.err_heading[-2]) / self.dt
        
        # rudder = (self.Kp_rudder*self.err_heading[-1])
        # rudder = rudder + self.rudder_rate * np.sign(rudder_cmd - self.rudder_pwm_L[-1])

        # Simple control
        if abs(self.err_heading[-1]) > 70:
            rudder = np.sign(self.err_heading[-1]) * 60
        elif abs(self.err_heading[-1]) > 40:
            rudder = np.sign(self.err_heading[-1]) * 30
        elif abs(self.err_heading[-1]) > 20:
            rudder = np.sign(self.err_heading[-1]) * 10
        elif abs(self.err_heading[-1]) > 10:
            rudder = np.sign(self.err_heading[-1]) * 5
        else: # abs(self.err_heading[-1]) < 10
            rudder = 0

        if rudder > self.rudder_lim[1]:
            rudder = self.rudder_lim[1]
        elif rudder < self.rudder_lim[0]:
            rudder = self.rudder_lim[0]
                    
        self.r_pwm_cmd = int(rudder/135*(1000)+1500) ###

        # fix rudder rate
        # self.r_pwm_cmd = int(self.rudder_pwm_L[-1] + np.sign(self.err_heading[-1]) * self.rudder_rate * self.dt)
        
        if self.r_pwm_cmd > self.rudder_pwm_lim[1]:
            self.r_pwm_cmd = self.rudder_pwm_lim[1]
        elif self.r_pwm_cmd < self.rudder_pwm_lim[0]:
            self.r_pwm_cmd = self.rudder_pwm_lim[0]
    
        # self.rudder_pwm_L = np.append(self.rudder_pwm_L, self.r_pwm)
        self.rudder_pwm_L = np.append(self.rudder_pwm_L, self.r_pwm_cmd)
        self.rudder_pwm_L = self.rudder_pwm_L[1:]
        # self.rudder_pwm_R = np.append(self.rudder_pwm_R, self.r_pwm)
        self.rudder_pwm_R = np.append(self.rudder_pwm_R, self.r_pwm_cmd)
        self.rudder_pwm_R = self.rudder_pwm_R[1:]
    
    def cal_self_rotate(self):
        thrust_pwm_L = 1500 - int((self.err_heading[-1] * self.Kp_self_rot)) 
        thrust_pwm_R = 1500 + int((self.err_heading[-1] * self.Kp_self_rot)) 

        if thrust_pwm_L > self.thrust_pwm_lim[1]:
            thrust_pwm_L = self.thrust_pwm_lim[1]
        elif thrust_pwm_L < self.thrust_pwm_lim[0]:
            thrust_pwm_L = self.thrust_pwm_lim[0]
        
        if thrust_pwm_R > self.thrust_pwm_lim[1]:
            thrust_pwm_R = self.thrust_pwm_lim[1]
        elif thrust_pwm_R < self.thrust_pwm_lim[0]:
            thrust_pwm_R = self.thrust_pwm_lim[0]

        self.thrust_pwm_L = np.append(self.thrust_pwm_L, thrust_pwm_L)
        self.thrust_pwm_L = self.thrust_pwm_L[1:]
        self.thrust_pwm_R = np.append(self.thrust_pwm_R, thrust_pwm_R)
        self.thrust_pwm_R = self.thrust_pwm_R[1:]

        self.rudder_pwm_L = np.append(self.rudder_pwm_L, 1500)
        self.rudder_pwm_L = self.rudder_pwm_L[1:]
        self.rudder_pwm_R = np.append(self.rudder_pwm_R, 1500)
        self.rudder_pwm_R = self.rudder_pwm_R[1:]

    def allocate_thrust(self):
        # Calculate Required Forces 
        Xreq = self.err_x[-1] * self.Kp_DP_x + (self.err_x[-1] - self.err_x[-2]) * self.Kd_DP_x
        Yreq = self.err_y[-1] * self.Kp_DP_y + (self.err_y[-1] - self.err_y[-2]) * self.Kd_DP_y
        Nreq = self.err_heading_next[-1] * self.Kp_DP_hdg + (self.err_heading_next[-1] - self.err_heading_next[-2]) * self.Kd_DP_hdg

        # Allocate thrust
        Config_Mat = np.array([[2, 0, 0, 0, 1, 0, -self.ly], \
                               [0, 2, 0, 0, 1, 0, self.ly], \
                               [0, 0, 2, 0, 0, 1, -self.lx], \
                               [0, 0, 0, 2, 0, 1, -self.lx], \
                               [1, 1, 0, 0, 0, 0, 0], \
                               [0, 0, 1, 1, 0, 0, 0], \
                               [-self.ly, self.ly, -self.lx, -self.lx, 0, 0, 0]])

        Req_Force_Vec = np.array([0, 0, 0, 0, Xreq, Yreq, Nreq]).transpose()
        temp = np.dot(np.linalg.inv(Config_Mat), Req_Force_Vec)
        Thrust_L = np.linalg.norm([temp[0], temp[1]]) * np.sign(temp[0])
        Thrust_R = np.linalg.norm([temp[2], temp[3]]) * np.sign(temp[1])
        Servo_L = np.rad2deg(np.arctan2(temp[1], temp[0])) 
        Servo_R = np.rad2deg(np.arctan2(temp[3], temp[2]))
        # Saturation
        if Servo_L < self.rudder_lim[0] : 
            Servo_L = self.rudder_lim[0]
        elif Servo_L > self.rudder_lim[1]:
            Servo_L = self.rudder_lim[1]
        
        if Servo_R < self.rudder_lim[0]:
            Servo_R = self.rudder_lim[0]
        elif Servo_R > self.rudder_lim[1]:
            Servo_R = self.rudder_lim[1]

        # Map pwm 
        thrust_pwm_L = 1500 + int(Thrust_L) # TODO: define the sign!
        thrust_pwm_R = 1500 + int(Thrust_R) # TODO: define the sign!

        if thrust_pwm_L > self.thrust_pwm_lim[1]:
            thrust_pwm_L = self.thrust_pwm_lim[1]
        elif thrust_pwm_L < self.thrust_pwm_lim[0]:
            thrust_pwm_L = self.thrust_pwm_lim[0]
        
        if thrust_pwm_R > self.thrust_pwm_lim[1]:
            thrust_pwm_R = self.thrust_pwm_lim[1]
        elif thrust_pwm_R < self.thrust_pwm_lim[0]:
            thrust_pwm_R = self.thrust_pwm_lim[0]

        self.thrust_pwm_L = np.append(self.thrust_pwm_L, thrust_pwm_L)
        self.thrust_pwm_L = self.thrust_pwm_L[1:]
        self.thrust_pwm_R = np.append(self.thrust_pwm_R, thrust_pwm_R)
        self.thrust_pwm_R = self.thrust_pwm_R[1:]

        rudder_pwm_L = 1500 -int(Servo_L/135*(1000))
        rudder_pwm_R = 1500 - int(Servo_R/135*(1000))
        self.rudder_pwm_L = np.append(self.rudder_pwm_L, rudder_pwm_L)
        self.rudder_pwm_L = self.rudder_pwm_L[1:]
        self.rudder_pwm_R = np.append(self.rudder_pwm_R, rudder_pwm_R)
        self.rudder_pwm_R = self.rudder_pwm_R[1:]

def main(args=None):
    rclpy.init(args=args)
    pwm_converter = PWMConverter()
    # pwm_converter.wait_for_topics()
    rclpy.spin(pwm_converter)
    pwm_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
