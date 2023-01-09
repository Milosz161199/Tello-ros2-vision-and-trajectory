# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

from tello_msgs.msg import FlightData, TelloResponse
from tello_msgs.srv import TelloAction

import socket
import json


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('test_node')
        self.control_method = False
        self.rc_srv_name = "/drone1/tello_action"
        
        self.vel_lin = 0.1
        self.vel_ang = 0.1
        self.max_vel_lin = 20.0
        self.max_vel_ang = 40.0
        self.lin_offset = 0.1
        self.ang_offset = 0.1
        
        self.curr_measured_x = 0
        self.curr_measured_y = 0
        self.curr_measured_z = 0
        self.curr_x = 0
        self.curr_y = 0
        self.curr_z = 0
        self.dest_x = 0
        self.dest_y = 0
        self.dest_z = 0
        
        self.reached_x = False
        self.reached_y = False
        self.reached_z = False
        
        self.new_cmd = False
        self.twist_cmd = Twist()
        
        self.sub_republisher = self.create_subscription(
            Odometry,
            '/repeater/tello_1/pose/info',
            self.republisher_callback,
            10)
        
        self.client_rc = self.create_client(
            TelloAction,
            self.rc_srv_name)
        
        while not self.client_rc.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Rc service not available - waiting for service")
            
        self.response_rc = self.create_subscription(
            TelloResponse,
            "tello_response",
            self.response_callback,
            10)
        
        self.pub_cmd_vel = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        
        self.get_logger().info("Control node has been started")
        
        self.client_rc.call_async(TelloAction.Request(cmd="takeoff"))

        self.get_logger().info("Drone took off")
        
        self.timer_period = 1/10  # 10Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.control_timer_period = 1 / 250  # 250Hz
        self.control_timer = self.create_timer(self.control_timer_period, self.control_timer_callback)
        
        
        
        # Just for tests
        self.x_arr = [1, 2, 0, 1]
        self.y_arr = [0, 1, 0, 0]
        self.z_arr = [1, 1, 1, 1]
        # self.y_arr = [0, 1, 2, 3]
        # self.z_arr = [1, 1, 2, 1]
        self.number_of_points = 4
        self.point = 0
        
        # self.test_flight()
        
    def response_callback(self):
        pass
    
    def republisher_callback(self, msg):
        self.curr_measured_x = msg.pose.pose.position.x
        self.curr_measured_y = msg.pose.pose.position.y
        self.curr_measured_z = msg.pose.pose.position.z
        

    def timer_callback(self):
        if self.new_cmd:
            print(":send")
            self.send_drone_cmd()
            self.new_cmd = False
        else:
            self.twist_cmd = Twist()
            self.send_drone_cmd()
            
    def control_timer_callback(self):
        self.reached_x = False
        self.reached_y = False
        self.reached_z = False
        if self.point < 4:
            self.new_cmd = True
            self.reached_x = self.set_x_velocity(self.x_arr[self.point])
            self.reached_y = self.set_y_velocity(self.y_arr[self.point])
            self.reached_z = self.set_z_velocity(self.z_arr[self.point])
        else:
            self.new_cmd = False
        
        
        
        if self.reached_x and self.reached_y and self.reached_z:
            self.point += 1
            self.twist_cmd = Twist()
            
            
    def set_x_velocity(self, dest_x):
        self.curr_x = self.curr_measured_x
        if (dest_x - self.lin_offset) < self.curr_x < (dest_x + self.lin_offset):
            self.twist_cmd.linear.x = 0.0
            return True
        elif (dest_x - self.lin_offset) > self.curr_x:
            self.twist_cmd.linear.x = self.vel_lin
            return False
        elif (dest_x + self.lin_offset) < self.curr_x:
            self.twist_cmd.linear.x = -self.vel_lin
            return False
            
    def set_y_velocity(self, dest_y):
        self.curr_y = self.curr_measured_y
        if self.curr_y - self.lin_offset < dest_y < self.curr_y + self.lin_offset:
            self.twist_cmd.linear.y = 0.0
            return True
        elif self.curr_y - self.lin_offset > dest_y:
            self.twist_cmd.linear.y = -self.vel_lin
            return False
        elif self.curr_y + self.lin_offset < dest_y:
            self.twist_cmd.linear.y = self.vel_lin
            return False
            
    def set_z_velocity(self, dest_z):
        self.curr_z = self.curr_measured_z
        if self.curr_z - self.lin_offset < dest_z < self.curr_z + self.lin_offset:
            self.twist_cmd.linear.z = 0.0
            return True
        elif self.curr_z - self.lin_offset > dest_z:
            self.twist_cmd.linear.z = -self.vel_lin
            return False
        elif self.curr_z + self.lin_offset < dest_z:
            self.twist_cmd.linear.z = self.vel_lin
            return False
        
    def send_drone_cmd(self):
        if self.control_method == True:
            multi = 10
            srv_req = TelloAction.Request()
            srv_req.cmd = f"rc {int(self.twist_cmd.linear.x*multi)} {int(self.twist_cmd.linear.y*multi)} {int(self.twist_cmd.linear.z*multi)} {int(-1*self.twist_cmd.angular.z*30 )}"
            self.client_rc.call_async(srv_req)
            self.get_logger().info(f"{srv_req.cmd}")
        else:
            self.pub_cmd_vel.publish(self.twist_cmd)
            self.get_logger().debug(f"{self.twist_cmd}")
        
    def test_flight(self):
        # self.set_x_velocity(2)
        # self.new_cmd = True
        # while not self.reached_x:
        #     self.set_x_velocity(2)
        #     self.new_cmd = True
        #     print(self.twist_cmd.linear.x)
            
        # self.get_logger().info("Reached")
                
        if len(self.x_arr) == len(self.y_arr) == len(self.z_arr):
            for (x, y, z) in zip(self.x_arr, self.y_arr, self.z_arr):
                self.reached_x = False
                self.reached_y = False
                self.reached_z = False
                self.new_cmd = True
                while not self.reached_x:
                    self.set_x_velocity(x)
                    
                self.set_y_velocity(y)
                self.set_z_velocity(z)
            
              
                            
def main(args=None):
    rclpy.init(args=args)

    test_node = MinimalPublisher()

    rclpy.spin(test_node)

    test_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
