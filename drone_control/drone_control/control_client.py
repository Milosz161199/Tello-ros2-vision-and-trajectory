import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose

from tello_msgs.msg import FlightData, TelloResponse
from tello_msgs.srv import TelloAction

import time

from action_detect.action import Detect


class ControlActionClient(Node):

    def __init__(self):
        super().__init__('control_action_client')
        # Control drone with service call or topic cmd_vel
        # topic - preferred
        self.control_with_srv = False
        # Operate in simulation or in laboratory
        self.operate_in_sim = True
        
        if self.operate_in_sim:
            self.rc_srv_name = "/drone1/tello_action"
            self.topic_cmd_name = "/drone1/cmd_vel"
        else:
            self.rc_srv_name = "/tello_action"
            self.topic_cmd_name = "/cmd_vel"
        
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
        self.msg_twist = Twist()
        
        self.current_time = 0
        self.last_time = 0
        
        self.kp_x = 1.68
        self.ki_x = 0.86
        self.kd_x = 1.68
        self.kp_y = 3
        self.ki_y = 0.1
        self.kd_y = 2
        self.kp_z = 16
        self.ki_z = 2.02
        self.kd_z = 10.8
        
        self.x_error = 0
        self.sum_x_error = 0
        self.derr_x_error = 0
        self.last_x_error = 0
        
        self.curr_measured_x = 0
        self.curr_measured_y = 0
        self.curr_measured_z = 0
        self.curr_x = 0
        self.curr_y = 0
        self.curr_z = 0
        
        self.x_arr = list()
        self.y_arr = list()
        self.z_arr = list()
        self.number_of_points = 0
        self.curr_point = 0
        self.start_control = False
        
        # Create client for paper detect action
        self._action_client = ActionClient(
            self,
            Detect,
            'Detect')
        
        if self.operate_in_sim:
            # Subscribe g2rr topic
            self._sub_republisher = self.create_subscription(
                Odometry,
                '/repeater/tello_1/pose/info',
                self.republisher_callback,
                10)
        else:
            # Subscribe optitrack republisher topic
            self._sub_optitrack = self.create_subscription(
                String,
                'optitrack_topic',
                self.optitrack_callback,
                10)
        
        # Create client for rc service
        self._client_rc = self.create_client(
            TelloAction,
            self.rc_srv_name)
        
        # Wait for service to connect
        while not self._client_rc.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Rc service not available - waiting for service")
            
        # Subscribe to tello response topic
        self.response_rc = self.create_subscription(
            TelloResponse,
            "tello_response",
            self.response_callback,
            10)
        
        self.get_logger().info("Control node has been started")
        
        # Take off drone
        self._client_rc.call_async(TelloAction.Request(cmd="takeoff"))

        self.get_logger().info("Drone took off")
        
        # Creat publisher for cmd_vel topic
        self._pub_cmd_vel = self.create_publisher(Twist, self.topic_cmd_name, 10)
        
        # Timer for drone control loop
        self.timer_period = 1/10  # 10Hz
        self._timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Timer for processing optitrack / g2rr data
        self.control_timer_period = 1 / 250  # 250Hz
        self._control_timer = self.create_timer(self.control_timer_period, self.control_timer_callback)
        
    def response_callback(self):
        pass
    
    def optitrack_callback(self, msg):
        # Update drone position
        self.curr_measured_x = msg.position.x
        self.curr_measured_y = msg.position.y
        self.curr_measured_z = msg.position.z
    
    def republisher_callback(self, msg):
        # Update drone position
        self.curr_measured_x = msg.pose.pose.position.x
        self.curr_measured_y = msg.pose.pose.position.y
        self.curr_measured_z = msg.pose.pose.position.z
    
    def send_goal(self, order):
        goal_msg = Detect.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        print(f"Number of points: {result.number_of_points}")
        self.number_of_points = result.number_of_points
        self.x_arr = result.x
        self.y_arr = result.y
        self.z_arr = result.z
        print(len(self.x_arr))
        print(len(self.y_arr))
        print(len(self.z_arr))
        time.sleep(5)
        self.start_control = True
        
    def timer_callback(self):
        if self.new_cmd:
            self.send_drone_cmd()
            self.new_cmd = False
        else:
            self.twist_cmd = Twist()
            self.send_drone_cmd()
            
    def control_timer_callback(self):
        if self.start_control:
            self.reached_x = False
            self.reached_y = False
            self.reached_z = False
            if self.curr_point < self.number_of_points - 1:
                print('point:', self.curr_point, '/', self.number_of_points)
                self.new_cmd = True
                self.reached_x = self.set_x_velocity(self.x_arr[self.curr_point])
                self.reached_y = self.set_y_velocity(self.y_arr[self.curr_point])
                self.reached_z = self.set_z_velocity(self.z_arr[self.curr_point])
            else:
                print('final point')
                self.twist_cmd = Twist()
                self.new_cmd = True
            
            if self.reached_x and self.reached_y and self.reached_z:
                self.curr_point += 1
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
        if self.control_with_srv:
            multi = 10
            srv_req = TelloAction.Request()
            srv_req.cmd = f"rc {int(self.twist_cmd.linear.x*multi)} {int(self.twist_cmd.linear.y*multi)} {int(self.twist_cmd.linear.z*multi)} {int(-1*self.twist_cmd.angular.z*30 )}"
            self._client_rc.call_async(srv_req)
            self.get_logger().info(f"{srv_req.cmd}")
        else:
            self._pub_cmd_vel.publish(self.twist_cmd)
            self.get_logger().debug(f"{self.twist_cmd}")
    
    def calc_pid_x(self):
        self.current_time = time.time() - self.last_time
        
        self.x_error = self.x_arr[0] - self.curr_x
        self.sum_x_error += self.x_error
        self.derr_x_error = (self.x_error - self.last_x_error) / self.current_time
        x_pid = self.kp_x * self.x_error + self.ki_x * self.sum_x_error + self.kd_x * self.derr_x_error
        self.last_x_error = self.x_error
        
        self.msg_twist.linear.x = x_pid
        
        self._drone_cmd_publisher.publish(self.msg_twist)

def main(args=None):
    rclpy.init(args=args)

    action_client = ControlActionClient()

    action_client.send_goal(0)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()