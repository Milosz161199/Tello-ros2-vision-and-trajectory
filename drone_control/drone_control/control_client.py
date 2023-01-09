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
        self._action_client = ActionClient(
            self,
            Detect,
            'Detect')
        
        # self._optitrack_subscriber = self.create_subscription(
        #     String,
        #     'optitrack_topic',
        #     self.optitrack_callback,
        #     10)
        
        self._republisher_subscriber = self.create_subscription(
            Odometry,
            '/repeater/tello_1/pose/info',
            self.republisher_callback,
            10)
        
        self._call_service = self.create_client(
            TelloAction,
            "/tello_action",
        )
        
        self._drone_cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
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
        
        # Just for tests
        # self.x_arr = [1, 0, 2, 3]
        # self.y_arr = [0, 0, 0, 0]
        # self.z_arr = [1, 1, 1, 1]
        # self.y_arr = [0, 1, 2, 3]
        # self.z_arr = [1, 1, 2, 1]
        self.number_of_points = 4
        

    def optitrack_callback(self, msg):
        self.curr_measured_x = msg.position.x
        self.curr_measured_y = msg.position.y
        self.curr_measured_z = msg.position.z
    
    def republisher_callback(self, msg):
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
        self.x_arr = result.x
        self.y_arr = result.y
        self.z_arr = result.z
        print(self.x_arr)
        # self.get_logger().info('Result: f'.format(result.x))
        rclpy.shutdown()
        
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