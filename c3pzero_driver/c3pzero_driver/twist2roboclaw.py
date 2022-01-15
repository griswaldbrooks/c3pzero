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

from . import roboclaw_3

from geometry_msgs.msg import Twist


class RoboclawTwistSubscriber(Node):

    def __init__(self):
        super().__init__('roboclaw_twist_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.rc = roboclaw_3.Roboclaw("/dev/ttyACM0",115200)

        if not self.rc.Open():
            print("failed to open port")
        self.rc_address = 0x80

        version = self.rc.ReadVersion(self.rc_address)
        if version[0]==False:
            self.get_logger().error('Retriving the Roboclaw version failed')
        else:
            self.get_logger().info('Roboclaw version: %s' % repr(version[1]))
        
        self.get_logger().info('Init complete')

    def listener_callback(self, msg):
        # self.get_logger().info('X_vel: %f, Z_rot: %f' % msg.linear.x, msg.angular.z)

        right_wheel = 0.4*msg.linear.x + (msg.angular.z * .54)/2
        left_wheel = 0.4*msg.linear.x - (msg.angular.z * .54)/2
        self.rc.SpeedM1(self.rc_address, int(18000 * right_wheel))
        self.rc.SpeedM2(self.rc_address, int(18000 * left_wheel))

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = RoboclawTwistSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
