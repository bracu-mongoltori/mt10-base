import rclpy
from rclpy.node import Node
from miniarm_interfaces.msg import IKTarget
from std_msgs.msg import String

import time

class UserInputTargetNode(Node):
    def __init__(self):
        super().__init__('user_input_target')
        self.publisher_ = self.create_publisher(IKTarget, 'mini_arm/ik_target', 10)
        self.pubCmd = self.create_publisher(String, 'mini_arm/cmd', 10)
        self.get_logger().info('UserInputTargetNode has been started.')

        self.sequence = [[0.0, 30.0], [10.0, 30.0], [0.0, 20.0], [-10.0, 30.0], [0.0, 30.0]]

        while(True):
            self.get_logger().info(f"\n\nInput the target position:")
            vx =input("Enter x (cm): ")
            if(vx == "extend"):
                msg = String()
                msg.data = "extend"
                self.pubCmd.publish(msg)
                self.get_logger().info(f"Sent Command: {msg.data}")
            elif(vx == "retract"):
                msg = String()
                msg.data = "retract"
                self.pubCmd.publish(msg)
                self.get_logger().info(f"Sent Command: {msg.data}")
            elif(vx == "hold"):
                msg = String()
                msg.data = "hold"
                self.pubCmd.publish(msg)
                self.get_logger().info(f"Sent Command: {msg.data}")
            elif(vx == "sequence"):
                for i in self.sequence:
                    msg = IKTarget()
                    msg.target_x = i[0]
                    msg.target_y = i[1]
                    msg.click = True
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Sent target: {msg.target_x}, {msg.target_y}")
                    time.sleep(4)
            else:
                x = float(vx)
                y = float(input("Enter y (cm): "))

                msg = IKTarget()
                msg.target_x = x
                msg.target_y = y
                msg.click = True
                self.publisher_.publish(msg)
                self.get_logger().info(f"Sent target: {msg.target_x}, {msg.target_y}")

def main(args=None):
    rclpy.init(args=args)
    node = UserInputTargetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()