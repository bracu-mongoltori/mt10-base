import serial
import time

import rclpy
from rclpy.node import Node
from miniarm_interfaces.msg import IKSolve
from std_msgs.msg import String

class MiniarmSer(Node):
    def __init__(self):
        super().__init__('miniarm_ser')
        self.pub_read = self.create_publisher(String, 'mini_arm/ser_read', 10)
        self.subscription_ = self.create_subscription(IKSolve, 'mini_arm/arm_angle', self.ik_solve_callback, 10)
        self.subCmd = self.create_subscription(String, 'mini_arm/cmd', self.cmd_callback, 10)
        self.serCheck = self.create_timer(0.05, self.read_ser)
        self.get_logger().info('Miniarm Seral Node has been started.')
        self.ser = None

        for i in range(10):
            try:
                self.ser = serial.Serial(f'/dev/ttyACM{i}', 9600)
                self.get_logger().info(f"Connected to /dev/ttyACM{i}")
                break
            except:
                self.ser = None
                self.get_logger().warn(f"Failed to connect to /dev/ttyACM{i}")
                time.sleep(0.05)
                pass
        if(self.ser is None):
            for i in range(10):
                try:
                    self.ser = serial.Serial(f'/dev/ttyUSB{i}', 9600)
                    self.get_logger().info(f"Connected to /dev/ttyUSB{i}")
                    break
                except:
                    self.get_logger().warn(f"Failed to connect to /dev/ttyUSB{i}")
                    time.sleep(0.05)
                    pass
    
    def cmd_callback(self, msg):
        if(self.ser is None):
            self.get_logger().error("Not connected.")
            return
        
        if(msg.data == "extend"):
            self.ser.write(f"extend\n".encode())
            self.get_logger().info(f"Sent Command: {msg.data}")
        elif(msg.data == "retract"):
            self.ser.write(f"retract\n".encode())
            self.get_logger().info(f"Sent Command: {msg.data}")
        elif(msg.data == "hold"):
            self.ser.write(f"hold\n".encode())
            self.get_logger().info(f"Sent Command: {msg.data}")
        else:
            self.get_logger().warn(f"Unknown Command: {msg.data}")
    
    def ik_solve_callback(self, msg):
        if(self.ser is None):
            self.get_logger().error("Not connected.")
            return
        
        clickVal = 1 if msg.click else 0
        self.ser.write(f"{msg.angle_1} {msg.angle_2},{clickVal}\n".encode())
        self.get_logger().info(f"Sent angles: {msg.angle_1}, {msg.angle_2}")
    
    def read_ser(self):
        if(self.ser is None):
            return
        
        while(self.ser.in_waiting > 0):
            line = self.ser.readline().decode().strip()
            self.get_logger().info(f"Received: {line}")

            if(line == "reached"):
                self.publish_read("reached")

    def publish_read(self, cmd:str):
        msg = String()
        msg.data = cmd
        self.pub_read.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = MiniarmSer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()