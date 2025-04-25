import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3
import math

class ZedF9Publisher(Node):
    def __init__(self):
        super().__init__('zed_f9_publisher')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Vector3, '/zed_f9', 10)
        self.get_logger().info('ZED-F9 publisher node started.')

    def listener_callback(self, msg: NavSatFix):
        lat = msg.latitude
        lon = msg.longitude

        # Use horizontal variance to compute standard deviation (accuracy)
        # This assumes x/y variances are equal, which is typical
        try:
            horizontal_variance = msg.position_covariance[0]
            accuracy = math.sqrt(horizontal_variance)
        except (IndexError, ValueError):
            accuracy = float('nan')  # if anything goes wrong

        # Prepare and publish Vector3 message
        out_msg = Vector3()
        out_msg.x = lat
        out_msg.y = lon
        out_msg.z = accuracy
        self.publisher_.publish(out_msg)

        self.get_logger().info(
            f'Published -> Lat: {lat}, Lon: {lon}, Acc: {accuracy:.2f} m'
        )

def main(args=None):
    rclpy.init(args=args)
    node = ZedF9Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
