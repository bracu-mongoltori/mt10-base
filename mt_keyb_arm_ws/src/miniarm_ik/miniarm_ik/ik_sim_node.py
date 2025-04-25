import rclpy
from rclpy.node import Node
from miniarm_interfaces.msg import IKTarget, IKSolve
import IK_Simulator.ik_sim as sim


class simNode(Node):
    def __init__(self):
        super().__init__('ik_sim_node')
        self.pub_target = self.create_publisher(IKTarget, 'mini_arm/ik_target', 10)
        self.sub_solve = self.create_subscription(IKSolve, 'mini_arm/arm_angle', self.ik_solve_callback, 10)
        self.get_logger().info('IK Simulator has been started.')

        self.create_timer(1/60.0, self.timer_callback)
    
    def timer_callback(self):
        msg = IKTarget()
        msg.target_x = sim.targetPx*1.0
        msg.target_y = sim.targetPy*1.0
        self.pub_target.publish(msg)
        sim.rerun()

    def ik_solve_callback(self, msg):
        # self.get_logger().info(f"Received angles: {msg.angle_1}, {msg.angle_2}")
        sim.setAngles(msg.angle_1, msg.angle_2, msg.angle_full_1, msg.angle_full_2)
        pass
def main(args=None):
    rclpy.init(args=args)
    node = simNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()