import rclpy
from rclpy.node import Node
from miniarm_interfaces.msg import IKTarget, IKSolve
from pkgs.ikSolver import calcAngles

class KeybIKNode(Node):
    def __init__(self):
        super().__init__('keyb_ik')
        self.subscription_ = self.create_subscription(IKTarget, 'mini_arm/ik_target', self.ik_target_callback, 10)
        self.publisher_ = self.create_publisher(IKSolve, 'mini_arm/arm_angle', 10)
        self.get_logger().info('KeybIKNode has been started.')

        self.len1:float = 18.5
        self.len2:float = 19.0

        self.LIMIT1:int = [0, 180]
        self.LIMIT2:int = [0, 165]
        self.a1Offs:int = 0
        self.a2Offs:int = 0

        warn1val = 0.02 * (self.LIMIT1[1] - self.LIMIT1[0])
        warn2val = 0.02 * (self.LIMIT1[1] - self.LIMIT1[0])
        self.warn1 = [self.LIMIT1[0] + warn1val, self.LIMIT1[1] - warn1val]
        self.warn2 = [self.LIMIT2[0] + warn2val, self.LIMIT2[1] - warn2val]

    def ik_target_callback(self, msg):
        ret = calcAngles((msg.target_x, msg.target_y), (self.len1, self.len2), (self.LIMIT1, self.LIMIT2), (self.a1Offs, self.a2Offs))
        
        self.get_logger().info(f"""
------------------------------------------
Received target: {msg.target_x}, {msg.target_y}

The base will move to {ret['ang1']:.2f} degrees.
The joint will move to {ret['ang2']:.2f} degrees.

        """)
        if(ret["dist"] > 0.1):
            self.get_logger().error(f"""
The target is out of reach.
The arm will reach the position of (x: {ret['head_position'][0]:.2f}cm, y:{ret['head_position'][1]:.2f}cm) with a distance of {ret['dist']:.2f} cm.

""")

        if(ret["ang1"] < self.warn1[0]):
            self.get_logger().warn("The base angle is too close to it's lower (right) limit.")
        elif(ret["ang1"] > self.warn1[1]):
            self.get_logger().warn("The base angle is too close to it's upper (left) limit.")
        
        if(ret["ang2"] < self.warn2[0]):
            self.get_logger().warn("The joint angle is too close to it's lower (right) limit.")
        elif(ret["ang2"] > self.warn2[1]):
            self.get_logger().warn("The joint angle is too close to it's upper (left) limit.")
        
        _msg = IKSolve()
        _msg.target = IKTarget()
        _msg.target.target_x = msg.target_x
        _msg.target.target_y = msg.target_y
        _msg.angle_full_1 = ret["ang1_full"]
        _msg.angle_full_2 = ret["ang2_full"]
        _msg.angle_1 = ret["ang1"]
        _msg.angle_2 = ret["ang2"]
        _msg.click = msg.click
        _msg.reached = IKTarget()
        _msg.reached.target_x = ret["head_position"][0]
        _msg.reached.target_y = ret["head_position"][1]
        _msg.dst = ret["dist"]
        self.publisher_.publish(_msg)




def main(args=None):
    rclpy.init(args=args)
    node = KeybIKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()