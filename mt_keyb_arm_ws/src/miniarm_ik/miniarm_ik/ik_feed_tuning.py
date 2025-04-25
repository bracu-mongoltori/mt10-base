import rclpy
from rclpy.node import Node
import cv2, numpy as np
import pkgs.params as params
import pkgs.keyPos_detection as kP
import pkgs.keyLayout as kl
from miniarm_interfaces.msg import IKTarget, IKSolve

import time

import threading

class IKFeedTuning(Node):
    def __init__(self):
        super().__init__('ik_feed_tuning')
        self.cap = cv2.VideoCapture(2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
        self.points = params.detected_corners
        self.wPoints = params.screen_corners
        self.trackers_initialized = False
        self.do_tracking = True
        self.publisher_ = self.create_publisher(IKTarget, 'mini_arm/ik_target', 10)
        self.subsc = self.create_subscription(IKSolve, 'mini_arm/arm_angle', self.ik_solve_callback, 10)
        
        cv2.namedWindow('Camera Feed')
        cv2.setMouseCallback('Camera Feed', self.on_mouse_click)

        self.lk_params = dict(
            winSize=(155, 155),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )

        self.target_key = 'e'

        self.headPos = None

        self.old_gray = None
        self.old_points = None

        self.sequencing = False
        self.timer = time.time()
        self.typing = "khalil sir"
        self.wInd = 0

        self.thrd = threading.Thread(target=self.run)
        self.thrd.daemon = True
        self.thrd.start()

    def ik_solve_callback(self, msg):
        self.headPos = [msg.reached.target_x, msg.reached.target_y]

    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.get_logger().info('--------------==--------------')
            if(len(self.points) < 4):
                self.get_logger().info(f'Cursor Position: ({x}, {y})')
                self.points.append((x, y))
                self.trackers_initialized = False
            else:
                wPos = kP.getPfrS((x,y))
                print(f"Screen Coordinates: {(x,y)}")
                print(f"Worls Coordinates: {wPos}")
        if event == cv2.EVENT_RBUTTONDOWN:
            if(len(self.wPoints) < 4):
                self.wPoints.append([x, y])
    
    def initialize_trackers(self, frame):
        points = np.array(self.points, dtype=np.float32).reshape(-1, 1, 2)
        return points
    def getPos(self):
        kP.setKeyboardCornersOnScreen(self.points)
        kps = kP.getScrPos(self.target_key)
        kpw = kP.getPfrS(kps)
        return kps, kpw

    def moveToKey(self):
        if(len(self.points) != 4 or len(self.wPoints) != 4):
            self.get_logger().error('Not enough points to calculate homography')
            return
        kps, kpw = self.getPos()
        if(kpw is not None):
            msg = IKTarget()
            msg.target_x = kpw[0]
            msg.target_y = kpw[1]
            msg.click = True
            self.publisher_.publish(msg)
        
    def run(self):
        print("Hello")
        while rclpy.ok():
            print("Ahoy")
            ret, frame = self.cap.read()
            _frm = frame.copy()
            if not ret:
                self.get_logger().error('Failed to capture image')
                break
            print("Hello")
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            if(len(self.points)==4):
                kP.setKeyboardCornersOnScreen(self.points)

                if(self.sequencing and time.time() - self.timer > 2.5):
                    self.target_key = self.typing[self.wInd]
                    self.moveToKey()
                    self.wInd = self.wInd + 1
                    self.timer = time.time()
                    if(self.wInd >= len(self.typing)):
                        self.wInd = 0
                        self.sequencing = False
                        self.get_logger().info('Typing sequence completed')

            if not self.trackers_initialized and self.points:
                self.old_points = self.initialize_trackers(frame)
                self.old_gray = gray_frame.copy()
                self.trackers_initialized = True
            
            if self.trackers_initialized and self.old_points is not None and self.do_tracking:
                new_points, status, _ = cv2.calcOpticalFlowPyrLK(
                    self.old_gray, gray_frame, self.old_points, None, **self.lk_params
                )

                valid_points = []
                for i, (new, old) in enumerate(zip(new_points, self.old_points)):
                    if status[i]:
                        a, b = new.ravel()
                        c, d = old.ravel()
                        valid_points.append((a, b))

                        cv2.circle(frame, (int(a), int(b)), 5, (0, 255, 0), -1)
                        cv2.line(frame, (int(c), int(d)), (int(a), int(b)), (255, 0, 0), 2)
                self.points = np.array(valid_points).astype(int).tolist()

                self.old_points = np.array(valid_points, dtype=np.float32).reshape(-1, 1, 2)
                self.old_gray = gray_frame.copy()

            kl.bindLayout(frame, self.target_key, self.headPos, self.points, self.do_tracking, self.wPoints, kP, params)
            print(frame.shape)
            cv2.imshow('Camera Feed', _frm)
            print("nono")
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                self.get_logger().info(f"""
detected_corners =  {self.points}
screen_corners = {self.wPoints}
            """)
                if(len(self.points)==4):
                    params.detected_corners = self.points
                    kP.setKeyboardCornersOnScreen(self.points)
            elif key == ord('r'):
                self.do_tracking = True
                self.get_logger().info('Reset key pressed')
                self.points = []
                self.wPoints = []
                self.trackers_initialized = False
                self.old_points = None
            elif key == ord('l'):
                self.wPoints = params.screen_corners
                self.points = params.detected_corners
                self.old_points = self.initialize_trackers(frame)
                self.do_tracking = False
            
            elif key == ord('c'):
                self.get_logger().info("Calculating Homography")
                self.do_tracking = True
            elif key == ord('i'):
                self.sequencing = True
                self.get_logger().info('Typing sequence initiated')
            
            elif(key>=97 and key<=122 or key==32):
                self.target_key = chr(key)
                self.do_tracking = False
                self.moveToKey()
                

        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = IKFeedTuning()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
