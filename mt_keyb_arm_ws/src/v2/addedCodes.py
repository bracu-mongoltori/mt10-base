import ik_solver_mine

import time

# Check if MPS is available (for macOS) and set it as the device, otherwise use CPU
device = ""

corners = [[137, 164], [1147, 558]]
worldpos = [[19.0,12.0],[-14.5,25.0]]

def remap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

target_key = ""

def on_press(self, key):
        global target_key
        try:
            key_str = key.char.upper() if hasattr(key, 'char') else key.name.upper()
            target_key = key_str
            self.pressed_keys.add(key_str)
        except AttributeError:
            pass


global tmr
        if(target_key!="" and time.time() - tmr > 1.0):
            sp = keyboard_overlay.key_positions[target_key]

            tmr = time.time()
            _x = remap(sp['x'], corners[0][0], corners[1][0], worldpos[0][0], worldpos[1][0])
            _y = remap(sp['y'], corners[0][1], corners[1][1], worldpos[0][1], worldpos[1][1])
            print((_x, _y))
            ik_solver_mine.reach(_x, _y)


# --------------------------------------------------------------------
import torch
from ultralytics import YOLO  # Import YOLO library
import cv2
import numpy as np
from collections import deque
from pynput import keyboard
import ik_solver_mine

import time

# Check if MPS is available (for macOS) and set it as the device, otherwise use CPU
device = ""

corners = [[137, 164], [1147, 558]]
worldpos = [[19.0,12.0],[-14.5,25.0]]

def remap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

if torch.cuda.is_available():
    device="cuda"
else:
    device="cpu" # use mps for mac M series of proccesors
print(f"Using device: {device}")

# Load the YOLO model+
model = YOLO('YOLO_Training and models/pt_files/keyboard_detection_model.pt')  # Replace with the path to your trained model

# Rolling average setup for angle smoothing
angle_buffer = deque(maxlen=5)  # Store the last 5 angle readings

# Known real-world width of the object (e.g., keyboard width in cm)
KNOWN_WIDTH = 44.2  # Replace with the real-world width of your object in cm
FOCAL_LENGTH = 1000  # Approximate focal length in pixels (tune based on camera calibration)

target_key = ""

class KeyboardOverlay:
    def __init__(self, keyboard_config=None):
        # Direct pixel measurements at 24 inches
        self.KEYBOARD_WIDTH_PX = 1068
        self.KEYBOARD_HEIGHT_PX = 450

        # Calculate scaling factor based on pixel measurements
        standard_key_px = self.KEYBOARD_WIDTH_PX / 19
        self.config = {
            'keyboard_width': self.KEYBOARD_WIDTH_PX,
            'keyboard_height': self.KEYBOARD_HEIGHT_PX,
            'standard_key_size': standard_key_px,
            'key_spacing': standard_key_px / 5,  # Small gap between keys
            'spacebar_width': standard_key_px * 6.265,  # 6x standard key
            'enter_width': standard_key_px * 2.25,  # 2.25x standard key
            'shift_width': standard_key_px * 2.25,  # 2.25x standard key
            'backspace_width': standard_key_px * 2,  # 2x standard key
            'tab_width': standard_key_px * 1.5,  # 1.5x standard key
            'caps_width': standard_key_px * 1.75,  # 1.75x standard key
            'ctrl_width': standard_key_px * 1.5,  # 1.5x standard key
            'alt_width': standard_key_px * 1.25,  # 1.25x standard key
            'win_width': standard_key_px * 1.25,  # 1.25x standard key
            'edge_margin': standard_key_px / 2,  # Half key size margin
            'scaling_factor': 1.0  # Direct pixel scaling
        }

        # Update with custom config if provided
        if keyboard_config:
            self.config.update(keyboard_config)

        # Store pixel measurements
        self.sf = self.config['scaling_factor']
        self.keyboard_width = int(self.config['keyboard_width'] * self.sf)
        self.keyboard_height = int(self.config['keyboard_height'] * self.sf)
        self.key_size = int(self.config['standard_key_size'] * self.sf)
        self.key_spacing = max(1, int(self.config['key_spacing'] * self.sf))

        # Colors (BGR format)
        self.BLUE = (255, 150, 0)
        self.GREEN = (0, 255, 0)
        self.WHITE = (255, 255, 255)
        self.RED = (0, 0, 255)

        # Store key positions and states
        self.key_positions = {}
        self.pressed_keys = set()

        # Initialize keyboard listener
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()

    def get_key_width(self, key):
        """Calculate width for different types of keys"""
        if key == 'SPACE':
            return int(self.config['spacebar_width'] * self.sf)
        elif key == 'ENTER':
            return int(self.config['enter_width'] * self.sf)
        elif key in ['LEFT_SHIFT', 'RIGHT_SHIFT']:
            return int(self.config['shift_width'] * self.sf)
        elif key == 'BACKSPACE':
            return int(self.config['backspace_width'] * self.sf)
        elif key in ['TAB']:
            return int(self.config['tab_width'] * self.sf)
        elif key in ['CAPS_LOCK']:
            return int(self.config['caps_width'] * self.sf)
        elif key in ['LEFT_CTRL', 'RIGHT_CTRL']:
            return int(self.config['ctrl_width'] * self.sf)
        elif key in ['LEFT_ALT', 'RIGHT_ALT', 'FN']:
            return int(self.config['alt_width'] * self.sf)
        elif key == 'LEFT_WIN':
            return int(self.config['win_width'] * self.sf)
        else:
            return self.key_size

    def get_display_text(self, key):
        """Convert internal key names to display text"""
        display_map = {
            'GRAVE': '`',
            'MINUS': '-',
            'EQUALS': '=',
            'LEFT_BRACKET': '[',
            'RIGHT_BRACKET': ']',
            'BACKSLASH': '\\',
            'SEMICOLON': ';',
            'QUOTE': "'",
            'COMMA': ',',
            'PERIOD': '.',
            'SLASH': '/',
            'SPACE': 'Space',
            'LEFT_SHIFT': 'Shift',
            'RIGHT_SHIFT': 'Shift',
            'LEFT_CTRL': 'Ctrl',
            'RIGHT_CTRL': 'Ctrl',
            'LEFT_ALT': 'Alt',
            'RIGHT_ALT': 'Alt',
            'LEFT_WIN': 'Win',
            'CAPS_LOCK': 'Caps',
            'FN': 'Fn'
        }
        return display_map.get(key, key)

    def on_press(self, key):
        global target_key
        try:
            key_str = key.char.upper() if hasattr(key, 'char') else key.name.upper()
            target_key = key_str
            self.pressed_keys.add(key_str)
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            key_str = key.char.upper() if hasattr(key, 'char') else key.name.upper()
            self.pressed_keys.discard(key_str)
        except AttributeError:
            pass
        if key == keyboard.Key.esc:
            return False

    def create_base_overlay(self):
        overlay = np.zeros((self.keyboard_height, self.keyboard_width, 4), dtype=np.uint8)
        margin = int(self.config['edge_margin'] * self.sf)
        cv2.rectangle(overlay, (margin, margin),
                      (self.keyboard_width - margin, self.keyboard_height - margin),
                      (*self.BLUE, 255), 2)

        key_rows = [
            ['ESC'] + [f'F{i}' for i in range(1, 13)],
            ['GRAVE', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', 'MINUS', 'EQUALS', 'BACKSPACE'],
            ['TAB', 'Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P', 'LEFT_BRACKET', 'RIGHT_BRACKET', 'BACKSLASH'],
            ['CAPS_LOCK', 'A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', 'SEMICOLON', 'QUOTE', 'ENTER'],
            ['LEFT_SHIFT', 'Z', 'X', 'C', 'V', 'B', 'N', 'M', 'COMMA', 'PERIOD', 'SLASH', 'RIGHT_SHIFT'],
            ['LEFT_CTRL', 'LEFT_WIN', 'LEFT_ALT', 'SPACE', 'RIGHT_ALT', 'FN', 'RIGHT_CTRL']
        ]

        y_offset = margin
        row_heights = [self.key_size * 0.8] + [self.key_size] * 5  # Smaller function key row

        for row_idx, row in enumerate(key_rows):
            x_offset = margin
            current_height = int(row_heights[row_idx])

            for key in row:
                width = self.get_key_width(key)

                self.key_positions[key] = {
                    'x': x_offset,
                    'y': y_offset,
                    'width': width,
                    'height': current_height
                }

                color = self.RED if key in self.pressed_keys else self.BLUE
                cv2.rectangle(overlay,
                              (x_offset, y_offset),
                              (x_offset + width, y_offset + current_height),
                              (*color, 255), 1)

                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.4 if row_idx == 0 else 0.5  # Smaller text for function keys
                display_text = self.get_display_text(key)
                text_size = cv2.getTextSize(display_text, font, font_scale, 1)[0]
                text_x = x_offset + (width - text_size[0]) // 2
                text_y = y_offset + (current_height + text_size[1]) // 2
                cv2.putText(overlay, display_text, (text_x, text_y),
                            font, font_scale, (*self.WHITE, 255), 1)

                x_offset += width + self.key_spacing
            y_offset += current_height + self.key_spacing

        return overlay

def nothing(x):
    pass

tmr = time.time()
def main():
    global cv2
    # Initialize camera
    # cap = cv2.VideoCapture(1)
    # if not cap.isOpened():
    #     print("Error: Could not open camera")
    #     return

    # Set camera resolution
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # Create keyboard overlay instance
    keyboard_overlay = KeyboardOverlay()

    # Create window and trackbars
    window_name = 'Keyboard Alignment Guide'
    cv2.namedWindow(window_name)
    cv2.createTrackbar('Opacity', window_name, 50, 100, nothing)
    cv2.createTrackbar('X Position', window_name, 50, 100, nothing)
    cv2.createTrackbar('Y Position', window_name, 50, 100, nothing)
    cv2.createTrackbar('Fine Scale', window_name, 100, 150, nothing)  # Finer scale adjustment

    import pyrealsense2 as rs
    import numpy as np
    import cv2

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    _device = pipeline_profile.get_device()
    device_product_line = str(_device.get_info(rs.camera_info.product_line))
    print(device_product_line)

    found_rgb = False
    for s in _device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        # print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    while True:
        # ret, frame = cap.read()
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        frame = np.asanyarray(color_frame.get_data())
        # print(frame.shape)
        # if not ret:
        #     break

        # Run YOLO object detection on the current frame
        results = model.predict(frame, device=device, verbose=False)

        # Process each detection
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Get bounding box coordinates as integers
                confidence = box.conf[0]  # Confidence score

                # Draw the bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

                # Calculate the perceived width in pixels
                perceived_width = x2 - x1

                # Estimate distance using the formula: Distance = (Real Width * Focal Length) / Perceived Width
                distance = (KNOWN_WIDTH * FOCAL_LENGTH) / perceived_width

                # Display the distance on the frame
                cv2.putText(frame, f"Distance: {distance:.2f} cm", (x1, y2 + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                # Crop the detected object region for angle estimation
                cropped_object = frame[y1:y2, x1:x2]

                # Convert to grayscale
                gray = cv2.cvtColor(cropped_object, cv2.COLOR_BGR2GRAY)

                # Apply Canny edge detection
                edges = cv2.Canny(gray, 50, 150)

                # Find contours
                contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                if contours:
                    # Find the largest contour
                    largest_contour = max(contours, key=cv2.contourArea)

                    # Get the minimum area rectangle
                    rect = cv2.minAreaRect(largest_contour)
                    angle = rect[-1]  # Angle of orientation

                    # Correct the angle if necessary
                    if angle < -45:
                        angle += 90
                    elif angle > 45:
                        angle -= 90

                    # Add the angle to the rolling buffer
                    angle_buffer.append(angle)

                    # Calculate the rolling average if buffer has enough readings
                    smoothed_angle = np.mean(angle_buffer) if len(angle_buffer) > 1 else angle

                    # Display the smoothed angle on the frame
                    cv2.putText(frame, f"Angle: {smoothed_angle:.2f} degrees", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    # Display guidance messages for the user with specific rotation instructions
                    if abs(smoothed_angle) < 1.5:  # Stricter alignment threshold
                        cv2.putText(frame, "Aligned! Move straight.", (50, 50),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    elif smoothed_angle > 1.5:
                        cv2.putText(frame, f"Rotate clockwise by {smoothed_angle:.2f} degrees", (50, 50),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    elif smoothed_angle < -1.5:
                        cv2.putText(frame, f"Rotate counterclockwise by {abs(smoothed_angle):.2f} degrees", (50, 50),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Overlay keyboard guide on the frame
        opacity = cv2.getTrackbarPos('Opacity', window_name) / 100.0
        x_pos = cv2.getTrackbarPos('X Position', window_name) / 100.0
        y_pos = cv2.getTrackbarPos('Y Position', window_name) / 100.0
        fine_scale = cv2.getTrackbarPos('Fine Scale', window_name) / 100.0

        overlay = keyboard_overlay.create_base_overlay()
        target_width = int(keyboard_overlay.KEYBOARD_WIDTH_PX * fine_scale)
        target_height = int(keyboard_overlay.KEYBOARD_HEIGHT_PX * fine_scale)
        x_offset = int((frame.shape[1] - target_width) * x_pos)
        y_offset = int((frame.shape[0] - target_height) * y_pos)

        resized_overlay = cv2.resize(overlay, (target_width, target_height))
        mask = np.zeros((frame.shape[0], frame.shape[1], 4), dtype=np.uint8)
        mask[y_offset:y_offset + target_height, x_offset:x_offset + target_width] = resized_overlay

        mask_alpha = mask[:, :, 3] / 255.0
        for c in range(3):
            frame[:, :, c] = frame[:, :, c] * (1 - mask_alpha * opacity) + mask[:, :, c] * (mask_alpha * opacity)

        global tmr
        if(target_key!="" and time.time() - tmr > 1.0):
            sp = keyboard_overlay.key_positions[target_key]

            tmr = time.time()
            _x = remap(sp['x'], corners[0][0], corners[1][0], worldpos[0][0], worldpos[1][0])
            _y = remap(sp['y'], corners[0][1], corners[1][1], worldpos[0][1], worldpos[1][1])
            print((_x, _y))
            ik_solver_mine.reach(_x, _y)
        # Display the final output
        cv2.imshow(window_name, frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    keyboard_overlay.listener.stop()
    # cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
