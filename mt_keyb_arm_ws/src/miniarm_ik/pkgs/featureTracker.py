import cv2
import numpy as np

# Global variables
tracking_points = []
trackers_initialized = False

def mouse_callback(event, x, y, flags, param):
    """Callback function to handle mouse clicks."""
    global tracking_points, trackers_initialized
    if event == cv2.EVENT_LBUTTONDOWN:
        tracking_points.append((x, y))
        trackers_initialized = False  # Reset tracker initialization
        print(f"Added tracking point: ({x}, {y})")

def initialize_trackers(frame):
    """Initialize tracking points."""
    global tracking_points
    points = np.array(tracking_points, dtype=np.float32).reshape(-1, 1, 2)
    return points

def main():
    global tracking_points, trackers_initialized

    # Open video feed
    cap = cv2.VideoCapture(2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cv2.namedWindow("Tracking")
    cv2.setMouseCallback("Tracking", mouse_callback)

    # Parameters for Lucas-Kanade optical flow
    lk_params = dict(
        winSize=(55, 55),
        maxLevel=2,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
    )

    old_gray = None
    old_points = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame.")
            break

        # Convert to grayscale for optical flow
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Initialize trackers when new points are added
        if not trackers_initialized and tracking_points:
            old_points = initialize_trackers(frame)
            old_gray = gray_frame.copy()
            trackers_initialized = True

        # Perform tracking if trackers are initialized
        if trackers_initialized and old_points is not None:
            # Calculate optical flow
            new_points, status, _ = cv2.calcOpticalFlowPyrLK(
                old_gray, gray_frame, old_points, None, **lk_params
            )

            # Filter valid points
            valid_points = []
            for i, (new, old) in enumerate(zip(new_points, old_points)):
                if status[i]:
                    a, b = new.ravel()
                    c, d = old.ravel()
                    valid_points.append((a, b))

                    # Draw the tracking
                    cv2.circle(frame, (int(a), int(b)), 5, (0, 255, 0), -1)
                    cv2.line(frame, (int(c), int(d)), (int(a), int(b)), (255, 0, 0), 2)

            # Update points for the next frame
            old_points = np.array(valid_points, dtype=np.float32).reshape(-1, 1, 2)
            old_gray = gray_frame.copy()

        # Show the frame
        cv2.imshow("Tracking", frame)

        # Exit on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
