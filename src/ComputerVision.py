import cv2 as cv
import numpy as np
from smbus2 import SMBus
import time

# Define I2C address for Arduino
ARDUINO_I2C_ADDRESS = 0x8

# Initialize I2C bus
bus = SMBus(1)

# Message ID for coordinate data
MESSAGE_ID = 0x01

class BallTracker:
    def __init__(self):
        self.prev_x = None
        self.prev_y = None
        self.prev_time = None
        # Store multiple previous positions for smoothing
        self.position_history = []
        self.velocity_history = []
        self.history_size = 5  # Number of frames to keep for smoothing
        
    def calculate_velocity(self, current_x, current_y, current_time):
        if self.prev_x is None or self.prev_y is None or self.prev_time is None:
            self.prev_x = current_x
            self.prev_y = current_y
            self.prev_time = current_time
            return 0, 0, 0, 0  # Initial velocity is 0
        
        # Calculate time difference in seconds
        dt = current_time - self.prev_time
        if dt == 0:
            return 0, 0, 0, 0
            
        # Calculate velocity components (pixels per second)
        velocity_x = (current_x - self.prev_x) / dt
        velocity_y = (current_y - self.prev_y) / dt
        
        # Add to velocity history
        self.velocity_history.append((velocity_x, velocity_y))
        if len(self.velocity_history) > self.history_size:
            self.velocity_history.pop(0)
            
        # Calculate smoothed velocity
        smoothed_vx = sum(v[0] for v in self.velocity_history) / len(self.velocity_history)
        smoothed_vy = sum(v[1] for v in self.velocity_history) / len(self.velocity_history)
        
        # Update previous values
        self.prev_x = current_x
        self.prev_y = current_y
        self.prev_time = current_time
        
        return velocity_x, velocity_y, smoothed_vx, smoothed_vy
    
def find_grid_square(x, y, n=5):
    cell_size = 256 / n
    row, col = int (y // cell_size), int(x // cell_size)
    
    return row, col

def detect_ball_and_platform():
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FPS, 30)
    tracker = BallTracker()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        frame = cv.resize(frame, (255, 255))
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Detect the yellow ball
        ball_color_lower = np.array([15, 140, 100])
        ball_color_upper = np.array([40, 255, 255])
        ball_mask = cv.inRange(hsv, ball_color_lower, ball_color_upper)
        
        # Find contours for the yellow ball
        ball_contours, _ = cv.findContours(ball_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        if ball_contours:
            largest_contour = max(ball_contours, key=cv.contourArea)
            ((x, y), radius) = cv.minEnclosingCircle(largest_contour)
            
            if radius > 10:
                # Calculate velocity
                current_time = time.time()
                velocity_x, velocity_y, _, _ = tracker.calculate_velocity(x, y, current_time)
                
                # Draw ball position
                cv.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv.circle(frame, (int(x), int(y)), 2, (0, 0, 255), -1)
                
                # Draw velocity vector
                vector_scale = 2.0
                end_x = int(x + velocity_x * vector_scale)
                end_y = int(y + velocity_y * vector_scale)
                cv.arrowedLine(frame, (int(x), int(y)), (end_x, end_y), (0, 255, 0), 2)
                
                # Display ball position
                print(f"Ball Position: ({int(x)}, {int(y)}), Velocity: ({velocity_x:.2f}, {velocity_y:.2f}) px/s")

        # Detect the platform corners and calculate its center
        red_lower = np.array([0, 120, 70])
        red_upper = np.array([10, 255, 255])
        platform_mask = cv.inRange(hsv, red_lower, red_upper)
        
        # Find contours for the platform corners
        platform_contours, _ = cv.findContours(platform_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        corner_points = []
        for contour in platform_contours:
            area = cv.contourArea(contour)
            if 50 < area < 500:  # Adjust as needed
                # Get the center of each contour
                M = cv.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    corner_points.append((cx, cy))
                    # Draw detected corner
                    cv.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        
        if len(corner_points) == 4:
            # Calculate the center of the platform
            center_x = sum(p[0] for p in corner_points) / 4
            center_y = sum(p[1] for p in corner_points) / 4
            cv.circle(frame, (int(center_x), int(center_y)), 5, (255, 0, 0), -1)
            print(f"Platform Center: ({int(center_x)}, {int(center_y)})")

            # Calculate coordinates of ball relative to center of platform
            ball_x_adjusted, ball_y_adjusted = x - center_x, y - center_y
        send_data_to_arduino(x, y, velocity_x, velocity_y, ball_x_adjusted, ball_y_adjusted)

        # Display frame with annotations
        cv.imshow('frame', frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()
    

def send_data_to_arduino(x=0, y=0, velocity_x=0, velocity_y=0, adjusted_x=0, adjusted_y=0):
    try:
        # Send position data to Arduino
        bus.write_i2c_block_data(ARDUINO_I2C_ADDRESS, MESSAGE_ID, [int(x), int(y), int(velocity_x), int(velocity_y), row, col])
        # Optionally send velocity data as well
        # You might need to scale/convert velocity values to fit in a byte
    except Exception as e:
        print(f"I2C Error: {e}")

if __name__ == "__main__":
    detect_ball_and_platform()