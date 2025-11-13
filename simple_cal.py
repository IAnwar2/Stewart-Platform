# Simple Auto Calibration System for Ball and Beam Control
# Interactive calibration tool for color detection, geometry, and servo limits
# Generates config.json file for use with ball tracking controller

import cv2
import numpy as np
import json
import math
import serial
import time
from datetime import datetime

class SimpleAutoCalibrator:
    """Interactive calibration system for ball and beam control setup."""
    
    def __init__(self):
        """Initialize calibration parameters and default values."""
        # Physical system parameters
        self.PLATFORM_DIAMETER = 0.3  # Known beam length in meters
        
        # Camera configuration
        self.CAM_INDEX = 1  # Default camera index
        self.FRAME_W, self.FRAME_H = 640, 480  # Frame dimensions
        
        # Calibration state tracking
        self.current_frame = None  # Current video frame
        self.phase = "color"  # Current phase: "color", "geometry", "limits", "complete"
        
        # Color calibration data
        self.hsv_samples = []  # Collected HSV color samples
        self.lower_hsv = None  # Lower HSV bound for ball detection
        self.upper_hsv = None  # Upper HSV bound for ball detection
        
        # Geometry calibration data
        self.peg_points = [[], [], []]  # Beam endpoint pixel coordinates
        self.pixel_to_meter_ratio = [None, None, None]  # Conversion ratio from pixels to meters
        # self.pixel_to_meter_ratio_ch1 = None  # Conversion ratio from pixels to meters
        # self.pixel_to_meter_ratio_ch2 = None
        # self.pixel_to_meter_ratio_ch3 = None
        
        # Servo hardware configuration
        self.servo = None  # Serial connection to servo
        self.servo_port = "COM3"  # Servo communication port
        self.neutral_angle = 15  # Servo neutral position angle
        
        # Position limit results
        self.position_min = [None, None, None]  # Minimum ball position in meters
        self.position_max = [None, None, None]  # Maximum ball position in meters

    def connect_servo(self):
        """Establish serial connection to servo motor for automated limit finding.
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            self.servo = serial.Serial(self.servo_port, 9600)
            time.sleep(2)  # Allow time for connection to stabilize
            print("[SERVO] Connected")
            return True
        except:
            print("[SERVO] Failed to connect - limits will be estimated")
            return False

    def send_servo_angle(self, angle, channel):
        """Send angle command to servo motor with safety clipping.
        
        Args:
            angle (float): Desired servo angle in degrees
        """
        if self.servo:
            # Clip angle to safe range and send as byte
            angle = int(np.clip(angle, 0, 30))
            self.servo.write(f"{channel} {angle}\n".encode("ascii"))
            # self.servo.write(bytes([angle]))

    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse click events for interactive calibration.
        
        Args:
            event: OpenCV mouse event type
            x, y: Mouse click coordinates
            flags: Additional event flags
            param: User data (unused)
        """
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.phase == "color":
                # Color sampling phase - collect HSV samples at click point
                self.sample_color(x, y)
            elif self.phase == "geometry_ch1" and len(self.peg_points[0]) < 2:
                # Geometry phase - collect beam endpoint coordinates
                self.peg_points[0].append((x, y))
                print(f"[GEO] Peg {len(self.peg_points[0])} selected for motor 1")
                if len(self.peg_points[0]) == 2:
                    self.calculate_geometry(1)
            elif self.phase == "geometry_ch2" and len(self.peg_points[1]) < 2:
                # Geometry phase - collect beam endpoint coordinates
                self.peg_points[1].append((x, y))
                print(f"[GEO] Peg {len(self.peg_points[1])} selected for motor 2")
                if len(self.peg_points[1]) == 2:
                    self.calculate_geometry(2)
            elif self.phase == "geometry_ch3" and len(self.peg_points[2]) < 2:
                # Geometry phase - collect beam endpoint coordinates
                self.peg_points[2].append((x, y))
                print(f"[GEO] Peg {len(self.peg_points[2])} selected for motor 3")
                if len(self.peg_points[2]) == 2:
                    self.calculate_geometry(3)

    def sample_color(self, x, y):
        """Sample HSV color values in a 5x5 region around click point.
        
        Args:
            x, y: Center coordinates for color sampling
        """
        if self.current_frame is None:
            return
        
        # Convert frame to HSV color space
        hsv = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2HSV)
        
        # Sample 5x5 region around click point
        for dy in range(-2, 3):
            for dx in range(-2, 3):
                px, py = x + dx, y + dy
                # Check bounds and collect valid samples
                if 0 <= px < hsv.shape[1] and 0 <= py < hsv.shape[0]:
                    self.hsv_samples.append(hsv[py, px])
        
        # Update HSV bounds based on collected samples
        if self.hsv_samples:
            samples = np.array(self.hsv_samples)
            
            # Calculate adaptive margins for each HSV channel
            h_margin = max(5, (np.max(samples[:, 0]) - np.min(samples[:, 0])) * 0.1)
            s_margin = max(10, (np.max(samples[:, 1]) - np.min(samples[:, 1])) * 0.15)
            v_margin = max(10, (np.max(samples[:, 2]) - np.min(samples[:, 2])) * 0.15)
            
            # Set lower bounds with margin
            self.lower_hsv = [
                max(0, np.min(samples[:, 0]) - h_margin),
                max(0, np.min(samples[:, 1]) - s_margin),
                max(0, np.min(samples[:, 2]) - v_margin)
            ]
            
            # Set upper bounds with margin
            self.upper_hsv = [
                min(179, np.max(samples[:, 0]) + h_margin),
                min(255, np.max(samples[:, 1]) + s_margin),
                min(255, np.max(samples[:, 2]) + v_margin)
            ]
            
            print(f"[COLOR] Samples: {len(self.hsv_samples)}")

    def calculate_geometry(self, channel):
        """Calculate pixel-to-meter conversion ratio from beam endpoint coordinates."""
        p1, p2 = self.peg_points[channel-1]
        
        # Calculate pixel distance between beam endpoints
        pixel_distance = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        
        # Convert to meters using known beam length
        if channel == 1:
            self.pixel_to_meter_ratio[0] = self.PLATFORM_DIAMETER / pixel_distance
            print(f"[GEO] Pixel-to-meter ratio motor 1: {self.pixel_to_meter_ratio[0]:.6f}")
            # Advance to limits geometry_ch2 phase
            self.phase = "geometry_ch2"
        elif channel == 2:
            self.pixel_to_meter_ratio[1] = self.PLATFORM_DIAMETER / pixel_distance
            print(f"[GEO] Pixel-to-meter ratio motor 2: {self.pixel_to_meter_ratio[1]:.6f}")
            # Advance to limits geometry_ch3 phase
            self.phase = "geometry_ch3"
        elif channel == 3:
            self.pixel_to_meter_ratio[2] = self.PLATFORM_DIAMETER / pixel_distance
            print(f"[GEO] Pixel-to-meter ratio motor 3: {self.pixel_to_meter_ratio[2]:.6f}")
            # Advance to limits calibration phase
            self.phase = "limits"

    def detect_ball_position(self, frame, channel):
        """Detect ball in frame and return position in meters from center.
        
        Args:
            frame: Input BGR image frame
            
        Returns:
            float or None: Ball position in meters from center, None if not detected
        """
        if not self.lower_hsv:
            return None
        
        if channel < 1 or channel > 3:
            return None
        if len(self.peg_points[channel-1]) < 2:
            # Pegs not fully selected for this motor yet
            return None
        if self.pixel_to_meter_ratio[channel-1] is None:
            return None
            
        # Convert to HSV and create color mask
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array(self.lower_hsv, dtype=np.uint8)
        upper = np.array(self.upper_hsv, dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        
        # Clean up mask with morphological operations
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        # Find contours in mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        
        # Get largest contour (assumed to be ball)
        largest = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(largest)
        
        # Filter out very small detections
        if radius < 5:
            return None
        
        # Distance along the peg line, 0 at midpoint 
        (x1, y1), (x2, y2) = self.peg_points[channel-1]

        # Direction vector along peg line
        dx = x2 - x1
        dy = y2 - y1
        length = math.hypot(dx, dy)
        if length == 0:
            # Degenerate case: both pegs at same point
            return None
        
        # Unit vector along the line
        ux = dx / length
        uy = dy / length

        # Unit vector perpendicular to beam (normal)
        nx = -uy
        ny = ux

        # Midpoint of the line segment between pegs
        mid_x = 0.5 * (x1 + x2)
        mid_y = 0.5 * (y1 + y2)

        # Vector from midpoint to ball
        wx = x - mid_x
        wy = y - mid_y

        # Signed distance (in pixels) along the line from the midpoint
        # pixel_offset = wx * ux + wy * uy
        pixel_offset = wx * nx + wy * ny
        

        # Convert pixel distance to meters using calibration
        meters_offset = pixel_offset * self.pixel_to_meter_ratio[channel-1]
        
        # # Convert pixel position to meters from center
        # center_x = frame.shape[1] // 2
        # pixel_offset = x - center_x
        # meters_offset = pixel_offset * self.pixel_to_meter_ratio[channel-1]
        
        return meters_offset

    def find_limits_automatically(self):
        """Use servo motor to automatically find ball position limits."""
        if not self.servo:
            # Estimate limits without servo if connection failed
            self.position_min[0] = -self.PLATFORM_DIAMETER / 2
            self.position_max[0] = self.PLATFORM_DIAMETER / 2
            self.position_min[1] = -self.PLATFORM_DIAMETER / 2
            self.position_max[1] = self.PLATFORM_DIAMETER / 2
            self.position_min[2] = -self.PLATFORM_DIAMETER / 2
            self.position_max[2] = self.PLATFORM_DIAMETER / 2
            print("[LIMITS] Estimated without servo")
            return
        
        print("[LIMITS] Finding limits with servo...")
        positions = [[]]
        
        # Test servo at different angles to find position range
        test_angles = [self.neutral_angle - 15, self.neutral_angle, self.neutral_angle + 15]
        
        for ch in (1, 2, 3):
            for angle in test_angles:
                # Move servo to test angle
                self.send_servo_angle(angle, ch)
                time.sleep(2)  # Wait for ball to settle
                
                # Collect multiple position measurements
                angle_positions = []
                start_time = time.time()
                while time.time() - start_time < 1.0:
                    ret, frame = self.cap.read()
                    if ret:
                        pos = self.detect_ball_position(frame)
                        if pos is not None:
                            angle_positions.append(pos)
                    time.sleep(0.05)
                
                # Calculate average position for this angle
                if angle_positions:
                    avg_pos = np.mean(angle_positions)
                    positions[ch-1].append(avg_pos)
                    print(f"[LIMITS] Motor {ch} with Angle {angle}: {avg_pos:.4f}m")
            
            # Return servo to neutral position
            self.send_servo_angle(self.neutral_angle, ch)
        
        for i in (0, 1, 2):
            # Determine position limits from collected data
            if len(positions[i]) >= 2:
                self.position_min[i] = min(positions[i])
                self.position_max[i] = max(positions[i])
                print(f"[LIMITS] Motor {i+1} with Range: {self.position_min[i]:.4f}m to {self.position_max[i]:.4f}m")
            else:
                print("[LIMITS] Failed to find limits")

    def save_config(self):
        """Save all calibration results to config.json file."""
        config = {
            "timestamp": datetime.now().isoformat(),
            "platform_diameter": float(self.PLATFORM_DIAMETER),
            "camera": {
                "index": int(self.CAM_INDEX),
                "frame_width": int(self.FRAME_W),
                "frame_height": int(self.FRAME_H)
            },
            "ball_detection": {
                "lower_hsv": [float(x) for x in self.lower_hsv] if self.lower_hsv else None,
                "upper_hsv": [float(x) for x in self.upper_hsv] if self.upper_hsv else None
            },
            "calibration": {
                "pixel_to_meter_ratio": {
                    "motor_1": float(self.pixel_to_meter_ratio[0]) if len(self.pixel_to_meter_ratio) > 0 else None,
                    "motor_2": float(self.pixel_to_meter_ratio[1]) if len(self.pixel_to_meter_ratio) > 1 else None,
                    "motor_3": float(self.pixel_to_meter_ratio[2]) if len(self.pixel_to_meter_ratio) > 2 else None
                },
                "position_min_m": {
                    "motor_1": float(self.position_min[0]) if len(self.position_min) > 0 else None,
                    "motor_2": float(self.position_min[1]) if len(self.position_min) > 1 else None,
                    "motor_3": float(self.position_min[2]) if len(self.position_min) > 2 else None
                },
                "position_max_m": {
                    "motor_1": float(self.position_max[0]) if len(self.position_max) > 0 else None,
                    "motor_2": float(self.position_max[1]) if len(self.position_max) > 1 else None,
                    "motor_3": float(self.position_max[2]) if len(self.position_max) > 2 else None
                },
                "peg_points": {
                    "motor_1": [float(x) for x in self.peg_points[0]] if len(self.peg_points) > 0 else None,
                    "motor_2": [float(x) for x in self.peg_points[1]] if len(self.peg_points) > 1 else None,
                    "motor_3": [float(x) for x in self.peg_points[2]] if len(self.peg_points) > 2 else None
                }
            },
            "servo": {
                "port": str(self.servo_port),
                "neutral_angle": int(self.neutral_angle)
            }
        }
        
        # Write configuration to JSON file
        with open("config.json", "w") as f:
            json.dump(config, f, indent=2)
        print("[SAVE] Configuration saved to config.json")

    def draw_overlay(self, frame):
        """Draw calibration status and instructions overlay on frame.
        
        Args:
            frame: Input BGR image frame
            
        Returns:
            numpy.ndarray: Frame with overlay graphics and text
        """
        overlay = frame.copy()
        
        # Phase-specific instruction text
        phase_text = {
            "color": "Click on ball to sample colors. Press 'c' when done.",
            "geometry_ch1": "Click on endpoints for Motor 1. Press '1' when done.",
            "geometry_ch2": "Click on endpoints for Motor 2. Press '2' when done.",
            "geometry_ch3": "Click on endpoints for Motor 3. Press '3' when done.",
            "limits": "Press 'l' to find limits automatically",
            "complete": "Calibration complete! Press 's' to save"
        }
        
        # Draw current phase and instructions
        cv2.putText(overlay, f"Phase: {self.phase}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(overlay, phase_text[self.phase], (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Show color calibration progress
        if self.hsv_samples:
            cv2.putText(overlay, f"Color samples: {len(self.hsv_samples)}", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Show geometry calibration points
        for x, channel_points in enumerate(self.peg_points):
            for i, peg in enumerate(channel_points):
                cv2.circle(overlay, peg, 8, (0, 255, 0), -1)
                cv2.putText(overlay, f"Motor {x}: Peg {i+1}", (peg[0]+10, peg[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Draw line between beam endpoints if both are selected
        for x, channel_points in enumerate(self.peg_points):
            if len(channel_points) == 2:
                cv2.line(overlay, channel_points[0], channel_points[1], (255, 0, 0), 2)
        
        # Show real-time ball detection if color calibration is complete
        if self.lower_hsv:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower = np.array(self.lower_hsv, dtype=np.uint8)
            upper = np.array(self.upper_hsv, dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            
            # Clean up mask
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            
            # Find and draw detected ball
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(largest)
                if radius > 5:
                    # Draw detection circle
                    cv2.circle(overlay, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(overlay, (int(x), int(y)), 3, (0, 255, 255), -1)
                    
                    # Show position if geometry calibration is complete
                    for i, ratio in self.pixel_to_meter_ratio:
                        if ratio:
                            pos = self.detect_ball_position(frame)
                            if pos is not None:
                                cv2.putText(overlay, f"Motor {i+1} Pos: {pos:.4f}m",
                                        (int(x)+20, int(y)+20),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Show final results if limit calibration is complete
        if len(self.position_min) == 3 and len(self.position_max) == 3:
            for i in (0, 1, 2):
                cv2.putText(overlay, f"Motor {i+1} Limits: {self.position_min[i]:.4f}m to {self.position_max[i]:.4f}m",
                        (10, overlay.shape[0] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        return overlay

    def run(self):
        """Main calibration loop with interactive GUI."""
        # Initialize camera capture
        self.cap = cv2.VideoCapture(self.CAM_INDEX, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.FRAME_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.FRAME_H)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize latency
        
        # Setup OpenCV window and mouse callback
        cv2.namedWindow("Auto Calibration")
        cv2.setMouseCallback("Auto Calibration", self.mouse_callback)
        
        # Attempt servo connection
        self.connect_servo()
        
        # Display instructions
        print("[INFO] Simple Auto Calibration")
        print("Phase 1: Click on ball to sample colors, press 'c' when done")
        print("Phase 2: Click on beam endpoints")
        print("Phase 3: Press 'l' to find limits")
        print("Press 's' to save, 'q' to quit")
        
        # Main calibration loop
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            self.current_frame = frame
            
            # Draw overlay and display frame
            display = self.draw_overlay(frame)
            cv2.imshow("Auto Calibration", display)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                # Quit calibration
                break
            elif key == ord('c') and self.phase == "color":
                # Complete color calibration phase
                if self.hsv_samples:
                    self.phase = "geometry_ch1"
                    print("[INFO] Color calibration complete. Click on motor 1 endpoints.")
            elif key == ord('l') and self.phase == "limits":
                # Start automatic limit finding
                self.find_limits_automatically()
                self.phase = "complete"
            elif key == ord('s') and self.phase == "complete":
                # Save configuration and exit
                self.save_config()
                break
        
        # Clean up resources
        self.cap.release()
        cv2.destroyAllWindows()
        if self.servo:
            self.servo.close()

if __name__ == "__main__":
    """Run calibration when script is executed directly."""
    calibrator = SimpleAutoCalibrator()
    calibrator.run()