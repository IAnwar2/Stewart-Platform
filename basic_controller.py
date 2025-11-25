import cv2
import numpy as np
import json
import serial
import time
import tkinter as tk
import matplotlib.pyplot as plt
from tkinter import ttk
from threading import Thread
import queue
from ball_detection import detect_ball_x

class BasicPIDController:
    def __init__(self, config_file="config.json"):
        """Initialize controller, load config, set defaults and queues."""
        # Load experiment and hardware config from JSON file
        with open(config_file, 'r') as f:
            self.config = json.load(f)
        # PID gains - TWO SETS for different error ranges
        # Coarse mode (large errors, far from setpoint): fast response, less oscillation
        self.Kp_coarse = [0.35, 0.35, 0.35]
        self.Ki_coarse = [0.05, 0.05, 0.05]
        self.Kd_coarse = [0.15, 0.15, 0.15]
        # Fine mode (small errors, near setpoint): precise convergence
        self.Kp_fine = [0.25, 0.25, 0.25]
        self.Ki_fine = [0.12, 0.12, 0.12]
        self.Kd_fine = [0.20, 0.20, 0.20]
        # Current operating gains (controlled by sliders in GUI)
        self.Kp = [0.25, 0.25, 0.25]
        self.Ki = [0.12, 0.12, 0.12]
        self.Kd = [0.20, 0.20, 0.20]
        # Error threshold for switching between coarse and fine modes
        self.error_threshold = 0.05  # meters (5 cm)
        # Scale factor for converting from pixels to meters
        self.scale_factor = self.config['calibration']['pixel_to_meter_ratio'] * self.config['camera']['frame_width'] / 2
        # Servo port name and center angle
        self.servo_port = self.config['servo']['port']
        self.neutral_angle = self.config['servo']['neutral_angle']
        self.servo = None
        # Controller-internal state
        self.setpoint = [0.0, 0.0, 0.0]
        self.integral = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]
        self.prev_position = [0.0, 0.0, 0.0]  # For low-pass filtering derivative
        self.derivative_filtered = [0.0, 0.0, 0.0]  # Filtered derivative
        # Anti-windup limits for integral
        self.integral_max = [0.5, 0.5, 0.5]
        self.integral_min = [-0.5, -0.5, -0.5]
        # Data logs for plotting results
        self.time_log = [[], [], []]
        self.position_log = [[], [], []]
        self.setpoint_log = [[], [], []]
        self.control_log = [[], [], []]
        self.mode_log = [[], [], []]  # Track which mode was used
        self.start_time = None
        # Thread-safe queue for most recent ball position measurement
        self.position_queue = [queue.Queue(maxsize=1), queue.Queue(maxsize=1), queue.Queue(maxsize=1)]
        self.running = False    # Main run flag for clean shutdown

        self.active_motors = [True, False, False] # Store the active motors
        self.ctrl_motor_idx = 0
        self.mult = [-1, 1, -1] # Some motors are positive in the wrong direction (probably cause of camera)
        # store last seen positions so control loop can use last-known value if camera misses a frame
        self.last_positions = [0.0, 0.0, 0.0]
        # previous timestamp for computing dt in control loop
        self.prev_control_time = None
        # limits for integral anti-windup
        self.integral_limits = [1000.0, 1000.0, 1000.0]


    def connect_servo(self):
        """Try to open serial connection to servo, return True if success."""
        try:
            self.servo = serial.Serial(self.servo_port, 9600)
            time.sleep(2)
            print("[SERVO] Connected")
            return True
        except Exception as e:
            print(f"[SERVO] Failed: {e}")
            return False

    def send_servo_angle(self, angle, channel):
        """Send angle command to servo motor (clipped for safety)."""
        if self.servo:
            servo_angle = self.neutral_angle + angle
            servo_angle = int(np.clip(servo_angle, -10, 30))
            try:
                self.servo.write(f"{channel} {servo_angle}\n".encode("ascii"))
            except Exception:
                print("[SERVO] Send failed")

    def update_pid(self, position, motor_idx, dt=0.033):
        """Perform adaptive PID calculation with two gain sets and anti-windup."""
        error = self.setpoint[motor_idx] - position
        abs_error = abs(error)
        
        # Adaptive gain selection based on error magnitude
        if abs_error > self.error_threshold:
            # Coarse mode: large errors, aggressive response
            Kp = self.Kp_coarse[motor_idx]
            Ki = self.Ki_coarse[motor_idx]
            Kd = self.Kd_coarse[motor_idx]
            mode = 1  # Coarse mode indicator
        else:
            # Fine mode: small errors, precise convergence
            Kp = self.Kp_fine[motor_idx]
            Ki = self.Ki_fine[motor_idx]
            Kd = self.Kd_fine[motor_idx]
            mode = 0  # Fine mode indicator
        
        # Proportional term
        P = Kp * error
        
        # Integral term with anti-windup
        self.integral[motor_idx] += error * dt
        # Clamp integral to prevent windup
        self.integral[motor_idx] = np.clip(
            self.integral[motor_idx], 
            self.integral_min[motor_idx], 
            self.integral_max[motor_idx]
        )
        I = Ki * self.integral[motor_idx]
        
        # Derivative term with first-order low-pass filter
        # Reduces noise sensitivity while maintaining responsiveness
        raw_derivative = (error - self.prev_error[motor_idx]) / dt
        alpha = 0.3  # Filter coefficient (0-1), lower = more filtering
        self.derivative_filtered[motor_idx] = (
            alpha * raw_derivative + 
            (1 - alpha) * self.derivative_filtered[motor_idx]
        )
        D = Kd * self.derivative_filtered[motor_idx]
        
        self.prev_error[motor_idx] = error
        self.prev_position[motor_idx] = position
        
        # PID output (limit to safe beam range)
        output = P + I + D
        output = np.clip(output, -20, 20)
        
        return output, mode

    def camera_thread(self):
        """Dedicated thread for video capture and ball detection."""
        cap = cv2.VideoCapture(self.config['camera']['index'], cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        while self.running:
            ret, frame = cap.read()
            if not ret:
                continue
            #frame = cv2.resize(frame, (320, 240))
            # Detect ball position in frame
            found, pos_m1_normalized, pos_m2_normalized, pos_m3_normalized, vis_frame = detect_ball_x(frame)
            if found:
                # Convert normalized to meters using scale
                position_m = [0, 0, 0]
                position_m[0] = pos_m1_normalized * self.scale_factor * self.mult[0]
                position_m[1] = -pos_m2_normalized * self.scale_factor * self.mult[1]
                position_m[2] = pos_m3_normalized * self.scale_factor * self.mult[2]
                # Always keep latest measurement only
                try:
                    for motor_idx in range(3):
                        if self.active_motors[motor_idx]:
                            if self.position_queue[motor_idx].full():
                                self.position_queue[motor_idx].get_nowait()
                            self.position_queue[motor_idx].put_nowait(position_m[motor_idx])
                            print(f"Motor {motor_idx + 1}: {position_m[motor_idx]}")
                except Exception:
                    pass
            # Show processed video with overlays
            cv2.imshow("Ball Tracking", vis_frame)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC exits
                self.running = False
                break
        cap.release()
        cv2.destroyAllWindows()

    def control_thread(self):
        """Runs PID control loop in parallel with GUI and camera."""
        if not self.connect_servo():
            print("[ERROR] No servo - running in simulation mode")
        self.start_time = time.time()
        # control loop will run at roughly 30-60 Hz depending on camera update
        while self.running:
            try:
                loop_time = time.time()
                if self.prev_control_time is None:
                    dt = 0.033
                else:
                    dt = max(1e-3, loop_time - self.prev_control_time)
                self.prev_control_time = loop_time

                # collect latest positions for all motors (non-blocking). Use last known if missing.
                positions = [None, None, None]
                for motor_idx in range(3):
                    positions[motor_idx] = self.last_positions[motor_idx]
                    if self.active_motors[motor_idx]:
                        try:
                            # get newest position if available
                            pos = self.position_queue[motor_idx].get_nowait()
                            positions[motor_idx] = pos
                            self.last_positions[motor_idx] = pos
                        except queue.Empty:
                            # no new measurement this loop: use last known
                            pass

                # compute PID outputs for all motors (so we can coordinate/decouple)
                raw_outputs = [0.0, 0.0, 0.0]
                for motor_idx in range(3):
                    if self.active_motors[motor_idx]:
                        # Wait for latest ball position from camera
                        position = self.position_queue[motor_idx].get(timeout=0.1)
                        # Compute control output using adaptive PID
                        control_output, mode = self.update_pid(position, motor_idx)
                        control_output = control_output/np.clip(self.active_motors.count(True), 1, 3)
                        # Send control command to servo (real or simulated)
                        self.send_servo_angle(control_output, motor_idx + 1)
                        # Log results for plotting
                        current_time = time.time() - self.start_time
                        self.time_log[motor_idx].append(current_time)
                        self.position_log[motor_idx].append(positions[motor_idx])
                        self.setpoint_log[motor_idx].append(self.setpoint[motor_idx])
                        self.control_log[motor_idx].append(control_output)
                        self.mode_log[motor_idx].append(mode)
                        mode_str = "COARSE" if mode else "FINE"
                        error = self.setpoint[motor_idx] - position
                        print(f"Motor {motor_idx + 1} [{mode_str}] Pos: {position:.3f}m, Error: {error:.3f}m, Output: {control_output:.1f}°")
                    else: # Set inactive to neutral angle
                        self.send_servo_angle(0, motor_idx + 1)
                        self.time_log[motor_idx].append(current_time)
                        self.position_log[motor_idx].append(0)
                        self.setpoint_log[motor_idx].append(self.setpoint[motor_idx])
                        self.control_log[motor_idx].append(0)
                        self.mode_log[motor_idx].append(-1)
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[CONTROL] Error: {e}")
                break
        if self.servo:
            # Return to neutral on exit
            self.send_servo_angle(0,1)
            self.send_servo_angle(0,2)
            self.send_servo_angle(0,3)
            self.servo.close()

    def create_gui(self):
        """Build Tkinter GUI with large sliders and labeled controls."""
        self.root = tk.Tk()
        self.root.title("Basic PID Controller")
        self.root.geometry("520x560")

        ctrl_m = self.ctrl_motor_idx

        # Title label
        ttk.Label(self.root, text="PID Gains", font=("Arial", 18, "bold")).pack(pady=10)

        # Kp slider
        ttk.Label(self.root, text="Kp (Proportional)", font=("Arial", 12)).pack()
        self.kp_var = tk.DoubleVar(value=self.Kp[ctrl_m])
        kp_slider = ttk.Scale(self.root, from_=0, to=10, variable=self.kp_var,
                              orient=tk.HORIZONTAL, length=500)
        kp_slider.pack(pady=5)
        self.kp_label = ttk.Label(self.root, text=f"Kp: {self.Kp[ctrl_m]:.1f}", font=("Arial", 11))
        self.kp_label.pack()

        # Ki slider
        ttk.Label(self.root, text="Ki (Integral)", font=("Arial", 12)).pack()
        self.ki_var = tk.DoubleVar(value=self.Ki[ctrl_m])
        ki_slider = ttk.Scale(self.root, from_=0, to=5, variable=self.ki_var,
                              orient=tk.HORIZONTAL, length=500)
        ki_slider.pack(pady=5)
        self.ki_label = ttk.Label(self.root, text=f"Ki: {self.Ki[ctrl_m]:.1f}", font=("Arial", 11))
        self.ki_label.pack()

        # Kd slider
        ttk.Label(self.root, text="Kd (Derivative)", font=("Arial", 12)).pack()
        self.kd_var = tk.DoubleVar(value=self.Kd[ctrl_m])
        kd_slider = ttk.Scale(self.root, from_=0, to=10, variable=self.kd_var,
                              orient=tk.HORIZONTAL, length=500)
        kd_slider.pack(pady=5)
        self.kd_label = ttk.Label(self.root, text=f"Kd: {self.Kd[ctrl_m]:.1f}", font=("Arial", 11))
        self.kd_label.pack()

        # Setpoint slider
        ttk.Label(self.root, text="Setpoint (meters)", font=("Arial", 12)).pack()
        pos_min = self.config['calibration']['position_min_m']
        pos_max = self.config['calibration']['position_max_m']
        self.setpoint_var = tk.DoubleVar(value=self.setpoint[ctrl_m])
        setpoint_slider = ttk.Scale(self.root, from_=pos_min, to=pos_max,
                                   variable=self.setpoint_var,
                                   orient=tk.HORIZONTAL, length=500)
        setpoint_slider.pack(pady=5)
        self.setpoint_label = ttk.Label(self.root, text=f"Setpoint: {self.setpoint[ctrl_m]:.3f}m", font=("Arial", 11))
        self.setpoint_label.pack()

        # Button group for actions
        button_frame = ttk.Frame(self.root)
        button_frame.pack(pady=20)
        ttk.Button(button_frame, text="Reset Integral",
                   command=self.reset_integral).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Plot Results",
                   command=self.plot_results).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Stop",
                   command=self.stop).pack(side=tk.LEFT, padx=5)

        # Control motor
        ttk.Label(self.root, text="Controlled Motor", font=("Arial", 10)).pack()
        control_frame = ttk.Frame(self.root)
        control_frame.pack(pady=5)
        self.ctrl_m_var = tk.IntVar(value=ctrl_m)
        ttk.Radiobutton(control_frame, text="Motor 1", variable=self.ctrl_m_var, value=0).pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(control_frame, text="Motor 2", variable=self.ctrl_m_var, value=1).pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(control_frame, text="Motor 3", variable=self.ctrl_m_var, value=2).pack(side=tk.LEFT, padx=5)

        # Active motors
        ttk.Label(self.root, text="Active Motors", font=("Arial", 10)).pack()
        active_frame = ttk.Frame(self.root)
        active_frame.pack(pady=5)
        self.act_m1_var = tk.IntVar(value=self.active_motors[0])
        ttk.Checkbutton(active_frame, text="Motor 1", variable=self.act_m1_var).pack(side=tk.LEFT, padx=5)
        self.act_m2_var = tk.IntVar(value=self.active_motors[1])
        ttk.Checkbutton(active_frame, text="Motor 2", variable=self.act_m2_var).pack(side=tk.LEFT, padx=5)
        self.act_m3_var = tk.IntVar(value=self.active_motors[2])
        ttk.Checkbutton(active_frame, text="Motor 3", variable=self.act_m3_var).pack(side=tk.LEFT, padx=5)

        # Schedule periodic GUI update
        self.update_gui()

    def update_gui(self):
        """Reflect latest values from sliders into program and update display."""
        if self.running:
            ctrl_m = self.ctrl_motor_idx
            # PID parameters
            self.Kp[ctrl_m] = self.kp_var.get()
            self.Ki[ctrl_m] = self.ki_var.get()
            self.Kd[ctrl_m] = self.kd_var.get()
            self.setpoint[ctrl_m] = self.setpoint_var.get()
            # Controlled motor
            self.ctrl_motor_idx = self.ctrl_m_var.get()
            # Active motors
            self.active_motors[0] = self.act_m1_var.get()
            self.active_motors[1] = self.act_m2_var.get()
            self.active_motors[2] = self.act_m3_var.get()
            # Update displayed values
            self.kp_label.config(text=f"Kp: {self.Kp[ctrl_m]:.3f}")
            self.ki_label.config(text=f"Ki: {self.Ki[ctrl_m]:.3f}")
            self.kd_label.config(text=f"Kd: {self.Kd[ctrl_m]:.3f}")
            self.setpoint_label.config(text=f"Setpoint: {self.setpoint[ctrl_m]:.3f}m")
            # Call again after 50 ms (if not stopped)
            self.root.after(50, self.update_gui)

    def reset_integral(self):
        """Clear integral error in PID (button handler)."""
        self.integral[self.ctrl_motor_idx] = 0.0
        print("[RESET] Integral term reset")

    def plot_results(self):
        """Show matplotlib plots of position, control, and mode logs."""
        if not self.time_log[0]:
            print("[PLOT] No data to plot")
            return
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))
        ctrl_m = self.ctrl_motor_idx
        
        # Ball position trace
        ax1.plot(self.time_log[ctrl_m], self.position_log[ctrl_m], label="Ball Position", linewidth=2, marker='o', markersize=3)
        ax1.plot(self.time_log[ctrl_m], self.setpoint_log[ctrl_m], label="Setpoint",
                 linestyle="--", linewidth=2)
        ax1.set_ylabel("Position (m)")
        ax1.set_title(f"Adaptive PID Control - Motor {ctrl_m + 1}")
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Control output trace
        ax2.plot(self.time_log[ctrl_m], self.control_log[ctrl_m], label="Control Output",
                 color="orange", linewidth=2)
        ax2.set_ylabel("Platform Angle (degrees)")
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Mode switching trace (coarse vs fine)
        coarse_times = []
        coarse_positions = []
        fine_times = []
        fine_positions = []
        
        for i, mode in enumerate(self.mode_log[ctrl_m]):
            if mode == 1:  # Coarse mode
                coarse_times.append(self.time_log[ctrl_m][i])
                coarse_positions.append(self.position_log[ctrl_m][i])
            elif mode == 0:  # Fine mode
                fine_times.append(self.time_log[ctrl_m][i])
                fine_positions.append(self.position_log[ctrl_m][i])
        
        if coarse_times:
            ax3.scatter(coarse_times, coarse_positions, label="Coarse Mode (|error| > 5cm)", 
                       color='red', s=30, alpha=0.6)
        if fine_times:
            ax3.scatter(fine_times, fine_positions, label="Fine Mode (|error| ≤ 5cm)", 
                       color='green', s=30, alpha=0.6)
        
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel("Position (m)")
        ax3.set_title("PID Control Mode")
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()

    def stop(self):
        """Stop everything and clean up threads and GUI."""
        self.running = False
        # Try to safely close all windows/resources
        try:
            self.root.quit()
            self.root.destroy()
        except Exception:
            pass

    def run(self):
        """Entry point: starts threads, launches GUI mainloop."""
        print("[INFO] Starting Basic PID Controller")
        print("Use sliders to tune PID gains in real-time")
        print("Close camera window or click Stop to exit")
        self.running = True

        # Start camera and control threads, mark as daemon for exit
        cam_thread = Thread(target=self.camera_thread, daemon=True)
        ctrl_thread = Thread(target=self.control_thread, daemon=True)
        cam_thread.start()
        ctrl_thread.start()

        # Build and run GUI in main thread
        self.create_gui()
        self.root.mainloop()

        # After GUI ends, stop everything
        self.running = False
        print("[INFO] Controller stopped")

if __name__ == "__main__":
    try:
        controller = BasicPIDController()
        controller.run()
    except FileNotFoundError:
        print("[ERROR] config.json not found. Run simple_autocal.py first.")
    except Exception as e:
        print(f"[ERROR] {e}")
