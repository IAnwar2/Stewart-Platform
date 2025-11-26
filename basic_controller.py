import cv2
import numpy as np
import json
import serial
import time
import math
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
        # PID gains (controlled by sliders in GUI)
        self.Kp = [16.049, 16.049, 16.049]
        self.Ki = [6.379, 6.379, 6.379]
        self.Kd = [7.160, 7.160, 7.160]
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
        # Data logs for plotting results
        self.time_log = [[], [], []]
        self.position_log = [[], [], []]
        self.setpoint_log = [[], [], []]
        self.control_log = [[], [], []]
        self.start_time = None
        # Thread-safe queue for most recent ball position measurement
        self.position_queue = [queue.Queue(maxsize=1), queue.Queue(maxsize=1), queue.Queue(maxsize=1)]
        self.running = False    # Main run flag for clean shutdown

        self.active_motors = [1, 1, 1] # Store the active motors
        self.ctrl_motor_idx = 0
        self.ctrl_all = 0

        # Geometry
        self.center = np.array(self.config['geometry']['center'])
        self.p1 = np.array(self.config['geometry']['motor_1_pos'])
        self.p2 = np.array(self.config['geometry']['motor_2_pos'])
        self.p3 = np.array(self.config['geometry']['motor_3_pos'])
        self.setpoint_xy = self.center
        self.trunc_m = 0.07 # distance before truncation begins
        self.flat_m = 0.005 # *affects distance where PID values are zero

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
            servo_angle = int(np.clip(servo_angle, 0, 40))
            try:
                self.servo.write(f"{channel} {servo_angle}\n".encode("ascii"))
            except Exception:
                print("[SERVO] Send failed")

    def update_pid(self, position, motor_idx, dt=0.033):
        """Perform PID calculation and return control output."""
        error = self.setpoint[motor_idx] - position  # Compute error
        # Reduce PID values as it approaches the centre
        # Kp
        Kp = self.deadbanding(error, self.Kp[motor_idx]) #if abs(error) > 0.01 else 0
        # Ki
        Ki = self.rising(error, self.Ki[motor_idx]) #if abs(error) > 0.01 else 0
        # Kd
        Kd = self.rising(error, self.Kd[motor_idx]) #if abs(error) > 0.01 else 0
        print (Kp, Ki, Kd)
        # Proportional term
        P = Kp * error
        # Integral term accumulation
        self.integral[motor_idx] += error * dt
        I = Ki * self.integral[motor_idx]
        # Derivative term calculation
        derivative = (error - self.prev_error[motor_idx]) / dt
        D = Kd * derivative
        self.prev_error[motor_idx] = error
        # PID output (limit to safe beam range)
        output = P + I + D
        output = np.clip(output, -20, 20)
        #print(error)
        return output
    
    def deadbanding(self, error, k):
        a = self.trunc_m
        b = self.flat_m
        x = abs(error)

        result = k
        result += np.heaviside(a-x, 0) * (k/(a-b)*(x-b)-k)
        result += np.heaviside(b-x, 0) * (k/(a-b)*(b-x))
        return result
    
    def rising(self, error, k):
        a = self.trunc_m
        b = self.flat_m
        x = abs(error)

        result = k
        result += np.heaviside(a-x, 0) * (k/(a-b)*(b-x)+k)
        result += np.heaviside(b-x, 0) * (k/(a-b)*(x-b))
        return result

    def camera_thread(self):
        """Dedicated thread for video capture and ball detection."""
        cap = cv2.VideoCapture(self.config['camera']['index'], cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        cv2.namedWindow("Ball Tracking")
        cv2.setMouseCallback("Ball Tracking", self.mouse_callback)
        while self.running:
            ret, frame = cap.read()
            if not ret:
                continue
            #frame = cv2.resize(frame, (320, 240))
            cv2.circle(frame, self.setpoint_xy, 8, (0, 255, 0), -1)
            cv2.circle(frame, self.p1, 2, (0, 255, 0), -1)
            cv2.circle(frame, self.p2, 2, (0, 255, 0), -1)
            cv2.circle(frame, self.p3, 2, (0, 255, 0), -1)

            # Detect ball position in frame
            found, pos_m1_normalized, pos_m2_normalized, pos_m3_normalized, vis_frame = detect_ball_x(frame)
            if found:
                # Convert normalized to meters using scale
                position_m = [0, 0, 0]
                position_m[0] = round(pos_m1_normalized, 10)
                position_m[1] = round(pos_m2_normalized, 10)
                position_m[2] = round(pos_m3_normalized, 10)
                # Always keep latest measurement only
                try:
                    for motor_idx in range(3):
                        if self.active_motors[motor_idx]:
                            if self.position_queue[motor_idx].full():
                                self.position_queue[motor_idx].get_nowait()
                            self.position_queue[motor_idx].put_nowait(position_m[motor_idx])
                            #print(f"Motor {motor_idx + 1}: {position_m[motor_idx]}")
                except Exception:
                    pass
            # Show processed video with overlays
            cv2.imshow("Ball Tracking", vis_frame)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC exits
                self.running = False
                break
        cap.release()
        cv2.destroyAllWindows()

    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse click events for interactive calibration.
        
        Args:
            event: OpenCV mouse event type
            x, y: Mouse click coordinates
            flags: Additional event flags
            param: User data (unused)
        """

        if event == cv2.EVENT_LBUTTONDOWN:
            self.setpoint_xy = [x, y]

            normalized_x = (x - self.center[0]) / self.center[0]
            normalized_y = (y - self.center[1]) / self.center[1]

            relative_p1 = self.p1[0] - self.center[0], self.p1[1] - self.center[1]
            relative_p2 = self.p2[0] - self.center[0], self.p2[1] - self.center[1]
            relative_p3 = self.p3[0] - self.center[0], self.p3[1] - self.center[1]

            unit_vector_m1_x = relative_p1[0] / math.sqrt(relative_p1[0] ** 2 + relative_p1[1] ** 2)
            unit_vector_m1_y = relative_p1[1] / math.sqrt(relative_p1[0] ** 2 + relative_p1[1] ** 2)

            unit_vector_m2_x = relative_p2[0] / math.sqrt(relative_p2[0] ** 2 + relative_p2[1] ** 2)
            unit_vector_m2_y = relative_p2[1] / math.sqrt(relative_p2[0] ** 2 + relative_p2[1] ** 2) 

            unit_vector_m3_x = relative_p3[0] / math.sqrt(relative_p3[0] ** 2 + relative_p3[1] ** 2)
            unit_vector_m3_y = relative_p3[1] / math.sqrt(relative_p3[0] ** 2 + relative_p3[1] ** 2)

            pos_along_m1 = (normalized_x * unit_vector_m1_x + normalized_y * unit_vector_m1_y) * self.scale_factor
            pos_along_m2 = (normalized_x * unit_vector_m2_x + normalized_y * unit_vector_m2_y) * self.scale_factor
            pos_along_m3 = (normalized_x * unit_vector_m3_x + normalized_y * unit_vector_m3_y) * self.scale_factor

            self.setpoint = [-pos_along_m1, -pos_along_m2, -pos_along_m3]

    def control_thread(self):
        """Runs PID control loop in parallel with GUI and camera."""
        if not self.connect_servo():
            print("[ERROR] No servo - running in simulation mode")
        self.start_time = time.time()
        while self.running:
            try:
                for motor_idx in range(3):
                    if self.active_motors[motor_idx]:
                        # Wait for latest ball position from camera
                        position = self.position_queue[motor_idx].get(timeout=0.1)
                        # Compute control output using PID
                        control_output = self.update_pid(position, motor_idx)
                        #control_output = control_output/np.clip(self.active_motors.count(True), 1, 2)
                        # Send control command to servo (real or simulated)
                        self.send_servo_angle(control_output, motor_idx + 1)
                        # Log results for plotting
                        current_time = time.time() - self.start_time
                        self.time_log[motor_idx].append(current_time)
                        self.position_log[motor_idx].append(position)
                        self.setpoint_log[motor_idx].append(self.setpoint[motor_idx])
                        self.control_log[motor_idx].append(control_output)
                        print(f"Motor {motor_idx + 1} Pos: {position:.3f}m, Output: {control_output:.1f}°")
                    else: # Set inactive to neutral angle
                        self.send_servo_angle(0, motor_idx + 1)
                        # Log results for plotting
                        current_time = time.time() - self.start_time
                        self.time_log[motor_idx].append(current_time)
                        self.position_log[motor_idx].append(0)
                        self.setpoint_log[motor_idx].append(self.setpoint[motor_idx])
                        self.control_log[motor_idx].append(0)
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
        kp_slider = ttk.Scale(self.root, from_=0, to=100, variable=self.kp_var,
                              orient=tk.HORIZONTAL, length=500)
        kp_slider.pack(pady=5)
        self.kp_label = ttk.Label(self.root, text=f"Kp: {self.Kp[ctrl_m]:.1f}", font=("Arial", 11))
        self.kp_label.pack()

        # Ki slider
        ttk.Label(self.root, text="Ki (Integral)", font=("Arial", 12)).pack()
        self.ki_var = tk.DoubleVar(value=self.Ki[ctrl_m])
        ki_slider = ttk.Scale(self.root, from_=0, to=10, variable=self.ki_var,
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
        #pos_min = self.config['calibration']['position_min_m']
        #pos_max = self.config['calibration']['position_max_m']
        #self.setpoint_var = tk.DoubleVar(value=self.setpoint[ctrl_m])
        # setpoint_slider = ttk.Scale(self.root, from_=pos_min, to=pos_max,
        #                            variable=self.setpoint_var,
        #                            orient=tk.HORIZONTAL, length=500)
        # setpoint_slider.pack(pady=5)
        self.setpoint_label = ttk.Label(self.root, text=f"Setpoint: ({self.setpoint[0]}, {self.setpoint[1]}, {self.setpoint[2]}) m", font=("Arial", 11))
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
        self.ctrl_all_var = tk.IntVar(value=self.ctrl_all)
        ttk.Checkbutton(button_frame, text="Cntrl All", variable=self.ctrl_all_var).pack(side=tk.LEFT, padx=5)

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
            # Controlled motor
            init_ctrl_m = self.ctrl_motor_idx
            self.ctrl_motor_idx = self.ctrl_m_var.get()
            ctrl_m = self.ctrl_motor_idx
            self.ctrl_all = self.ctrl_all_var.get()
            # PID parameters
            if self.ctrl_all:
                kp_var = self.kp_var.get()
                ki_var = self.ki_var.get()
                kd_var = self.kd_var.get()
                self.Kp = [kp_var, kp_var, kp_var]
                self.Ki = [ki_var, ki_var, ki_var]
                self.Kd = [kd_var, kd_var, kd_var]
            elif init_ctrl_m == ctrl_m:
                self.Kp[ctrl_m] = self.kp_var.get()
                self.Ki[ctrl_m] = self.ki_var.get()
                self.Kd[ctrl_m] = self.kd_var.get()
            else:
                self.kp_var.set(self.Kp[ctrl_m])
                self.ki_var.set(self.Ki[ctrl_m])
                self.kd_var.set(self.Kd[ctrl_m])
            # Active motors
            self.active_motors[0] = self.act_m1_var.get()
            self.active_motors[1] = self.act_m2_var.get()
            self.active_motors[2] = self.act_m3_var.get()
            # Update displayed values
            self.kp_label.config(text=f"Kp: {self.Kp[ctrl_m]:.3f}")
            self.ki_label.config(text=f"Ki: {self.Ki[ctrl_m]:.3f}")
            self.kd_label.config(text=f"Kd: {self.Kd[ctrl_m]:.3f}")
            self.setpoint_label.config(text=f"Setpoint: ({self.setpoint[0]:.3f}, {self.setpoint[1]:.3f}, {self.setpoint[2]:.3f}) m")
            # Call again after 50 ms (if not stopped)
            self.root.after(50, self.update_gui)

    def reset_integral(self):
        """Clear integral error in PID (button handler)."""
        self.integral[0] = 0.0
        self.integral[1] = 0.0
        self.integral[2] = 0.0
        print("[RESET] Integral term reset")

    def plot_results(self):
        """Show matplotlib plots of position and control logs."""
        if not self.time_log:
            print("[PLOT] No data to plot")
            return
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
        ctrl_m = self.ctrl_motor_idx
        # Ball position trace
        ax1.plot(self.time_log[ctrl_m], self.position_log[ctrl_m], label="Ball Position", linewidth=2)
        ax1.plot(self.time_log[ctrl_m], self.setpoint_log[ctrl_m], label="Setpoint",
                 linestyle="--", linewidth=2)
        ax1.set_ylabel("Position (m)")
        ax1.set_title(f"Basic PID Control (Kp={self.Kp[ctrl_m]:.1f}, Ki={self.Ki[ctrl_m]:.1f}, Kd={self.Kd[ctrl_m]:.1f})")
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        # Control output trace
        ax2.plot(self.time_log[ctrl_m], self.control_log[ctrl_m], label="Control Output",
                 color="orange", linewidth=2)
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Platform Angle (degrees)")
        ax2.legend()
        ax2.grid(True, alpha=0.3)
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
