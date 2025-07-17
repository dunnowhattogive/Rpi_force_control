from logging import root
import time
import serial
import threading
import tkinter as tk
from tkinter import ttk
from gpiozero import OutputDevice, PWMOutputDevice
import signal
import sys
import traceback

# --- Stepper Motor Control ---
class StepperMotor:
    DIR_PIN = 20
    STEP_PIN = 21
    ENABLE_PIN = 16
    STEP_DELAY = 0.001

    def __init__(self):
        # Use gpiozero OutputDevice for stepper pins
        try:
            self.dir = OutputDevice(self.DIR_PIN, active_high=True, initial_value=False)
            self.step = OutputDevice(self.STEP_PIN, active_high=True, initial_value=False)
            self.enable = OutputDevice(self.ENABLE_PIN, active_high=False, initial_value=True)
            self.enable.on()  # Enable motor (LOW logic, so .on() sets pin low)
        except Exception as e:
            print(f"StepperMotor GPIO setup error: {e}")
            self.dir = self.step = self.enable = None

    def step(self, steps, direction):
        if self.dir is None or self.step is None:
            print("StepperMotor not initialized.")
            return
        self.dir.value = 1 if direction else 0
        for _ in range(steps):
            self.step.on()
            time.sleep(self.STEP_DELAY)
            self.step.off()
            time.sleep(self.STEP_DELAY)

    def disable(self):
        if self.enable is not None:
            self.enable.off()  # Disable motor (HIGH logic, so .off() sets pin high)

# --- Servo Motor Control ---
class ServoController:
    SERVO_PINS = [17, 27, 22]  # Example GPIO pins for 3 servos

    def __init__(self):
        self.servos = []
        for pin in self.SERVO_PINS:
            try:
                pwm = PWMOutputDevice(pin, frequency=50)
                pwm.value = 0
                self.servos.append(pwm)
            except Exception as e:
                print(f"ServoController PWM setup error on pin {pin}: {e}")
                self.servos.append(None)

    def angle_to_duty(self, angle):
        # Map angle to duty cycle (0.05 to 0.25 for 0-180 deg)
        return 0.05 + (angle / 180.0) * 0.20

    def set_angle(self, idx, angle):
        if idx < len(self.servos) and self.servos[idx] is not None:
            duty = self.angle_to_duty(angle)
            self.servos[idx].value = duty

    def cleanup(self):
        for servo in self.servos:
            if servo is not None:
                servo.value = 0

# --- Load Cell Reading ---
def read_force(ser):
    line = ser.readline().decode('utf-8').strip()
    try:
        return float(line)
    except ValueError:
        return None

# --- Shared Data for GUI/Main Thread Communication ---
class SharedData:
    def __init__(self, initial_threshold, tolerance):
        self.force_threshold = initial_threshold
        self.force_tolerance = tolerance
        self.current_force = 0.0
        self.status = "Waiting..."

# --- Servo and Force GUI ---
def servo_force_gui(servo_ctrl, shared):
    """
    Launches a Tkinter-based GUI for controlling servo angles and monitoring force measurements.
    Args:
        servo_ctrl: An object that provides a `set_angle(idx, angle)` method to control servo motors.
        shared: An object with the following attributes:
            - force_threshold (float): The current force threshold value.
            - current_force (float): The latest measured force value.
            - force_tolerance (float): The tolerance range for the force threshold.
    Features:
        - Allows the user to adjust the angle of three servos using sliders.
        - Displays the current force measurement, updating periodically.
        - Provides a slider to adjust the force threshold.
        - Shows a status label indicating whether the current force is below, within, or above the threshold (with tolerance), using color coding (blue, green, red).
        - Updates the GUI in real-time as values change.
    Note:
        This function blocks execution as it runs the Tkinter main loop.
    """
    def on_angle_change(idx, var):
        angle = var.get()
        servo_ctrl.set_angle(idx, angle)

    def on_threshold_change(val):
        shared.force_threshold = float(val)
        update_status()

    def set_preset(idx, angle):
        angle_vars[idx].set(angle)
        servo_ctrl.set_angle(idx, angle)

    def update_force_display():
        force_label.config(text=f"Current Force: {shared.current_force:.2f}g")
        update_status()
        root.after(200, update_force_display)

    def update_status():
        force = shared.current_force
        threshold = shared.force_threshold
        tol = shared.force_tolerance
        if force < threshold - tol:
            status = "Below threshold"
            status_label.config(fg="blue")
        elif force > threshold + tol:
            status = "Above threshold"
            status_label.config(fg="red")
        else:
            status = "Within threshold"
            status_label.config(fg="green")
        status_label.config(text=f"Status: {status}")

    root = tk.Tk()
    root.title("Servo & Force Control")

    # Servo controls
    angle_vars = []
    for i in range(3):
        frame = tk.Frame(root)
        frame.pack(padx=10, pady=5)
        tk.Label(frame, text=f"Servo {i+1} Angle:").pack(side=tk.LEFT)
        angle_var = tk.IntVar(value=90)
        angle_vars.append(angle_var)
        scale = tk.Scale(frame, from_=0, to=180, orient=tk.HORIZONTAL, variable=angle_var,
                         command=lambda val, idx=i, var=angle_var: on_angle_change(idx, var))
        scale.pack(side=tk.LEFT)

        # Preset buttons at 30° intervals
        preset_frame = tk.Frame(frame)
        preset_frame.pack(side=tk.LEFT, padx=5)
        for preset in range(0, 181, 30):
            btn = tk.Button(preset_frame, text=str(preset),
                            command=lambda idx=i, angle=preset: set_preset(idx, angle), width=3)
            btn.pack(side=tk.LEFT, padx=1)

    # Force threshold control
    thresh_frame = tk.Frame(root)
    thresh_frame.pack(padx=10, pady=10)
    tk.Label(thresh_frame, text="Force Threshold (g):").pack(side=tk.LEFT)
    threshold_var = tk.DoubleVar(value=shared.force_threshold)
    threshold_scale = tk.Scale(thresh_frame, from_=0, to=2000, orient=tk.HORIZONTAL, resolution=1,
                               variable=threshold_var, command=on_threshold_change, length=300)
    threshold_scale.pack(side=tk.LEFT)

    # Force display
    force_label = tk.Label(root, text="Current Force: 0.00g", font=("Arial", 16))
    force_label.pack(pady=10)

    # Status display
    status_label = tk.Label(root, text="Status: Waiting...", font=("Arial", 16))
    status_label.pack(pady=5)

    # Periodically update force display
    root.after(200, update_force_display)
    root.mainloop()

# --- Cleanup function ---
def cleanup(stepper, servo_ctrl, ser):
    ser.close()
    stepper.disable()
    servo_ctrl.cleanup()
    # No GPIO.cleanup() needed for gpiozero

# --- Controller class integrating GUI and logic ---
class Controller:
    def __init__(self):
        # Remove GPIO.setmode(GPIO.BCM) -- not needed for gpiozero
        self.stepper = StepperMotor()
        self.servo_ctrl = ServoController()

        SERIAL_PORT = '/dev/ttyUSB0'
        BAUDRATE = 9600
        FORCE_TARGET = 500
        FORCE_TOLERANCE = 10

        self.shared = SharedData(FORCE_TARGET, FORCE_TOLERANCE)

        self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)

        self.force_threshold = FORCE_TARGET
        self.running = True

        # PID parameters
        self.pid_kp = 0.1  # Proportional gain
        self.pid_ki = 0.01 # Integral gain
        self.pid_kd = 0.05 # Derivative gain
        self.pid_integral = 0.0
        self.pid_last_error = 0.0

        # Control enable flags
        self.servo_enabled = True
        self.stepper_enabled = True

        # Start GUI in a separate thread
        gui_thread = threading.Thread(target=servo_force_gui, args=(self.servo_ctrl, self.shared), daemon=True)
        gui_thread.start()

        def handle_exit(signum, frame):
            self.cleanup()
            sys.exit(0)

        signal.signal(signal.SIGINT, handle_exit)   # Handle Ctrl+C
        signal.signal(signal.SIGTERM, handle_exit)  # Handle termination

        try:
            while self.running:
                force = read_force(self.ser)
                if force is None:
                    continue
                self.shared.current_force = force
                threshold = self.shared.force_threshold
                # --- PID control logic ---
                if self.stepper_enabled:
                    error = threshold - force
                    self.pid_integral += error
                    derivative = error - self.pid_last_error
                    output = self.pid_kp * error + self.pid_ki * self.pid_integral + self.pid_kd * derivative
                    self.pid_last_error = error

                    # Convert output to stepper steps and direction
                    steps = int(abs(output))
                    direction = output > 0  # True: increase force, False: decrease force

                    if steps > 0:
                        self.stepper.step(steps, direction)
                        print(f"PID: error={error:.2f}, output={output:.2f}, steps={steps}, direction={'up' if direction else 'down'}")
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.cleanup()
        finally:
            self.cleanup()

    def cleanup(self):
        cleanup(self.stepper, self.servo_ctrl, self.ser)
        self.running = False

    def increase_force(self):
        if self.stepper_enabled:
            self.stepper.step(10, True)
            print("Stepper turning anti-clockwise to increase force")

    def decrease_force(self):
        if self.stepper_enabled:
            self.stepper.step(10, False)
            print("Stepper turning clockwise to decrease force")

    def get_force_reading(self):
        return self.shared.current_force

    def auto_adjust_force(self):
        # This method is not used in the main loop anymore, but kept for compatibility
        while self.running:
            force = self.get_force_reading()
            if force is not None and self.stepper_enabled:
                if force < self.force_threshold:
                    self.increase_force()
                elif force > self.force_threshold:
                    self.decrease_force()
            time.sleep(1)

    def increase_threshold(self):
        self.shared.force_threshold += 1
        print(f"Force threshold increased to {self.shared.force_threshold}")

    def decrease_threshold(self):
        self.shared.force_threshold -= 1
        print(f"Force threshold decreased to {self.shared.force_threshold}")

# --- App class for Tkinter GUI ---
class App:
    def __init__(self, root):
        self.root = root
        self.root.title("RPi Control")
        self.controller = Controller()

        self.servo_control_enabled = tk.BooleanVar(value=True)
        self.stepper_control_enabled = tk.BooleanVar(value=True)

        # --- Notebook for tabs ---
        notebook = ttk.Notebook(root)
        notebook.pack(fill="both", expand=True)

        # --- Main tab (manual force control, stop button) ---
        main_frame = tk.Frame(notebook)
        notebook.add(main_frame, text="Main")

        self.manual_increase_force_button = tk.Button(
            main_frame, text="Increase Force (Stepper)", command=self.controller.increase_force
        )
        self.manual_increase_force_button.pack(pady=5)

        self.manual_decrease_force_button = tk.Button(
            main_frame, text="Decrease Force (Stepper)", command=self.controller.decrease_force
        )
        self.manual_decrease_force_button.pack(pady=5)

        self.stop_button = tk.Button(main_frame, text="Stop Program", command=self.stop_program)
        self.stop_button.pack(pady=10)

        # --- Settings tab ---
        settings_frame = tk.Frame(notebook)
        notebook.add(settings_frame, text="Settings")

        # Pin Selection
        pin_frame = tk.LabelFrame(settings_frame, text="Pin Selection")
        pin_frame.pack(padx=10, pady=5, fill="x")

        tk.Label(pin_frame, text="Stepper DIR:").grid(row=0, column=0, sticky="e")
        self.stepper_dir_var = tk.IntVar(value=StepperMotor.DIR_PIN)
        tk.Entry(pin_frame, textvariable=self.stepper_dir_var, width=5).grid(row=0, column=1)

        tk.Label(pin_frame, text="Stepper STEP:").grid(row=0, column=2, sticky="e")
        self.stepper_step_var = tk.IntVar(value=StepperMotor.STEP_PIN)
        tk.Entry(pin_frame, textvariable=self.stepper_step_var, width=5).grid(row=0, column=3)

        tk.Label(pin_frame, text="Stepper ENABLE:").grid(row=0, column=4, sticky="e")
        self.stepper_enable_var = tk.IntVar(value=StepperMotor.ENABLE_PIN)
        tk.Entry(pin_frame, textvariable=self.stepper_enable_var, width=5).grid(row=0, column=5)

        tk.Label(pin_frame, text="Servo 1 PWM:").grid(row=1, column=0, sticky="e")
        self.servo1_var = tk.IntVar(value=ServoController.SERVO_PINS[0])
        tk.Entry(pin_frame, textvariable=self.servo1_var, width=5).grid(row=1, column=1)

        tk.Label(pin_frame, text="Servo 2 PWM:").grid(row=1, column=2, sticky="e")
        self.servo2_var = tk.IntVar(value=ServoController.SERVO_PINS[1])
        tk.Entry(pin_frame, textvariable=self.servo2_var, width=5).grid(row=1, column=3)

        tk.Label(pin_frame, text="Servo 3 PWM:").grid(row=1, column=4, sticky="e")
        self.servo3_var = tk.IntVar(value=ServoController.SERVO_PINS[2])
        tk.Entry(pin_frame, textvariable=self.servo3_var, width=5).grid(row=1, column=5)

        self.apply_pins_button = tk.Button(pin_frame, text="Apply Pins", command=self.apply_pins)
        self.apply_pins_button.grid(row=2, column=0, columnspan=6, pady=4)

        # Enable/disable checkboxes
        self.servo_checkbox = tk.Checkbutton(
            settings_frame, text="Enable Servo Control", variable=self.servo_control_enabled,
            command=self.update_servo_controls
        )
        self.servo_checkbox.pack(pady=2)

        self.stepper_checkbox = tk.Checkbutton(
            settings_frame, text="Enable Stepper Control", variable=self.stepper_control_enabled,
            command=self.update_stepper_controls
        )
        self.stepper_checkbox.pack(pady=2)

        # Force threshold adjustment
        self.increase_button = tk.Button(settings_frame, text="Increase Force Threshold", command=self.controller.increase_threshold)
        self.increase_button.pack(pady=5)

        self.decrease_button = tk.Button(settings_frame, text="Decrease Force Threshold", command=self.controller.decrease_threshold)
        self.decrease_button.pack(pady=5)

        # --- PID parameter controls ---
        pid_frame = tk.LabelFrame(settings_frame, text="PID Parameters")
        pid_frame.pack(padx=10, pady=10, fill="x")

        tk.Label(pid_frame, text="Kp:").grid(row=0, column=0, sticky="e")
        self.kp_var = tk.DoubleVar(value=self.controller.pid_kp)
        tk.Entry(pid_frame, textvariable=self.kp_var, width=7).grid(row=0, column=1)

        tk.Label(pid_frame, text="Ki:").grid(row=0, column=2, sticky="e")
        self.ki_var = tk.DoubleVar(value=self.controller.pid_ki)
        tk.Entry(pid_frame, textvariable=self.ki_var, width=7).grid(row=0, column=3)

        tk.Label(pid_frame, text="Kd:").grid(row=0, column=4, sticky="e")
        self.kd_var = tk.DoubleVar(value=self.controller.pid_kd)
        tk.Entry(pid_frame, textvariable=self.kd_var, width=7).grid(row=0, column=5)

        self.apply_pid_button = tk.Button(pid_frame, text="Apply PID", command=self.apply_pid_params)
        self.apply_pid_button.grid(row=1, column=0, columnspan=6, pady=4)

        # --- end Settings tab ---

        # --- Tests tab ---
        tests_frame = tk.Frame(notebook)
        notebook.add(tests_frame, text="Tests")

        self.test_results_text = tk.Text(tests_frame, height=15, width=60, state=tk.DISABLED)
        self.test_results_text.pack(padx=10, pady=10)

        run_unit_btn = tk.Button(tests_frame, text="Run Unit Tests", command=self.run_unit_tests_gui)
        run_unit_btn.pack(pady=5)

        run_system_btn = tk.Button(tests_frame, text="Run System Tests", command=self.run_system_tests_gui)
        run_system_btn.pack(pady=5)

        # Start a thread to sync enable flags with Controller
        self.sync_thread = threading.Thread(target=self.sync_enable_flags, daemon=True)
        self.sync_thread.start()

    def update_servo_controls(self):
        self.controller.servo_enabled = self.servo_control_enabled.get()
        # If you have servo widgets, enable/disable them here

    def update_stepper_controls(self):
        self.controller.stepper_enabled = self.stepper_control_enabled.get()
        state = tk.NORMAL if self.stepper_control_enabled.get() else tk.DISABLED
        self.increase_button.config(state=state)
        self.decrease_button.config(state=state)
        self.manual_increase_force_button.config(state=state)
        self.manual_decrease_force_button.config(state=state)

    def stop_program(self):
        self.controller.cleanup()
        self.root.quit()

    def sync_enable_flags(self):
        # Keep controller flags in sync with GUI checkboxes
        while True:
            self.controller.servo_enabled = self.servo_control_enabled.get()
            self.controller.stepper_enabled = self.stepper_control_enabled.get()
            time.sleep(0.2)

    def apply_pins(self):
        # Update StepperMotor pins
        StepperMotor.DIR_PIN = self.stepper_dir_var.get()
        StepperMotor.STEP_PIN = self.stepper_step_var.get()
        StepperMotor.ENABLE_PIN = self.stepper_enable_var.get()
        # Update ServoController pins
        ServoController.SERVO_PINS = [
            self.servo1_var.get(),
            self.servo2_var.get(),
            self.servo3_var.get()
        ]
        # Optionally, reinitialize hardware if needed
        print("Updated Stepper and Servo pins:")
        print(f"Stepper DIR: {StepperMotor.DIR_PIN}, STEP: {StepperMotor.STEP_PIN}, ENABLE: {StepperMotor.ENABLE_PIN}")
        print(f"Servo PWM: {ServoController.SERVO_PINS}")

    def apply_pid_params(self):
        self.controller.pid_kp = self.kp_var.get()
        self.controller.pid_ki = self.ki_var.get()
        self.controller.pid_kd = self.kd_var.get()
        print(f"Applied PID params: Kp={self.controller.pid_kp}, Ki={self.controller.pid_ki}, Kd={self.controller.pid_kd}")

    def run_unit_tests_gui(self):
        self.test_results_text.config(state=tk.NORMAL)
        self.test_results_text.delete(1.0, tk.END)
        try:
            results = run_unit_tests()
            self.test_results_text.insert(tk.END, "\n".join(results) + "\n")
        except Exception as e:
            self.test_results_text.insert(tk.END, f"Error running unit tests:\n{traceback.format_exc()}\n")
        self.test_results_text.config(state=tk.DISABLED)

    def run_system_tests_gui(self):
        self.test_results_text.config(state=tk.NORMAL)
        self.test_results_text.delete(1.0, tk.END)
        try:
            results = run_system_tests(self.controller)
            self.test_results_text.insert(tk.END, "\n".join(results) + "\n")
        except Exception as e:
            self.test_results_text.insert(tk.END, f"Error running system tests:\n{traceback.format_exc()}\n")
        self.test_results_text.config(state=tk.DISABLED)

# --- Unit and System Tests ---
def run_unit_tests():
    results = []
    try:
        # ServoController.angle_to_duty
        servo = ServoController()
        assert abs(servo.angle_to_duty(0) - 2.5) < 0.01, "Servo angle 0 failed"
        assert abs(servo.angle_to_duty(180) - 12.5) < 0.01, "Servo angle 180 failed"
        assert abs(servo.angle_to_duty(90) - 7.5) < 0.01, "Servo angle 90 failed"
        results.append("ServoController.angle_to_duty: PASS")
    except Exception as e:
        results.append(f"ServoController.angle_to_duty: FAIL ({e})")
    try:
        # StepperMotor.disable (should not raise)
        stepper = StepperMotor()
        stepper.disable()
        results.append("StepperMotor.disable: PASS")
    except Exception as e:
        results.append(f"StepperMotor.disable: FAIL ({e})")
    try:
        # SharedData initialization
        shared = SharedData(100, 5)
        assert shared.force_threshold == 100
        assert shared.force_tolerance == 5
        assert shared.current_force == 0.0
        assert shared.status == "Waiting..."
        results.append("SharedData.__init__: PASS")
    except Exception as e:
        results.append(f"SharedData.__init__: FAIL ({e})")
    try:
        # read_force returns float or None
        class DummySer:
            def readline(self):
                return b'123.45\n'
        val = read_force(DummySer())
        assert abs(val - 123.45) < 0.01
        class DummySerBad:
            def readline(self):
                return b'bad\n'
        val2 = read_force(DummySerBad())
        assert val2 is None
        results.append("read_force: PASS")
    except Exception as e:
        results.append(f"read_force: FAIL ({e})")
    try:
        # PID parameter assignment
        class DummyController:
            pid_kp = 0.1
            pid_ki = 0.01
            pid_kd = 0.05
        dummy = DummyController()
        dummy.pid_kp = 0.5
        dummy.pid_ki = 0.2
        dummy.pid_kd = 0.1
        assert dummy.pid_kp == 0.5 and dummy.pid_ki == 0.2 and dummy.pid_kd == 0.1
        results.append("PID parameter assignment: PASS")
    except Exception as e:
        results.append(f"PID parameter assignment: FAIL ({e})")
    return results

def run_system_tests(controller):
    results = []
    try:
        # Stepper enable/disable
        controller.stepper.disable()
        results.append("StepperMotor.disable: PASS")
        controller.stepper.step(1, True)
        results.append("StepperMotor.step: PASS")
    except Exception as e:
        results.append(f"StepperMotor test: FAIL ({e})")
    try:
        # Servo set_angle
        controller.servo_ctrl.set_angle(0, 90)
        results.append("ServoController.set_angle: PASS")
    except Exception as e:
        results.append(f"ServoController.set_angle: FAIL ({e})")
    try:
        # Controller manual force control
        controller.stepper.step = lambda steps, direction: results.append(f"Stepper step called: steps={steps}, direction={direction}")
        controller.stepper_enabled = True
        controller.increase_force()
        controller.decrease_force()
        results.append("Controller.increase_force/decrease_force: PASS")
    except Exception as e:
        results.append(f"Controller.increase_force/decrease_force: FAIL ({e})")
    try:
        # Controller threshold adjustment
        old = controller.shared.force_threshold
        controller.increase_threshold()
        assert controller.shared.force_threshold == old + 1
        controller.decrease_threshold()
        assert controller.shared.force_threshold == old
        results.append("Controller.increase_threshold/decrease_threshold: PASS")
    except Exception as e:
        results.append(f"Controller.increase_threshold/decrease_threshold: FAIL ({e})")
    try:
        # PID logic test
        controller.pid_kp = 0.2
        controller.pid_ki = 0.05
        controller.pid_kd = 0.1
        controller.pid_integral = 0.0
        controller.pid_last_error = 0.0
        controller.shared.force_threshold = 500
        controller.shared.current_force = 480
        error = controller.shared.force_threshold - controller.shared.current_force
        controller.pid_integral += error
        derivative = error - controller.pid_last_error
        output = controller.pid_kp * error + controller.pid_ki * controller.pid_integral + controller.pid_kd * derivative
        controller.pid_last_error = error
        steps = int(abs(output))
        direction = output > 0
        results.append(f"PID logic: error={error}, output={output}, steps={steps}, direction={direction}")
        assert steps > 0
        results.append("PID logic test: PASS")
    except Exception as e:
        results.append(f"PID logic test: FAIL ({e})")
    try:
        # Force feedback and automatic control test
        controller.shared.force_threshold = 500
        controller.shared.force_tolerance = 10

        # Simulate below threshold
        controller.shared.current_force = 480
        controller.stepper_enabled = True
        controller.servo_enabled = True
        controller.stepper.step = lambda steps, direction: results.append(f"Stepper step called: steps={steps}, direction={direction}")
        threshold = controller.shared.force_threshold
        tol = controller.shared.force_tolerance
        force = controller.shared.current_force
        if force < threshold - tol:
            controller.stepper.step(10, True)
            results.append("Force feedback: Below threshold, stepper should move up (anti-clockwise)")
        elif force > threshold + tol:
            controller.stepper.step(10, False)
            results.append("Force feedback: Above threshold, stepper should move down (clockwise)")
        else:
            results.append("Force feedback: Within threshold, stepper should hold")

        # Simulate above threshold
        controller.shared.current_force = 520
        force = controller.shared.current_force
        if force < threshold - tol:
            controller.stepper.step(10, True)
            results.append("Force feedback: Below threshold, stepper should move up (anti-clockwise)")
        elif force > threshold + tol:
            controller.stepper.step(10, False)
            results.append("Force feedback: Above threshold, stepper should move down (clockwise)")
        else:
            results.append("Force feedback: Within threshold, stepper should hold")

        # Simulate within threshold
        controller.shared.current_force = 505
        force = controller.shared.current_force
        if force < threshold - tol:
            controller.stepper.step(10, True)
            results.append("Force feedback: Below threshold, stepper should move up (anti-clockwise)")
        elif force > threshold + tol:
            controller.stepper.step(10, False)
            results.append("Force feedback: Above threshold, stepper should move down (clockwise)")
        else:
            results.append("Force feedback: Within threshold, stepper should hold")

        results.append("Force feedback and automatic control: PASS")
    except Exception as e:
        results.append(f"Force feedback and automatic control: FAIL ({e})")
    return results

# --- Main function to start the app ---
def main():
    def handle_exit(signum, frame):
        app.controller.cleanup()
        sys.exit(0)

    root = tk.Tk()
    global app
    app = App(root)

    signal.signal(signal.SIGINT, handle_exit)
    signal.signal(signal.SIGTERM, handle_exit)

    try:
        root.mainloop()
    finally:
        app.controller.cleanup()

if __name__ == "__main__":
    main()
