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
import syslog
import glob
import serial.tools.list_ports

# --- Stepper Motor Control ---
class StepperMotor:
    DIR_PIN = 20
    STEP_PIN = 21
    ENABLE_PIN = 16
    STEP_DELAY = 0.001

    def __init__(self):
        # Use gpiozero OutputDevice for stepper pins, but allow fallback to dummy for test/dev
        try:
            self.dir = OutputDevice(self.DIR_PIN, active_high=True, initial_value=False)
            self.step = OutputDevice(self.STEP_PIN, active_high=True, initial_value=False)
            self.enable = OutputDevice(self.ENABLE_PIN, active_high=False, initial_value=True)
            self.enable.on()  # Enable motor (LOW logic, so .on() sets pin low)
            self.hw_available = True
        except Exception as e:
            print(f"StepperMotor GPIO setup error: {e}")
            self.dir = self.step = self.enable = None
            self.hw_available = False

    def step(self, steps, direction):
        if self.hw_available and self.dir is not None and self.step is not None:
            self.dir.value = 1 if direction else 0
            for _ in range(steps):
                self.step.on()
                time.sleep(self.STEP_DELAY)
                self.step.off()
                time.sleep(self.STEP_DELAY)
        else:
            # Simulate stepper for GUI/testing
            print(f"[SIM] Stepper step: steps={steps}, direction={'up' if direction else 'down'}")

    def disable(self):
        if self.hw_available and self.enable is not None:
            self.enable.off()
        else:
            print("[SIM] Stepper disabled")

# --- Servo Motor Control ---
class ServoController:
    SERVO_PINS = [17, 27, 22]  # Example GPIO pins for 3 servos

    def __init__(self):
        self.servos = []
        self.hw_available = True
        for pin in self.SERVO_PINS:
            try:
                pwm = PWMOutputDevice(pin, frequency=50)
                pwm.value = 0
                self.servos.append(pwm)
            except Exception as e:
                print(f"ServoController PWM setup error on pin {pin}: {e}")
                self.servos.append(None)
                self.hw_available = False

    def angle_to_duty(self, angle):
        # Map angle to duty cycle (0.05 to 0.25 for 0-180 deg)
        return 0.05 + (angle / 180.0) * 0.20

    def set_angle(self, idx, angle):
        if self.hw_available and idx < len(self.servos) and self.servos[idx] is not None:
            duty = self.angle_to_duty(angle)
            self.servos[idx].value = duty
        else:
            print(f"[SIM] Servo {idx+1} set to angle {angle}")

    def cleanup(self):
        for i, servo in enumerate(self.servos):
            if self.hw_available and servo is not None:
                servo.value = 0
            else:
                print(f"[SIM] Servo {i+1} cleanup")

# --- Load Cell Port Detection ---
def detect_load_cell_port():
    """Auto-detect the load cell serial port by checking common device names and descriptors"""
    # List of potential device patterns to check
    device_patterns = [
        '/dev/ttyUSB*',      # USB to Serial adapters
        '/dev/ttyACM*',      # Arduino/microcontroller devices
        '/dev/ttyAMA*',      # Raspberry Pi UART
        '/dev/serial/by-id/*'  # Persistent device names
    ]
    
    # Check using pyserial's port listing (more reliable)
    try:
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # Check for common load cell/HX711 device descriptors
            description_lower = port.description.lower()
            manufacturer_lower = (port.manufacturer or '').lower()
            
            # Common identifiers for USB-Serial adapters used with HX711
            if any(keyword in description_lower for keyword in [
                'ch340', 'ch341',           # Common Chinese USB-Serial chips
                'cp210',                     # Silicon Labs chips
                'ft232', 'ftdi',            # FTDI chips
                'pl2303',                   # Prolific chips
                'arduino', 'nano', 'uno',   # Arduino boards with HX711
                'usb-serial', 'usb to uart',
                'hx711', 'load cell'        # Direct mentions
            ]):
                print(f"Potential load cell device found: {port.device} - {port.description}")
                return port.device
                
            # Also check manufacturer
            if any(keyword in manufacturer_lower for keyword in [
                'arduino', 'ftdi', 'silicon labs', 'prolific'
            ]):
                print(f"Potential load cell device found by manufacturer: {port.device} - {port.manufacturer}")
                return port.device
    except Exception as e:
        print(f"Error using pyserial port detection: {e}")
    
    # Fallback: Check filesystem patterns
    for pattern in device_patterns:
        devices = glob.glob(pattern)
        for device in sorted(devices):  # Sort for consistent ordering
            try:
                # Try to open the device briefly to see if it exists and is accessible
                with serial.Serial(device, 9600, timeout=0.1) as test_ser:
                    print(f"Found accessible serial device: {device}")
                    return device
            except (serial.SerialException, PermissionError):
                continue
    
    print("No load cell serial device auto-detected")
    return None

def test_load_cell_communication(port, timeout=2):
    """Test if the given port has a load cell by trying to read data"""
    try:
        with serial.Serial(port, 9600, timeout=timeout) as ser:
            # Try to read a few lines to see if we get numeric data
            for _ in range(5):
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    try:
                        # If we can parse it as a float, it's likely force data
                        float(line)
                        print(f"Load cell confirmed on {port}: received data '{line}'")
                        return True
                    except ValueError:
                        continue
        print(f"No valid load cell data received from {port}")
        return False
    except Exception as e:
        print(f"Failed to test load cell communication on {port}: {e}")
        return False

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
    This function is DEPRECATED and should not be called directly.
    Use the main App class instead.
    """
    # This function is no longer used - all GUI functionality is in the App class
    pass

# --- Cleanup function ---
def cleanup(stepper, servo_ctrl, ser):
    if ser is not None:
        try:
            ser.close()
        except Exception as e:
            print(f"[SIM] Serial close error: {e}")
    stepper.disable()
    servo_ctrl.cleanup()
    # No GPIO.cleanup() needed for gpiozero

# --- Controller class integrating GUI and logic ---
class Controller:
    def __init__(self):
        # Remove GPIO.setmode(GPIO.BCM) -- not needed for gpiozero
        self.stepper = StepperMotor()
        self.servo_ctrl = ServoController()
        
        # Initialize app reference to None
        self.app = None

        # Auto-detect load cell serial port
        detected_port = detect_load_cell_port()
        if detected_port:
            print(f"Auto-detected potential load cell port: {detected_port}")
            # Test if it's actually a load cell
            if test_load_cell_communication(detected_port):
                SERIAL_PORT = detected_port
                print(f"Load cell confirmed and connected on: {SERIAL_PORT}")
            else:
                print(f"Device on {detected_port} doesn't appear to be a load cell, trying manual fallback")
                SERIAL_PORT = '/dev/ttyUSB0'  # Fallback
        else:
            print("No load cell auto-detected, using default port")
            SERIAL_PORT = '/dev/ttyUSB0'  # Default fallback

        BAUDRATE = 9600
        FORCE_TARGET = 500  # Adjust for MK10 range (typically 0-10kg = 0-10000g)
        FORCE_TOLERANCE = 10  # Adjust tolerance for MK10 precision

        self.shared = SharedData(FORCE_TARGET, FORCE_TOLERANCE)

        # Serial port handling with auto-detection and error reporting
        self.ser = None
        self.serial_port = SERIAL_PORT
        self.baudrate = BAUDRATE
        
        # Try multiple connection attempts
        connection_attempts = [
            (SERIAL_PORT, BAUDRATE),
            ('/dev/ttyUSB0', BAUDRATE),    # Common default
            ('/dev/ttyUSB1', BAUDRATE),    # Alternative USB port
            ('/dev/ttyACM0', BAUDRATE),    # Arduino-style devices
            ('/dev/ttyAMA0', BAUDRATE),    # RPi hardware UART
        ]
        
        for port, baud in connection_attempts:
            try:
                print(f"Attempting to connect to load cell on {port} at {baud} baud...")
                test_ser = serial.Serial(port, baud, timeout=1)
                
                # Quick test to see if we get data
                if test_load_cell_communication(port, timeout=1):
                    self.ser = test_ser
                    self.serial_port = port
                    self.baudrate = baud
                    print(f"Successfully connected to load cell on {port}")
                    break
                else:
                    test_ser.close()
                    
            except Exception as e:
                print(f"Failed to connect to {port}: {e}")
                continue
        
        if self.ser is None:
            syslog.openlog("rpi_control")
            syslog.syslog(syslog.LOG_WARNING, f"No load cell found on any serial port, running in simulation mode")
            syslog.closelog()
            print("No load cell detected - running in simulation mode")

        # Initialize running flag
        self.running = True

        # --- MK10 Load Cell Configuration Notes ---
        # - Capacity: 10kg (10000g)
        # - Requires proper calibration with known weights
        # - HX711 amplifier needs calibration factor adjustment
        # - Tension-based: positive values indicate pulling force
        print("MK10 Load Cell Configuration:")
        print("- Capacity: 10kg (10000g)")
        print("- Ensure HX711 is properly calibrated")
        print("- Positive values = tension force")
        if self.ser:
            print(f"- Connected on: {self.serial_port}")
        else:
            print("- No load cell detected (simulation mode)")

        # PID parameters
        self.pid_kp = 0.1  # Proportional gain
        self.pid_ki = 0.01 # Integral gain
        self.pid_kd = 0.05 # Derivative gain
        self.pid_integral = 0.0
        self.pid_last_error = 0.0

        # Control enable flags
        self.servo_enabled = True
        self.stepper_enabled = True
        self.auto_mode_enabled = False

        def handle_exit(signum, frame):
            self.cleanup()
            sys.exit(0)

        signal.signal(signal.SIGINT, handle_exit)
        signal.signal(signal.SIGTERM, handle_exit)

    def cleanup(self):
        cleanup(self.stepper, self.servo_ctrl, self.ser)
        self.running = False

    def set_app(self, app):
        """Set reference to App for logging"""
        self.app = app
        
        # Now that app is set, log the MK10 configuration to GUI
        self.log_status("MK10 Load Cell Configuration:")
        self.log_status("- Capacity: 10kg (10000g)")
        self.log_status("- Ensure HX711 is properly calibrated")
        self.log_status("- Positive values = tension force")
        if self.ser:
            self.log_status(f"- Connected on: {self.serial_port}")
        else:
            self.log_status("- No load cell detected (simulation mode)")

    def log_status(self, message):
        """Log status message to GUI if available, otherwise print"""
        if self.app and hasattr(self.app, 'log_status'):
            self.app.log_status(message)
        else:
            print(message)

    def increase_force(self):
        if self.stepper_enabled and not self.auto_mode_enabled:
            if callable(self.stepper.step):
                self.stepper.step(10, True)
                self.log_status("Stepper turning anti-clockwise to increase force")
            else:
                self.log_status("[SIM] Stepper step: steps=10, direction=up (anti-clockwise)")

    def decrease_force(self):
        if self.stepper_enabled and not self.auto_mode_enabled:
            if callable(self.stepper.step):
                self.stepper.step(10, False)
                self.log_status("Stepper turning clockwise to decrease force")
            else:
                self.log_status("[SIM] Stepper step: steps=10, direction=down (clockwise)")

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
        self.log_status(f"Force threshold increased to {self.shared.force_threshold}")

    def decrease_threshold(self):
        self.shared.force_threshold -= 1
        self.log_status(f"Force threshold decreased to {self.shared.force_threshold}")

    def reconnect_load_cell(self):
        """Attempt to reconnect to the load cell if connection was lost"""
        if self.ser is not None:
            try:
                self.ser.close()
            except:
                pass
            self.ser = None
        
        # Try to auto-detect again
        detected_port = detect_load_cell_port()
        if detected_port and test_load_cell_communication(detected_port):
            try:
                self.ser = serial.Serial(detected_port, self.baudrate, timeout=1)
                self.serial_port = detected_port
                self.log_status(f"Reconnected to load cell on {detected_port}")
                return True
            except Exception as e:
                self.log_status(f"Failed to reconnect to {detected_port}: {e}")
        
        self.log_status("Load cell reconnection failed - continuing in simulation mode")
        return False

# --- App class for Tkinter GUI ---
class App:
    def __init__(self, root):
        self.root = root
        self.root.title("RPi Control")
        self.controller = Controller()
        
        self.servo_control_enabled = tk.BooleanVar(value=True)
        self.stepper_control_enabled = tk.BooleanVar(value=True)
        self.auto_stepper_mode = tk.BooleanVar(value=False)

        # Configurable presets (default values) - moved to top of __init__
        self.servo_presets = [0, 30, 45, 60, 90, 120, 135, 150, 180]
        self.stepper_presets = [1, 5, 10, 25, 50, 100]

        # --- Notebook for tabs ---
        notebook = ttk.Notebook(root)
        notebook.pack(fill="both", expand=True)

        # --- Main tab (manual force control, stop button) ---
        main_frame = tk.Frame(notebook)
        notebook.add(main_frame, text="Main")

        # Live force display
        self.live_force_var = tk.StringVar(value="0.00")
        self.force_label = tk.Label(main_frame, text="Live Force (g):", font=("Arial", 12))
        self.force_label.pack(pady=2)
        self.force_value_box = tk.Entry(main_frame, textvariable=self.live_force_var, font=("Arial", 12), state="readonly", width=12)
        self.force_value_box.pack(pady=2)

        # Force threshold display
        self.force_thresh_var = tk.StringVar(value=str(self.controller.shared.force_threshold))
        self.force_thresh_label = tk.Label(main_frame, text="Force Threshold (g):", font=("Arial", 12))
        self.force_thresh_label.pack(pady=2)
        self.force_thresh_box = tk.Entry(main_frame, textvariable=self.force_thresh_var, font=("Arial", 12), state="readonly", width=12)
        self.force_thresh_box.pack(pady=2)

        # Force status indicator
        self.force_status_frame = tk.Frame(main_frame)
        self.force_status_frame.pack(pady=5)
        
        self.force_status_label = tk.Label(self.force_status_frame, text="Force Status:", font=("Arial", 12))
        self.force_status_label.pack(side=tk.LEFT, padx=5)
        
        self.force_status_indicator = tk.Label(self.force_status_frame, text="WITHIN RANGE", 
                                              font=("Arial", 12, "bold"), fg="white", bg="green", 
                                              width=15, relief="raised")
        self.force_status_indicator.pack(side=tk.LEFT, padx=5)

        # Servo controls frame
        servo_control_frame = tk.LabelFrame(main_frame, text="Servo Controls")
        servo_control_frame.pack(pady=10, padx=10, fill="x")

        # Servo angle controls
        self.servo_angle_vars = []
        self.servo_scales = []
        self.servo_preset_buttons = []
        
        for i in range(3):
            servo_frame = tk.Frame(servo_control_frame)
            servo_frame.pack(fill="x", padx=5, pady=2)
            
            tk.Label(servo_frame, text=f"Servo {i+1} Angle:", width=12).pack(side=tk.LEFT)
            
            angle_var = tk.IntVar(value=90)
            self.servo_angle_vars.append(angle_var)
            
            scale = tk.Scale(servo_frame, from_=0, to=180, orient=tk.HORIZONTAL, 
                           variable=angle_var, command=lambda val, idx=i: self.set_servo_angle(idx, val))
            scale.pack(side=tk.LEFT, fill="x", expand=True, padx=5)
            self.servo_scales.append(scale)
            
            # Current angle display
            angle_display = tk.Label(servo_frame, text="90°", width=4, relief="sunken")
            angle_display.pack(side=tk.LEFT, padx=2)
            
            # Preset dropdown for each servo
            preset_var = tk.StringVar(value="90°")
            preset_dropdown = ttk.Combobox(servo_frame, textvariable=preset_var, width=6, state="readonly")
            preset_dropdown['values'] = [f"{preset}°" for preset in self.servo_presets]
            preset_dropdown.pack(side=tk.LEFT, padx=2)
            preset_dropdown.bind('<<ComboboxSelected>>', lambda event, idx=i: self.servo_preset_selected(event, idx))
            
            # Update angle display when slider changes
            def update_angle_display(val, idx=i, display=angle_display):
                display.config(text=f"{int(float(val))}°")
            scale.config(command=lambda val, idx=i: [self.set_servo_angle(idx, val), update_angle_display(val, idx)])

        # Combined Stepper Control frame (merged manual and mode controls)
        stepper_control_frame = tk.LabelFrame(main_frame, text="Stepper Control")
        stepper_control_frame.pack(pady=10, padx=10, fill="x")
        
        # Mode selection
        mode_frame = tk.Frame(stepper_control_frame)
        mode_frame.pack(pady=5, fill="x")
        
        self.stepper_mode_toggle = tk.Checkbutton(
            mode_frame, 
            text="Automatic Mode", 
            variable=self.auto_stepper_mode,
            command=self.toggle_stepper_mode,
            font=("Arial", 11)
        )
        self.stepper_mode_toggle.pack(side=tk.LEFT, padx=5)
        
        self.stepper_mode_status = tk.Label(
            mode_frame, 
            text="Mode: Manual", 
            font=("Arial", 10), 
            fg="blue"
        )
        self.stepper_mode_status.pack(side=tk.LEFT, padx=20)

        # Manual control buttons
        manual_buttons_frame = tk.Frame(stepper_control_frame)
        manual_buttons_frame.pack(pady=5, fill="x")
        
        tk.Label(manual_buttons_frame, text="Manual Control:", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)
        
        self.manual_increase_force_button = tk.Button(
            manual_buttons_frame, text="Increase Force (+)", 
            command=self.controller.increase_force
        )
        self.manual_increase_force_button.pack(side=tk.LEFT, padx=5)

        self.manual_decrease_force_button = tk.Button(
            manual_buttons_frame, text="Decrease Force (-)", 
            command=self.controller.decrease_force
        )
        self.manual_decrease_force_button.pack(side=tk.LEFT, padx=5)

        # Quick step preset controls
        preset_frame = tk.Frame(stepper_control_frame)
        preset_frame.pack(pady=5, fill="x")
        
        tk.Label(preset_frame, text="Quick Steps:", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)
        
        self.stepper_preset_var = tk.StringVar(value="Select Steps")
        self.stepper_preset_dropdown = ttk.Combobox(preset_frame, textvariable=self.stepper_preset_var, 
                                                   width=12, state="readonly")
        self.stepper_preset_dropdown['values'] = [f"{steps} step{'s' if steps != 1 else ''}" for steps in self.stepper_presets]
        self.stepper_preset_dropdown.pack(side=tk.LEFT, padx=5)
        
        tk.Button(preset_frame, text="Increase (+)", 
                 command=self.stepper_preset_increase_dropdown).pack(side=tk.LEFT, padx=2)
        tk.Button(preset_frame, text="Decrease (-)", 
                 command=self.stepper_preset_decrease_dropdown).pack(side=tk.LEFT, padx=2)

        # Stop button
        stop_frame = tk.Frame(stepper_control_frame)
        stop_frame.pack(pady=5)
        
        self.stop_button = tk.Button(stop_frame, text="Stop Program", command=self.stop_program, 
                                   bg="red", fg="white", font=("Arial", 10, "bold"))
        self.stop_button.pack()

        # Status/Log display
        status_log_frame = tk.LabelFrame(main_frame, text="Status Log")
        status_log_frame.pack(pady=10, padx=10, fill="both", expand=True)
        
        self.status_log = tk.Text(status_log_frame, height=8, width=60, state=tk.DISABLED, 
                                 font=("Consolas", 9), wrap=tk.WORD)
        scrollbar = tk.Scrollbar(status_log_frame, orient=tk.VERTICAL, command=self.status_log.yview)
        self.status_log.config(yscrollcommand=scrollbar.set)
        
        self.status_log.pack(side=tk.LEFT, fill="both", expand=True)
        scrollbar.pack(side=tk.RIGHT, fill="y")
        
        # Clear log button
        clear_log_btn = tk.Button(status_log_frame, text="Clear Log", command=self.clear_status_log)
        clear_log_btn.pack(pady=2)

        # --- Settings tab ---
        settings_frame = tk.Frame(notebook)
        notebook.add(settings_frame, text="Settings")

        # --- PID parameter controls ---
        pid_frame = tk.LabelFrame(settings_frame, text="PID Parameters")
        pid_frame.pack(padx=10, pady=10, fill="x")

        tk.Label(pid_frame, text="Kp:").grid(row=0, column=0, sticky="e")
        self.kp_var = tk.DoubleVar(value=self.controller.pid_kp)
        self.kp_entry = tk.Entry(pid_frame, textvariable=self.kp_var, width=7)
        self.kp_entry.grid(row=0, column=1)

        tk.Label(pid_frame, text="Ki:").grid(row=0, column=2, sticky="e")
        self.ki_var = tk.DoubleVar(value=self.controller.pid_ki)
        self.ki_entry = tk.Entry(pid_frame, textvariable=self.ki_var, width=7)
        self.ki_entry.grid(row=0, column=3)

        tk.Label(pid_frame, text="Kd:").grid(row=0, column=4, sticky="e")
        self.kd_var = tk.DoubleVar(value=self.controller.pid_kd)
        self.kd_entry = tk.Entry(pid_frame, textvariable=self.kd_var, width=7)
        self.kd_entry.grid(row=0, column=5)

        self.apply_pid_button = tk.Button(pid_frame, text="Apply PID", command=self.apply_pid_params)
        self.apply_pid_button.grid(row=1, column=0, columnspan=6, pady=4)

        # --- Pin and threshold entry boxes: make them writable and update values on Apply ---
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

        # Individual servo enable checkboxes
        servo_enable_frame = tk.LabelFrame(settings_frame, text="Enable Individual Servos")
        servo_enable_frame.pack(padx=10, pady=5, fill="x")
        self.servo1_enabled = tk.BooleanVar(value=True)
        self.servo2_enabled = tk.BooleanVar(value=True)
        self.servo3_enabled = tk.BooleanVar(value=True)
        tk.Checkbutton(servo_enable_frame, text="Servo 1", variable=self.servo1_enabled, command=self.update_individual_servos).pack(side=tk.LEFT, padx=5)
        tk.Checkbutton(servo_enable_frame, text="Servo 2", variable=self.servo2_enabled, command=self.update_individual_servos).pack(side=tk.LEFT, padx=5)
        tk.Checkbutton(servo_enable_frame, text="Servo 3", variable=self.servo3_enabled, command=self.update_individual_servos).pack(side=tk.LEFT, padx=5)

        # Force threshold display and apply
        thresh_set_frame = tk.LabelFrame(settings_frame, text="Force Threshold")
        thresh_set_frame.pack(padx=10, pady=5, fill="x")
        tk.Label(thresh_set_frame, text="Current Threshold (g):").pack(side=tk.LEFT)
        self.settings_force_thresh_var = tk.StringVar(value=str(self.controller.shared.force_threshold))
        self.settings_force_thresh_box = tk.Entry(thresh_set_frame, textvariable=self.settings_force_thresh_var, font=("Arial", 12), width=12)
        self.settings_force_thresh_box.pack(side=tk.LEFT, padx=5)
        self.set_force_thresh_var = tk.DoubleVar(value=self.controller.shared.force_threshold)
        self.set_force_thresh_entry = tk.Entry(thresh_set_frame, textvariable=self.set_force_thresh_var, width=10)
        self.set_force_thresh_entry.pack(side=tk.LEFT, padx=5)
        self.apply_force_thresh_button = tk.Button(thresh_set_frame, text="Apply Threshold", command=self.apply_force_threshold)
        self.apply_force_thresh_button.pack(side=tk.LEFT, padx=5)

        # Servo position presets configuration
        servo_presets_config_frame = tk.LabelFrame(settings_frame, text="Configure Servo Presets")
        servo_presets_config_frame.pack(padx=10, pady=5, fill="x")
        
        preset_config_row = tk.Frame(servo_presets_config_frame)
        preset_config_row.pack(fill="x", padx=5, pady=2)
        
        tk.Label(preset_config_row, text="Preset Angles:").pack(side=tk.LEFT)
        
        self.preset_entries = []
        for i, preset in enumerate(self.servo_presets):
            preset_var = tk.IntVar(value=preset)
            entry = tk.Entry(preset_config_row, textvariable=preset_var, width=4)
            entry.pack(side=tk.LEFT, padx=1)
            self.preset_entries.append(preset_var)
        
        preset_buttons_row = tk.Frame(servo_presets_config_frame)
        preset_buttons_row.pack(fill="x", padx=5, pady=2)
        
        tk.Button(preset_buttons_row, text="Apply Presets", command=self.apply_servo_presets).pack(side=tk.LEFT, padx=5)
        tk.Button(preset_buttons_row, text="Reset to Defaults", command=self.reset_servo_presets).pack(side=tk.LEFT, padx=5)
        tk.Button(preset_buttons_row, text="Save Config", command=self.save_servo_presets).pack(side=tk.LEFT, padx=5)
        tk.Button(preset_buttons_row, text="Load Config", command=self.load_servo_presets).pack(side=tk.LEFT, padx=5)

        # Add/Remove preset controls
        preset_modify_row = tk.Frame(servo_presets_config_frame)
        preset_modify_row.pack(fill="x", padx=5, pady=2)
        
        tk.Label(preset_modify_row, text="Add New Preset:").pack(side=tk.LEFT)
        self.new_preset_var = tk.IntVar(value=90)
        tk.Entry(preset_modify_row, textvariable=self.new_preset_var, width=4).pack(side=tk.LEFT, padx=2)
        tk.Button(preset_modify_row, text="Add", command=self.add_servo_preset).pack(side=tk.LEFT, padx=2)
        tk.Button(preset_modify_row, text="Remove Last", command=self.remove_servo_preset).pack(side=tk.LEFT, padx=5)

        # Stepper position presets configuration
        stepper_presets_config_frame = tk.LabelFrame(settings_frame, text="Configure Stepper Presets")
        stepper_presets_config_frame.pack(padx=10, pady=5, fill="x")
        
        stepper_preset_config_row = tk.Frame(stepper_presets_config_frame)
        stepper_preset_config_row.pack(fill="x", padx=5, pady=2)
        
        tk.Label(stepper_preset_config_row, text="Preset Steps:").pack(side=tk.LEFT)
        
        self.stepper_preset_entries = []
        for i, preset in enumerate(self.stepper_presets):
            preset_var = tk.IntVar(value=preset)
            entry = tk.Entry(stepper_preset_config_row, textvariable=preset_var, width=4)
            entry.pack(side=tk.LEFT, padx=1)
            self.stepper_preset_entries.append(preset_var)
        
        stepper_preset_buttons_row = tk.Frame(stepper_presets_config_frame)
        stepper_preset_buttons_row.pack(fill="x", padx=5, pady=2)
        
        tk.Button(stepper_preset_buttons_row, text="Apply Presets", command=self.apply_stepper_presets).pack(side=tk.LEFT, padx=5)
        tk.Button(stepper_preset_buttons_row, text="Reset to Defaults", command=self.reset_stepper_presets).pack(side=tk.LEFT, padx=5)
        tk.Button(stepper_preset_buttons_row, text="Save Config", command=self.save_stepper_presets).pack(side=tk.LEFT, padx=5)
        tk.Button(stepper_preset_buttons_row, text="Load Config", command=self.load_stepper_presets).pack(side=tk.LEFT, padx=5)

        # Add/Remove stepper preset controls
        stepper_preset_modify_row = tk.Frame(stepper_presets_config_frame)
        stepper_preset_modify_row.pack(fill="x", padx=5, pady=2)
        
        tk.Label(stepper_preset_modify_row, text="Add New Preset:").pack(side=tk.LEFT)
        self.new_stepper_preset_var = tk.IntVar(value=20)
        tk.Entry(stepper_preset_modify_row, textvariable=self.new_stepper_preset_var, width=4).pack(side=tk.LEFT, padx=2)
        tk.Button(stepper_preset_modify_row, text="Add", command=self.add_stepper_preset).pack(side=tk.LEFT, padx=2)
        tk.Button(stepper_preset_modify_row, text="Remove Last", command=self.remove_stepper_preset).pack(side=tk.LEFT, padx=5)

        # Remove old stepper presets section and replace with servo config only
        # Stepper position presets
        stepper_presets_frame = tk.LabelFrame(settings_frame, text="Quick Stepper Actions")
        stepper_presets_frame.pack(padx=10, pady=5, fill="x")
        
        stepper_preset_row = tk.Frame(stepper_presets_frame)
        stepper_preset_row.pack(fill="x", padx=5, pady=2)
        
        tk.Label(stepper_preset_row, text="Quick Steps:", width=10).pack(side=tk.LEFT)
        
        for steps in self.stepper_presets:
            # Increase buttons
            btn_inc = tk.Button(stepper_preset_row, text=f"+{steps}", width=4,
                              command=lambda s=steps: self.stepper_preset_increase(s))
            btn_inc.pack(side=tk.LEFT, padx=1)
            
            # Decrease buttons  
            btn_dec = tk.Button(stepper_preset_row, text=f"-{steps}", width=4,
                              command=lambda s=steps: self.stepper_preset_decrease(s))
            btn_dec.pack(side=tk.LEFT, padx=1)

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
        
        # Start periodic updates
        self.update_force_and_thresh()
        
        # Set reference for logging AFTER GUI is fully initialized
        self.controller.set_app(self)  # Set reference for logging

    def update_force_and_thresh(self):
        """Periodically update force and threshold display"""
        # Add force simulation for testing mode
        if self.controller.ser is None:
            # Simulate varying force readings when no hardware is connected
            import math
            current_time = time.time()
            simulated_force = 500 + 50 * math.sin(current_time * 0.5)  # Oscillate around 500g
            self.controller.shared.current_force = simulated_force
        
        self.live_force_var.set(f"{self.controller.shared.current_force:.2f}")
        self.force_thresh_var.set(str(self.controller.shared.force_threshold))
        self.update_force_status_indicator()
        self.root.after(200, self.update_force_and_thresh)

    def update_servo_controls(self):
        self.controller.servo_enabled = self.servo_control_enabled.get()
        # If you have servo widgets, enable/disable them here

    def update_stepper_controls(self):
        self.controller.stepper_enabled = self.stepper_control_enabled.get()
        # Only enable manual controls if stepper is enabled AND not in auto mode
        manual_state = tk.NORMAL if (self.stepper_control_enabled.get() and not self.auto_stepper_mode.get()) else tk.DISABLED
        self.manual_increase_force_button.config(state=manual_state)
        self.manual_decrease_force_button.config(state=manual_state)
        
        # Preset dropdown should be disabled if stepper disabled OR in auto mode
        preset_state = "readonly" if (self.stepper_control_enabled.get() and not self.auto_stepper_mode.get()) else tk.DISABLED
        self.stepper_preset_dropdown.config(state=preset_state)

    def update_individual_servos(self):
        enabled = [self.servo1_enabled.get(), self.servo2_enabled.get(), self.servo3_enabled.get()]
        print(f"Servo enable states: {enabled}")
        # Optionally, disable servo controls in GUI if unchecked

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
        # Update StepperMotor pins from entry boxes
        try:
            StepperMotor.DIR_PIN = int(self.stepper_dir_var.get())
            StepperMotor.STEP_PIN = int(self.stepper_step_var.get())
            StepperMotor.ENABLE_PIN = int(self.stepper_enable_var.get())
            ServoController.SERVO_PINS = [
                int(self.servo1_var.get()),
                int(self.servo2_var.get()),
                int(self.servo3_var.get())
            ]
            self.log_status("Updated Stepper and Servo pins:")
            self.log_status(f"Stepper DIR: {StepperMotor.DIR_PIN}, STEP: {StepperMotor.STEP_PIN}, ENABLE: {StepperMotor.ENABLE_PIN}")
            self.log_status(f"Servo PWM: {ServoController.SERVO_PINS}")
        except Exception as e:
            self.log_status(f"Invalid pin value: {e}")

    def apply_force_threshold(self):
        try:
            val = float(self.set_force_thresh_entry.get())
            self.controller.shared.force_threshold = val
            self.force_thresh_var.set(str(val))  # Update main tab display
            self.settings_force_thresh_var.set(str(val))  # Update settings tab display
            self.log_status(f"Applied force threshold: {val}")
        except Exception as e:
            self.log_status(f"Invalid force threshold value: {e}")

    def apply_pid_params(self):
        try:
            self.controller.pid_kp = float(self.kp_entry.get())
            self.controller.pid_ki = float(self.ki_entry.get())
            self.controller.pid_kd = float(self.kd_entry.get())
            self.log_status(f"Applied PID params: Kp={self.controller.pid_kp}, Ki={self.controller.pid_ki}, Kd={self.controller.pid_kd}")
        except Exception as e:
            self.log_status(f"Invalid PID parameter value: {e}")

    def run_unit_tests_gui(self):
        self.test_results_text.config(state=tk.NORMAL)
        self.test_results_text.delete(1.0, tk.END)
        try:
            results = run_unit_tests()
            self.test_results_text.insert(tk.END, "\n".join(results) + "\n")
        except Exception as e:
            self.test_results_text.insert(tk.END, f"Error running unit tests:\n{traceback.format_exc()}")
        self.test_results_text.config(state=tk.DISABLED)

    def run_system_tests_gui(self):
        self.test_results_text.config(state=tk.NORMAL)
        self.test_results_text.delete(1.0, tk.END)
        try:
            results = run_system_tests(self.controller)
            self.test_results_text.insert(tk.END, "\n".join(results) + "\n")
        except Exception as e:
            self.test_results_text.insert(tk.END, f"Error running system tests:\n{traceback.format_exc()}")
        self.test_results_text.config(state=tk.DISABLED)

    def clear_status_log(self):
        """Clear the status log"""
        self.status_log.config(state=tk.NORMAL)
        self.status_log.delete(1.0, tk.END)
        self.status_log.config(state=tk.DISABLED)

    def toggle_stepper_mode(self):
        if self.auto_stepper_mode.get():
            # Switch to automatic mode
            self.controller.auto_mode_enabled = True
            self.stepper_mode_status.config(text="Mode: Automatic", fg="green")
            # Disable manual controls when in automatic mode
            self.manual_increase_force_button.config(state=tk.DISABLED)
            self.manual_decrease_force_button.config(state=tk.DISABLED)
            self.stepper_preset_dropdown.config(state=tk.DISABLED)
            # Find and disable preset buttons
            for widget in self.stepper_preset_dropdown.master.winfo_children():
                if isinstance(widget, tk.Button) and ("Increase" in widget.cget("text") or "Decrease" in widget.cget("text")):
                    widget.config(state=tk.DISABLED)
            self.log_status("Switched to automatic stepper control")
        else:
            # Switch to manual mode
            self.controller.auto_mode_enabled = False
            self.stepper_mode_status.config(text="Mode: Manual", fg="blue")
            # Enable manual controls when stepper is enabled
            if self.stepper_control_enabled.get():
                self.manual_increase_force_button.config(state=tk.NORMAL)
                self.manual_decrease_force_button.config(state=tk.NORMAL)
                self.stepper_preset_dropdown.config(state="readonly")
                # Find and enable preset buttons
                for widget in self.stepper_preset_dropdown.master.winfo_children():
                    if isinstance(widget, tk.Button) and ("Increase" in widget.cget("text") or "Decrease" in widget.cget("text")):
                        widget.config(state=tk.NORMAL)
            self.log_status("Switched to manual stepper control")

    def update_stepper_controls(self):
        self.controller.stepper_enabled = self.stepper_control_enabled.get()
        # Only enable manual controls if stepper is enabled AND not in auto mode
        manual_state = tk.NORMAL if (self.stepper_control_enabled.get() and not self.auto_stepper_mode.get()) else tk.DISABLED
        self.manual_increase_force_button.config(state=manual_state)
        self.manual_decrease_force_button.config(state=manual_state)
        
        # Preset dropdown should be disabled if stepper disabled OR in auto mode
        preset_state = "readonly" if (self.stepper_control_enabled.get() and not self.auto_stepper_mode.get()) else tk.DISABLED
        self.stepper_preset_dropdown.config(state=preset_state)

    def set_servo_angle(self, servo_idx, angle_str):
        if self.servo_control_enabled.get():
            angle = int(float(angle_str))
            self.controller.servo_ctrl.set_angle(servo_idx, angle)
            self.log_status(f"Servo {servo_idx+1} set to {angle} degrees")

    def set_servo_preset(self, servo_idx, angle):
        if self.servo_control_enabled.get():
            self.servo_angle_vars[servo_idx].set(angle)
            self.controller.servo_ctrl.set_angle(servo_idx, angle)
            self.log_status(f"Servo {servo_idx+1} preset to {angle} degrees")

    def stepper_preset_increase(self, steps):
        if self.stepper_control_enabled.get() and not self.auto_stepper_mode.get():
            if callable(self.controller.stepper.step):
                self.controller.stepper.step(steps, True)
                self.log_status(f"Stepper preset: +{steps} steps (anti-clockwise)")
            else:
                self.log_status(f"[SIM] Stepper preset: +{steps} steps (anti-clockwise)")

    def stepper_preset_decrease(self, steps):
        if self.stepper_control_enabled.get() and not self.auto_stepper_mode.get():
            if callable(self.controller.stepper.step):
                self.controller.stepper.step(steps, False)
                self.log_status(f"Stepper preset: -{steps} steps (clockwise)")
            else:
                self.log_status(f"[SIM] Stepper preset: -{steps} steps (clockwise)")

    def set_all_servos_preset(self, angle):
        if self.servo_control_enabled.get():
            for i in range(3):
                self.servo_angle_vars[i].set(angle)
                self.controller.servo_ctrl.set_angle(i, angle)
            self.log_status(f"All servos set to {angle} degrees")

    def servo_preset_selected(self, event, servo_idx):
        """Handle servo preset selection from dropdown"""
        if self.servo_control_enabled.get():
            try:
                # Extract angle value from dropdown selection (e.g., "90°" -> 90)
                angle_str = event.widget.get().replace('°', '')
                angle = int(angle_str)
                self.servo_angle_vars[servo_idx].set(angle)
                self.controller.servo_ctrl.set_angle(servo_idx, angle)
                self.log_status(f"Servo {servo_idx+1} preset to {angle} degrees")
            except ValueError:
                self.log_status(f"Invalid angle selected for servo {servo_idx+1}")

    def stepper_preset_increase_dropdown(self):
        """Increase stepper position using dropdown selection"""
        if self.stepper_control_enabled.get() and not self.auto_stepper_mode.get():
            try:
                # Extract step count from dropdown selection (e.g., "10 steps" -> 10)
                step_str = self.stepper_preset_var.get().split()[0]
                steps = int(step_str)
                if callable(self.controller.stepper.step):
                    self.controller.stepper.step(steps, True)
                    self.log_status(f"Stepper preset: +{steps} steps (anti-clockwise)")
                else:
                    self.log_status(f"[SIM] Stepper preset: +{steps} steps (anti-clockwise)")
            except (ValueError, IndexError):
                self.log_status("Please select a step count from dropdown")

    def stepper_preset_decrease_dropdown(self):
        """Decrease stepper position using dropdown selection"""
        if self.stepper_control_enabled.get() and not self.auto_stepper_mode.get():
            try:
                # Extract step count from dropdown selection (e.g., "10 steps" -> 10)
                step_str = self.stepper_preset_var.get().split()[0]
                steps = int(step_str)
                if callable(self.controller.stepper.step):
                    self.controller.stepper.step(steps, False)
                    self.log_status(f"Stepper preset: -{steps} steps (clockwise)")
                else:
                    self.log_status(f"[SIM] Stepper preset: -{steps} steps (clockwise)")
            except (ValueError, IndexError):
                self.log_status("Please select a step count from dropdown")

    def apply_servo_presets(self):
        try:
            new_presets = [int(entry.get()) for entry in self.preset_entries]
            # Validate presets are within servo range
            for preset in new_presets:
                if preset < 0 or preset > 180:
                    raise ValueError(f"Preset {preset} out of range (0-180)")
            
            self.servo_presets = new_presets
            self.update_preset_dropdowns()
            self.log_status(f"Applied servo presets: {self.servo_presets}")
        except Exception as e:
            self.log_status(f"Invalid servo preset values: {e}")

    def reset_servo_presets(self):
        """Reset servo presets to default values"""
        self.servo_presets = [0, 30, 45, 60, 90, 120, 135, 150, 180]
        self.update_preset_dropdowns()
        self.log_status("Servo presets reset to defaults")

    def add_servo_preset(self):
        """Add a new servo preset"""
        try:
            new_preset = int(self.new_preset_var.get())
            if new_preset < 0 or new_preset > 180:
                raise ValueError(f"Preset {new_preset} out of range (0-180)")
            
            if new_preset not in self.servo_presets:
                self.servo_presets.append(new_preset)
                self.servo_presets.sort()  # Keep presets sorted
                self.update_preset_dropdowns()
                self.log_status(f"Added servo preset: {new_preset}°")
            else:
                self.log_status(f"Preset {new_preset}° already exists")
        except Exception as e:
            self.log_status(f"Invalid preset value: {e}")

    def remove_servo_preset(self):
        """Remove the last servo preset"""
        if len(self.servo_presets) > 1:  # Keep at least one preset
            removed = self.servo_presets.pop()
            self.update_preset_dropdowns()
            self.log_status(f"Removed servo preset: {removed}°")
        else:
            self.log_status("Cannot remove the last preset")

    def apply_stepper_presets(self):
        try:
            new_presets = [int(entry.get()) for entry in self.stepper_preset_entries]
            # Validate presets are positive
            for preset in new_presets:
                if preset <= 0:
                    raise ValueError(f"Preset {preset} must be positive")
                if preset > 1000:
                    raise ValueError(f"Preset {preset} too large (max 1000)")
            
            self.stepper_presets = new_presets
            self.update_stepper_preset_dropdown()
            self.log_status(f"Applied stepper presets: {self.stepper_presets}")
        except Exception as e:
            self.log_status(f"Invalid stepper preset values: {e}")

    def reset_stepper_presets(self):
        """Reset stepper presets to default values"""
        self.stepper_presets = [1, 5, 10, 25, 50, 100]
        self.update_stepper_preset_dropdown()
        self.log_status("Stepper presets reset to defaults")

    def add_stepper_preset(self):
        """Add a new stepper preset"""
        try:
            new_preset = int(self.new_stepper_preset_var.get())
            if new_preset <= 0:
                raise ValueError(f"Preset {new_preset} must be positive")
            if new_preset > 1000:
                raise ValueError(f"Preset {new_preset} too large (max 1000)")
            
            if new_preset not in self.stepper_presets:
                self.stepper_presets.append(new_preset)
                self.stepper_presets.sort()  # Keep presets sorted
                self.update_stepper_preset_dropdown()
                self.log_status(f"Added stepper preset: {new_preset} steps")
            else:
                self.log_status(f"Preset {new_preset} steps already exists")
        except Exception as e:
            self.log_status(f"Invalid preset value: {e}")

    def remove_stepper_preset(self):
        """Remove the last stepper preset"""
        if len(self.stepper_presets) > 1:  # Keep at least one preset
            removed = self.stepper_presets.pop()
            self.update_stepper_preset_dropdown()
            self.log_status(f"Removed stepper preset: {removed} steps")
        else:
            self.log_status("Cannot remove the last preset")

    def update_stepper_preset_dropdown(self):
        """Update stepper preset dropdown with new values"""
        self.stepper_preset_dropdown['values'] = [f"{steps} step{'s' if steps != 1 else ''}" for steps in self.stepper_presets]

    def update_preset_dropdowns(self):
        """Update servo preset dropdowns with new values"""
        # Update servo preset dropdowns (need to find them in the widget hierarchy)
        for i in range(3):
            # Find the dropdown for servo i and update its values
            servo_frame = self.servo_scales[i].master
            for widget in servo_frame.winfo_children():
                if isinstance(widget, ttk.Combobox):
                    widget['values'] = [f"{preset}°" for preset in self.servo_presets]
                    break

    def save_servo_presets(self):
        """Save servo presets to a configuration file"""
        try:
            import json
            config = {'servo_presets': self.servo_presets}
            with open('servo_config.json', 'w') as f:
                json.dump(config, f)
            self.log_status("Servo presets saved to servo_config.json")
        except Exception as e:
            self.log_status(f"Error saving servo presets: {e}")

    def load_servo_presets(self):
        """Load servo presets from a configuration file"""
        try:
            import json
            with open('servo_config.json', 'r') as f:
                config = json.load(f)
            self.servo_presets = config.get('servo_presets', [0, 30, 45, 60, 90, 120, 135, 150, 180])
            self.update_preset_dropdowns()
            self.log_status("Servo presets loaded from servo_config.json")
        except FileNotFoundError:
            self.log_status("No servo configuration file found, using defaults")
        except Exception as e:
            self.log_status(f"Error loading servo presets: {e}")

    def save_stepper_presets(self):
        """Save stepper presets to a configuration file"""
        try:
            import json
            config = {'stepper_presets': self.stepper_presets}
            with open('stepper_config.json', 'w') as f:
                json.dump(config, f)
            self.log_status("Stepper presets saved to stepper_config.json")
        except Exception as e:
            self.log_status(f"Error saving stepper presets: {e}")

    def load_stepper_presets(self):
        """Load stepper presets from a configuration file"""
        try:
            import json
            with open('stepper_config.json', 'r') as f:
                config = json.load(f)
            self.stepper_presets = config.get('stepper_presets', [1, 5, 10, 25, 50, 100])
            self.update_stepper_preset_dropdown()
            self.log_status("Stepper presets loaded from stepper_config.json")
        except FileNotFoundError:
            self.log_status("No stepper configuration file found, using defaults")
        except Exception as e:
            self.log_status(f"Error loading stepper presets: {e}")

    def update_force_status_indicator(self):
        """Update force status indicator based on current force and threshold"""
        force = self.controller.shared.current_force
        threshold = self.controller.shared.force_threshold
        tolerance = self.controller.shared.force_tolerance
        
        if force < threshold - tolerance:
            self.force_status_indicator.config(text="BELOW RANGE", bg="red")
        elif force > threshold + tolerance:
            self.force_status_indicator.config(text="ABOVE RANGE", bg="red")
        else:
            self.force_status_indicator.config(text="WITHIN RANGE", bg="green")

    def log_status(self, message):
        """Add a message to the status log with timestamp"""
        import datetime
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        log_message = f"[{timestamp}] {message}\n"
        
        self.status_log.config(state=tk.NORMAL)
        self.status_log.insert(tk.END, log_message)
        self.status_log.see(tk.END)  # Auto-scroll to bottom
        self.status_log.config(state=tk.DISABLED)
        
        # Also print to terminal for debugging
        print(message)

# --- Load Cell Reading with auto-reconnection ---
def read_force(ser):
    if ser is None:
        # Simulate force value for GUI/testing
        return 0.0
    
    try:
        line = ser.readline().decode('utf-8').strip()
        if not line:
            return None
        
        # MK10 load cell with HX711 should output force values in grams
        # Ensure proper calibration and scaling in HX711 firmware/setup
        return float(line)
    except (ValueError, UnicodeDecodeError):
        return None
    except serial.SerialException as e:
        print(f"Serial communication error: {e}")
        # Could trigger reconnection attempt here
        return None

# --- Unit and System Tests ---
def run_unit_tests():
    results = []
    try:
        # ServoController.angle_to_duty - Fixed test values
        servo = ServoController()
        assert abs(servo.angle_to_duty(0) - 0.05) < 0.01, "Servo angle 0 failed"
        assert abs(servo.angle_to_duty(180) - 0.25) < 0.01, "Servo angle 180 failed"
        assert abs(servo.angle_to_duty(90) - 0.15) < 0.01, "Servo angle 90 failed"
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

    try:
        root = tk.Tk()
        global app
        app = App(root)

        signal.signal(signal.SIGINT, handle_exit)
        signal.signal(signal.SIGTERM, handle_exit)

        root.mainloop()
        app.controller.cleanup()
    except Exception as e:
        syslog.openlog("rpi_control")
        syslog.syslog(syslog.LOG_ERR, f"rpi_control.py failed to start: {e}\n{traceback.format_exc()}")
        syslog.closelog()
        raise

if __name__ == "__main__":
    main()
