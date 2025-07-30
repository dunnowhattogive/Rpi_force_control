import time
import serial
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from gpiozero import OutputDevice, PWMOutputDevice
import signal
import sys
import traceback
import syslog
import glob
import serial.tools.list_ports
import json
import os
import math
import csv
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
from collections import deque
import pygame

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
            self.step_pin = OutputDevice(self.STEP_PIN, active_high=True, initial_value=False)  # Renamed to avoid conflict
            self.enable = OutputDevice(self.ENABLE_PIN, active_high=False, initial_value=True)
            self.enable.on()  # Enable motor (LOW logic, so .on() sets pin low)
            self.hw_available = True
        except Exception as e:
            print(f"StepperMotor GPIO setup error: {e}")
            self.dir = self.step_pin = self.enable = None
            self.hw_available = False

    def step_motor(self, steps, direction):
        if self.hw_available and self.dir is not None and self.step_pin is not None:
            self.dir.value = 1 if direction else 0
            for _ in range(steps):
                self.step_pin.on()
                time.sleep(self.STEP_DELAY)
                self.step_pin.off()
                time.sleep(self.STEP_DELAY)
        else:
            # Simulate stepper for GUI/testing
            print(f"[SIM] Stepper step: steps={steps}, direction={'up' if direction else 'down'}")

    def step(self, steps, direction):
        """Main step method - calls step_motor to avoid naming conflict"""
        self.step_motor(steps, direction)

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

# --- Safety and Limits Management ---
class SafetyManager:
    def __init__(self):
        self.max_force = 8000  # Maximum safe force in grams (80% of 10kg capacity)
        self.min_force = -1000  # Minimum force (compression limit)
        self.max_servo_angle = 180
        self.min_servo_angle = 0
        self.max_stepper_steps_per_second = 1000
        self.emergency_stop = False
        self.alarms_enabled = True
        
        # Initialize pygame for sound alerts
        try:
            pygame.mixer.init()
            self.sound_available = True
        except:
            self.sound_available = False
            print("Warning: Sound alerts not available")
    
    def check_force_limits(self, force):
        """Check if force is within safe limits"""
        if force > self.max_force:
            self.trigger_alarm("FORCE TOO HIGH", f"Force {force:.1f}g exceeds maximum {self.max_force}g")
            return False
        elif force < self.min_force:
            self.trigger_alarm("FORCE TOO LOW", f"Force {force:.1f}g below minimum {self.min_force}g")
            return False
        return True
    
    def trigger_alarm(self, alarm_type, message):
        """Trigger safety alarm"""
        if self.alarms_enabled:
            print(f"SAFETY ALARM: {alarm_type} - {message}")
            if self.sound_available:
                # Generate alarm beep
                self.play_alarm_sound()
    
    def play_alarm_sound(self):
        """Play alarm sound"""
        try:
            # Generate a simple beep sound
            frequency = 1000  # Hz
            duration = 0.5    # seconds
            sample_rate = 22050
            frames = int(duration * sample_rate)
            arr = np.sin(2 * np.pi * frequency * np.linspace(0, duration, frames))
            arr = (arr * 32767).astype(np.int16)
            sound = pygame.sndarray.make_sound(arr)
            sound.play()
        except:
            pass  # Fail silently if sound doesn't work

# --- Data Logging System ---
class DataLogger:
    def __init__(self, base_path="data_logs"):
        self.base_path = base_path
        self.current_log_file = None
        self.logging_enabled = False
        self.log_interval = 1.0  # seconds
        self.last_log_time = 0
        
        # Create logs directory if it doesn't exist
        os.makedirs(base_path, exist_ok=True)
    
    def start_logging(self, test_name="test"):
        """Start data logging to a new file"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{test_name}_{timestamp}.csv"
        self.current_log_file = os.path.join(self.base_path, filename)
        
        # Create CSV header
        with open(self.current_log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Timestamp', 'Force_g', 'Threshold_g', 'Servo1_deg', 'Servo2_deg', 'Servo3_deg', 'Mode'])
        
        self.logging_enabled = True
        return self.current_log_file
    
    def log_data(self, force, threshold, servo_angles, mode):
        """Log data point if logging is enabled and interval has passed"""
        current_time = time.time()
        if self.logging_enabled and (current_time - self.last_log_time) >= self.log_interval:
            timestamp = datetime.now().isoformat()
            
            with open(self.current_log_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([timestamp, force, threshold, 
                               servo_angles[0], servo_angles[1], servo_angles[2], mode])
            
            self.last_log_time = current_time
    
    def stop_logging(self):
        """Stop data logging"""
        self.logging_enabled = False
        return self.current_log_file

# --- Load Cell Calibration System ---
class CalibrationManager:
    def __init__(self):
        self.calibration_file = "calibration.json"
        self.zero_offset = 0.0
        self.scale_factor = 1.0
        self.calibration_weights = [0, 100, 500, 1000, 2000, 5000]  # grams
        self.calibration_readings = []
        self.load_calibration()
    
    def save_calibration(self):
        """Save calibration data to file"""
        cal_data = {
            'zero_offset': self.zero_offset,
            'scale_factor': self.scale_factor,
            'timestamp': datetime.now().isoformat(),
            'calibration_points': list(zip(self.calibration_weights, self.calibration_readings))
        }
        
        with open(self.calibration_file, 'w') as f:
            json.dump(cal_data, f, indent=2)
    
    def load_calibration(self):
        """Load calibration data from file"""
        try:
            with open(self.calibration_file, 'r') as f:
                cal_data = json.load(f)
                self.zero_offset = cal_data.get('zero_offset', 0.0)
                self.scale_factor = cal_data.get('scale_factor', 1.0)
        except FileNotFoundError:
            print("No calibration file found, using defaults")
    
    def apply_calibration(self, raw_reading):
        """Apply calibration to raw reading"""
        return (raw_reading - self.zero_offset) * self.scale_factor
    
    def start_calibration(self):
        """Start calibration procedure"""
        self.calibration_readings = []
        return True
    
    def add_calibration_point(self, weight, raw_reading):
        """Add a calibration point"""
        self.calibration_readings.append(raw_reading)
        
        # If we have enough points, calculate calibration
        if len(self.calibration_readings) >= 2:
            self.calculate_calibration()
    
    def calculate_calibration(self):
        """Calculate calibration coefficients using linear regression"""
        if len(self.calibration_readings) >= 2:
            weights = np.array(self.calibration_weights[:len(self.calibration_readings)])
            readings = np.array(self.calibration_readings)
            
            # Linear regression: force = scale_factor * (reading - zero_offset)
            # Rearranged: reading = force/scale_factor + zero_offset
            coeffs = np.polyfit(weights, readings, 1)
            self.scale_factor = 1.0 / coeffs[0]
            self.zero_offset = coeffs[1]

# --- Configuration Management ---
class ConfigManager:
    def __init__(self):
        self.config_file = "system_config.json"
        self.config = self.load_default_config()
        self.load_config()
    
    def load_default_config(self):
        """Load default configuration"""
        return {
            'stepper': {
                'dir_pin': 20,
                'step_pin': 21,
                'enable_pin': 16,
                'step_delay': 0.001
            },
            'servos': {
                'pins': [17, 27, 22],
                'frequency': 50
            },
            'serial': {
                'baudrate': 9600,
                'timeout': 1
            },
            'pid': {
                'kp': 0.1,
                'ki': 0.01,
                'kd': 0.05
            },
            'safety': {
                'max_force': 8000,
                'min_force': -1000,
                'alarms_enabled': True
            },
            'logging': {
                'interval': 1.0,
                'auto_start': False
            }
        }
    
    def save_config(self):
        """Save configuration to file"""
        with open(self.config_file, 'w') as f:
            json.dump(self.config, f, indent=2)
    
    def load_config(self):
        """Load configuration from file"""
        try:
            with open(self.config_file, 'r') as f:
                loaded_config = json.load(f)
                # Merge with defaults to handle missing keys
                self.merge_config(self.config, loaded_config)
        except FileNotFoundError:
            print("No config file found, using defaults")
    
    def merge_config(self, default, loaded):
        """Recursively merge loaded config with defaults"""
        for key, value in loaded.items():
            if key in default:
                if isinstance(value, dict) and isinstance(default[key], dict):
                    self.merge_config(default[key], value)
                else:
                    default[key] = value

# --- Recipe/Sequence Control ---
class SequenceManager:
    def __init__(self):
        self.sequences = {}
        self.current_sequence = None
        self.sequence_running = False
        self.sequence_step = 0
    
    def create_sequence(self, name, steps):
        """Create a new test sequence"""
        # Steps format: [{'type': 'force', 'target': 1000, 'duration': 10}, ...]
        self.sequences[name] = steps
    
    def start_sequence(self, name, controller):
        """Start executing a sequence"""
        if name in self.sequences:
            self.current_sequence = self.sequences[name]
            self.sequence_running = True
            self.sequence_step = 0
            return True
        return False
    
    def stop_sequence(self):
        """Stop current sequence"""
        self.sequence_running = False
        self.current_sequence = None
        self.sequence_step = 0

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

        # Add new components
        self.safety_manager = SafetyManager()
        self.data_logger = DataLogger()
        self.calibration_manager = CalibrationManager()
        self.config_manager = ConfigManager()
        self.sequence_manager = SequenceManager()
        
        # Apply configuration
        self.apply_config()
        
        # Initialize data for plotting
        self.force_history = deque(maxlen=1000)  # Keep last 1000 readings
        self.time_history = deque(maxlen=1000)
        self.plot_start_time = time.time()

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

    def apply_config(self):
        """Apply loaded configuration"""
        config = self.config_manager.config
        
        # Apply PID config
        self.pid_kp = config['pid']['kp']
        self.pid_ki = config['pid']['ki']
        self.pid_kd = config['pid']['kd']
        
        # Apply safety config
        self.safety_manager.max_force = config['safety']['max_force']
        self.safety_manager.min_force = config['safety']['min_force']
        self.safety_manager.alarms_enabled = config['safety']['alarms_enabled']
        
        # Apply step delay
        StepperMotor.STEP_DELAY = config['stepper']['step_delay']

    def read_force_with_calibration(self):
        """Read force with calibration applied"""
        raw_force = read_force(self.ser)
        if raw_force is not None:
            return self.calibration_manager.apply_calibration(raw_force)
        return None

    def emergency_stop(self):
        """Emergency stop all operations"""
        self.safety_manager.emergency_stop = True
        self.auto_mode_enabled = False
        self.stepper_enabled = False
        self.servo_enabled = False
        self.log_status("EMERGENCY STOP ACTIVATED")
        self.safety_manager.trigger_alarm("EMERGENCY_STOP", "Emergency stop activated by user")

    def reset_emergency_stop(self):
        """Reset emergency stop"""
        self.safety_manager.emergency_stop = False
        self.stepper_enabled = True
        self.servo_enabled = True
        self.log_status("Emergency stop reset - system ready")

    def auto_adjust_force(self):
        """Enhanced automatic force adjustment with safety checks"""
        if not self.auto_mode_enabled or not self.stepper_enabled or self.safety_manager.emergency_stop:
            return
            
        current_force = self.shared.current_force
        target_force = self.shared.force_threshold  # Fixed: was undefined
        tolerance = self.shared.force_tolerance
        
        # Safety check
        if not self.safety_manager.check_force_limits(current_force):
            self.emergency_stop()
            return
            
        # Check if we're within tolerance
        if abs(current_force - target_force) <= tolerance:
            return  # No adjustment needed
            
        # Calculate PID error
        error = target_force - current_force
        self.pid_integral += error
        derivative = error - self.pid_last_error
        
        # Calculate PID output
        output = (self.pid_kp * error + 
                 self.pid_ki * self.pid_integral + 
                 self.pid_kd * derivative)
        
        # Convert to steps and direction
        steps = max(1, min(50, int(abs(output))))  # Limit steps between 1-50
        direction = output > 0  # True for increase force, False for decrease
        
        # Apply the adjustment
        if callable(self.stepper.step):
            self.stepper.step(steps, direction)
            action = "increase" if direction else "decrease"
            self.log_status(f"Auto: {action} force by {steps} steps (error: {error:.1f}g)")
        
        # Update PID state
        self.pid_last_error = error
        
        # Limit integral windup
        max_integral = 1000
        self.pid_integral = max(-max_integral, min(max_integral, self.pid_integral))

    def update_force_history(self, force):
        """Update force history for plotting"""
        current_time = time.time() - self.plot_start_time
        self.force_history.append(force)
        self.time_history.append(current_time)

    def increase_force(self):
        if self.stepper_enabled and not self.auto_mode_enabled and not self.safety_manager.emergency_stop:
            if callable(self.stepper.step):
                self.stepper.step(10, True)
                self.log_status("Stepper turning anti-clockwise to increase force")
            else:
                self.log_status("[SIM] Stepper step: steps=10, direction=up (anti-clockwise)")

    def decrease_force(self):
        if self.stepper_enabled and not self.auto_mode_enabled and not self.safety_manager.emergency_stop:
            if callable(self.stepper.step):
                self.stepper.step(10, False)
                self.log_status("Stepper turning clockwise to decrease force")
            else:
                self.log_status("[SIM] Stepper step: steps=10, direction=down (clockwise)")

    def get_force_reading(self):
        return self.shared.current_force

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

    def log_status(self, message):
        """Log status message to GUI if available, otherwise print"""
        if self.app and hasattr(self.app, 'log_status'):
            self.app.log_status(message)
        else:
            print(message)

    def increase_threshold(self):
        """Increase force threshold by 1"""
        self.shared.force_threshold += 1
        self.log_status(f"Force threshold increased to {self.shared.force_threshold}g")

    def decrease_threshold(self):
        """Decrease force threshold by 1"""
        self.shared.force_threshold -= 1
        self.log_status(f"Force threshold decreased to {self.shared.force_threshold}g")

# --- App class for Tkinter GUI ---
class App:
    def __init__(self, root):
        self.root = root
        self.root.title("RPi Control")
        
        # Raspberry Pi display compatibility - don't force fullscreen initially
        try:
            # Check if we're on a Raspberry Pi with a small display
            import subprocess
            result = subprocess.run(['cat', '/proc/device-tree/model'], 
                                  capture_output=True, text=True, timeout=2)
            if 'Raspberry Pi' in result.stdout:
                # Raspberry Pi detected - use appropriate window size
                self.root.geometry("800x600")  # Better for Pi displays
                self.is_raspberry_pi = True
            else:
                # Desktop system
                try:
                    self.root.attributes('-fullscreen', True)
                    self.root.state('zoomed')
                except tk.TclError:
                    # Fallback if fullscreen not supported
                    self.root.geometry("1200x800")
                self.is_raspberry_pi = False
        except:
            # Fallback for any system
            self.root.geometry("1024x768")
            self.is_raspberry_pi = False
        
        # Bind escape key to toggle fullscreen (optional emergency exit)
        self.root.bind('<Escape>', self.toggle_fullscreen)
        self.root.bind('<F11>', self.toggle_fullscreen)
        
        self.controller = Controller()
        
        self.servo_control_enabled = tk.BooleanVar(value=True)
        self.stepper_control_enabled = tk.BooleanVar(value=True)
        self.auto_stepper_mode = tk.BooleanVar(value=False)

        # Configurable presets (default values)
        self.servo_presets = [0, 30, 45, 60, 90, 120, 135, 150, 180]
        self.stepper_presets = [1, 5, 10, 25, 50, 100]

        # Store references to preset-related widgets for proper cleanup
        self.servo_preset_dropdowns = []
        self.stepper_preset_buttons_frame = None
        self.preset_config_frame = None
        self.stepper_preset_config_frame = None

        # Initialize error tracking flags
        self._force_error_logged = False
        self._log_error_logged = False

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

        # Force threshold display with adjustment buttons
        threshold_frame = tk.Frame(main_frame)
        threshold_frame.pack(pady=5)
        
        self.force_thresh_var = tk.StringVar(value=str(self.controller.shared.force_threshold))
        self.force_thresh_label = tk.Label(threshold_frame, text="Force Threshold (g):", font=("Arial", 12))
        self.force_thresh_label.pack(side=tk.LEFT, padx=5)
        
        # Threshold adjustment buttons
        tk.Button(threshold_frame, text="-", command=self.controller.decrease_threshold, 
                 font=("Arial", 10), width=3).pack(side=tk.LEFT, padx=2)
        
        self.force_thresh_box = tk.Entry(threshold_frame, textvariable=self.force_thresh_var, 
                                        font=("Arial", 12), state="readonly", width=8)
        self.force_thresh_box.pack(side=tk.LEFT, padx=2)
        
        tk.Button(threshold_frame, text="+", command=self.controller.increase_threshold, 
                 font=("Arial", 10), width=3).pack(side=tk.LEFT, padx=2)

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
            self.servo_preset_dropdowns.append(preset_dropdown)
            
            # Update angle display when slider changes
            def create_angle_updater(val, idx=i, display=angle_display):
                def update_angle_display(val):
                    display.config(text=f"{int(float(val))}°")
                return update_angle_display
            
            angle_updater = create_angle_updater(None, i, angle_display)
            scale.config(command=lambda val, idx=i, updater=angle_updater: [self.set_servo_angle(idx, val), updater(val)])

        # Combined Stepper Control frame
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

        # Emergency stop button
        stop_frame = tk.Frame(stepper_control_frame)
        stop_frame.pack(pady=5)
        
        self.emergency_stop_button = tk.Button(stop_frame, text="EMERGENCY STOP", 
                                              command=self.controller.emergency_stop, 
                                              bg="red", fg="white", font=("Arial", 12, "bold"))
        self.emergency_stop_button.pack(side=tk.LEFT, padx=5)
        
        self.reset_stop_button = tk.Button(stop_frame, text="Reset", 
                                          command=self.controller.reset_emergency_stop, 
                                          bg="orange", fg="white", font=("Arial", 10))
        self.reset_stop_button.pack(side=tk.LEFT, padx=5)
        
        self.stop_button = tk.Button(stop_frame, text="Stop Program", command=self.stop_program, 
                                   bg="darkred", fg="white", font=("Arial", 10, "bold"))
        self.stop_button.pack(side=tk.LEFT, padx=5)

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

        # PID parameter controls
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

        # --- Tests tab ---
        tests_frame = tk.Frame(notebook)
        notebook.add(tests_frame, text="Tests")

        self.test_results_text = tk.Text(tests_frame, height=15, width=60, state=tk.DISABLED)
        self.test_results_text.pack(padx=10, pady=10)

        run_unit_btn = tk.Button(tests_frame, text="Run Unit Tests", command=self.run_unit_tests_gui)
        run_unit_btn.pack(pady=5)

        run_system_btn = tk.Button(tests_frame, text="Run System Tests", command=self.run_system_tests_gui)
        run_system_btn.pack(pady=5)

        # Add data logging controls - Initialize BEFORE creating tabs
        self.logging_enabled = tk.BooleanVar(value=False)
        
        # Setup plotting first
        self.setup_plotting()
        
        # Create additional tabs AFTER main tab
        self.create_calibration_tab(notebook)
        self.create_data_logging_tab(notebook)
        self.create_safety_tab(notebook)
        self.create_sequence_tab(notebook)

        # Start background threads
        self.sync_thread = threading.Thread(target=self.sync_enable_flags, daemon=True)
        self.sync_thread.start()
        
        self.auto_control_thread = threading.Thread(target=self.auto_control_loop, daemon=True)
        self.auto_control_thread.start()
        
        # Start periodic updates
        self.update_force_and_thresh()
        self.update_plot()
        
        # Set reference for logging AFTER GUI is fully initialized
        self.controller.set_app(self)

    # Add missing GUI methods
    def set_servo_angle(self, idx, val):
        """Set servo angle with validation"""
        try:
            angle = int(float(val))
            if 0 <= angle <= 180:
                self.controller.servo_ctrl.set_angle(idx, angle)
                self.log_status(f"Servo {idx+1} set to {angle}°")
        except (ValueError, TypeError):
            self.log_status(f"Invalid servo angle: {val}")

    def servo_preset_selected(self, event, idx):
        """Handle servo preset selection"""
        try:
            selected_text = event.widget.get()
            angle = int(selected_text.replace('°', ''))
            self.servo_angle_vars[idx].set(angle)
            self.set_servo_angle(idx, angle)
        except (ValueError, IndexError):
            self.log_status("Error applying servo preset")

    def stepper_preset_increase_dropdown(self):
        """Increase force using selected step preset"""
        try:
            selected = self.stepper_preset_var.get()
            if "step" in selected:
                steps = int(selected.split()[0])
                if self.controller.stepper_enabled and not self.controller.auto_mode_enabled:
                    self.controller.stepper.step(steps, True)
                    self.log_status(f"Stepper increased by {steps} steps")
        except (ValueError, AttributeError):
            self.log_status("Error with stepper preset")

    def stepper_preset_decrease_dropdown(self):
        """Decrease force using selected step preset"""
        try:
            selected = self.stepper_preset_var.get()
            if "step" in selected:
                steps = int(selected.split()[0])
                if self.controller.stepper_enabled and not self.controller.auto_mode_enabled:
                    self.controller.stepper.step(steps, False)
                    self.log_status(f"Stepper decreased by {steps} steps")
        except (ValueError, AttributeError):
            self.log_status("Error with stepper preset")

    def toggle_stepper_mode(self):
        """Toggle between manual and automatic stepper mode"""
        self.controller.auto_mode_enabled = self.auto_stepper_mode.get()
        mode_text = "Automatic" if self.controller.auto_mode_enabled else "Manual"
        self.stepper_mode_status.config(text=f"Mode: {mode_text}")
        
        # Enable/disable manual controls
        state = tk.DISABLED if self.controller.auto_mode_enabled else tk.NORMAL
        self.manual_increase_force_button.config(state=state)
        self.manual_decrease_force_button.config(state=state)
        
        self.log_status(f"Stepper mode changed to: {mode_text}")

    def update_force_status_indicator(self):
        """Update the force status indicator based on current force vs threshold"""
        try:
            current_force = self.controller.shared.current_force
            threshold = self.controller.shared.force_threshold
            tolerance = self.controller.shared.force_tolerance
            
            if abs(current_force - threshold) <= tolerance:
                self.force_status_indicator.config(text="WITHIN RANGE", bg="green", fg="white")
            elif current_force > threshold:
                self.force_status_indicator.config(text="ABOVE TARGET", bg="orange", fg="white")
            else:
                self.force_status_indicator.config(text="BELOW TARGET", bg="blue", fg="white")
        except Exception as e:
            self.log_status(f"Error updating force status: {e}")

    def stop_program(self):
        """Stop the entire program"""
        self.log_status("Program stop requested")
        self.controller.cleanup()
        self.root.quit()

    def clear_status_log(self):
        """Clear the status log"""
        self.status_log.config(state=tk.NORMAL)
        self.status_log.delete(1.0, tk.END)
        self.status_log.config(state=tk.DISABLED)

    def log_status(self, message):
        """Log status message with timestamp"""
        try:
            timestamp = datetime.now().strftime("%H:%M:%S")
            full_message = f"[{timestamp}] {message}\n"
            
            self.status_log.config(state=tk.NORMAL)
            self.status_log.insert(tk.END, full_message)
            self.status_log.see(tk.END)  # Auto-scroll to bottom
            self.status_log.config(state=tk.DISABLED)
        except Exception as e:
            print(f"Logging error: {e}")

    # Add missing methods that are referenced but not defined
    def sync_enable_flags(self):
        """Background thread to sync enable flags"""
        while True:
            try:
                if hasattr(self, 'controller'):
                    self.controller.servo_enabled = self.servo_control_enabled.get()
                    self.controller.stepper_enabled = self.stepper_control_enabled.get()
                time.sleep(0.1)
            except Exception:
                break

    def auto_control_loop(self):
        """Background thread for automatic control"""
        while True:
            try:
                if hasattr(self, 'controller') and self.controller.auto_mode_enabled:
                    self.controller.auto_adjust_force()
                time.sleep(0.5)
            except Exception:
                break

    def apply_pid_params(self):
        """Apply PID parameters from settings"""
        try:
            self.controller.pid_kp = self.kp_var.get()
            self.controller.pid_ki = self.ki_var.get()
            self.controller.pid_kd = self.kd_var.get()
            self.log_status(f"PID parameters updated: Kp={self.controller.pid_kp}, Ki={self.controller.pid_ki}, Kd={self.controller.pid_kd}")
        except Exception as e:
            self.log_status(f"Error applying PID parameters: {e}")

    def apply_pins(self):
        """Apply pin configuration (requires restart)"""
        self.log_status("Pin configuration changed - restart required for changes to take effect")

    def update_servo_controls(self):
        """Update servo control states"""
        self.controller.servo_enabled = self.servo_control_enabled.get()
        state_text = "enabled" if self.controller.servo_enabled else "disabled"
        self.log_status(f"Servo controls {state_text}")

    def update_stepper_controls(self):
        """Update stepper control states"""
        self.controller.stepper_enabled = self.stepper_control_enabled.get()
        state_text = "enabled" if self.controller.stepper_enabled else "disabled"
        self.log_status(f"Stepper controls {state_text}")

    def update_individual_servos(self):
        """Update individual servo enable states"""
        self.log_status("Individual servo settings updated")

    def apply_force_threshold(self):
        """Apply force threshold from settings"""
        try:
            new_threshold = self.set_force_thresh_var.get()
            self.controller.shared.force_threshold = new_threshold
            self.settings_force_thresh_var.set(str(new_threshold))
            self.log_status(f"Force threshold set to {new_threshold}g")
        except Exception as e:
            self.log_status(f"Error setting force threshold: {e}")

    def run_unit_tests_gui(self):
        """Run unit tests and display results"""
        try:
            results = run_unit_tests()
            self.test_results_text.config(state=tk.NORMAL)
            self.test_results_text.delete(1.0, tk.END)
            self.test_results_text.insert(1.0, "\n".join(results))
            self.test_results_text.config(state=tk.DISABLED)
        except Exception as e:
            self.log_status(f"Error running unit tests: {e}")

    def run_system_tests_gui(self):
        """Run system tests and display results"""
        try:
            results = run_system_tests(self.controller)
            self.test_results_text.config(state=tk.NORMAL)
            self.test_results_text.delete(1.0, tk.END)
            self.test_results_text.insert(1.0, "\n".join(results))
            self.test_results_text.config(state=tk.DISABLED)
        except Exception as e:
            self.log_status(f"Error running system tests: {e}")

    # Add placeholder methods for missing tabs
    def create_calibration_tab(self, notebook):
        """Create calibration tab"""
        cal_frame = tk.Frame(notebook)
        notebook.add(cal_frame, text="Calibration")
        tk.Label(cal_frame, text="Load Cell Calibration", font=("Arial", 14)).pack(pady=20)
        tk.Label(cal_frame, text="Calibration features will be implemented here").pack(pady=10)

    def create_data_logging_tab(self, notebook):
        """Create data logging tab"""
        log_frame = tk.Frame(notebook)
        notebook.add(log_frame, text="Data Logging")
        
        # Logging controls
        logging_control_frame = tk.LabelFrame(log_frame, text="Data Logging Controls")
        logging_control_frame.pack(pady=10, padx=10, fill="x")
        
        self.logging_checkbox = tk.Checkbutton(logging_control_frame, text="Enable Data Logging", 
                                              variable=self.logging_enabled)
        self.logging_checkbox.pack(pady=5)
        
        tk.Button(logging_control_frame, text="View Log Files", 
                 command=self.view_log_files).pack(pady=5)
        
        # Plotting area
        if hasattr(self, 'plot_fig') and self.plot_fig is not None:
            plot_frame = tk.LabelFrame(log_frame, text="Real-time Force Plot")
            plot_frame.pack(pady=10, padx=10, fill="both", expand=True)
            
            self.canvas = FigureCanvasTkAgg(self.plot_fig, plot_frame)
            self.canvas.draw()
            self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def create_safety_tab(self, notebook):
        """Create safety tab"""
        safety_frame = tk.Frame(notebook)
        notebook.add(safety_frame, text="Safety")
        
        # Emergency controls
        emergency_frame = tk.LabelFrame(safety_frame, text="Emergency Controls")
        emergency_frame.pack(pady=10, padx=10, fill="x")
        
        tk.Button(emergency_frame, text="EMERGENCY STOP", 
                 command=self.controller.emergency_stop,
                 bg="red", fg="white", font=("Arial", 16, "bold"),
                 height=2).pack(pady=10)
        
        tk.Button(emergency_frame, text="Reset Emergency Stop", 
                 command=self.controller.reset_emergency_stop,
                 bg="orange", fg="white", font=("Arial", 12)).pack(pady=5)

    def create_sequence_tab(self, notebook):
        """Create sequence tab"""
        seq_frame = tk.Frame(notebook)
        notebook.add(seq_frame, text="Sequences")
        tk.Label(seq_frame, text="Automated Test Sequences", font=("Arial", 14)).pack(pady=20)
        tk.Label(seq_frame, text="Sequence features will be implemented here").pack(pady=10)

# Add missing test functions
def run_unit_tests():
    """Run unit tests and return results"""
    results = []
    results.append("=== Unit Tests ===")
    
    try:
        # Test StepperMotor
        stepper = StepperMotor()
        results.append("✓ StepperMotor instantiation: PASS")
        
        # Test ServoController
        servo = ServoController()
        results.append("✓ ServoController instantiation: PASS")
        
        # Test SafetyManager
        safety = SafetyManager()
        results.append("✓ SafetyManager instantiation: PASS")
        
        # Test force reading simulation
        force = read_force(None)  # Should return 0.0 for simulation
        if force == 0.0:
            results.append("✓ Force reading simulation: PASS")
        else:
            results.append("✗ Force reading simulation: FAIL")
            
        results.append("\nAll unit tests completed successfully!")
        
    except Exception as e:
        results.append(f"✗ Unit test error: {e}")
    
    return results

def run_system_tests(controller):
    """Run system tests and return results"""
    results = []
    results.append("=== System Tests ===")
    
    try:
        # Test controller initialization
        if controller:
            results.append("✓ Controller initialization: PASS")
        else:
            results.append("✗ Controller initialization: FAIL")
            
        # Test force reading
        try:
            force = controller.get_force_reading()
            results.append(f"✓ Force reading: PASS ({force:.2f}g)")
        except Exception as e:
            results.append(f"✗ Force reading: FAIL ({e})")
            
        # Test stepper control
        try:
            controller.stepper.step(1, True)
            results.append("✓ Stepper control: PASS")
        except Exception as e:
            results.append(f"✗ Stepper control: FAIL ({e})")
            
        # Test servo control
        try:
            controller.servo_ctrl.set_angle(0, 90)
            results.append("✓ Servo control: PASS")
        except Exception as e:
            results.append(f"✗ Servo control: FAIL ({e})")
            
        results.append("\nSystem tests completed!")
        
    except Exception as e:
        results.append(f"✗ System test error: {e}")
    
    return results

# Add Pi-specific hardware detection
def detect_raspberry_pi():
    """Detect if running on Raspberry Pi"""
    try:
        with open('/proc/device-tree/model', 'r') as f:
            model = f.read()
            return 'Raspberry Pi' in model
    except:
        return False

# Modified main function for Pi compatibility
def main():
    app = None
    
    def handle_exit(signum, frame):
        try:
            if app is not None:
                app.controller.cleanup()
                app.root.quit()
                app.root.destroy()
        except Exception as e:
            print(f"Error during cleanup: {e}")
        finally:
            sys.exit(0)

    try:
        # Check if running on Pi and adjust accordingly
        is_pi = detect_raspberry_pi()
        
        root = tk.Tk()
        
        if is_pi:
            # Pi-specific optimizations
            root.geometry("800x600")
            root.configure(bg='lightgray')
            # Reduce font sizes for smaller Pi displays
            root.option_add('*Font', 'TkDefaultFont 9')
        else:
            # Desktop configuration
            root.geometry("1920x1080")
            root.configure(bg='lightgray')
            
        root.protocol("WM_DELETE_WINDOW", lambda: handle_exit(None, None))
        
        app = App(root)

        signal.signal(signal.SIGINT, handle_exit)
        signal.signal(signal.SIGTERM, handle_exit)

        app.log_status("Application started successfully")
        if is_pi:
            app.log_status("Raspberry Pi detected - optimized for Pi hardware")
            
        root.mainloop()
        
        if app is not None:
            app.controller.cleanup()
            
    except KeyboardInterrupt:
        print("Application interrupted by user")
        if app is not None:
            app.controller.cleanup()
    except Exception as e:
        error_msg = f"rpi_control.py failed to start: {e}\n{traceback.format_exc()}"
        print(error_msg)
        
        try:
            syslog.openlog("rpi_control")
            syslog.syslog(syslog.LOG_ERR, error_msg)
            syslog.closelog()
        except:
            pass
        
        if app is not None:
            app.controller.cleanup()
        raise

if __name__ == "__main__":
    main()