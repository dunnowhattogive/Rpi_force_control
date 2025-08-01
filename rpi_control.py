import time
import serial
import threading
import sys
import traceback
import glob
import serial.tools.list_ports
import json
import os
import math
import csv
import signal
from datetime import datetime
import numpy as np
from collections import deque

os.environ["GPIOZERO_PIN_FACTORY"] = "native"
# Audio support with fallback
try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False
    print("Warning: pygame not available - audio alerts disabled")

# GUI support with fallback
try:
    import tkinter as tk
    from tkinter import ttk
    from tkinter import messagebox  # <-- Add this import
    TKINTER_AVAILABLE = True
except ImportError:
    TKINTER_AVAILABLE = False
    print("Error: tkinter not available - GUI disabled")
    sys.exit(1)

# Plotting support with fallback
try:
    import matplotlib.pyplot as plt
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("Warning: matplotlib not available - plotting disabled")

# Syslog support with fallback
try:
    import syslog
    SYSLOG_AVAILABLE = True
except ImportError:
    SYSLOG_AVAILABLE = False

# GPIO support with robust hardware detection
try:
    import gpiod
    # Simple OutputDevice using gpiod
    class OutputDevice:
        def __init__(self, pin, active_high=True, initial_value=False):
            self.pin = pin
            self.active_high = active_high
            self.chip = gpiod.Chip('gpiochip4')
            self.line = self.chip.get_line(pin)
            self.line.request(consumer="rpi_control", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[initial_value if active_high else not initial_value])
            self.value = initial_value

        def on(self):
            self.value = True
            self.line.set_value(1 if self.active_high else 0)

        def off(self):
            self.value = False
            self.line.set_value(0 if self.active_high else 1)

        def close(self):
            try:
                self.line.release()
                self.chip.close()
            except Exception:
                pass

    # Simple PWMOutputDevice using software PWM (very basic, not for production)
    import threading

    class PWMOutputDevice:
        def __init__(self, pin, frequency=50):
            self.pin = pin
            self.frequency = frequency
            self.duty_cycle = 0.0
            self.chip = gpiod.Chip('gpiochip4')
            self.line = self.chip.get_line(pin)
            self.line.request(consumer="rpi_control_pwm", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
            self._running = True
            self._thread = threading.Thread(target=self._pwm_loop, daemon=True)
            self._thread.start()

        @property
        def value(self):
            return self.duty_cycle

        @value.setter
        def value(self, v):
            self.duty_cycle = max(0.0, min(1.0, float(v)))

        def _pwm_loop(self):
            period = 1.0 / self.frequency
            while self._running:
                high_time = period * self.duty_cycle
                low_time = period - high_time
                if high_time > 0:
                    self.line.set_value(1)
                    time.sleep(high_time)
                if low_time > 0:
                    self.line.set_value(0)
                    time.sleep(low_time)

        def close(self):
            self._running = False
            try:
                self.line.set_value(0)
                self.line.release()
                self.chip.close()
            except Exception:
                pass

    GPIO_AVAILABLE = True
    PI_HARDWARE = True
    GPIO_HARDWARE_WORKING = True
    print("gpiod GPIO available")
except ImportError:
    print("Warning: gpiod not available - running in simulation mode")
    GPIO_AVAILABLE = False
    PI_HARDWARE = False
    GPIO_HARDWARE_WORKING = False
    # Create dummy classes for development/testing
    class OutputDevice:
        def __init__(self, pin, **kwargs):
            self.pin = pin
            self.value = False
        def on(self): pass
        def off(self): pass
        def close(self): pass

    class PWMOutputDevice:
        def __init__(self, pin, **kwargs):
            self.pin = pin
            self.value = 0
        def close(self): pass

# --- Load Cell Reading with auto-reconnection ---
def read_force(ser):
    if ser is None:
        # Simulate force value for GUI/testing with minimal variation
        import random
        # Get the current simulated force or start at 0
        base_force = getattr(read_force, 'simulated_force', 0.0)
        # Add very small random variation to simulate sensor noise
        variation = random.uniform(-0.5, 0.5)
        read_force.simulated_force = max(0, min(10000, base_force + variation))
        return read_force.simulated_force
    
    try:
        line = ser.readline().decode('utf-8').strip()
        if not line:
            return None
        
        # MK10 load cell with HX711 should output force values in grams
        return float(line)
    except (ValueError, UnicodeDecodeError):
        return None
    except serial.SerialException:
        return None

# --- Stepper Motor Control ---
class StepperMotor:
    DIR_PIN = 20
    STEP_PIN = 21
    ENABLE_PIN = 16
    STEP_DELAY = 0.001

    def __init__(self):
        try:
            # Use GPIO_AVAILABLE for hardware mode
            if GPIO_AVAILABLE:
                self.dir = OutputDevice(self.DIR_PIN, active_high=True, initial_value=False)
                self.step_pin = OutputDevice(self.STEP_PIN, active_high=True, initial_value=False)
                self.enable = OutputDevice(self.ENABLE_PIN, active_high=False, initial_value=True)
                self.enable.on()  # Enable motor (LOW logic)
                self.hw_available = True
            else:
                self.dir = self.step_pin = self.enable = None
                self.hw_available = False
        except Exception as e:
            print(f"StepperMotor GPIO setup error: {e}")
            self.dir = self.step_pin = self.enable = None
            self.hw_available = False

    def step(self, steps, direction):
        """Main step method"""
        if self.hw_available and self.dir is not None and self.step_pin is not None:
            self.dir.value = 1 if direction else 0
            for _ in range(steps):
                self.step_pin.on()
                time.sleep(self.STEP_DELAY)
                self.step_pin.off()
                time.sleep(self.STEP_DELAY)

    def disable(self):
        if self.hw_available and self.enable is not None:
            self.enable.off()

# --- Servo Motor Control ---
class ServoController:
    SERVO_PINS = [17, 27, 22]

    def __init__(self):
        self.servos = []
        self.hw_available = GPIO_AVAILABLE
        for pin in self.SERVO_PINS:
            try:
                if GPIO_AVAILABLE:
                    pwm = PWMOutputDevice(pin, frequency=50)
                    pwm.value = 0
                    self.servos.append(pwm)
                else:
                    self.servos.append(None)
            except Exception as e:
                print(f"ServoController PWM setup error on pin {pin}: {e}")
                self.servos.append(None)
                self.hw_available = False

    def angle_to_duty(self, angle):
        return 0.05 + (angle / 180.0) * 0.20

    def set_angle(self, idx, angle):
        if self.hw_available and idx < len(self.servos) and self.servos[idx] is not None:
            duty = self.angle_to_duty(angle)
            self.servos[idx].value = duty

    def cleanup(self):
        for servo in self.servos:
            if self.hw_available and servo is not None:
                servo.value = 0

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
                return port.device
                
            # Also check manufacturer
            if any(keyword in manufacturer_lower for keyword in [
                'arduino', 'ftdi', 'silicon labs', 'prolific'
            ]):
                return port.device
    except Exception as e:
        pass
    
    # Fallback: Check filesystem patterns
    for pattern in device_patterns:
        devices = glob.glob(pattern)
        for device in sorted(devices):  # Sort for consistent ordering
            try:
                # Try to open the device briefly to see if it exists and is accessible
                with serial.Serial(device, 9600, timeout=0.1) as test_ser:
                    return device
            except (serial.SerialException, PermissionError):
                continue
    
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
                        return True
                    except ValueError:
                        continue
        return False
    except Exception as e:
        return False

# --- Shared Data for GUI/Main Thread Communication ---
class SharedData:
    def __init__(self, initial_threshold, tolerance):
        self.force_threshold = initial_threshold
        self.force_tolerance = tolerance
        self.current_force = 0.0
        self.status = "Waiting..."

# --- Cleanup function ---
def cleanup(stepper, servo_ctrl, ser):
    if ser is not None:
        try:
            ser.close()
        except Exception as e:
            print(f"Serial close error: {e}")
    stepper.disable()
    servo_ctrl.cleanup()

# --- Safety and Limits Management ---
class SafetyManager:
    def __init__(self):
        self.max_force = 8000
        self.min_force = -1000
        self.max_servo_angle = 180
        self.min_servo_angle = 0
        self.max_stepper_steps_per_second = 1000
        self.emergency_stop = False
        self.alarms_enabled = True
        
        # Initialize pygame for sound alerts
        if PYGAME_AVAILABLE:
            try:
                pygame.mixer.init()
                self.sound_available = True
            except:
                self.sound_available = False
                print("Warning: Sound alerts not available")
        else:
            self.sound_available = False
    
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
        print(f"SAFETY ALARM: {alarm_type} - {message}")
        if self.alarms_enabled and self.sound_available:
            try:
                self.play_alarm_sound()
            except Exception as e:
                print(f"Sound alarm error: {e}")

    def play_alarm_sound(self):
        """Play alarm sound"""
        try:
            frequency = 1000
            duration = 0.5
            sample_rate = 22050
            frames = int(duration * sample_rate)
            arr = np.sin(2 * np.pi * frequency * np.linspace(0, duration, frames))
            arr = (arr * 32767).astype(np.int16)
            sound = pygame.sndarray.make_sound(arr)
            sound.play()
        except:
            pass

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
        if not self.logging_enabled or not self.current_log_file:
            return
            
        current_time = time.time()
        if (current_time - self.last_log_time) >= self.log_interval:
            try:
                timestamp = datetime.now().isoformat()
                
                # Ensure servo_angles is a list of 3 values
                if not isinstance(servo_angles, (list, tuple)) or len(servo_angles) != 3:
                    servo_angles = [0, 0, 0]
                
                with open(self.current_log_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([timestamp, force, threshold, 
                                   servo_angles[0], servo_angles[1], servo_angles[2], mode])
                
                self.last_log_time = current_time
            except Exception as e:
                print(f"Logging error: {e}")

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
            pass  # Use defaults silently

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
        """Load default configuration with much more aggressive PID settings"""
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
                'kp': 2.0,    # Much more aggressive - increased from 0.5
                'ki': 0.2,    # Much more aggressive - increased from 0.05
                'kd': 0.3     # Much more aggressive - increased from 0.1
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
            pass  # Use defaults silently
    
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
        
        # Initialize predefined sequences
        self.create_default_sequences()
    
    def create_default_sequences(self):
        """Create default test sequences"""
        self.sequences = {
            "Basic Force Test": [
                {"type": "force", "target": 250, "duration": 3},
                {"type": "force", "target": 500, "duration": 5},
                {"type": "force", "target": 750, "duration": 3},
                {"type": "force", "target": 500, "duration": 2}
            ],
            "Ramp Test": [
                {"type": "force", "target": 200, "duration": 2},
                {"type": "force", "target": 400, "duration": 3},
                {"type": "force", "target": 600, "duration": 3},
                {"type": "force", "target": 800, "duration": 3},
                {"type": "force", "target": 1000, "duration": 4},
                {"type": "force", "target": 0, "duration": 2}
            ],
            "Cyclic Test": [
                {"type": "force", "target": 300, "duration": 2},
                {"type": "force", "target": 700, "duration": 2},
                {"type": "force", "target": 300, "duration": 2},
                {"type": "force", "target": 700, "duration": 2},
                {"type": "force", "target": 300, "duration": 2},
                {"type": "force", "target": 0, "duration": 1}
            ],
            "Custom": [
                {"type": "force", "target": 500, "duration": 5}
            ]
        }
    
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
        self.stepper = StepperMotor()
        self.servo_ctrl = ServoController()
        self.app = None

        # Auto-detect load cell serial port
        detected_port = detect_load_cell_port()
        if detected_port and test_load_cell_communication(detected_port):
            SERIAL_PORT = detected_port
        else:
            SERIAL_PORT = '/dev/ttyUSB0'

        BAUDRATE = 9600
        FORCE_TARGET = 500
        FORCE_TOLERANCE = 10

        self.shared = SharedData(FORCE_TARGET, FORCE_TOLERANCE)

        # Serial port handling
        self.ser = None
        self.serial_port = SERIAL_PORT
        self.baudrate = BAUDRATE
        
        connection_attempts = [
            (SERIAL_PORT, BAUDRATE),
            ('/dev/ttyUSB0', BAUDRATE),
            ('/dev/ttyUSB1', BAUDRATE),
            ('/dev/ttyACM0', BAUDRATE),
            ('/dev/ttyAMA0', BAUDRATE),
        ]
        
        for port, baud in connection_attempts:
            try:
                test_ser = serial.Serial(port, baud, timeout=1)
                if test_load_cell_communication(port, timeout=1):
                    self.ser = test_ser
                    self.serial_port = port
                    self.baudrate = baud
                    break
                else:
                    test_ser.close()
            except Exception:
                continue
        
        if self.ser is None and SYSLOG_AVAILABLE:
            try:
                syslog.openlog("rpi_control")
                syslog.syslog(syslog.LOG_WARNING, "No load cell found, running in simulation mode")
                syslog.closelog()
            except:
                pass

        self.running = True

        # PID parameters - aggressive for faster response
        self.pid_kp = 2.0
        self.pid_ki = 0.2
        self.pid_kd = 0.3
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

        # Initialize components
        self.safety_manager = SafetyManager()
        self.data_logger = DataLogger()
        self.calibration_manager = CalibrationManager()
        self.config_manager = ConfigManager()
        self.sequence_manager = SequenceManager()
        
        self.apply_config()
        
        # Initialize data for plotting
        self.force_history = deque(maxlen=1000)
        self.time_history = deque(maxlen=1000)
        self.plot_start_time = time.time()
        
        # Rate limiting for automatic control
        self.last_auto_adjustment = 0
        self.auto_adjustment_interval = 0.1
        
        # Simulation variables
        self.simulated_force = 0.0
        self.simulated_position = 0

    def cleanup(self):
        cleanup(self.stepper, self.servo_ctrl, self.ser)
        self.running = False

    def set_app(self, app):
        """Set reference to App for logging"""
        self.app = app
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
        
        self.pid_kp = config['pid'].get('kp', 2.0)
        self.pid_ki = config['pid'].get('ki', 0.2)
        self.pid_kd = config['pid'].get('kd', 0.3)
        
        self.safety_manager.max_force = config['safety']['max_force']
        self.safety_manager.min_force = config['safety']['min_force']
        self.safety_manager.alarms_enabled = config['safety']['alarms_enabled']
        
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

    def update_simulated_force(self, steps, direction):
        """Update simulated force with faster response"""
        force_per_step = 5.0
        
        if direction:
            force_change = steps * force_per_step
            self.simulated_force += force_change
        else:
            force_change = steps * force_per_step
            self.simulated_force -= force_change
        
        import random
        noise = random.uniform(-0.2, 0.2)
        self.simulated_force += noise
        
        self.simulated_force = max(0, min(10000, self.simulated_force))
        self.shared.current_force = self.simulated_force
        read_force.simulated_force = self.simulated_force

    def auto_adjust_force(self):
        """Enhanced automatic force adjustment"""
        if not self.auto_mode_enabled or not self.stepper_enabled or self.safety_manager.emergency_stop:
            return
            
        current_time = time.time()
        if current_time - self.last_auto_adjustment < 0.1:
            return
            
        current_force = self.shared.current_force
        target_force = self.shared.force_threshold
        tolerance = self.shared.force_tolerance
        
        if not self.safety_manager.check_force_limits(current_force):
            self.emergency_stop()
            return
            
        error = target_force - current_force
        
        if abs(error) <= tolerance:
            self.pid_integral *= 0.9
            self.log_status(f"Auto: Within tolerance (error: {error:.1f}g, tolerance: ±{tolerance}g)")
            return
            
        # PID calculation
        p_term = self.pid_kp * error * 3.0
        
        self.pid_integral += error * 0.5
        max_integral = 500
        self.pid_integral = max(-max_integral, min(max_integral, self.pid_integral))
        i_term = self.pid_ki * self.pid_integral
        
        derivative = error - self.pid_last_error
        d_term = self.pid_kd * derivative
        
        output = p_term + i_term + d_term
        
        direction = error > 0
        
        error_abs = abs(error)
        if error_abs > tolerance * 20:
            steps = min(100, max(50, int(error_abs / 5)))
        elif error_abs > tolerance * 10:
            steps = min(50, max(25, int(error_abs / 8)))
        elif error_abs > tolerance * 5:
            steps = min(30, max(15, int(error_abs / 10)))
        elif error_abs > tolerance * 2:
            steps = min(20, max(8, int(error_abs / 15)))
        else:
            steps = min(10, max(3, int(error_abs / 20)))
        
        if callable(self.stepper.step):
            self.stepper.step(steps, direction)
            action = "INCREASE" if direction else "DECREASE"
            self.log_status(f"Auto: {action} force by {steps} steps (error: {error:.1f}g, target: {target_force}g, current: {current_force:.1f}g)")
            
            if self.ser is None:
                self.update_simulated_force(steps, direction)
        
        self.pid_last_error = error
        self.last_auto_adjustment = current_time

    def update_force_history(self, force):
        """Update force history for plotting"""
        current_time = time.time() - self.plot_start_time
        self.force_history.append(force)
        self.time_history.append(current_time)

    def increase_force(self):
        if self.stepper_enabled and not self.auto_mode_enabled and not self.safety_manager.emergency_stop:
            self.stepper.step(10, True)
            self.log_status("Stepper turning anti-clockwise to increase force")
            # Update simulated force if in simulation mode
            if self.ser is None:
                self.update_simulated_force(10, True)

    def decrease_force(self):
        if self.stepper_enabled and not self.auto_mode_enabled and not self.safety_manager.emergency_stop:
            self.stepper.step(10, False)
            self.log_status("Stepper turning clockwise to decrease force")
            # Update simulated force if in simulation mode
            if self.ser is None:
                self.update_simulated_force(10, False)

    def get_force_reading(self):
        return self.shared.current_force

    def log_status(self, message):
        """Log status message with timestamp"""
        try:
            timestamp = datetime.now().strftime("%H:%M:%S")
            log_message = f"[{timestamp}] {message}"
            
            # Use app's logging if available, otherwise print
            if self.app and hasattr(self.app, 'log_status'):
                self.app.log_status(message)
            else:
                print(log_message)
        except Exception as e:
            print(f"[{datetime.now().strftime('%H:%M:%S')}] Logging error: {e}")

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
        
        self.shutting_down = False
        self.force_update_id = None
        self.plot_update_id = None
        
        self.setup_display()
        
        self.root.bind('<Escape>', self.toggle_fullscreen)
        self.root.bind('<F11>', self.toggle_fullscreen)
        
        self.controller = Controller()
        
        # --- Servo and stepper controls: all servos disabled at startup ---
        self.servo_control_enabled = tk.BooleanVar(value=False)
        self.stepper_control_enabled = tk.BooleanVar(value=True)
        self.auto_stepper_mode = tk.BooleanVar(value=False)

        self.servo_presets = [0, 30, 45, 60, 90, 120, 135, 150, 180]
        self.stepper_presets = [1, 5, 10, 25, 50, 100]

        self.servo_preset_dropdowns = []
        self._force_error_logged = False
        self._plot_error_logged = False

        self.logging_enabled = tk.BooleanVar(value=False)
        self.step_start_time = None  # <-- Initialize here

        # --- Individual servo enable checkboxes: all disabled at startup ---
        self.servo1_enabled = tk.BooleanVar(value=False)
        self.servo2_enabled = tk.BooleanVar(value=False)
        self.servo3_enabled = tk.BooleanVar(value=False)

        self.setup_gui()
        self.setup_plotting()
        self.start_background_threads()
        
        self.update_force_and_thresh()
        self.update_plot()
        
        self.controller.set_app(self)
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def setup_display(self):
        """Setup display configuration"""
        try:
            import subprocess
            result = subprocess.run(['cat', '/proc/device-tree/model'], 
                                  capture_output=True, text=True, timeout=2)
            if 'Raspberry Pi' in result.stdout:
                self.root.geometry("800x600")
                self.is_raspberry_pi = True
            else:
                try:
                    self.root.attributes('-fullscreen', True)
                    self.root.state('zoomed')
                except tk.TclError:
                    self.root.geometry("1200x800")
                self.is_raspberry_pi = False
        except:
            self.root.geometry("1024x768")
            self.is_raspberry_pi = False

    def setup_gui(self):
        """Setup the main GUI components"""
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True)

        self.create_main_tab(notebook)
        self.create_settings_tab(notebook)
        self.create_tests_tab(notebook)
        self.create_calibration_tab(notebook)
        self.create_data_logging_tab(notebook)
        self.create_safety_tab(notebook)
        self.create_sequence_tab(notebook)

    def create_main_tab(self, notebook):
        """Create main control tab"""
        main_frame = tk.Frame(notebook)
        notebook.add(main_frame, text="Main")

        # Live force display
        self.live_force_var = tk.StringVar(value="0.00")
        self.force_label = tk.Label(main_frame, text="Live Force (g):", font=("Arial", 12))
        self.force_label.pack(pady=2)
        self.force_value_box = tk.Entry(main_frame, textvariable=self.live_force_var, 
                                       font=("Arial", 12), state="readonly", width=12)
        self.force_value_box.pack(pady=2)

        # Force threshold display with adjustment buttons
        threshold_frame = tk.Frame(main_frame)
        threshold_frame.pack(pady=5)
        
        self.force_thresh_var = tk.StringVar(value=str(self.controller.shared.force_threshold))
        self.force_thresh_label = tk.Label(threshold_frame, text="Force Threshold (g):", font=("Arial", 12))
        self.force_thresh_label.pack(side=tk.LEFT, padx=5)
        
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

        # Servo controls
        self.create_servo_controls(main_frame)
        
        # Stepper controls
        self.create_stepper_controls(main_frame)
        
        # Status log
        self.create_status_log(main_frame)

    def create_servo_controls(self, parent):
        """Create servo control interface"""
        servo_control_frame = tk.LabelFrame(parent, text="Servo Controls")
        servo_control_frame.pack(pady=10, padx=10, fill="x")

        self.servo_angle_vars = []
        self.servo_scales = []
        
        for i in range(3):
            servo_frame = tk.Frame(servo_control_frame)
            servo_frame.pack(fill="x", padx=5, pady=2)
            
            tk.Label(servo_frame, text=f"Servo {i+1} Angle:", width=12).pack(side=tk.LEFT)
            
            angle_var = tk.IntVar(value=90)
            self.servo_angle_vars.append(angle_var)
            
            scale = tk.Scale(servo_frame, from_=0, to=180, orient=tk.HORIZONTAL, 
                           variable=angle_var)
            scale.pack(side=tk.LEFT, fill="x", expand=True, padx=5)
            self.servo_scales.append(scale)
            
            angle_display = tk.Label(servo_frame, text="90°", width=4, relief="sunken")
            angle_display.pack(side=tk.LEFT, padx=2)
            
            preset_var = tk.StringVar(value="90°")
            preset_dropdown = ttk.Combobox(servo_frame, textvariable=preset_var, width=6, state="readonly")
            preset_dropdown['values'] = [f"{preset}°" for preset in self.servo_presets]
            preset_dropdown.pack(side=tk.LEFT, padx=2)
            preset_dropdown.bind('<<ComboboxSelected>>', lambda event, idx=i: self.servo_preset_selected(event, idx))
            self.servo_preset_dropdowns.append(preset_dropdown)
            
            # Update callbacks
            def create_callbacks(idx, display):
                def update_display_and_servo(val):
                    display.config(text=f"{int(float(val))}°")
                    self.set_servo_angle(idx, val)
                return update_display_and_servo
            
            callback = create_callbacks(i, angle_display)
            scale.config(command=callback)

    def create_stepper_controls(self, parent):
        """Create stepper control interface"""
        stepper_control_frame = tk.LabelFrame(parent, text="Stepper Control")
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

        # Manual controls
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

        # Quick step presets
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

        # Emergency controls
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

    def create_status_log(self, parent):
        """Create status log interface"""
        status_log_frame = tk.LabelFrame(parent, text="Status Log")
        status_log_frame.pack(pady=10, padx=10, fill="both", expand=True)
        
        self.status_log = tk.Text(status_log_frame, height=8, width=60, state=tk.DISABLED, 
                                 font=("Consolas", 9), wrap=tk.WORD)
        scrollbar = tk.Scrollbar(status_log_frame, orient=tk.VERTICAL, command=self.status_log.yview)
        self.status_log.config(yscrollcommand=scrollbar.set)
        
        self.status_log.pack(side=tk.LEFT, fill="both", expand=True)
        scrollbar.pack(side=tk.RIGHT, fill="y")
        
        clear_log_btn = tk.Button(status_log_frame, text="Clear Log", command=self.clear_status_log)
        clear_log_btn.pack(pady=2)

    def create_settings_tab(self, notebook):
        """Create settings tab"""
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

    def create_tests_tab(self, notebook):
        """Create tests tab"""
        tests_frame = tk.Frame(notebook)
        notebook.add(tests_frame, text="Tests")

        # Create main container with scrollable area
        main_container = tk.Frame(tests_frame)
        main_container.pack(fill="both", expand=True, padx=10, pady=10)

        # Test results display
        results_frame = tk.LabelFrame(main_container, text="Test Results")
        results_frame.pack(fill="both", expand=True, pady=(0, 10))

        self.test_results_text = tk.Text(results_frame, height=15, width=80, state=tk.DISABLED, 
                                        font=("Consolas", 9), wrap=tk.WORD)
        test_scrollbar = tk.Scrollbar(results_frame, orient=tk.VERTICAL, command=self.test_results_text.yview)
        self.test_results_text.config(yscrollcommand=test_scrollbar.set)
        
        self.test_results_text.pack(side=tk.LEFT, fill="both", expand=True)
        test_scrollbar.pack(side=tk.RIGHT, fill="y")

        # Test control buttons
        button_frame = tk.LabelFrame(main_container, text="Test Controls")
        button_frame.pack(fill="x", pady=(0, 10))

        # Row 1: Basic tests
        row1 = tk.Frame(button_frame)
        row1.pack(fill="x", padx=5, pady=5)
        
        tk.Button(row1, text="Run Unit Tests", command=self.run_unit_tests_gui,
                 bg="lightblue", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)

        tk.Button(row1, text="Run System Tests", command=self.run_system_tests_gui,
                 bg="lightgreen", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)

        tk.Button(row1, text="Run Functionality Check", command=self.run_functionality_check_gui,
                 bg="lightyellow", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)

        # Row 2: Component tests
        row2 = tk.Frame(button_frame)
        row2.pack(fill="x", padx=5, pady=5)

        tk.Button(row2, text="Test Stepper Motor", command=self.test_stepper_motor,
                 bg="lightcoral", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)

        tk.Button(row2, text="Test Servo Motors", command=self.test_servo_motors,
                 bg="lightcyan", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)

        tk.Button(row2, text="Test Force Reading", command=self.test_force_reading,
                 bg="lightpink", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)

        # Row 3: Advanced tests
        row3 = tk.Frame(button_frame)
        row3.pack(fill="x", padx=5, pady=5)

        tk.Button(row3, text="Test PID Control", command=self.test_pid_control,
                 bg="lightsteelblue", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)

        tk.Button(row3, text="Test Safety Systems", command=self.test_safety_systems,
                 bg="lightsalmon", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)

        tk.Button(row3, text="Run All Tests", command=self.run_all_tests,
                 bg="lightgray", font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=5)

        # Row 4: Utility buttons
        row4 = tk.Frame(button_frame)
        row4.pack(fill="x", padx=5, pady=5)

        tk.Button(row4, text="Clear Results", command=self.clear_test_results,
                 bg="white", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)

        tk.Button(row4, text="Save Results", command=self.save_test_results,
                 bg="lightgoldenrodyellow", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)

    def run_unit_tests(self):
        """Run unit tests and return results"""
        results = []
        try:
            # Test stepper motor initialization
            stepper = StepperMotor()
            results.append("✅ StepperMotor initialization: PASS")
            
            # Test servo controller initialization
            servo_ctrl = ServoController()
            results.append("✅ ServoController initialization: PASS")
            
            # Test safety manager initialization
            safety = SafetyManager()
            results.append("✅ SafetyManager initialization: PASS")
            
            # Test data logger initialization
            logger = DataLogger()
            results.append("✅ DataLogger initialization: PASS")
            
            # Test calibration manager initialization
            cal_mgr = CalibrationManager()
            results.append("✅ CalibrationManager initialization: PASS")
            
            # Test config manager initialization
            config_mgr = ConfigManager()
            results.append("✅ ConfigManager initialization: PASS")
            
            # Test sequence manager initialization
            seq_mgr = SequenceManager()
            results.append("✅ SequenceManager initialization: PASS")
            
            # Test shared data initialization
            shared = SharedData(500, 10)
            results.append("✅ SharedData initialization: PASS")
            
            # Test force reading function
            force = read_force(None)  # Test simulation mode
            if force is not None:
                results.append("✅ Force reading function: PASS")
            else:
                results.append("❌ Force reading function: FAIL")
            
            # Test port detection function
            try:
                detect_load_cell_port()
                results.append("✅ Port detection function: PASS")
            except Exception as e:
                results.append(f"❌ Port detection function: FAIL - {e}")
            
            results.append("\n📊 Unit Tests Summary:")
            results.append("All basic component initializations tested")
            
        except Exception as e:
            results.append(f"❌ Unit test failed: {e}")
            results.append(traceback.format_exc())
        
        return results

    def run_system_tests(self):
        """Run system tests and return results"""
        results = []
        try:
            # Test force reading with controller
            force = self.controller.read_force_with_calibration() or 0.0
            results.append(f"✅ Force reading test: {force:.2f}g")
            
            # Test servo control
            try:
                self.controller.servo_ctrl.set_angle(0, 90)
                results.append("✅ Servo control test: PASS")
            except Exception as e:
                results.append(f"❌ Servo control test: FAIL - {e}")
            
            # Test stepper control
            try:
                self.controller.stepper.step(1, True)
                results.append("✅ Stepper control test: PASS")
            except Exception as e:
                results.append(f"❌ Stepper control test: FAIL - {e}")
            
            # Test safety limits
            try:
                safe = self.controller.safety_manager.check_force_limits(1000)
                results.append(f"✅ Safety limits test: {'PASS' if safe else 'FAIL'}")
            except Exception as e:
                results.append(f"❌ Safety limits test: FAIL - {e}")
            
            # Test PID calculation
            try:
                self.controller.auto_adjust_force()
                results.append("✅ PID control test: PASS")
            except Exception as e:
                results.append(f"❌ PID control test: FAIL - {e}")
            
            # Test data logging
            try:
                log_file = self.controller.data_logger.start_logging("system_test")
                if log_file:
                    self.controller.data_logger.log_data(100, 150, [90, 90, 90], "Test")
                    self.controller.data_logger.stop_logging()
                    results.append("✅ Data logging test: PASS")
                else:
                    results.append("❌ Data logging test: FAIL")
            except Exception as e:
                results.append(f"❌ Data logging test: FAIL - {e}")
            
            # Test configuration management
            try:
                self.controller.config_manager.save_config()
                results.append("✅ Configuration management test: PASS")
            except Exception as e:
                results.append(f"❌ Configuration management test: FAIL - {e}")
            
            # Test calibration system
            try:
                self.controller.calibration_manager.save_calibration()
                results.append("✅ Calibration system test: PASS")
            except Exception as e:
                results.append(f"❌ Calibration system test: FAIL - {e}")
            
            # Test sequence management
            try:
                seq_started = self.controller.sequence_manager.start_sequence("Basic Force Test", self.controller)
                if seq_started:
                    self.controller.sequence_manager.stop_sequence()
                    results.append("✅ Sequence management test: PASS")
                else:
                    results.append("❌ Sequence management test: FAIL")
            except Exception as e:
                results.append(f"❌ Sequence management test: FAIL - {e}")
            
            results.append("\n📊 System Tests Summary:")
            results.append("All system integration tests completed")
            
        except Exception as e:
            results.append(f"❌ System test failed: {e}")
            results.append(traceback.format_exc())
    
        return results

    def run_unit_tests_gui(self):
        """Run unit tests and display results in GUI"""
        try:
            self.test_results_text.config(state=tk.NORMAL)
            self.test_results_text.insert(tk.END, "\n--- UNIT TESTS ---\n")
            
            results = self.run_unit_tests()
            for result in results:
                self.test_results_text.insert(tk.END, result + "\n")
                
            self.test_results_text.see(tk.END)
            self.test_results_text.config(state=tk.DISABLED)
            self.log_status("Unit tests completed")
        except Exception as e:
            self.log_status(f"Error running unit tests: {e}")

    def run_system_tests_gui(self):
        """Run system tests and display results in GUI"""
        try:
            self.test_results_text.config(state=tk.NORMAL)
            self.test_results_text.insert(tk.END, "\n--- SYSTEM TESTS ---\n")
            
            results = self.run_system_tests()
            for result in results:
                self.test_results_text.insert(tk.END, result + "\n")
                
            self.test_results_text.see(tk.END)
            self.test_results_text.config(state=tk.DISABLED)
            self.log_status("System tests completed")
        except Exception as e:
            self.log_status(f"Error running system tests: {e}")

    def run_functionality_check_gui(self):
        """Run comprehensive functionality check and display results in GUI"""
        try:
            self.test_results_text.config(state=tk.NORMAL)
            self.test_results_text.insert(tk.END, "\n" + "="*60 + "\n")
            self.test_results_text.insert(tk.END, "FUNCTIONALITY CHECK STARTED\n")
            self.test_results_text.insert(tk.END, "="*60 + "\n")
            
            results = self.run_comprehensive_functionality_check()
            for result in results:
                self.test_results_text.insert(tk.END, result + "\n")
                
            self.test_results_text.see(tk.END)
            self.test_results_text.config(state=tk.DISABLED)
            self.log_status("Functionality check completed")
        except Exception as e:
            self.log_status(f"Error running functionality check: {e}")

    def run_comprehensive_functionality_check(self):
        """Comprehensive functionality check"""
        results = []
        
        try:
            # Header
            results.append(f"Functionality Check - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            results.append("-" * 50)
            
            # 1. Hardware Detection Tests
            results.append("\n🔧 HARDWARE DETECTION TESTS:")
            results.append("-" * 30)
            
            # GPIO availability
            if GPIO_AVAILABLE:
                results.append("✅ GPIO library available")
            else:
                results.append("⚠️  GPIO library not available (simulation mode)")
            
            # Serial port detection
            detected_port = detect_load_cell_port()
            if detected_port:
                results.append(f"✅ Load cell port detected: {detected_port}")
            else:
                results.append("⚠️  No load cell port detected (simulation mode)")
            
            # Test serial communication
            if self.controller.ser:
                results.append(f"✅ Serial communication active on {self.controller.serial_port}")
            else:
                results.append("⚠️  No serial communication (simulation mode)")
            
            # 2. Component Tests
            results.append("\n🔨 COMPONENT TESTS:")
            results.append("-" * 20)
            
            # Test stepper motor
            try:
                if self.controller.stepper.hw_available:
                    results.append("✅ Stepper motor initialized successfully")
                else:
                    results.append("⚠️  Stepper motor in simulation mode")
            except Exception as e:
                results.append(f"❌ Stepper motor test failed: {e}")
            
            # Test servo controller
            try:
                if self.controller.servo_ctrl.hw_available:
                    results.append("✅ Servo controller initialized successfully")
                else:
                    results.append("⚠️  Servo controller in simulation mode")
            except Exception as e:
                results.append(f"❌ Servo controller test failed: {e}")
            
            # 3. Force Reading Tests
            results.append("\n📊 FORCE READING TESTS:")
            results.append("-" * 25)
            
            force_readings = []
            for i in range(5):
                force = read_force(self.controller.ser)
                if force is not None:
                    force_readings.append(force)
                time.sleep(0.1)
            
            if force_readings:
                avg_force = sum(force_readings) / len(force_readings)
                results.append(f"✅ Force readings successful: {len(force_readings)}/5")
                results.append(f"   Average: {avg_force:.2f}g")
            else:
                results.append("❌ No force readings obtained")
            
            # 4. Control System Tests
            results.append("\n🎮 CONTROL SYSTEM TESTS:")
            results.append("-" * 25)
            
            # Test stepper control
            try:
                original_force = self.controller.shared.current_force
                self.controller.stepper.step(5, True)
                time.sleep(0.2)
                
                if self.controller.ser is None:
                    self.controller.update_simulated_force(5, True)
                
                new_force = self.controller.shared.current_force
                results.append(f"✅ Stepper control test: {original_force:.2f}g → {new_force:.2f}g")
            except Exception as e:
                results.append(f"❌ Stepper control test failed: {e}")
            
            # Test servo control
            try:
                for i in range(3):
                    self.controller.servo_ctrl.set_angle(i, 90)
                results.append("✅ Servo control test: All servos set to 90°")
            except Exception as e:
                results.append(f"❌ Servo control test failed: {e}")
            
            # 5. Safety System Tests
            results.append("\n🛡️ SAFETY SYSTEMS TESTS:")
            results.append("-" * 22)
            
            try:
                safe_force = self.controller.safety_manager.check_force_limits(1000)
                unsafe_force = self.controller.safety_manager.check_force_limits(15000)
                
                if safe_force and not unsafe_force:
                    results.append("✅ Force limit checking works correctly")
                else:
                    results.append("❌ Force limit checking failed")
            except Exception as e:
                results.append(f"❌ Safety system test failed: {e}")
            
            # 6. Summary
            results.append("\n📋 FUNCTIONALITY CHECK SUMMARY:")
            results.append("-" * 30)
            
            success_count = sum(1 for r in results if r.startswith("✅"))
            warning_count = sum(1 for r in results if r.startswith("⚠️"))
            failure_count = sum(1 for r in results if r.startswith("❌"))
            
            results.append(f"✅ Successful tests: {success_count}")
            results.append(f"⚠️  Warnings: {warning_count}")
            results.append(f"❌ Failed tests: {failure_count}")
            
            if failure_count == 0:
                results.append("\n🎉 ALL FUNCTIONALITY CHECKS PASSED!")
            elif warning_count > 0 and failure_count == 0:
                results.append("\n✅ FUNCTIONALITY CHECKS PASSED WITH WARNINGS")
            else:
                results.append(f"\n⚠️  FUNCTIONALITY CHECKS COMPLETED WITH {failure_count} FAILURES")
            
        except Exception as e:
            results.append(f"❌ Functionality check error: {e}")
            results.append(traceback.format_exc())
        
        return results

    def test_stepper_motor(self):
        """Test stepper motor functionality"""
        try:
            self.test_results_text.config(state=tk.NORMAL)
            self.test_results_text.insert(tk.END, "\n--- STEPPER MOTOR TEST ---\n")
            
            # Test initialization
            if self.controller.stepper.hw_available:
                self.test_results_text.insert(tk.END, "✅ Stepper hardware available\n")
            else:
                self.test_results_text.insert(tk.END, "⚠️  Stepper in simulation mode\n")
            
            # Test stepping
            original_force = self.controller.shared.current_force
            self.controller.stepper.step(10, True)
            time.sleep(0.5)
            if self.controller.ser is None:
                self.controller.update_simulated_force(10, True)
            new_force = self.controller.shared.current_force
            
            self.test_results_text.insert(tk.END, f"Force change: {original_force:.2f}g → {new_force:.2f}g\n")
            self.test_results_text.insert(tk.END, "✅ Stepper motor test completed\n")
            
            self.test_results_text.see(tk.END)
            self.test_results_text.config(state=tk.DISABLED)
            
        except Exception as e:
            self.test_results_text.insert(tk.END, f"❌ Stepper test failed: {e}\n")
            self.test_results_text.config(state=tk.DISABLED)

    def test_servo_motors(self):
        """Test servo motor functionality"""
        try:
            self.test_results_text.config(state=tk.NORMAL)
            self.test_results_text.insert(tk.END, "\n--- SERVO MOTORS TEST ---\n")
            
            test_angles = [0, 90, 180, 90]
            for angle in test_angles:
                for i in range(3):
                    self.controller.servo_ctrl.set_angle(i, angle)
                self.test_results_text.insert(tk.END, f"Set all servos to {angle}°\n")
                time.sleep(0.5)
            
            self.test_results_text.insert(tk.END, "✅ Servo motors test completed\n")
            self.test_results_text.see(tk.END)
            self.test_results_text.config(state=tk.DISABLED)
            
        except Exception as e:
            self.test_results_text.insert(tk.END, f"❌ Servo test failed: {e}\n")
            self.test_results_text.config(state=tk.DISABLED)

    def test_force_reading(self):
        """Test force reading functionality"""
        try:
            self.test_results_text.config(state=tk.NORMAL)
            self.test_results_text.insert(tk.END, "\n--- FORCE READING TEST ---\n")
            
            readings = []
            for i in range(10):
                force = read_force(self.controller.ser)
                if force is not None:
                    readings.append(force)
                    self.test_results_text.insert(tk.END, f"Reading {i+1}: {force:.2f}g\n")
                time.sleep(0.1)
            
            if readings:
                avg = sum(readings) / len(readings)
                self.test_results_text.insert(tk.END, f"Average force: {avg:.2f}g\n")
                self.test_results_text.insert(tk.END, f"✅ Got {len(readings)}/10 readings\n")
            else:
                self.test_results_text.insert(tk.END, "❌ No force readings obtained\n")
            
            self.test_results_text.see(tk.END)
            self.test_results_text.config(state=tk.DISABLED)
            
        except Exception as e:
            self.test_results_text.insert(tk.END, f"❌ Force reading test failed: {e}\n")
            self.test_results_text.config(state=tk.DISABLED)

    def test_pid_control(self):
        """Test PID control functionality"""
        try:
            self.test_results_text.config(state=tk.NORMAL)
            self.test_results_text.insert(tk.END, "\n--- PID CONTROL TEST ---\n")
            
            # Save original state
            original_auto = self.controller.auto_mode_enabled
            original_threshold = self.controller.shared.force_threshold
            
            # Enable auto mode
            self.controller.auto_mode_enabled = True
            self.controller.shared.force_threshold = 500
            
            self.test_results_text.insert(tk.END, "Testing PID control for 5 cycles...\n")
            
            for i in range(5):
                error_before = abs(self.controller.shared.force_threshold - self.controller.shared.current_force)
                self.controller.auto_adjust_force()
                error_after = abs(self.controller.shared.force_threshold - self.controller.shared.current_force)
                
                self.test_results_text.insert(tk.END, f"Cycle {i+1}: Error {error_before:.1f}g → {error_after:.1f}g\n")
                time.sleep(0.2)
            
            # Restore original state
            self.controller.auto_mode_enabled = original_auto
            self.controller.shared.force_threshold = original_threshold
            
            self.test_results_text.insert(tk.END, "✅ PID control test completed\n")
            self.test_results_text.see(tk.END)
            self.test_results_text.config(state=tk.DISABLED)
            
        except Exception as e:
            self.test_results_text.insert(tk.END, f"❌ PID control test failed: {e}\n")
            self.test_results_text.config(state=tk.DISABLED)

    def test_safety_systems(self):
        """Test safety system functionality"""
        try:
            self.test_results_text.config(state=tk.NORMAL)
            self.test_results_text.insert(tk.END, "\n--- SAFETY SYSTEMS TEST ---\n")
            
            # Test force limits
            safe_result = self.controller.safety_manager.check_force_limits(1000)
            unsafe_result = self.controller.safety_manager.check_force_limits(15000)
            
            self.test_results_text.insert(tk.END, f"Safe force (1000g): {'✅ PASS' if safe_result else '❌ FAIL'}\n")
            self.test_results_text.insert(tk.END, f"Unsafe force (15000g): {'✅ PASS' if not unsafe_result else '❌ FAIL'}\n")
            
            # Test emergency stop
            self.controller.emergency_stop()
            estop_active = self.controller.safety_manager.emergency_stop
            self.test_results_text.insert(tk.END, f"Emergency stop: {'✅ ACTIVATED' if estop_active else '❌ FAILED'}\n")
            
            # Reset emergency stop
            self.controller.reset_emergency_stop()
            estop_reset = not self.controller.safety_manager.emergency_stop
            self.test_results_text.insert(tk.END, f"Emergency reset: {'✅ SUCCESS' if estop_reset else '❌ FAILED'}\n")
            
            self.test_results_text.insert(tk.END, "✅ Safety systems test completed\n")
            self.test_results_text.see(tk.END)
            self.test_results_text.config(state=tk.DISABLED)
            
        except Exception as e:
            self.test_results_text.insert(tk.END, f"❌ Safety test failed: {e}\n")
            self.test_results_text.config(state=tk.DISABLED)

    def run_all_tests(self):
        """Run all available tests"""
        self.clear_test_results()
        
        self.test_results_text.config(state=tk.NORMAL)
        self.test_results_text.insert(tk.END, "RUNNING ALL TESTS...\n")
        self.test_results_text.insert(tk.END, "="*60 + "\n")
        self.test_results_text.config(state=tk.DISABLED)
        
        # Run each test with a small delay
        tests = [
            self.run_unit_tests_gui,
            self.run_system_tests_gui,
            self.run_functionality_check_gui,
            self.test_stepper_motor,
            self.test_servo_motors,
            self.test_force_reading,
            self.test_pid_control,
            self.test_safety_systems
        ]
        
        for test in tests:
            try:
                test()
                time.sleep(0.5)  # Small delay between tests
            except Exception as e:
                self.test_results_text.config(state=tk.NORMAL)
                self.test_results_text.insert(tk.END, f"❌ Test failed: {e}\n")
                self.test_results_text.config(state=tk.DISABLED)
        
        self.test_results_text.config(state=tk.NORMAL)
        self.test_results_text.insert(tk.END, "\n" + "="*60 + "\n")
        self.test_results_text.insert(tk.END, "ALL TESTS COMPLETED\n")
        self.test_results_text.see(tk.END)
        self.test_results_text.config(state=tk.DISABLED)

    def clear_test_results(self):
        """Clear the test results display"""
        self.test_results_text.config(state=tk.NORMAL)
        self.test_results_text.delete(1.0, tk.END)
        self.test_results_text.config(state=tk.DISABLED)

    def save_test_results(self):
        """Save test results to file"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"test_results_{timestamp}.txt"
            filepath = os.path.join("test_results", filename)
            
            # Create directory if it doesn't exist
            os.makedirs("test_results", exist_ok=True)
            
            # Get all text from results display
            results_text = self.test_results_text.get(1.0, tk.END)
            
            with open(filepath, 'w') as f:
                f.write(f"RPi Control Test Results\n")
                f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write("="*60 + "\n\n")
                f.write(results_text)
            
            self.log_status(f"Test results saved to: {filepath}")
            
        except Exception as e:
            self.log_status(f"Error saving test results: {e}")

    def create_calibration_tab(self, notebook):
        """Create calibration tab"""
        cal_frame = tk.Frame(notebook)
        notebook.add(cal_frame, text="Calibration")

        # Calibration instructions
        inst_frame = tk.LabelFrame(cal_frame, text="Calibration Instructions")
        inst_frame.pack(padx=10, pady=10, fill="x")
        
        instructions = """
1. Remove all weight from the load cell and click 'Zero Calibration'
2. Place known weights on the load cell
3. Enter the weight value and click 'Add Calibration Point'
4. Repeat with different weights for better accuracy
5. Click 'Apply Calibration' to save the calibration
        """
        tk.Label(inst_frame, text=instructions, justify=tk.LEFT, font=("Arial", 9)).pack(padx=10, pady=5)

        # Current calibration info
        info_frame = tk.LabelFrame(cal_frame, text="Current Calibration")
        info_frame.pack(padx=10, pady=5, fill="x")
        
        tk.Label(info_frame, text=f"Zero Offset: {self.controller.calibration_manager.zero_offset:.2f}").pack(anchor="w", padx=5)
        tk.Label(info_frame, text=f"Scale Factor: {self.controller.calibration_manager.scale_factor:.4f}").pack(anchor="w", padx=5)

        # Calibration controls
        control_frame = tk.LabelFrame(cal_frame, text="Calibration Controls")
        control_frame.pack(padx=10, pady=5, fill="x")
        
        tk.Label(control_frame, text="Known Weight (g):").grid(row=0, column=0, sticky="e", padx=5)
        self.cal_weight_var = tk.DoubleVar(value=0)
        tk.Entry(control_frame, textvariable=self.cal_weight_var, width=10).grid(row=0, column=1, padx=5)
        
        tk.Button(control_frame, text="Zero Calibration", command=self.zero_calibration).grid(row=0, column=2, padx=5)
        tk.Button(control_frame, text="Add Cal Point", command=self.add_calibration_point).grid(row=0, column=3, padx=5)
        tk.Button(control_frame, text="Apply Calibration", command=self.apply_calibration).grid(row=0, column=4, padx=5)

        # Calibration results
        results_frame = tk.LabelFrame(cal_frame, text="Calibration Points")
        results_frame.pack(padx=10, pady=5, fill="both", expand=True)
        
        self.cal_results_text = tk.Text(results_frame, height=10, width=60, state=tk.DISABLED, font=("Consolas", 9))
        cal_scrollbar = tk.Scrollbar(results_frame, orient=tk.VERTICAL, command=self.cal_results_text.yview)
        self.cal_results_text.config(yscrollcommand=cal_scrollbar.set)
        
        self.cal_results_text.pack(side=tk.LEFT, fill="both", expand=True)
        cal_scrollbar.pack(side=tk.RIGHT, fill="y")

    def create_data_logging_tab(self, notebook):
        """Create data logging tab"""
        log_frame = tk.Frame(notebook)
        notebook.add(log_frame, text="Data Logging")

        # Logging controls
        control_frame = tk.LabelFrame(log_frame, text="Logging Controls")
        control_frame.pack(padx=10, pady=10, fill="x")
        
        tk.Label(control_frame, text="Test Name:").grid(row=0, column=0, sticky="e", padx=5)
        self.test_name_var = tk.StringVar(value="test")
        tk.Entry(control_frame, textvariable=self.test_name_var, width=20).grid(row=0, column=1, padx=5)
        
        self.logging_enabled = tk.BooleanVar(value=False)
        tk.Checkbutton(control_frame, text="Enable Logging", variable=self.logging_enabled, 
                      command=self.toggle_logging).grid(row=0, column=2, padx=5)

        # Current log status
        status_frame = tk.LabelFrame(log_frame, text="Logging Status")
        status_frame.pack(padx=10, pady=5, fill="x")
        
        self.current_log_var = tk.StringVar(value="No active log")
        tk.Label(status_frame, textvariable=self.current_log_var, font=("Arial", 10)).pack(anchor="w", padx=5, pady=5)

        # Log file management
        file_frame = tk.LabelFrame(log_frame, text="Log File Management")
        file_frame.pack(padx=10, pady=5, fill="x")
        
        tk.Button(file_frame, text="View Log Files", command=self.view_log_files).pack(side=tk.LEFT, padx=5, pady=5)
        tk.Button(file_frame, text="Start Logging", command=self.start_logging).pack(side=tk.LEFT, padx=5, pady=5)
        tk.Button(file_frame, text="Stop Logging", command=self.stop_logging).pack(side=tk.LEFT, padx=5, pady=5)

        # Plotting section (if matplotlib available)
        if MATPLOTLIB_AVAILABLE and hasattr(self, 'plot_fig') and self.plot_fig:
            plot_frame = tk.LabelFrame(log_frame, text="Real-time Plot")
            plot_frame.pack(padx=10, pady=5, fill="both", expand=True)
            
            self.canvas = FigureCanvasTkAgg(self.plot_fig, plot_frame)
            self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def create_safety_tab(self, notebook):
        """Create safety tab"""
        safety_frame = tk.Frame(notebook)
        notebook.add(safety_frame, text="Safety")

        # Safety limits
        limits_frame = tk.LabelFrame(safety_frame, text="Force Limits")
        limits_frame.pack(padx=10, pady=10, fill="x")
        
        tk.Label(limits_frame, text="Max Force (g):").grid(row=0, column=0, sticky="e", padx=5)
        self.max_force_var = tk.IntVar(value=self.controller.safety_manager.max_force)
        tk.Entry(limits_frame, textvariable=self.max_force_var, width=10).grid(row=0, column=1, padx=5)
        
        tk.Label(limits_frame, text="Min Force (g):").grid(row=0, column=2, sticky="e", padx=5)
        self.min_force_var = tk.IntVar(value=self.controller.safety_manager.min_force)
        tk.Entry(limits_frame, textvariable=self.min_force_var, width=10).grid(row=0, column=3, padx=5)
        
        tk.Button(limits_frame, text="Apply Limits", command=self.apply_safety_limits).grid(row=0, column=4, padx=5)

        # Alarm settings
        alarm_frame = tk.LabelFrame(safety_frame, text="Alarm Settings")
        alarm_frame.pack(padx=10, pady=5, fill="x")
        
        self.alarms_enabled_var = tk.BooleanVar(value=self.controller.safety_manager.alarms_enabled)
        tk.Checkbutton(alarm_frame, text="Enable Audio Alarms", variable=self.alarms_enabled_var, 
                      command=self.toggle_alarms).pack(anchor="w", padx=5, pady=5)

        # Emergency controls
        emergency_frame = tk.LabelFrame(safety_frame, text="Emergency Controls")
        emergency_frame.pack(padx=10, pady=5, fill="x")
        
        tk.Button(emergency_frame, text="EMERGENCY STOP", command=self.controller.emergency_stop, 
                 bg="red", fg="white", font=("Arial", 14, "bold")).pack(side=tk.LEFT, padx=5, pady=5)
        tk.Button(emergency_frame, text="Reset Emergency Stop", command=self.controller.reset_emergency_stop, 
                 bg="orange", fg="white", font=("Arial", 12)).pack(side=tk.LEFT, padx=5, pady=5)

    def create_sequence_tab(self, notebook):
        """Create test sequence tab"""
        seq_frame = tk.Frame(notebook)
        notebook.add(seq_frame, text="Sequences")

        # Sequence selection
        select_frame = tk.LabelFrame(seq_frame, text="Test Sequences")
        select_frame.pack(padx=10, pady=10, fill="x")
        
        tk.Label(select_frame, text="Select Sequence:").grid(row=0, column=0, sticky="e", padx=5)
        self.sequence_var = tk.StringVar()
        sequence_combo = ttk.Combobox(select_frame, textvariable=self.sequence_var, 
                                     values=list(self.controller.sequence_manager.sequences.keys()), 
                                     state="readonly", width=20)
        sequence_combo.grid(row=0, column=1, padx=5)
        
        tk.Button(select_frame, text="Start Sequence", command=self.start_sequence).grid(row=0, column=2, padx=5)
        tk.Button(select_frame, text="Stop Sequence", command=self.stop_sequence).grid(row=0, column=3, padx=5)

        # Sequence status
        status_frame = tk.LabelFrame(seq_frame, text="Sequence Status")
        status_frame.pack(padx=10, pady=5, fill="x")
        
        self.seq_status_var = tk.StringVar(value="No sequence running")
        tk.Label(status_frame, textvariable=self.seq_status_var, font=("Arial", 10)).pack(anchor="w", padx=5, pady=5)

        # Sequence details
        details_frame = tk.LabelFrame(seq_frame, text="Sequence Details")
        details_frame.pack(padx=10, pady=5, fill="both", expand=True)
        
        self.seq_details_text = tk.Text(details_frame, height=15, width=60, state=tk.DISABLED, font=("Consolas", 9))
        seq_scrollbar = tk.Scrollbar(details_frame, orient=tk.VERTICAL, command=self.seq_details_text.yview)
        self.seq_details_text.config(yscrollcommand=seq_scrollbar.set)
        
        self.seq_details_text.pack(side=tk.LEFT, fill="both", expand=True)
        seq_scrollbar.pack(side=tk.RIGHT, fill="y")

    def zero_calibration(self):
        """Zero the calibration (set current reading as zero point)"""
        try:
            current_reading = self.controller.shared.current_force
            if current_reading is not None:
                self.controller.calibration_manager.zero_offset = current_reading
                self.log_status(f"Zero calibration set at {current_reading:.2f}")
            else:
                self.log_status("Error: No force reading available for zero calibration")
        except Exception as e:
            self.log_status(f"Error setting zero calibration: {e}")

    def add_calibration_point(self):
        """Add a calibration point"""
        try:
            weight = self.cal_weight_var.get()
            current_reading = self.controller.shared.current_force
            
            # Validate calibration inputs
            if weight < 0 or weight > 20000:
                self.log_status("Error: Calibration weight must be between 0 and 20000g")
                return
            if current_reading is None:
                self.log_status("Error: No force reading available for calibration")
                return
            
            self.controller.calibration_manager.add_calibration_point(weight, current_reading)
            
            # Update results display
            self.cal_results_text.config(state=tk.NORMAL)
            self.cal_results_text.insert(tk.END, f"Weight: {weight}g, Reading: {current_reading:.2f}g\n")
            self.cal_results_text.see(tk.END)
            self.cal_results_text.config(state=tk.DISABLED)
            
            self.log_status(f"Calibration point added: {weight}g -> {current_reading:.2f}g")
        except Exception as e:
            self.log_status(f"Error adding calibration point: {e}")

    def apply_calibration(self):
        """Apply and save calibration"""
        try:
            self.controller.calibration_manager.save_calibration()
            self.log_status("Calibration applied and saved")
        except Exception as e:
            self.log_status(f"Error applying calibration: {e}")

    def start_sequence(self):
        """Start selected test sequence"""
        try:
            sequence_name = self.sequence_var.get()
            if not sequence_name:
                self.log_status("Error: No sequence selected")
                return
                
            # Validate sequence steps
            sequence = self.controller.sequence_manager.sequences.get(sequence_name, [])
            if not sequence:
                self.log_status(f"Error: Sequence '{sequence_name}' is empty")
                return
                
            # Validate each step has required fields
            for i, step in enumerate(sequence):
                if not all(key in step for key in ['type', 'target', 'duration']):
                    self.log_status(f"Error: Step {i+1} is missing required fields")
                    return
                    
            success = self.controller.sequence_manager.start_sequence(sequence_name, self.controller)
            if success:
                self.seq_status_var.set(f"Running: {sequence_name}")
                
                # Display sequence details
                self.seq_details_text.config(state=tk.NORMAL)
                self.seq_details_text.delete(1.0, tk.END)
                
                # Show sequence steps
                for i, step in enumerate(sequence):
                    step_info = f"Step {i+1}: {step['type'].capitalize()} - Target: {step['target']}g, Duration: {step['duration']}s\n"
                    self.seq_details_text.insert(tk.END, step_info)
                    
                self.seq_details_text.config(state=tk.DISABLED)
                self.log_status(f"Started sequence: {sequence_name}")
                
                # Start sequence execution timer
                self.sequence_timer_id = self.root.after(100, self.execute_sequence_step)
            else:
                self.log_status(f"Failed to start sequence: {sequence_name}")
        except Exception as e:
            self.log_status(f"Error starting sequence: {e}")
            
    def execute_sequence_step(self):
        """Execute current step in the sequence and advance to next step when complete"""
        if not self.controller.sequence_manager.sequence_running:
            return
            
        try:
            # Get current sequence and step
            sequence = self.controller.sequence_manager.current_sequence
            step_idx = self.controller.sequence_manager.sequence_step
            
            if step_idx >= len(sequence):
                # Sequence complete
                self.complete_sequence()
                return
                
            # Get current step details
            step = sequence[step_idx]
            step_type = step.get('type', '')
            target = step.get('target', 0)
            duration = step.get('duration', 1)
            
            # Update the UI to show current step
            self.seq_status_var.set(f"Running step {step_idx+1}/{len(sequence)}: {step_type.capitalize()} - {target}g")
            
            # Highlight current step in the sequence details
            self.seq_details_text.config(state=tk.NORMAL)
            self.seq_details_text.tag_remove("current_step", "1.0", tk.END)
            
            # Find and tag the current step line
            line_start = f"{step_idx+1}.0"
            line_end = f"{step_idx+2}.0"
            self.seq_details_text.tag_add("current_step", line_start, line_end)
            self.seq_details_text.tag_config("current_step", background="yellow")
            self.seq_details_text.config(state=tk.DISABLED)
            
            # Execute step based on type
            if step_type == 'force':
                # Set the force threshold to the target
                self.controller.shared.force_threshold = target
                
                # Check if this is a newly started step
                if not hasattr(self, 'step_start_time') or self.step_start_time is None:
                    self.step_start_time = time.time()
                    self.controller.auto_mode_enabled = True
                    self.log_status(f"Step {step_idx+1}: Setting force to {target}g for {duration}s")
                
                # Calculate elapsed time
                elapsed = time.time() - self.step_start_time
                remaining = max(0, duration - elapsed)
                
                # Update status with remaining time
                self.seq_status_var.set(f"Step {step_idx+1}/{len(sequence)}: Force {target}g ({remaining:.1f}s remaining)")
                
                # Move to next step if duration is complete
                if elapsed >= duration:
                    self.step_start_time = None
                    self.controller.sequence_manager.sequence_step += 1
                    self.log_status(f"Step {step_idx+1} complete")
            
            # Schedule next check
            self.sequence_timer_id = self.root.after(100, self.execute_sequence_step)
            
        except Exception as e:
            self.log_status(f"Error executing sequence step: {e}")
            self.stop_sequence()
    
    def complete_sequence(self):
        """Handle sequence completion"""
        self.controller.sequence_manager.stop_sequence()
        self.controller.auto_mode_enabled = False
        self.seq_status_var.set("Sequence completed")
        
        # Clean up highlighting
        self.seq_details_text.config(state=tk.NORMAL)
        self.seq_details_text.tag_remove("current_step", "1.0", tk.END)
        self.seq_details_text.config(state=tk.DISABLED)
        
        self.log_status("Sequence completed successfully")
        
        # Optional: return to a safe state
        self.controller.shared.force_threshold = 0

    def stop_sequence(self):
        """Stop current test sequence"""
        try:
            # Cancel any pending timer
            if hasattr(self, 'sequence_timer_id') and self.sequence_timer_id:
                self.root.after_cancel(self.sequence_timer_id)
                self.sequence_timer_id = None
                
            # Reset step timing
            if hasattr(self, 'step_start_time'):
                self.step_start_time = None
                
            self.controller.sequence_manager.stop_sequence()
            self.controller.auto_mode_enabled = False
            self.seq_status_var.set("No sequence running")
            
            # Clean up highlighting
            self.seq_details_text.config(state=tk.NORMAL)
            self.seq_details_text.tag_remove("current_step", "1.0", tk.END)
            self.seq_details_text.config(state=tk.DISABLED)
            
            self.log_status("Sequence stopped")
        except Exception as e:
            self.log_status(f"Error stopping sequence: {e}")

    def setup_plotting(self):
        """Setup real-time plotting"""
        if MATPLOTLIB_AVAILABLE:
            try:
                self.plot_fig = Figure(figsize=(8, 4), dpi=100)
                self.plot_ax = self.plot_fig.add_subplot(111)
                self.plot_ax.set_title('Force vs Time')
                self.plot_ax.set_xlabel('Time (s)')
                self.plot_ax.set_ylabel('Force (g)')
                self.plot_line, = self.plot_ax.plot([], [], 'b-')
                self.plot_threshold_line, = self.plot_ax.plot([], [], 'r--')
                self.plot_ax.set_ylim(0, 1000)
                self.plot_ax.set_xlim(0, 60)
                self.plot_ax.grid(True)
                self.plot_fig.tight_layout()
            except Exception as e:
                if not self._plot_error_logged:
                    print(f"Error setting up plotting: {e}")
                    self._plot_error_logged = True
                self.plot_fig = None
        else:
            self.plot_fig = None

    def update_plot(self):
        """Update the real-time plot"""
        if self.shutting_down:
            return
            
        try:
            if MATPLOTLIB_AVAILABLE and self.plot_fig:
                # Get latest data
                times = list(self.controller.time_history)
                forces = list(self.controller.force_history)
                
                if times and forces:
                    # Update data
                    self.plot_line.set_data(times, forces)
                    
                    # Update threshold line
                    threshold = self.controller.shared.force_threshold
                    max_time = times[-1] if times else 60
                    self.plot_threshold_line.set_data([0, max_time], [threshold, threshold])
                    
                    # Adjust limits if needed
                    max_force = max(forces) if forces else 1000
                    min_force = min(forces) if forces else 0
                    margin = max(100, max_force * 0.1)
                    
                    if max_force + margin > self.plot_ax.get_ylim()[1]:
                        self.plot_ax.set_ylim(0, max_force + margin)
                    
                    if max_time > self.plot_ax.get_xlim()[1]:
                        self.plot_ax.set_xlim(0, max_time + 5)
                    
                    # Redraw
                    self.plot_fig.canvas.draw_idle()
                    self.plot_fig.canvas.flush_events()
        except Exception as e:
            if not self._plot_error_logged:
                print(f"Error updating plot: {e}")
                self._plot_error_logged = True
                
        # Schedule next update if not shutting down
        if not self.shutting_down:
            self.plot_update_id = self.root.after(1000, self.update_plot)

    def update_force_and_thresh(self):
        """Update force display and threshold on GUI"""
        if self.shutting_down:
            return
            
        try:
            # Read current force
            current_force = self.controller.get_force_reading()
            if current_force is not None:
                # Update force display
                self.live_force_var.set(f"{current_force:.1f}")
                
                # Update force threshold display
                self.force_thresh_var.set(f"{self.controller.shared.force_threshold:.1f}")
                self.settings_force_thresh_var.set(f"{self.controller.shared.force_threshold:.1f}")
                
                # Update force history for plotting
                self.controller.update_force_history(current_force)
                
                # Update force status indicator
                tolerance = self.controller.shared.force_tolerance
                target = self.controller.shared.force_threshold
                
                if abs(current_force - target) <= tolerance:
                    self.force_status_indicator.config(text="WITHIN RANGE", bg="green")
                elif current_force > target + tolerance:
                    self.force_status_indicator.config(text="ABOVE RANGE", bg="orange")
                else:
                    self.force_status_indicator.config(text="BELOW RANGE", bg="blue")
                
                # Auto-adjust force if enabled
                if self.controller.auto_mode_enabled:
                    self.controller.auto_adjust_force()
                    
                # Update data logging if enabled
                if self.controller.data_logger.logging_enabled:
                    servo_angles = [var.get() for var in self.servo_angle_vars] if hasattr(self, 'servo_angle_vars') else [0, 0, 0]
                    mode = "Auto" if self.controller.auto_mode_enabled else "Manual"
                    self.controller.data_logger.log_data(current_force, target, servo_angles, mode)
        except Exception as e:
            if not self._force_error_logged:
                print(f"Error updating force display: {e}")
                self._force_error_logged = True

        # Schedule next update if not shutting down
        if not self.shutting_down:
            self.force_update_id = self.root.after(100, self.update_force_and_thresh)

    def start_background_threads(self):
        """Start background processes"""
        # These are actually timer-based callbacks, not threads
        self.force_update_id = self.root.after(100, self.update_force_and_thresh)
        self.plot_update_id = self.root.after(1000, self.update_plot)

    def toggle_fullscreen(self, event=None):
        """Toggle fullscreen mode"""
        self.root.attributes('-fullscreen', not self.root.attributes('-fullscreen'))
        return "break"  # Prevents default handler

    def on_closing(self):
        """Handle window closing"""
        self.shutting_down = True
        
        # Cancel pending callbacks
        if self.force_update_id:
            self.root.after_cancel(self.force_update_id)
        if self.plot_update_id:
            self.root.after_cancel(self.plot_update_id)
            
        # Stop sequence if running
        if hasattr(self, 'sequence_timer_id') and self.sequence_timer_id:
            self.root.after_cancel(self.sequence_timer_id)
            
        # Clean up controller resources
        self.controller.cleanup()
        self.root.destroy()

    def stop_program(self):
        """Stop the program completely"""
        if tk.messagebox.askyesno("Confirm Exit", "Are you sure you want to exit?"):
            self.on_closing()

    def toggle_stepper_mode(self):
        """Toggle between manual and automatic stepper control"""
        auto_mode = self.auto_stepper_mode.get()
        self.controller.auto_mode_enabled = auto_mode
        
        # Update UI
        if auto_mode:
            self.stepper_mode_status.config(text="Mode: Automatic", fg="red")
            self.manual_increase_force_button.config(state=tk.DISABLED)
            self.manual_decrease_force_button.config(state=tk.DISABLED)
        else:
            self.stepper_mode_status.config(text="Mode: Manual", fg="blue")
            self.manual_increase_force_button.config(state=tk.NORMAL)
            self.manual_decrease_force_button.config(state=tk.NORMAL)
            
        self.log_status(f"Stepper mode set to {'Automatic' if auto_mode else 'Manual'}")

    def stepper_preset_increase_dropdown(self):
        """Use selected preset to increase force"""
        if not self.stepper_control_enabled.get():
            return
            
        try:
            selected = self.stepper_preset_var.get()
            if selected:
                # Extract number of steps from selection
                steps = int(selected.split()[0])
                self.controller.stepper.step(steps, True)
                self.log_status(f"Increased force by {steps} steps")
                
                # Update simulated force if needed
                if self.controller.ser is None:
                    self.controller.update_simulated_force(steps, True)
        except (ValueError, AttributeError) as e:
            self.log_status(f"Error applying preset: {e}")

    def stepper_preset_decrease_dropdown(self):
        """Use selected preset to decrease force"""
        if not self.stepper_control_enabled.get():
            return
            
        try:
            selected = self.stepper_preset_var.get()
            if selected:
                # Extract number of steps from selection
                steps = int(selected.split()[0])
                self.controller.stepper.step(steps, False)
                self.log_status(f"Decreased force by {steps} steps")
                
                # Update simulated force if needed
                if self.controller.ser is None:
                    self.controller.update_simulated_force(steps, False)
        except (ValueError, AttributeError) as e:
            self.log_status(f"Error applying preset: {e}")

    def servo_preset_selected(self, event, idx):
        """Handle servo preset selection"""
        try:
            dropdown = self.servo_preset_dropdowns[idx]
            selected = dropdown.get()
            if selected:
                # Extract angle from selection
                angle = int(selected.strip('°'))
                # Update servo angle
                self.servo_angle_vars[idx].set(angle)
                self.set_servo_angle(idx, angle)
                self.log_status(f"Servo {idx+1} set to {angle}°")
        except (ValueError, IndexError) as e:
            self.log_status(f"Error applying servo preset: {e}")

    def set_servo_angle(self, idx, angle):
        """Set servo angle if servo control is enabled"""
        if self.servo_control_enabled.get() and self.controller.servo_enabled:
            # Check if individual servo is enabled
            servo_enabled = [self.servo1_enabled.get(), self.servo2_enabled.get(), self.servo3_enabled.get()]
            if idx < len(servo_enabled) and servo_enabled[idx]:
                self.controller.servo_ctrl.set_angle(idx, int(float(angle)))

    def apply_pid_params(self):
        """Apply PID parameters from settings tab"""
        try:
            self.controller.pid_kp = float(self.kp_var.get())
            self.controller.pid_ki = float(self.ki_var.get())
            self.controller.pid_kd = float(self.kd_var.get())
            
            # Reset PID state
            self.controller.pid_integral = 0.0
            self.controller.pid_last_error = 0.0
            
            # Update config
            self.controller.config_manager.config['pid']['kp'] = self.controller.pid_kp
            self.controller.config_manager.config['pid']['ki'] = self.controller.pid_ki
            self.controller.config_manager.config['pid']['kd'] = self.controller.pid_kd
            self.controller.config_manager.save_config()
            
            self.log_status(f"PID parameters updated: Kp={self.controller.pid_kp}, Ki={self.controller.pid_ki}, Kd={self.controller.pid_kd}")
        except ValueError as e:
            self.log_status(f"Error applying PID parameters: {e}")

    def apply_pins(self):
        """Apply pin configuration"""
        try:
            # This requires a restart to take effect properly
            StepperMotor.DIR_PIN = self.stepper_dir_var.get()
            StepperMotor.STEP_PIN = self.stepper_step_var.get()
            StepperMotor.ENABLE_PIN = self.stepper_enable_var.get()
            
            ServoController.SERVO_PINS[0] = self.servo1_var.get()
            ServoController.SERVO_PINS[1] = self.servo2_var.get()
            ServoController.SERVO_PINS[2] = self.servo3_var.get()
            
            # Update config
            self.controller.config_manager.config['stepper']['dir_pin'] = StepperMotor.DIR_PIN
            self.controller.config_manager.config['stepper']['step_pin'] = StepperMotor.STEP_PIN
            self.controller.config_manager.config['stepper']['enable_pin'] = StepperMotor.ENABLE_PIN
            self.controller.config_manager.config['servos']['pins'] = ServoController.SERVO_PINS
            self.controller.config_manager.save_config()
            
            self.log_status("Pin configuration updated - restart required for changes to take effect")
        except ValueError as e:
            self.log_status(f"Error applying pin configuration: {e}")

    def update_servo_controls(self):
        """Update servo control based on checkbox state"""
        enabled = self.servo_control_enabled.get()
        self.controller.servo_enabled = enabled
        
        # Update UI state
        state = tk.NORMAL if enabled else tk.DISABLED
        for scale in self.servo_scales:
            scale.config(state=state)
        
        self.log_status(f"Servo control {'enabled' if enabled else 'disabled'}")

    def update_stepper_controls(self):
        """Update stepper control based on checkbox state"""
        enabled = self.stepper_control_enabled.get()
        self.controller.stepper_enabled = enabled
        
        # Update UI state
        state = tk.NORMAL if enabled else tk.DISABLED
        self.manual_increase_force_button.config(state=state)
        self.manual_decrease_force_button.config(state=state)
        self.stepper_preset_dropdown.config(state="readonly" if enabled else tk.DISABLED)
        
        self.log_status(f"Stepper control {'enabled' if enabled else 'disabled'}")

    def update_individual_servos(self):
        """Update individual servo enable states"""
        states = [self.servo1_enabled.get(), self.servo2_enabled.get(), self.servo3_enabled.get()]
        for i, enabled in enumerate(states):
            state_text = "enabled" if enabled else "disabled"
            self.log_status(f"Servo {i+1} {state_text}")

    def apply_force_threshold(self):
        """Apply force threshold from settings tab"""
        try:
            new_threshold = float(self.set_force_thresh_var.get())
            if 0 <= new_threshold <= 10000:
                self.controller.shared.force_threshold = new_threshold
                self.force_thresh_var.set(f"{new_threshold:.1f}")
                self.settings_force_thresh_var.set(f"{new_threshold:.1f}")
                self.log_status(f"Force threshold set to {new_threshold:.1f}g")
            else:
                self.log_status("Error: Threshold must be between 0 and 10000g")
        except ValueError as e:
            self.log_status(f"Error setting threshold: {e}")

    def apply_safety_limits(self):
        """Apply safety limits from safety tab"""
        try:
            max_force = int(self.max_force_var.get())
            min_force = int(self.min_force_var.get())
            
            if min_force < max_force:
                self.controller.safety_manager.max_force = max_force
                self.controller.safety_manager.min_force = min_force
                
                # Update config
                self.controller.config_manager.config['safety']['max_force'] = max_force
                self.controller.config_manager.config['safety']['min_force'] = min_force
                self.controller.config_manager.save_config()
                
                self.log_status(f"Safety limits updated: Min={min_force}g, Max={max_force}g")
            else:
                self.log_status("Error: Min force must be less than max force")
        except ValueError as e:
            self.log_status(f"Error setting safety limits: {e}")

    def toggle_alarms(self):
        """Toggle audio alarms"""
        enabled = self.alarms_enabled_var.get()
        self.controller.safety_manager.alarms_enabled = enabled
        
        # Update config
        self.controller.config_manager.config['safety']['alarms_enabled'] = enabled
        self.controller.config_manager.save_config()
        
        self.log_status(f"Audio alarms {'enabled' if enabled else 'disabled'}")

    def view_log_files(self):
        """View log files in system file explorer"""
        log_path = self.controller.data_logger.base_path
        try:
            import subprocess
            import platform
            
            os_name = platform.system()
            if os_name == "Windows":
                os.startfile(log_path)
            elif os_name == "Darwin":  # macOS
                subprocess.run(['open', log_path])
            else:  # Linux
                subprocess.run(['xdg-open', log_path])
                
            self.log_status(f"Opening log directory: {log_path}")
        except Exception as e:
            self.log_status(f"Error opening log directory: {e}")

    def start_logging(self):
        """Start data logging"""
        try:
            test_name = self.test_name_var.get() or "test"
            log_file = self.controller.data_logger.start_logging(test_name)
            
            if log_file:
                self.logging_enabled.set(True)
                self.current_log_var.set(f"Logging to: {os.path.basename(log_file)}")
                self.log_status(f"Data logging started: {os.path.basename(log_file)}")
            else:
                self.logging_enabled.set(False)
                self.log_status("Error starting data logging")
        except Exception as e:
            self.logging_enabled.set(False)
            self.log_status(f"Error starting data logging: {e}")

    def stop_logging(self):
        """Stop data logging"""
        try:
            log_file = self.controller.data_logger.stop_logging()
            
            self.logging_enabled.set(False)
            self.current_log_var.set("No active log")
            
            if log_file:
                self.log_status(f"Data logging stopped: {os.path.basename(log_file)}")
            else:
                self.log_status("Data logging stopped")
        except Exception as e:
            self.log_status(f"Error stopping data logging: {e}")

    def log_status(self, message):
        """Log status message to the status log"""
        try:
            if self.status_log:
                timestamp = datetime.now().strftime("%H:%M:%S")
                log_message = f"[{timestamp}] {message}"
                
                self.status_log.config(state=tk.NORMAL)
                self.status_log.insert(tk.END, log_message + "\n")
                self.status_log.see(tk.END)
                self.status_log.config(state=tk.DISABLED)
        except Exception as e:
            print(f"Error logging status: {e}")

    def clear_status_log(self):
        """Clear the status log"""
        try:
            self.status_log.config(state=tk.NORMAL)
            self.status_log.delete(1.0, tk.END)
            self.status_log.config(state=tk.DISABLED)
        except Exception as e:
            print(f"Error clearing status log: {e}")

    def toggle_logging(self):
        """Toggle data logging on/off"""
        if self.logging_enabled.get():
            self.start_logging()
        else:
            self.stop_logging()
# Remove the following erroneous code at the end of the file:
#        traceback.print_exc()
#    finally:
#        # Cleanup
#        try:
#            if 'app' in locals() and hasattr(app, 'controller'):
#                print("Cleaning up...")
#                app.controller.cleanup()
#        except:
#            pass
#        print("Application shutdown complete.")

# Add a proper main entry point for Tkinter app
if __name__ == "__main__":
    print("RPi Force Control - Direct execution detected")
    root = tk.Tk()
    app = App(root)
    root.mainloop()
else:
    print(f"RPi Force Control - Module imported as {__name__}")
