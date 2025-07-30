#!/usr/bin/env python3
"""
Raspberry Pi Control System Functionality Check
Comprehensive testing of all system components
"""

import os
import sys
import time
import traceback
import subprocess
import importlib
import json
from datetime import datetime

class FunctionalityChecker:
    def __init__(self):
        self.test_results = []
        self.passed_tests = 0
        self.failed_tests = 0
        self.warnings = 0
        
    def log_test(self, test_name, status, message="", details=""):
        """Log test result"""
        result = {
            'test': test_name,
            'status': status,
            'message': message,
            'details': details,
            'timestamp': datetime.now().isoformat()
        }
        self.test_results.append(result)
        
        if status == "PASS":
            self.passed_tests += 1
            print(f"✅ {test_name}: PASS - {message}")
        elif status == "FAIL":
            self.failed_tests += 1
            print(f"❌ {test_name}: FAIL - {message}")
        elif status == "WARN":
            self.warnings += 1
            print(f"⚠️  {test_name}: WARNING - {message}")
        
        if details:
            print(f"   Details: {details}")

    def check_python_environment(self):
        """Check Python version and basic environment"""
        try:
            version = sys.version_info
            if version.major == 3 and version.minor >= 7:
                self.log_test("Python Version", "PASS", f"Python {version.major}.{version.minor}.{version.micro}")
            else:
                self.log_test("Python Version", "FAIL", f"Python {version.major}.{version.minor} - requires 3.7+")
        except Exception as e:
            self.log_test("Python Version", "FAIL", str(e))

    def check_required_modules(self):
        """Check if all required Python modules are available"""
        required_modules = [
            'tkinter', 'serial', 'threading', 'json', 'os', 'time',
            'csv', 'datetime', 'math', 'sys', 'signal', 'traceback',
            'subprocess', 'glob', 'collections', 'syslog'
        ]
        
        optional_modules = [
            'numpy', 'matplotlib', 'pygame', 'gpiozero', 'psutil'
        ]
        
        # Test required modules
        for module in required_modules:
            try:
                importlib.import_module(module)
                self.log_test(f"Module: {module}", "PASS", "Available")
            except ImportError:
                self.log_test(f"Module: {module}", "FAIL", "Missing required module")
        
        # Test optional modules
        for module in optional_modules:
            try:
                importlib.import_module(module)
                self.log_test(f"Module: {module}", "PASS", "Available")
            except ImportError:
                self.log_test(f"Module: {module}", "WARN", "Optional module missing")

    def check_raspberry_pi_detection(self):
        """Check Raspberry Pi detection"""
        try:
            if os.path.exists('/proc/device-tree/model'):
                with open('/proc/device-tree/model', 'r') as f:
                    model = f.read().strip()
                if 'Raspberry Pi' in model:
                    self.log_test("Pi Detection", "PASS", f"Detected: {model}")
                else:
                    self.log_test("Pi Detection", "WARN", f"Non-Pi device: {model}")
            else:
                self.log_test("Pi Detection", "WARN", "Not running on Raspberry Pi")
        except Exception as e:
            self.log_test("Pi Detection", "FAIL", str(e))

    def check_gpio_access(self):
        """Check GPIO access and permissions"""
        try:
            # Check if gpiozero is available
            import gpiozero
            self.log_test("GPIO Library", "PASS", "gpiozero available")
            
            # Check GPIO permissions
            gpio_files = ['/dev/gpiomem', '/sys/class/gpio']
            for gpio_file in gpio_files:
                if os.path.exists(gpio_file):
                    if os.access(gpio_file, os.R_OK | os.W_OK):
                        self.log_test(f"GPIO Access: {gpio_file}", "PASS", "Read/Write access")
                    else:
                        self.log_test(f"GPIO Access: {gpio_file}", "WARN", "Limited access - may need group membership")
                else:
                    self.log_test(f"GPIO Access: {gpio_file}", "WARN", "GPIO file not found")
                    
        except ImportError:
            self.log_test("GPIO Library", "FAIL", "gpiozero not available")
        except Exception as e:
            self.log_test("GPIO Access", "FAIL", str(e))

    def check_serial_ports(self):
        """Check serial port access and detection"""
        try:
            import serial.tools.list_ports
            ports = serial.tools.list_ports.comports()
            
            if ports:
                self.log_test("Serial Ports", "PASS", f"Found {len(ports)} serial ports")
                for port in ports:
                    details = f"Port: {port.device}, Description: {port.description}"
                    self.log_test("Serial Port Details", "PASS", "", details)
            else:
                self.log_test("Serial Ports", "WARN", "No serial ports detected")
                
            # Check common serial device files
            common_ports = ['/dev/ttyUSB0', '/dev/ttyACM0', '/dev/ttyAMA0']
            for port in common_ports:
                if os.path.exists(port):
                    if os.access(port, os.R_OK | os.W_OK):
                        self.log_test(f"Serial Access: {port}", "PASS", "Read/Write access")
                    else:
                        self.log_test(f"Serial Access: {port}", "WARN", "Limited access - may need dialout group")
                        
        except ImportError:
            self.log_test("Serial Library", "FAIL", "pyserial not available")
        except Exception as e:
            self.log_test("Serial Ports", "FAIL", str(e))

    def check_display_system(self):
        """Check display system and GUI capabilities"""
        try:
            import tkinter as tk
            
            # Check if DISPLAY environment variable is set
            display = os.environ.get('DISPLAY')
            if display:
                self.log_test("Display Environment", "PASS", f"DISPLAY={display}")
            else:
                wayland = os.environ.get('WAYLAND_DISPLAY')
                if wayland:
                    self.log_test("Display Environment", "PASS", f"WAYLAND_DISPLAY={wayland}")
                else:
                    self.log_test("Display Environment", "WARN", "No display environment detected")
            
            # Test basic tkinter functionality
            try:
                root = tk.Tk()
                root.withdraw()  # Hide the window
                self.log_test("Tkinter GUI", "PASS", "Tkinter window creation successful")
                root.destroy()
            except Exception as e:
                self.log_test("Tkinter GUI", "FAIL", f"Tkinter error: {e}")
                
        except ImportError:
            self.log_test("Tkinter Library", "FAIL", "tkinter not available")
        except Exception as e:
            self.log_test("Display System", "FAIL", str(e))

    def check_audio_system(self):
        """Check audio system for alerts"""
        try:
            import pygame
            pygame.mixer.init()
            self.log_test("Audio System", "PASS", "pygame audio initialized")
            pygame.mixer.quit()
        except ImportError:
            self.log_test("Audio Library", "FAIL", "pygame not available")
        except Exception as e:
            self.log_test("Audio System", "WARN", f"Audio initialization failed: {e}")

    def check_file_permissions(self):
        """Check file system permissions and directory structure"""
        current_dir = os.getcwd()
        
        # Check read/write access to current directory
        if os.access(current_dir, os.R_OK | os.W_OK):
            self.log_test("Directory Access", "PASS", f"Read/Write access to {current_dir}")
        else:
            self.log_test("Directory Access", "FAIL", f"Limited access to {current_dir}")
        
        # Check if main script exists
        main_script = "rpi_control.py"
        if os.path.exists(main_script):
            self.log_test("Main Script", "PASS", f"{main_script} found")
            if os.access(main_script, os.R_OK):
                self.log_test("Script Access", "PASS", "Main script readable")
            else:
                self.log_test("Script Access", "FAIL", "Cannot read main script")
        else:
            self.log_test("Main Script", "WARN", f"{main_script} not found in current directory")

    def check_system_resources(self):
        """Check system resources and performance"""
        try:
            import psutil
            
            # Memory check
            memory = psutil.virtual_memory()
            if memory.total > 512 * 1024 * 1024:  # 512MB
                self.log_test("System Memory", "PASS", f"{memory.total // (1024*1024)}MB available")
            else:
                self.log_test("System Memory", "WARN", f"Low memory: {memory.total // (1024*1024)}MB")
            
            # CPU check
            cpu_count = psutil.cpu_count()
            self.log_test("CPU Cores", "PASS", f"{cpu_count} cores detected")
            
            # Disk space check
            disk = psutil.disk_usage('/')
            free_gb = disk.free // (1024*1024*1024)
            if free_gb > 1:
                self.log_test("Disk Space", "PASS", f"{free_gb}GB free")
            else:
                self.log_test("Disk Space", "WARN", f"Low disk space: {free_gb}GB")
                
        except ImportError:
            self.log_test("System Monitoring", "WARN", "psutil not available for resource monitoring")
        except Exception as e:
            self.log_test("System Resources", "FAIL", str(e))

    def test_core_functionality(self):
        """Test core application functionality"""
        try:
            # Import main modules from the application
            sys.path.insert(0, '.')
            
            # Test if we can import the main classes (without running GUI)
            test_code = """
import sys
sys.path.insert(0, '.')

# Test basic imports
from rpi_control import StepperMotor, ServoController, SafetyManager
from rpi_control import DataLogger, CalibrationManager, ConfigManager

# Test class instantiation
stepper = StepperMotor()
servo = ServoController()
safety = SafetyManager()
logger = DataLogger()
calibration = CalibrationManager()
config = ConfigManager()

print("All core classes instantiated successfully")
"""
            
            result = subprocess.run([sys.executable, '-c', test_code], 
                                  capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                self.log_test("Core Classes", "PASS", "All main classes can be instantiated")
            else:
                self.log_test("Core Classes", "FAIL", f"Import/instantiation error: {result.stderr}")
                
        except FileNotFoundError:
            self.log_test("Core Classes", "WARN", "Main script not found for testing")
        except Exception as e:
            self.log_test("Core Classes", "FAIL", str(e))

    def check_configuration_files(self):
        """Check configuration file handling"""
        try:
            # Test JSON configuration
            test_config = {
                "test": True,
                "timestamp": datetime.now().isoformat()
            }
            
            with open("test_config.json", "w") as f:
                json.dump(test_config, f)
            
            with open("test_config.json", "r") as f:
                loaded_config = json.load(f)
            
            if loaded_config["test"]:
                self.log_test("Config Files", "PASS", "JSON configuration read/write successful")
            else:
                self.log_test("Config Files", "FAIL", "Configuration data corruption")
                
            # Cleanup
            os.remove("test_config.json")
            
        except Exception as e:
            self.log_test("Config Files", "FAIL", str(e))

    def check_autoboot_configuration(self):
        """Check auto-boot and startup configuration"""
        try:
            # Check autostart desktop entry
            autostart_file = os.path.expanduser("~/.config/autostart/rpi-control.desktop")
            if os.path.exists(autostart_file):
                with open(autostart_file, 'r') as f:
                    content = f.read()
                if "Hidden=false" in content:
                    self.log_test("GUI Autostart", "PASS", "Desktop autostart enabled")
                else:
                    self.log_test("GUI Autostart", "WARN", "Desktop autostart disabled")
            else:
                self.log_test("GUI Autostart", "WARN", "Desktop autostart not configured")
            
            # Check systemd service
            try:
                result = subprocess.run(['systemctl', '--user', 'is-enabled', 'rpi-control.service'],
                                      capture_output=True, text=True)
                if result.returncode == 0:
                    self.log_test("Systemd Service", "PASS", "Service enabled for auto-start")
                    
                    # Check if service is running
                    result = subprocess.run(['systemctl', '--user', 'is-active', 'rpi-control.service'],
                                          capture_output=True, text=True)
                    if result.returncode == 0:
                        self.log_test("Service Status", "PASS", "Service currently running")
                    else:
                        self.log_test("Service Status", "WARN", "Service not currently running")
                else:
                    self.log_test("Systemd Service", "WARN", "Service not enabled for auto-start")
            except FileNotFoundError:
                self.log_test("Systemd Service", "WARN", "systemctl not available")
            
            # Check auto-login configuration
            autologin_file = "/etc/systemd/system/getty@tty1.service.d/autologin.conf"
            if os.path.exists(autologin_file):
                self.log_test("Auto-login", "PASS", "Auto-login to desktop configured")
            else:
                self.log_test("Auto-login", "WARN", "Manual login required")
                
        except Exception as e:
            self.log_test("Autoboot Configuration", "FAIL", str(e))

    def run_all_checks(self):
        """Run all functionality checks"""
        print("=" * 60)
        print("Raspberry Pi Control System Functionality Check")
        print("=" * 60)
        print()
        
        # Run all checks
        self.check_python_environment()
        self.check_required_modules()
        self.check_raspberry_pi_detection()
        self.check_gpio_access()
        self.check_serial_ports()
        self.check_display_system()
        self.check_audio_system()
        self.check_file_permissions()
        self.check_system_resources()
        self.check_configuration_files()
        self.check_autoboot_configuration()  # Add autoboot check
        self.test_core_functionality()
        
        # Summary
        print("\n" + "=" * 60)
        print("FUNCTIONALITY CHECK SUMMARY")
        print("=" * 60)
        print(f"✅ Passed Tests: {self.passed_tests}")
        print(f"❌ Failed Tests: {self.failed_tests}")
        print(f"⚠️  Warnings: {self.warnings}")
        print(f"📊 Total Tests: {len(self.test_results)}")
        
        # Overall assessment
        if self.failed_tests == 0:
            if self.warnings == 0:
                print("\n🎉 EXCELLENT: All systems operational!")
                status = "EXCELLENT"
            else:
                print("\n✅ GOOD: System functional with minor issues")
                status = "GOOD"
        elif self.failed_tests <= 2:
            print("\n⚠️  FAIR: System may work with limitations")
            status = "FAIR"
        else:
            print("\n❌ POOR: Significant issues detected")
            status = "POOR"
        
        # Recommendations
        print("\n📋 RECOMMENDATIONS:")
        if self.failed_tests > 0:
            print("• Install missing dependencies using install_and_run.sh")
            print("• Check user group memberships (gpio, dialout, audio)")
            print("• Verify hardware connections and permissions")
        
        if self.warnings > 0:
            print("• Review warning messages for potential optimizations")
            print("• Consider installing optional packages for full functionality")
        
        print("\n🔧 NEXT STEPS:")
        print("1. Review failed tests and install missing components")
        print("2. Run the installation script: ./install_and_run.sh")
        print("3. Test the main application: python rpi_control.py")
        print("4. Check hardware connections if using real devices")
        
        # Save detailed results
        try:
            results_file = f"functionality_check_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            with open(results_file, 'w') as f:
                json.dump({
                    'summary': {
                        'status': status,
                        'passed': self.passed_tests,
                        'failed': self.failed_tests,
                        'warnings': self.warnings,
                        'total': len(self.test_results)
                    },
                    'tests': self.test_results
                }, f, indent=2)
            print(f"\n📄 Detailed results saved to: {results_file}")
        except Exception as e:
            print(f"\n⚠️  Could not save results file: {e}")
        
        return status

if __name__ == "__main__":
    checker = FunctionalityChecker()
    result = checker.run_all_checks()
    
    # Exit with appropriate code
    exit_codes = {
        "EXCELLENT": 0,
        "GOOD": 0,
        "FAIR": 1,
        "POOR": 2
    }
    sys.exit(exit_codes.get(result, 2))
