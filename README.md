# Raspberry Pi Control System

A comprehensive Raspberry Pi control system for managing stepper motors, servo motors, and MK10 tension-based load cells with automatic force feedback control, real-time plotting, and advanced safety features.

## 🍓 Raspberry Pi Compatibility

**✅ FULLY RASPBERRY PI COMPATIBLE**

This project is specifically designed and optimized for Raspberry Pi with the following compatibility features:

### 🔧 Hardware Compatibility
- **Raspberry Pi Models**: Pi 4B, Pi 3B+, Pi 3B, Pi 2B, Pi Zero 2W (GPIO-enabled models)
- **Raspberry Pi OS**: Bullseye, Buster, and newer versions
- **GPIO Libraries**: Native gpiozero integration with fallback support
- **Serial Interfaces**: USB, UART, and GPIO serial communication
- **Display Support**: HDMI, DSI touchscreens, headless operation
- **Architecture**: ARM64 and ARM32 support

### 🚀 Pi-Specific Optimizations
- **Auto-Detection**: Automatically detects Pi hardware and adjusts performance
- **Resource Management**: Optimized memory usage and CPU scheduling for Pi
- **Power Efficiency**: Reduced update rates and smart resource allocation
- **GPIO Safety**: Proper pin management with hardware-safe defaults
- **Boot Integration**: Systemd service with auto-start capabilities
- **Performance Scaling**: Adaptive GUI and plotting based on Pi model

### 📦 Installation Compatibility
- **Automated Setup**: One-command installation script for Pi OS
- **Dependency Management**: Pi-specific package versions and sources
- **User Permissions**: Automatic GPIO, serial, and audio group configuration
- **Service Integration**: Systemd user services with lingering support
- **Auto-Boot**: Optional GUI auto-start on Pi boot

## 🎯 Features

### 🔧 Hardware Support
- **Stepper Motor Control**: NEMA 17 with Easy Driver controller (DIR/STEP/ENABLE)
- **Servo Control**: Up to 3 SG90 servo motors with PWM control and individual enable/disable
- **Load Cell Integration**: MK10 tension-based load cell (10kg capacity) with HX711 amplifier
- **Auto-Detection**: Intelligent serial port detection for load cells (CH340, FTDI, CP210x)
- **Cross-Platform**: Runs on Raspberry Pi (primary), Windows, macOS, and Linux
- **Simulation Mode**: Full functionality without hardware for development/testing

### 🎮 Control Modes
- **Manual Control**: Direct stepper/servo positioning via intuitive GUI
- **Automatic Control**: Advanced PID-based force feedback control with safety limits
- **Preset System**: Configurable angle/step presets for quick positioning
- **Emergency Stop**: Immediate system shutdown with safety lockouts
- **Real-time Monitoring**: Live force readings, status indicators, and data plotting

### 💻 Software Features
- **Multi-tab GUI**: Professional interface with Main, Settings, Calibration, Data Logging, Safety, Sequences, and Tests tabs
- **Real-time Plotting**: Live force data visualization with matplotlib integration
- **Data Logging**: CSV export with timestamps, force data, servo positions, and modes
- **Safety Management**: Force limits, alarms, emergency stops, and audio alerts
- **Configuration Management**: JSON-based settings with backup and restore
- **Sequence Control**: Automated test sequences with step-by-step execution
- **Comprehensive Testing**: Built-in unit tests, system tests, and functionality checks
- **Calibration System**: Advanced load cell calibration with multiple weight points

### 🛡️ Safety Features
- **Force Limits**: Configurable maximum/minimum force thresholds
- **Emergency Stop**: Hardware and software emergency shutdown
- **Audio Alerts**: Sound notifications for safety violations
- **Automatic Shutdown**: PID control with safety override mechanisms
- **Data Logging**: Complete audit trail of all operations
- **Hardware Monitoring**: Real-time system resource and hardware status

## 🔌 Hardware Requirements

### Essential Components
- **Raspberry Pi**: Pi 4B (recommended) or Pi 3B+ with Raspberry Pi OS
- **Stepper Motor**: NEMA 17 (E21H4N-2.5-900) with Easy Driver controller
- **Servo Motors**: 3x SG90 or compatible micro servos
- **Load Cell**: MK10 tension load cell (10kg capacity)
- **Amplifier**: HX711 24-bit ADC load cell amplifier
- **Connectivity**: USB-to-Serial adapter (CH340, FTDI, or CP210x)
- **Power Supplies**: 5V/3A (Pi), 12V/2A (stepper), 5V/2A (servos/sensors)

### Raspberry Pi Specific Requirements
- **MicroSD Card**: 32GB+ Class 10 (recommended: SanDisk Extreme)
- **Cooling**: Heat sinks for Pi 4B, fan for continuous operation
- **GPIO Access**: Enable SPI/I2C in raspi-config
- **Power Supply**: Official Pi PSU or equivalent 5V/3A USB-C (Pi 4) or micro-USB (Pi 3)
- **Case**: GPIO-accessible case with ventilation

### Recommended Pi Accessories
- **Display**: 7" Official Pi Touchscreen for standalone operation
- **Camera**: Pi Camera Module for visual monitoring
- **HAT**: Prototyping HAT for secure connections
- **Breakout Board**: GPIO breakout for easier wiring
- **Storage**: USB SSD for improved performance (optional)

### Wiring Connections

#### Stepper Motor (Easy Driver)
```
Raspberry Pi → Easy Driver
GPIO 20     → DIR (Direction control)
GPIO 21     → STEP (Step pulse)
GPIO 16     → ENABLE (Motor enable/disable)
5V          → VCC (Logic power)
GND         → GND (Common ground)

Easy Driver → NEMA 17
A+/A-       → Phase A windings
B+/B-       → Phase B windings
12V PSU     → Motor power (M+ terminal)
GND         → Motor ground (GND terminal)
```

#### Servo Motors
```
Raspberry Pi → Servos
GPIO 17     → Servo 1 PWM (Orange/Signal wire)
GPIO 27     → Servo 2 PWM (Orange/Signal wire)
GPIO 22     → Servo 3 PWM (Orange/Signal wire)
5V PSU      → All servo power (Red/+ wires)
GND         → All servo ground (Brown/- wires)
```

#### Load Cell System
```
MK10 Load Cell → HX711 Amplifier
Red wire       → E+ (Excitation positive)
Black wire     → E- (Excitation negative)
White wire     → A- (Signal negative)
Green wire     → A+ (Signal positive)

HX711 → USB-Serial Adapter
VCC   → 5V (Power)
GND   → GND (Ground)
DT    → TX (Data transmission)
SCK   → RX (Clock - optional for this setup)

USB-Serial → Raspberry Pi
USB connector → Any available USB port
```

## 🚀 Installation

### Raspberry Pi Installation (Recommended)
```bash
# 1. Update Raspberry Pi OS
sudo apt update && sudo apt upgrade -y

# 2. Clone the repository
git clone https://github.com/your-repo/RPi_control.git
cd RPi_control

# 3. Run Pi-optimized installation
chmod +x install_and_run.sh
./install_and_run.sh

# 4. Configure auto-boot (choose option during install)
# Option 1: GUI auto-start when user logs in
# Option 3: Full auto-boot with auto-login

# 5. Reboot to apply all settings
sudo reboot

# 6. Validate installation
./validate_system.sh
```

### Pi-Specific Configuration
```bash
# Enable GPIO interfaces
sudo raspi-config
# Interface Options → SPI → Enable
# Interface Options → I2C → Enable
# Interface Options → Serial Port → Disable login shell, Enable hardware

# Configure GPU memory (for better GUI performance)
echo "gpu_mem=128" | sudo tee -a /boot/config.txt

# Enable hardware watchdog (optional)
echo "dtparam=watchdog=on" | sudo tee -a /boot/config.txt

# Optimize for headless operation (optional)
sudo systemctl disable bluetooth
sudo systemctl disable wifi-powersave

# Reboot after configuration changes
sudo reboot
```

### Auto-Boot Configuration

The system includes a systemd service file (`rpicontrol.service`) that can automatically start the GUI on boot. Here's how to configure it:

#### Check Current Auto-Boot Status
```bash
# Check if service is enabled
systemctl --user is-enabled rpicontrol.service

# Check if service is running
systemctl --user is-active rpicontrol.service

# View service status
systemctl --user status rpicontrol.service
```

#### Enable Auto-Boot GUI
```bash
# Copy service file to systemd directory (if not already there)
cp rpicontrol.service ~/.config/systemd/user/

# Reload systemd configuration
systemctl --user daemon-reload

# Enable the service to start on boot
systemctl --user enable rpicontrol.service

# Start the service immediately (optional)
systemctl --user start rpicontrol.service

# Enable lingering to start user services without login
sudo loginctl enable-linger $USER
```

#### Disable Auto-Boot GUI
```bash
# Stop the service
systemctl --user stop rpicontrol.service

# Disable auto-start
systemctl --user disable rpicontrol.service
```

#### Auto-Login Setup (Required for GUI on Boot)
For the GUI to start automatically on boot, you need auto-login enabled:

```bash
# Enable auto-login to desktop
sudo raspi-config
# Choose: System Options → Boot / Auto Login → Desktop Autologin

# Or manually configure
sudo systemctl set-default graphical.target
sudo mkdir -p /etc/systemd/system/getty@tty1.service.d/
echo '[Service]
ExecStart=
ExecStart=-/sbin/agetty --autologin sigma --noclear %I $TERM' | sudo tee /etc/systemd/system/getty@tty1.service.d/autologin.conf
```

#### Service Configuration
The current service file (`rpicontrol.service`) is configured for:
- **User**: `sigma` (update this to match your username)
- **Working Directory**: `/home/sigma/projects/RPi_control`
- **Virtual Environment**: `/home/sigma/projects/RPi_control/venv`
- **Display**: `:0` (main display)
- **Auto-restart**: Enabled with 3-second delay

#### Update Service File for Your System
```bash
# Edit the service file to match your paths
nano rpicontrol.service

# Update these lines:
# User=your_username
# ExecStart=/bin/bash -c 'source /home/your_username/path/to/venv/bin/activate && python /home/your_username/path/to/rpi_control.py'
# WorkingDirectory=/home/your_username/path/to/project
```

### Current Auto-Boot Status

**YES**, the GUI can run on boot if properly configured:

1. ✅ **Service File Exists**: `rpicontrol.service` is provided
2. ⚠️ **Requires Setup**: Service needs to be enabled manually
3. ⚠️ **Requires Auto-Login**: Desktop auto-login must be configured
4. ⚠️ **Path-Specific**: Service paths must match your installation

### Quick Auto-Boot Setup
```bash
# 1. Update service file paths
sed -i 's/sigma/$(whoami)/g' rpicontrol.service
sed -i 's|/home/sigma/projects/RPi_control|$(pwd)|g' rpicontrol.service

# 2. Install service
mkdir -p ~/.config/systemd/user
cp rpicontrol.service ~/.config/systemd/user/
systemctl --user daemon-reload

# 3. Enable service
systemctl --user enable rpicontrol.service
sudo loginctl enable-linger $USER

# 4. Configure auto-login
sudo raspi-config  # System Options → Boot / Auto Login → Desktop Autologin

# 5. Reboot to test
sudo reboot
```

### Manual Installation
```bash
# System dependencies
sudo apt update && sudo apt upgrade -y
sudo apt install python3 python3-pip python3-venv python3-dev build-essential
sudo apt install python3-tk python3-numpy python3-matplotlib python3-pygame
sudo apt install python3-serial python3-gpiozero pcmanfm

# Create project directory
mkdir -p /home/$(whoami)/rpi_control
cd /home/$(whoami)/rpi_control

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install Python dependencies
pip install -r requirements.txt

# Configure user permissions
sudo usermod -a -G gpio,dialout,audio $(whoami)

# Create configuration files
cp system_config.json.template system_config.json
```

## 🎮 Usage

### Starting the Application

#### Auto-Boot Mode (Recommended)
If you've configured auto-boot, the GUI will start automatically when the Pi boots and logs into the desktop.

#### Manual Desktop/GUI Mode
```bash
cd /home/$(whoami)/rpi_control
./run.sh
```

#### Service Mode (Background)
```bash
# Start service manually
systemctl --user start rpicontrol

# Check service status
systemctl --user status rpicontrol

# View service logs
journalctl --user -u rpicontrol -f
```

#### Desktop Shortcut
Double-click the "RPi Control System" icon on the desktop (created during installation).

### GUI Interface Overview

#### 📊 Main Tab
- **Live Force Display**: Real-time load cell readings in grams with colored status indicator
- **Force Threshold**: Current target force value with quick adjustment buttons
- **Force Status**: Visual indicator (WITHIN RANGE/ABOVE TARGET/BELOW TARGET)
- **Servo Controls**: Individual servo angle sliders with preset dropdowns and angle displays
- **Stepper Control**: Manual/automatic mode toggle with direction buttons and step presets
- **Status Log**: Timestamped event logging with auto-scroll and filtering

#### ⚙️ Settings Tab
- **PID Parameters**: Real-time tuning of proportional, integral, derivative gains
- **Pin Configuration**: Modify GPIO pin assignments with validation
- **Force Threshold**: Set and apply target force values
- **Component Enable/Disable**: Individual control over servos and stepper motor
- **Preset Management**: Configure servo angles and stepper step counts

#### 🎯 Calibration Tab
- **Load Cell Calibration**: Multi-point calibration with known weights
- **Zero Offset**: Set baseline reading with no load
- **Scale Factor**: Calculate and apply calibration coefficients
- **Calibration History**: View and restore previous calibrations

#### 📈 Data Logging Tab
- **Real-time Plotting**: Live force data visualization with zoom and pan
- **CSV Export**: Timestamped data logging with configurable intervals
- **Log Management**: View, organize, and export log files
- **Test Naming**: Custom test identifiers for organized data collection

#### 🛡️ Safety Tab
- **Emergency Controls**: Large emergency stop button with reset functionality
- **Safety Limits**: Configure maximum/minimum force thresholds
- **Audio Alarms**: Enable/disable sound alerts for safety violations
- **Limit Validation**: Real-time checking with immediate feedback

#### 🔄 Sequences Tab
- **Automated Testing**: Create and execute multi-step test sequences
- **Sequence Editor**: Define force targets, hold times, and ramp rates
- **Execution Control**: Start, pause, stop, and monitor sequence progress
- **Sequence Library**: Save and reuse common test procedures

#### 🧪 Tests Tab
- **Unit Tests**: Component-level functionality verification
- **System Tests**: End-to-end testing with hardware simulation
- **Performance Tests**: Benchmark system response and accuracy
- **Test Reports**: Detailed pass/fail results with recommendations

### Configuration Management

#### System Configuration (`system_config.json`)
```json
{
    "stepper": {
        "dir_pin": 20,
        "step_pin": 21,
        "enable_pin": 16,
        "step_delay": 0.001
    },
    "servos": {
        "pins": [17, 27, 22],
        "frequency": 50
    },
    "serial": {
        "baudrate": 9600,
        "timeout": 1
    },
    "pid": {
        "kp": 0.1,
        "ki": 0.01,
        "kd": 0.05
    },
    "safety": {
        "max_force": 8000,
        "min_force": -1000,
        "alarms_enabled": true
    },
    "logging": {
        "interval": 1.0,
        "auto_start": false
    }
}
```

#### Calibration Data (`calibration.json`)
```json
{
    "zero_offset": 0.0,
    "scale_factor": 1.0,
    "timestamp": "2024-01-01T12:00:00",
    "calibration_points": [
        [0, 0.0],
        [1000, 1023.45],
        [2000, 2046.90]
    ]
}
```

### MK10 Load Cell Setup

#### Hardware Installation
1. **Mechanical Mounting**: Secure load cell with proper alignment for tension measurement
2. **Electrical Connection**: Wire according to color code (Red=E+, Black=E-, White/Green=signal)
3. **Amplifier Setup**: Connect HX711 with stable 5V power supply
4. **Serial Interface**: Use quality USB-to-Serial adapter with stable drivers

#### Calibration Procedure
1. **Zero Calibration**: Remove all loads and click "Zero Load Cell"
2. **Weight Points**: Apply known weights (500g, 1kg, 2kg, 5kg) and record readings
3. **Linear Regression**: System automatically calculates calibration coefficients
4. **Verification**: Test with additional weights to verify accuracy
5. **Save Configuration**: Store calibration data for automatic loading

#### Expected Performance
- **Resolution**: 0.1g (with proper calibration)
- **Accuracy**: ±0.5% of full scale (±50g at 10kg)
- **Update Rate**: 10-80 Hz (configurable)
- **Temperature Stability**: ±0.02% per °C

## 🔧 API Reference

### Controller Class
```python
from rpi_control import Controller

# Initialize controller
controller = Controller()

# Manual control
controller.increase_force()        # Increment stepper position
controller.decrease_force()        # Decrement stepper position
controller.increase_threshold()    # Raise force target
controller.decrease_threshold()    # Lower force target

# Automatic control
controller.auto_mode_enabled = True   # Enable PID control
controller.emergency_stop()          # Immediate system shutdown
controller.reset_emergency_stop()    # Clear emergency state

# Data access
current_force = controller.get_force_reading()
controller.log_status("Custom message")
```

### Hardware Classes
```python
from rpi_control import StepperMotor, ServoController

# Stepper motor control
stepper = StepperMotor()
stepper.step(steps=100, direction=True)  # 100 steps anti-clockwise
stepper.disable()                        # Power down motor

# Servo control
servo_ctrl = ServoController()
servo_ctrl.set_angle(idx=0, angle=90)    # Set servo 0 to 90 degrees
servo_ctrl.cleanup()                     # Reset all servos to 0°
```

### Data and Safety
```python
from rpi_control import DataLogger, SafetyManager

# Data logging
logger = DataLogger()
log_file = logger.start_logging("test_session")
logger.log_data(force=150.5, threshold=200, servo_angles=[90,45,0], mode="Auto")
logger.stop_logging()

# Safety management
safety = SafetyManager()
safety.max_force = 5000  # Set maximum safe force
is_safe = safety.check_force_limits(current_force)
safety.trigger_alarm("CUSTOM_ALARM", "Test message")
```

## 🔍 Troubleshooting

### Raspberry Pi Specific Issues

#### ❌ GPIO Permission Denied
**Symptoms**: "GPIO setup error" or permission denied
```bash
# Check Pi model and GPIO support
cat /proc/device-tree/model

# Install Pi-specific GPIO libraries
sudo apt install python3-gpiozero python3-rpi.gpio python3-lgpio

# Add user to gpio group
sudo usermod -a -G gpio $USER

# Check GPIO group membership
groups $USER

# Enable GPIO in boot config
sudo raspi-config
# Interface Options → SPI/I2C → Enable

# Verify GPIO device files
ls -la /dev/gpiomem /sys/class/gpio

# Test GPIO access
python3 -c "from gpiozero import LED; print('GPIO OK')"
```

#### ❌ Pi-Specific Display Issues
**Symptoms**: GUI crashes or display problems
```bash
# Check Pi display configuration
tvservice -s  # HDMI status
vcgencmd get_config int | grep -E "(hdmi|display)"

# For Pi touchscreen
sudo apt install raspberrypi-ui-mods

# Test display with simple GUI
python3 -c "
import tkinter as tk
root = tk.Tk()
root.title('Pi Display Test')
tk.Label(root, text='Pi Display Working!').pack()
root.mainloop()
"

# Configure display rotation (if needed)
echo "display_rotate=1" | sudo tee -a /boot/config.txt  # 90 degrees

# Force HDMI output
echo "hdmi_force_hotplug=1" | sudo tee -a /boot/config.txt
```

#### ❌ Pi Performance Issues
**Symptoms**: Slow GUI or high CPU usage
```bash
# Check Pi model and specs
cat /proc/cpuinfo | grep -E "(model|processor)"
free -h  # Memory usage
df -h    # Disk usage

# Monitor system performance
htop
iostat 1 5

# Pi performance optimizations
sudo nano /boot/config.txt
# Add these lines:
# arm_freq=1500          # Overclock (Pi 3B+/4B only)
# gpu_mem=128            # GPU memory split
# disable_overscan=1     # Full screen usage

# Optimize Python for Pi
pip install --upgrade pip
pip install numpy --only-binary=numpy  # Use compiled version

# Reduce GUI update frequency (automatically handled)
python functionality_check.py | grep "Raspberry Pi"
```

#### ❌ Serial Communication on Pi
**Symptoms**: Load cell not detected
```bash
# Check Pi serial configuration
sudo raspi-config
# Interface Options → Serial Port
# "Would you like a login shell accessible over serial?" → No
# "Would you like the serial port hardware enabled?" → Yes

# List serial devices
ls -la /dev/tty*
dmesg | grep tty

# Test USB-serial adapter
lsusb  # Look for CH340, FTDI, CP210x
sudo minicom -D /dev/ttyUSB0 -b 9600

# Check serial permissions
groups $USER | grep dialout
sudo usermod -a -G dialout $USER  # Add to dialout group

# Test Pi hardware UART (if using GPIO serial)
echo "enable_uart=1" | sudo tee -a /boot/config.txt
sudo reboot
```

#### ❌ Pi Audio Issues
**Symptoms**: No sound alerts
```bash
# Check Pi audio configuration
aplay -l  # List audio devices
amixer scontrols  # Audio controls

# Set audio output
sudo raspi-config
# Advanced Options → Audio → Force 3.5mm jack

# Or set HDMI audio
sudo raspi-config
# Advanced Options → Audio → Force HDMI

# Test audio
aplay /usr/share/sounds/alsa/Front_Left.wav

# Install missing audio packages
sudo apt install alsa-utils pulseaudio-utils

# Configure audio for pygame
export SDL_AUDIODRIVER=alsa
python3 -c "import pygame; pygame.mixer.init(); print('Audio OK')"
```

### Pi Boot and Service Issues

#### ❌ Service Won't Start on Pi
**Symptoms**: Auto-boot fails
```bash
# Check Pi boot process
systemctl --user status rpicontrol
journalctl --user -u rpicontrol -n 50

# Verify Pi-specific paths in service
cat ~/.config/systemd/user/rpicontrol.service
# Should show your actual Pi username and paths

# Check display environment for Pi
echo $DISPLAY  # Should be :0
who  # Check logged in users

# Test manual service start
systemctl --user start rpicontrol
systemctl --user status rpicontrol

# Check lingering for user services
loginctl show-user $USER | grep Linger
sudo loginctl enable-linger $USER
```

#### ❌ Pi Auto-Login Issues
**Symptoms**: Stuck at login screen
```bash
# Check Pi auto-login configuration
sudo systemctl get-default  # Should be graphical.target
cat /etc/systemd/system/getty@tty1.service.d/autologin.conf

# Reconfigure auto-login via raspi-config
sudo raspi-config
# System Options → Boot / Auto Login → Desktop Autologin

# Check Pi desktop environment
ps aux | grep -E "(lxde|openbox|wayfire)"

# Test Pi desktop start
startx  # Manual desktop start
```

## 🧪 Pi-Specific Testing

### Raspberry Pi Validation
```bash
# Run Pi-specific functionality check
python functionality_check.py

# Check Pi hardware detection
python -c "
import subprocess
try:
    result = subprocess.run(['cat', '/proc/device-tree/model'], 
                          capture_output=True, text=True, timeout=2)
    if 'Raspberry Pi' in result.stdout:
        print(f'✅ Detected: {result.stdout.strip()}')
    else:
        print('❌ Not a Raspberry Pi')
except:
    print('❌ Pi detection failed')
"

# Test Pi GPIO functionality
python -c "
try:
    from gpiozero import OutputDevice
    led = OutputDevice(18)  # Safe test pin
    print('✅ GPIO library working')
    led.close()
except Exception as e:
    print(f'❌ GPIO test failed: {e}')
"

# Check Pi performance metrics
python -c "
import psutil
import time
cpu_temp = None
try:
    with open('/sys/class/thermal/thermal_zone0/temp') as f:
        cpu_temp = int(f.read()) / 1000
except:
    pass

print(f'CPU Usage: {psutil.cpu_percent()}%')
print(f'RAM Usage: {psutil.virtual_memory().percent}%')
if cpu_temp:
    print(f'CPU Temp: {cpu_temp}°C')
print(f'CPU Cores: {psutil.cpu_count()}')
"
```

### Pi Performance Benchmarks
```bash
# Pi-specific performance test
python -c "
import time
from rpi_control import Controller

# Test Pi-optimized force reading
controller = Controller()
start = time.time()
for i in range(50):  # Reduced for Pi
    controller.get_force_reading()
    time.sleep(0.01)
rate = 50 / (time.time() - start)
print(f'Pi Force Reading Rate: {rate:.1f} Hz')

# Test Pi GUI performance
import tkinter as tk
root = tk.Tk()
root.withdraw()
start = time.time()
for i in range(50):  # Reduced for Pi
    root.update()
rate = 50 / (time.time() - start)
print(f'Pi GUI Update Rate: {rate:.1f} Hz')
root.destroy()
"
```

## 🍓 Pi Deployment Recommendations

### Production Pi Setup
1. **Use Pi 4B**: 4GB+ RAM recommended for best performance
2. **Quality SD Card**: SanDisk Extreme or Samsung EVO Select
3. **Proper Cooling**: Heat sinks + fan for 24/7 operation
4. **Stable Power**: Official Pi PSU or equivalent quality
5. **GPIO Protection**: Use level shifters for 5V devices
6. **Backup Strategy**: Regular SD card images and config backups

### Pi Network Configuration
```bash
# Enable SSH for remote access
sudo systemctl enable ssh
sudo systemctl start ssh

# Configure WiFi (if needed)
sudo raspi-config
# System Options → Wireless LAN

# Set static IP (optional)
sudo nano /etc/dhcpcd.conf
# Add:
# interface wlan0
# static ip_address=192.168.1.100/24
# static routers=192.168.1.1
# static domain_name_servers=8.8.8.8

# Enable VNC for remote desktop (optional)
sudo raspi-config
# Interface Options → VNC → Enable
```

### Pi Security Considerations
```bash
# Change default password
passwd

# Update Pi firmware
sudo rpi-update

# Configure firewall
sudo ufw enable
sudo ufw allow ssh
sudo ufw allow 5900  # VNC if enabled

# Disable unnecessary services
sudo systemctl disable bluetooth
sudo systemctl disable cups  # If no printing needed
```

## 📜 License

This project is licensed under the **MIT License** - see the LICENSE file for details.

## 🙋 Support

### Getting Help
1. **Read Documentation**: Start with this README and troubleshooting section
2. **Run Functionality Check**: Use `python functionality_check.py` to identify issues
3. **Check Hardware**: Verify all connections and power supplies
4. **Search Issues**: Look through existing GitHub issues for similar problems
5. **Create Issue**: Submit detailed bug report with system information

### Issue Report Template
```markdown
**System Information**
- Raspberry Pi Model: [Pi 4B/3B+/etc.]
- OS Version: [Raspberry Pi OS version]
- Python Version: [3.8/3.9/etc.]
- Hardware: [Connected devices]

**Problem Description**
[Clear description of the issue]

**Steps to Reproduce**
1. [First step]
2. [Second step]
3. [etc.]

**Expected Behavior**
[What should happen]

**Actual Behavior**
[What actually happens]

**Logs and Output**
```
[Paste relevant log output]
```

**Functionality Check Results**
[Output from python functionality_check.py]
```

### Community
- **GitHub Discussions**: General questions and feature requests
- **Issue Tracker**: Bug reports and technical problems
- **Wiki**: Community-contributed guides and modifications
- **Examples**: Sample configurations and use cases

## 📊 Changelog

### Version 3.0.0 (Current)
- ✨ **New Features**:
  - **Full Raspberry Pi Optimization**: Native Pi hardware detection and performance scaling
  - **Pi-Specific Installation**: Automated Pi OS setup with all dependencies
  - **GPIO Integration**: Native gpiozero support with hardware-safe defaults
  - **Pi Boot Integration**: Systemd service with auto-start and lingering support
  - Real-time force data plotting with Pi-optimized performance
  - Advanced safety management with audio alerts
  - Comprehensive calibration system with multi-point support
  - Automated test sequences with customizable steps
  - Data logging with CSV export and analysis tools
  - Comprehensive functionality checking and validation

- 🔧 **Pi-Specific Improvements**:
  - **Hardware Auto-Detection**: Automatic Pi model detection and optimization
  - **Resource Management**: Memory and CPU optimizations for Pi hardware
  - **Display Compatibility**: Support for Pi touchscreens, HDMI, and headless operation
  - **Serial Integration**: Enhanced USB and GPIO serial communication
  - **Audio Support**: Pi audio jack and HDMI audio configuration
  - **GPIO Safety**: Proper pin management with cleanup and error handling
  - **Performance Scaling**: Adaptive update rates based on Pi model capabilities
  - Cross-platform compatibility maintained (Pi primary, Windows/macOS/Linux secondary)

- 🐛 **Bug Fixes**:
  - Fixed serial port detection on various USB adapters
  - Resolved GPIO permission issues with proper group management
  - Improved tkinter compatibility across Python versions
  - Fixed memory leaks in real-time plotting
  - Corrected PID controller integral windup issues

### Version 2.1.0
- Added automatic load cell port detection and reconnection
- Enhanced error handling with detailed logging
- Improved GUI responsiveness and status indicators
- Added comprehensive unit and system testing framework
- Optimized performance for continuous operation

### Version 2.0.0
- Integrated MK10 load cell support with HX711 amplifier
- Implemented PID-based automatic force control
- Added professional GUI with tabbed interface
- Enhanced safety features with emergency stops
- Comprehensive configuration management system

### Version 1.0.0
- Initial release with basic stepper and servo control
- Simple GUI interface with manual controls
- Hardware abstraction with simulation mode
- Basic testing and validation framework

---

**🍓 Optimized for Raspberry Pi - Made with ❤️ for the Pi community**

*This project is specifically designed and tested on Raspberry Pi hardware with comprehensive Pi OS integration.*