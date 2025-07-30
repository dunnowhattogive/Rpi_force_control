# Fresh Raspberry Pi Setup Guide

Complete step-by-step instructions to run the RPi Control System on a fresh Raspberry Pi.

## 🍓 Prerequisites

### Hardware Requirements
- **Raspberry Pi 5** (8GB recommended), **Pi 4B** (4GB+ RAM recommended), or Pi 3B+
- **MicroSD Card**: 32GB+ Class 10 (A2 rating recommended for Pi 5)
- **Power Supply**: 
  - **Pi 5**: Official Pi 5 PSU (5V/5A USB-C) - **ESSENTIAL for stability**
  - **Pi 4**: Official Pi PSU (5V/3A USB-C) or equivalent
  - **Pi 3**: micro-USB (5V/2.5A minimum)
- **Display**: HDMI monitor/TV or Pi touchscreen
- **Input**: USB keyboard and mouse
- **Internet**: Ethernet cable or WiFi access

### Pi 5 Specific Requirements
- **Active Cooling**: Fan or official Pi 5 cooler (Pi 5 runs hotter than Pi 4)
- **Quality PSU**: 5V/5A power supply is critical for Pi 5 stability
- **Fast SD Card**: A2-rated microSD cards recommended for better Pi 5 performance

### Optional Hardware
- **Case**: GPIO-accessible case with good ventilation (especially important for Pi 5)
- **Heat Sinks**: For continuous operation
- **USB Hub**: If using multiple USB devices

## 📀 Step 1: Flash Raspberry Pi OS

### Using Raspberry Pi Imager (Recommended)
1. **Download Pi Imager**: https://www.raspberrypi.org/software/
2. **Insert SD Card** into your computer
3. **Run Pi Imager** and select:
   - **OS**: Raspberry Pi OS (64-bit) with desktop
   - **Storage**: Your SD card
4. **Click Settings Gear** (⚙️) and configure:
   - ✅ Enable SSH (set username/password)
   - ✅ Configure WiFi (if using wireless)
   - ✅ Set username: `sigma` (or your preferred username)
   - ✅ Set password: (choose a secure password)
   - ✅ Set locale settings
5. **Write** to SD card and wait for completion

### Alternative: Manual Download
```bash
# Download Raspberry Pi OS image
wget https://downloads.raspberrypi.org/raspios_arm64/images/raspios_arm64-2023-12-06/2023-12-05-raspios-bookworm-arm64.img.xz

# Flash using dd (Linux/macOS)
sudo dd if=2023-12-05-raspios-bookworm-arm64.img of=/dev/sdX bs=4M status=progress
```

## 🚀 Step 2: First Boot Setup

### Initial Boot
1. **Insert SD card** into Raspberry Pi
2. **Connect** display, keyboard, mouse
3. **Power on** the Pi
4. **Wait** for first boot (can take 2-3 minutes)

### Complete Setup Wizard
1. **Welcome Screen**: Click "Next"
2. **Country**: Select your country, language, timezone
3. **User Account**: 
   - Username: `sigma` (or match what you set in imager)
   - Password: (your chosen password)
4. **WiFi**: Select and configure your network (if using WiFi)
5. **Update**: Let it check for updates (recommended)
6. **Reboot** when prompted

### Enable Essential Interfaces
```bash
# Open Pi configuration tool
sudo raspi-config

# Navigate and enable:
# 3 Interface Options → P2 SSH → Enable
# 3 Interface Options → P4 SPI → Enable  
# 3 Interface Options → P5 I2C → Enable
# 3 Interface Options → P6 Serial Port:
#   - "login shell accessible over serial?" → No
#   - "serial port hardware enabled?" → Yes
# 1 System Options → S5 Boot / Auto Login → B4 Desktop Autologin (optional)

# Finish and reboot
```

## 🌐 Step 3: Network and Remote Access

### WiFi Configuration (if needed)
```bash
# If WiFi wasn't configured during setup
sudo raspi-config
# 1 System Options → S1 Wireless LAN

# Or manually edit
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
# Add:
network={
    ssid="YourWiFiName"
    psk="YourWiFiPassword"
}
```

### Enable SSH (for remote access)
```bash
# SSH should be enabled, verify:
sudo systemctl enable ssh
sudo systemctl start ssh

# Find Pi's IP address
hostname -I
ip addr show wlan0  # for WiFi
ip addr show eth0   # for Ethernet

# Test SSH from another computer:
# ssh sigma@192.168.1.xxx
```

### Optional: Enable VNC (Remote Desktop)
```bash
sudo raspi-config
# 3 Interface Options → P3 VNC → Enable

# Or manually:
sudo systemctl enable vncserver-x11-serviced
```

## 🔧 Step 4: System Updates and Dependencies

### Update System
```bash
# Update package lists and system
sudo apt update
sudo apt upgrade -y

# Pi 5 specific: Update firmware
sudo rpi-update

# For Pi 5: Check firmware version
vcgencmd version

# Reboot after updates
sudo reboot
```

### Pi 5 Specific Configuration
```bash
# Pi 5 uses different config file location
# Check which config file exists:
if [ -f /boot/firmware/config.txt ]; then
    echo "Pi 5 detected - using /boot/firmware/config.txt"
    CONFIG_FILE="/boot/firmware/config.txt"
else
    echo "Pi 4 or earlier - using /boot/config.txt"
    CONFIG_FILE="/boot/config.txt"
fi

# Pi 5 optimizations
if [ "$CONFIG_FILE" = "/boot/firmware/config.txt" ]; then
    echo "# Pi 5 optimizations" | sudo tee -a $CONFIG_FILE
    echo "gpu_mem=128" | sudo tee -a $CONFIG_FILE
    echo "arm_boost=1" | sudo tee -a $CONFIG_FILE
    echo "dtparam=spi=on" | sudo tee -a $CONFIG_FILE
    echo "dtparam=i2c=on" | sudo tee -a $CONFIG_FILE
fi
```

### Install Essential Packages
```bash
# Development tools
sudo apt install -y git curl wget build-essential

# Python development
sudo apt install -y python3 python3-pip python3-venv python3-dev

# System libraries
sudo apt install -y libffi-dev libssl-dev

# Hardware libraries  
sudo apt install -y python3-gpiozero python3-rpi.gpio python3-lgpio

# GUI and graphics - handle tkinter package name variations
# Try multiple package names for tkinter (varies by Pi OS version)
if sudo apt install -y python3-tkinter; then
    echo "✓ python3-tkinter installed"
elif sudo apt install -y python3-tk; then
    echo "✓ python3-tk installed"
else
    echo "⚠️ tkinter installation may have failed"
    # Install development packages that might help
    sudo apt install -y tk-dev python3-dev libffi-dev
fi

# Test tkinter installation
if python3 -c "import tkinter; print('tkinter OK')" 2>/dev/null; then
    echo "✓ tkinter verified working"
else
    echo "❌ tkinter test failed - may need manual fix"
fi

# Continue with other GUI packages
sudo apt install -y xorg xinit x11-xserver-utils

# Scientific computing (Pi-optimized versions)
sudo apt install -y python3-numpy python3-matplotlib python3-scipy
sudo apt install -y libatlas-base-dev libopenblas-dev
sudo apt install -y libfreetype6-dev libpng-dev

# Serial communication
sudo apt install -y python3-serial minicom screen

# Audio support
sudo apt install -y python3-pygame alsa-utils pulseaudio

# File management
sudo apt install -y pcmanfm

# Optional: System monitoring
sudo apt install -y htop iotop
```

## 📁 Step 5: Download RPi Control Project

### Clone from Repository
```bash
# Navigate to home directory
cd ~

# Create projects directory
mkdir -p projects
cd projects

# Clone the project (replace with actual repository URL)
git clone https://github.com/your-username/RPi_control.git
cd RPi_control

# Verify files are present
ls -la
# Should see: rpi_control.py, install_and_run.sh, requirements.txt, etc.
```

### Alternative: Download as ZIP
```bash
# If git clone doesn't work, download manually
cd ~/projects
wget https://github.com/your-username/RPi_control/archive/main.zip
unzip main.zip
mv RPi_control-main RPi_control
cd RPi_control
```

## ⚙️ Step 6: Install RPi Control System

### Automated Installation (Recommended)
```bash
# Make installation script executable
chmod +x install_and_run.sh

# Run the automated installer
./install_and_run.sh

# During installation, you'll be prompted:
# Choose option (1-3) [default: 2]:
# 1 = GUI auto-start when user logs in
# 2 = Manual start only (default)  
# 3 = Full auto-boot (auto-login + GUI start)

# For first-time users, choose option 3 for full auto-boot
```

### Installation Process
The installer will:
1. ✅ Detect Raspberry Pi hardware
2. ✅ Install system dependencies
3. ✅ Create virtual environment
4. ✅ Install Python packages
5. ✅ Configure user permissions (gpio, dialout, audio groups)
6. ✅ Set up systemd service
7. ✅ Create desktop shortcuts
8. ✅ Configure auto-boot (if selected)
9. ✅ Run functionality check

### Post-Installation
```bash
# Reboot to apply all changes
sudo reboot

# After reboot, validate installation
cd ~/projects/RPi_control
./validate_system.sh
```

## 🔌 Step 7: Hardware Connections (Optional)

If you have the actual hardware components, wire them according to the connection diagrams:

### Stepper Motor (Easy Driver)
```
Pi GPIO 20 → Easy Driver DIR
Pi GPIO 21 → Easy Driver STEP  
Pi GPIO 16 → Easy Driver ENABLE
Pi 5V      → Easy Driver VCC
Pi GND     → Easy Driver GND
```

### Servo Motors
```
Pi GPIO 17 → Servo 1 Signal (Orange)
Pi GPIO 27 → Servo 2 Signal (Orange)
Pi GPIO 22 → Servo 3 Signal (Orange)
5V PSU     → All Servo Power (Red)
GND        → All Servo Ground (Brown)
```

### Load Cell System
```
MK10 Load Cell → HX711 Amplifier
HX711 → USB-Serial Adapter
USB-Serial → Pi USB Port
```

## 🎮 Step 8: First Run

### Start the Application
```bash
# Method 1: Direct execution
cd ~/projects/RPi_control
./run.sh

# Method 2: Desktop shortcut
# Double-click "RPi Control System" icon on desktop

# Method 3: Service mode
./manage_service.sh start

# Method 4: Auto-boot (if configured)
# Will start automatically on next boot
```

### Verify Operation
1. **GUI Opens**: Main window with tabs should appear
2. **Status Log**: Shows "Application started successfully"
3. **Force Reading**: Shows simulated values if no hardware
4. **Controls Responsive**: Servo sliders and stepper buttons work
5. **No Errors**: Check status log for any red error messages

## 🧪 Step 9: Testing and Validation

### Run System Tests
```bash
cd ~/projects/RPi_control

# Comprehensive functionality check
python functionality_check.py

# Quick system validation  
./validate_system.sh

# Check service status
./manage_service.sh status

# View application logs
./manage_service.sh logs
```

### Expected Test Results
- ✅ Python 3.8+ detected
- ✅ All required modules available
- ✅ Raspberry Pi hardware detected
- ✅ GPIO libraries functional
- ✅ Display system working
- ✅ Configuration files accessible
- ⚠️ Hardware components in simulation mode (if no physical hardware)

## 🔧 Step 10: Configuration and Customization

### Basic Configuration
```bash
# Edit system configuration
nano ~/projects/RPi_control/system_config.json

# Key settings to adjust:
# - GPIO pin assignments
# - PID parameters  
# - Safety limits
# - Serial port settings
```

### Service Management
```bash
cd ~/projects/RPi_control

# Enable auto-start on boot
./manage_service.sh enable

# Disable auto-start
./manage_service.sh disable

# Start/stop manually
./manage_service.sh start
./manage_service.sh stop

# View live logs
./manage_service.sh logs
```

### Desktop Integration
```bash
# Desktop shortcut created automatically at:
~/Desktop/RPi_Control.desktop

# Autostart entry (if auto-boot enabled):
~/.config/autostart/rpi-control.desktop
```

## 🚨 Troubleshooting Common Issues

### ❌ tkinter Installation Issues
**Symptoms**: "unable to locate package python3-tkinter" or "No module named 'tkinter'"

```bash
# Try different package names for tkinter
# Method 1: Try both common package names
sudo apt update
sudo apt install -y python3-tkinter || sudo apt install -y python3-tk

# Method 2: Install development packages
sudo apt install -y tk-dev python3-dev libffi-dev

# Method 3: Full tkinter rebuild (if above fails)
sudo apt install -y tcl-dev tk-dev python3-dev
pip3 install --upgrade pip
pip3 install tkinter-rebuild || echo "tkinter-rebuild not available"

# Method 4: Check if tkinter is built into Python
python3 -c "
import sys
print(f'Python version: {sys.version}')
try:
    import tkinter
    print('✅ tkinter is working')
    tkinter.Tk().withdraw()
    print('✅ tkinter GUI test passed')
except ImportError as e:
    print(f'❌ tkinter import failed: {e}')
except Exception as e:
    print(f'❌ tkinter GUI test failed: {e}')
"

# Method 5: Alternative GUI test
python3 -c "
import tkinter as tk
root = tk.Tk()
root.title('Pi Display Test')
label = tk.Label(root, text='If you see this, GUI is working!')
label.pack(pady=20)
button = tk.Button(root, text='Close', command=root.quit)
button.pack(pady=10)
# Uncomment next line to show test window
# root.mainloop()
root.destroy()
print('GUI components test: PASS')
"
```

### ❌ Pi OS Version Specific Issues

#### Raspberry Pi OS Lite (No Desktop)
```bash
# Install desktop environment if running Pi OS Lite
sudo apt install -y raspberrypi-ui-mods
sudo apt install -y lightdm

# Install additional desktop packages
sudo apt install -y lxde-core lxappearance
sudo systemctl set-default graphical.target

# Reboot to desktop
sudo reboot
```

#### Bookworm vs Bullseye Differences
```bash
# Check Pi OS version
cat /etc/os-release

# For Pi OS Bookworm (2023+):
sudo apt install -y python3-tk python3-tkinter

# For Pi OS Bullseye (2021-2022):
sudo apt install -y python3-tkinter

# For older versions:
sudo apt install -y python-tk python3-tk
```

### ❌ Alternative: Headless Testing
If GUI continues to have issues, test core functionality without GUI:

```bash
# Run headless test
cd ~/projects/RPi_control
./run_headless.sh

# Manual core test
python3 -c "
from rpi_control import Controller, StepperMotor, ServoController, SafetyManager
print('✓ Core imports successful')

controller = Controller()
print('✓ Controller initialized')

# Test components
stepper = StepperMotor()
servo = ServoController()
safety = SafetyManager()
print('✅ All core components working without GUI')
"
```

### ❌ Display Issues on Different Pi Models

#### Pi with Small Display (3.5" screens)
```bash
# Optimize for small displays
echo "hdmi_group=2" | sudo tee -a /boot/config.txt
echo "hdmi_mode=87" | sudo tee -a /boot/config.txt
echo "hdmi_cvt=480 320 60 6 0 0 0" | sudo tee -a /boot/config.txt

# Reduce GUI font sizes
echo "gtk-font-name='Sans 8'" > ~/.gtkrc-2.0
```

#### Pi with No Display (Headless)
```bash
# Enable VNC for remote GUI access
sudo raspi-config
# Interface Options → VNC → Enable

# Or install VNC manually
sudo apt install -y realvnc-vnc-server
sudo systemctl enable vncserver-x11-serviced
sudo systemctl start vncserver-x11-serviced

# Set VNC resolution
echo "framebuffer_width=1920" | sudo tee -a /boot/config.txt
echo "framebuffer_height=1080" | sudo tee -a /boot/config.txt
```

## ✅ Success Checklist

After completing this guide, you should have:

- [ ] Fresh Raspberry Pi OS installed and updated
- [ ] **tkinter working** (test with `python3 -c "import tkinter; print('OK')"`)
- [ ] RPi Control System installed and functional
- [ ] GUI starting successfully (or headless mode working)
- [ ] All system tests passing
- [ ] Service management working
- [ ] Hardware connections ready (if applicable)
- [ ] Remote access configured (if desired)

### tkinter Verification Commands
```bash
# Quick tkinter test
python3 -c "import tkinter; print('tkinter imported successfully')"

# Full GUI test
python3 -c "
import tkinter as tk
root = tk.Tk()
root.title('Test')
tk.Label(root, text='GUI Working!').pack()
root.after(1000, root.quit)  # Auto-close after 1 second
root.mainloop()
print('GUI test completed successfully')
"

# RPi Control specific test
cd ~/projects/RPi_control
python3 -c "
from rpi_control import App
import tkinter as tk
print('✅ RPi Control GUI imports working')
"
```

**🎉 Congratulations! Your Raspberry Pi Control System is ready to use!**

---

*Having tkinter issues? Try the headless mode first, then work on GUI fixes.*
After completing this guide, you should have:

- [ ] Fresh Raspberry Pi OS installed and updated
- [ ] Correct power supply for your Pi model (5A for Pi 5, 3A for Pi 4)
- [ ] Active cooling installed (especially important for Pi 5)
- [ ] RPi Control System installed and functional
- [ ] GUI starting automatically (if configured)
- [ ] All system tests passing
- [ ] Service management working
- [ ] Hardware connections ready (if applicable)
- [ ] Remote access configured (if desired)
- [ ] Temperature monitoring working (Pi 5: <70°C under load)

**🎉 Congratulations! Your Raspberry Pi Control System is ready to use!**

**Pi 5 Users**: Enjoy the enhanced performance and responsiveness!

---

*Need help? Check the troubleshooting section or create an issue on GitHub.*
