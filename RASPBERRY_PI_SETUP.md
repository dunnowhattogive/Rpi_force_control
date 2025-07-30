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

# GUI and graphics
sudo apt install -y python3-tk python3-tkinter

# Scientific computing (Pi-optimized)
sudo apt install -y python3-numpy python3-matplotlib python3-scipy
sudo apt install -y libatlas-base-dev libopenblas-dev

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

### GUI Won't Start
```bash
# Check display environment
echo $DISPLAY  # Should show :0

# Test basic GUI
python3 -c "import tkinter; tkinter.Tk().mainloop()"

# Check for errors
journalctl --user -u rpicontrol -n 50
```

### Permission Errors
```bash
# Check group membership
groups $USER

# Should include: gpio dialout audio

# If missing, add groups:
sudo usermod -a -G gpio,dialout,audio $USER
# Log out and back in, or reboot
```

### Service Won't Start
```bash
# Check service file
cat ~/.config/systemd/user/rpicontrol.service

# Verify paths match your installation
# Edit if needed:
nano ~/.config/systemd/user/rpicontrol.service

# Reload and restart
systemctl --user daemon-reload
systemctl --user restart rpicontrol
```

### Performance Issues
```bash
# Check Pi temperature
vcgencmd measure_temp

# Monitor resources
htop

# Optimize Pi performance
sudo nano /boot/config.txt
# Add: gpu_mem=128

# Restart after changes
sudo reboot
```

### Pi 5 Specific Issues

#### ❌ Pi 5 Power Problems
**Symptoms**: Random reboots, performance issues, or won't boot
```bash
# Check Pi 5 power status
vcgencmd get_throttled
# 0x0 = no issues
# 0x50000 = under-voltage detected

# Monitor voltage
vcgencmd measure_volts
# Should be close to 5.0V

# Check if using correct PSU
# Pi 5 REQUIRES 5V/5A (25W) power supply
# Pi 4 PSU (5V/3A) will cause under-voltage on Pi 5
```

#### ❌ Pi 5 Overheating
**Symptoms**: Thermal throttling, performance drops
```bash
# Check Pi 5 temperature
vcgencmd measure_temp
# Idle: <50°C, Load: <70°C, Critical: >80°C

# Check for thermal throttling
vcgencmd get_throttled
# If non-zero, add cooling

# Install active cooling for Pi 5
# Official Pi 5 Active Cooler recommended
```

#### ❌ Pi 5 Config File Location
**Symptoms**: Configuration changes not taking effect
```bash
# Pi 5 uses different config location
ls -la /boot/firmware/config.txt  # Pi 5
ls -la /boot/config.txt           # Pi 4 and earlier

# Edit correct config file for Pi 5
sudo nano /boot/firmware/config.txt

# For Pi 4 and earlier
sudo nano /boot/config.txt
```

#### ❌ Pi 5 GPIO Compatibility
**Symptoms**: GPIO errors or hardware not responding
```bash
# Verify Pi 5 GPIO library versions
pip list | grep -i gpio

# Update to latest gpiozero for Pi 5 support
pip install --upgrade gpiozero

# Test Pi 5 GPIO
python3 -c "
import gpiozero
print(f'gpiozero version: {gpiozero.__version__}')
from gpiozero import OutputDevice
test = OutputDevice(18)
print('Pi 5 GPIO test: PASS')
test.close()
"
```

## 📱 Step 11: Remote Access (Optional)

### SSH Access
```bash
# From another computer:
ssh sigma@192.168.1.xxx

# Run commands remotely:
cd ~/projects/RPi_control
./manage_service.sh status
```

### VNC Remote Desktop
```bash
# Enable VNC (if not already done)
sudo raspi-config
# 3 Interface Options → P3 VNC → Enable

# Connect using VNC Viewer from another computer
# Address: 192.168.1.xxx:5900
```

### Web Access (Future Enhancement)
The project could be extended with a web interface for remote monitoring and control.

## 🎯 Next Steps

### Hardware Integration
1. **Purchase Components**: Order stepper motor, servos, load cell, amplifiers
2. **Wiring**: Follow connection diagrams carefully
3. **Calibration**: Use calibration tab for load cell setup
4. **Testing**: Verify all hardware functions correctly

### Advanced Configuration
1. **PID Tuning**: Adjust parameters for your specific hardware
2. **Safety Limits**: Set appropriate force and position limits  
3. **Sequences**: Create automated test procedures
4. **Data Logging**: Configure for your measurement requirements

### System Optimization
1. **Performance**: Monitor and optimize for your specific application
2. **Reliability**: Set up monitoring and automatic restarts
3. **Backup**: Regular configuration and data backups
4. **Updates**: Keep system and software updated

### Pi 5 Specific Recommendations
1. **Monitor Temperature**: Use `vcgencmd measure_temp` regularly
2. **Check Power**: Ensure 5V/5A PSU is being used
3. **Active Cooling**: Install fan or official cooler for continuous operation
4. **Performance**: Pi 5 can handle higher update rates and more complex operations
5. **Future Expansion**: Pi 5's PCIe slot enables advanced interfaces

## 📚 Additional Resources

### Documentation
- **Main README**: Complete project documentation
- **API Reference**: Programming interface details
- **Troubleshooting**: Common issues and solutions

### Community
- **GitHub Issues**: Report bugs and get help
- **Discussions**: Share ideas and improvements
- **Wiki**: Community-contributed guides

### Pi-Specific Resources
- **Raspberry Pi Foundation**: https://www.raspberrypi.org/
- **Pi Forums**: https://www.raspberrypi.org/forums/
- **GPIO Pinout**: https://pinout.xyz/

---

## ✅ Success Checklist

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
