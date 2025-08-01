# Raspberry Pi Control System

A comprehensive Raspberry Pi control system for managing stepper motors, servo motors, and MK10 tension-based load cells with automatic force feedback control, real-time plotting, and advanced safety features.

## 🍓 Raspberry Pi Compatibility

**✅ FULLY RASPBERRY PI COMPATIBLE**

This project is specifically designed and optimized for Raspberry Pi with the following compatibility features:

### 🔧 Hardware Compatibility
- **Raspberry Pi Models**: Pi 5 (8GB recommended), Pi 4B, Pi 3B+, Pi 3B, Pi 2B, Pi Zero 2W (GPIO-enabled models)
- **Raspberry Pi OS**: Bookworm, Bullseye, Buster, and newer versions
- **GPIO Libraries**: Native `gpiod` integration (replaces gpiozero)
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
- **Comprehensive Testing**: Built-in unit tests, system tests, and comprehensive functionality validation
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
- **Raspberry Pi**: Pi 5 (8GB recommended), Pi 4B (4GB+ RAM recommended) or Pi 3B+ with Raspberry Pi OS
- **Stepper Motor**: NEMA 17 (E21H4N-2.5-900) with Easy Driver controller
- **Servo Motors**: 3x SG90 or compatible micro servos
- **Load Cell**: MK10 tension load cell (10kg capacity)
- **Amplifier**: HX711 24-bit ADC load cell amplifier
- **Connectivity**: USB-to-Serial adapter (CH340, FTDI, or CP210x)
- **Power Supplies**: 5V/3A (Pi), 12V/2A (stepper), 5V/2A (servos/sensors)

### Raspberry Pi Specific Requirements
- **MicroSD Card**: 32GB+ Class 10 (recommended: SanDisk Extreme)
- **Cooling**: Heat sinks for Pi 4B/5, active cooling for Pi 5 under continuous operation
- **GPIO Access**: Enable SPI/I2C in raspi-config
- **Power Supply**: 
  - **Pi 5**: Official Pi 5 PSU (5V/5A USB-C) or equivalent
  - **Pi 4**: Official Pi PSU (5V/3A USB-C) or equivalent
  - **Pi 3**: micro-USB (5V/2.5A minimum)
- **Case**: GPIO-accessible case with ventilation (Pi 5 requires better airflow)

### Pi 5 Specific Considerations
- **Enhanced Performance**: Pi 5's improved CPU/GPU provides better real-time performance
- **Power Requirements**: Higher power consumption requires 5A PSU for stability
- **GPIO Compatibility**: All GPIO functions remain compatible with existing pin assignments
- **USB 3.0**: Faster USB ports improve serial communication reliability
- **PCIe Support**: Future expansion capabilities for advanced interfaces
- **Active Cooling**: Recommended for continuous operation due to higher performance

## 🚀 Installation

### Quick Setup (Recommended)
```bash
# 1. Clone the repository
git clone https://github.com/your-repo/RPi_control.git
cd RPi_control

# 2. Run comprehensive setup
chmod +x setup.sh
./setup.sh

# 3. Start the application
./manage.sh run
```

### Setup Options
```bash
./setup.sh    # Interactive setup with multiple options:
              # 1. Full installation (recommended)
              # 2. Fix existing installation  
              # 3. Virtual environment only
              # 4. Service configuration only
```

## 🎮 Usage

### Starting the Application

```bash
# Manual GUI start
./manage.sh run

# Service management
./manage.sh start      # Start as background service
./manage.sh stop       # Stop service
./manage.sh status     # Check service status
./manage.sh logs       # View live logs

# Auto-boot configuration
./manage.sh enable     # Enable auto-start on boot
./manage.sh disable    # Disable auto-start
```

### System Management

```bash
# System operations
./manage.sh fix        # Fix common issues
./manage.sh update     # Update packages
./manage.sh test       # Run functionality tests
./manage.sh validate   # Validate configuration

# Monitoring
./manage.sh monitor    # System resource monitor
./manage.sh temp       # Check Pi temperature
./manage.sh backup     # Backup configuration
```

## 🔧 Script Structure

The system now uses only two main scripts:

### **Primary Scripts**
- **`setup.sh`** - Complete installation and configuration
- **`manage.sh`** - All system management operations  
- **`rpi_control.py`** - Main application
- **`functionality_check.py`** - System validation

### **Compatibility Scripts**
- **`start_gui.sh`** - Redirects to `manage.sh run` (for backward compatibility)

### **Configuration Files**
- **`rpicontrol.service`** - Systemd service definition
- **`requirements.txt`** - Consolidated Python package dependencies
- **`system_config.json`** - Application configuration (created during setup)

## 📁 File Structure

```
RPi_control/
├── setup.sh              # Complete system setup
├── manage.sh             # System management
├── rpi_control.py        # Main application
├── functionality_check.py # System validation
├── rpicontrol.service    # Service definition
├── requirements.txt      # Consolidated Python dependencies
├── start_gui.sh          # Compatibility redirect
├── README.md             # This file
├── RASPBERRY_PI_SETUP.md # Pi-specific setup guide
├── venv/                 # Virtual environment (created by setup)
├── data_logs/            # Data logging output
└── config_backups/       # Configuration backups
```

## 🔧 Management Commands

### Setup Script (`./setup.sh`)
```bash
./setup.sh               # Interactive mode with options:
                         # 1. Full installation (recommended)
                         # 2. Fix existing installation
                         # 3. Virtual environment only
                         # 4. Service configuration only
                         
# Note: If multiple requirements files exist, the setup script
# will automatically consolidate them into a single requirements.txt
```

### Management Script (`./manage.sh`)
```bash
# Application Control
./manage.sh run          # Start GUI manually
./manage.sh start        # Start as service
./manage.sh stop         # Stop service
./manage.sh restart      # Restart service

# Service Management  
./manage.sh enable       # Enable auto-start
./manage.sh disable      # Disable auto-start
./manage.sh status       # Show status
./manage.sh logs         # View live logs

# System Operations
./manage.sh fix          # Fix common issues
./manage.sh update       # Update packages  
./manage.sh test         # Run tests
./manage.sh validate     # Validate config

# Monitoring & Maintenance
./manage.sh monitor      # Resource monitor
./manage.sh temp         # Pi temperature
./manage.sh gpio         # GPIO status
./manage.sh backup       # Backup config
./manage.sh restore      # Restore config
./manage.sh clean        # Clean temp files
```

### **Functionality Validation**
```bash
# Run comprehensive functionality check
python functionality_check.py

# Quick validation via management script
./manage.sh validate

# Check specific components
./manage.sh test
```

---

**🍓 Optimized for Raspberry Pi 5 & 4 - Made with ❤️ for the Pi community**

*This project is specifically designed and tested on Raspberry Pi hardware with comprehensive Pi OS integration.*

**Note:**  
> **GPIO Library Change:**  
> This project now uses [gpiod](https://libgpiod.readthedocs.io/) for GPIO access instead of gpiozero.  
> Make sure your system has the `gpiod` Python package and the underlying system library installed.  
> See `requirements.txt` and the installation section for details.