#!/bin/bash

# Raspberry Pi Control System Installation and Setup Script
# Optimized for Raspberry Pi OS (Debian-based systems)

set -e  # Exit on any error

echo "======================================================"
echo "Raspberry Pi Control System Installation Script"
echo "======================================================"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Detect system information
print_status "Detecting system information..."
ARCH=$(uname -m)
OS_VERSION=$(lsb_release -rs 2>/dev/null || echo "Unknown")
KERNEL=$(uname -r)

echo "Architecture: $ARCH"
echo "OS Version: $OS_VERSION"
echo "Kernel: $KERNEL"

# Check if running on Raspberry Pi
if [[ -f /proc/device-tree/model ]]; then
    PI_MODEL=$(cat /proc/device-tree/model 2>/dev/null || echo "Unknown Pi")
    print_success "Detected Raspberry Pi: $PI_MODEL"
    IS_PI=true
else
    print_warning "Not detected as Raspberry Pi - continuing with generic Linux setup"
    IS_PI=false
fi

# Update system packages
print_status "Updating system packages..."
sudo apt update
sudo apt upgrade -y

# Install system dependencies
print_status "Installing system dependencies..."

# Essential packages
sudo apt install -y \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    build-essential \
    git \
    curl \
    wget

# GUI and display packages
sudo apt install -y \
    python3-tk \
    python3-tkinter \
    xorg \
    xinit \
    x11-xserver-utils

# File manager for log viewing
sudo apt install -y pcmanfm

# Serial communication dependencies
sudo apt install -y \
    python3-serial \
    minicom \
    screen

# Scientific computing packages (Pi-optimized)
print_status "Installing scientific computing packages..."
sudo apt install -y \
    python3-numpy \
    python3-matplotlib \
    python3-scipy \
    libatlas-base-dev \
    libopenblas-dev \
    libhdf5-dev \
    libhdf5-serial-dev \
    pkg-config

# Audio system for alerts
sudo apt install -y \
    python3-pygame \
    alsa-utils \
    pulseaudio

# GPIO and hardware access (Pi-specific)
if [[ $IS_PI == true ]]; then
    print_status "Installing Raspberry Pi specific packages..."
    sudo apt install -y \
        python3-gpiozero \
        python3-rpi.gpio \
        rpi.gpio-common \
        raspi-config
fi

# Development tools (optional)
sudo apt install -y \
    python3-pytest \
    python3-coverage \
    vim \
    nano

# Create project directory if it doesn't exist
PROJECT_DIR="/home/$(whoami)/rpi_control"
if [[ ! -d "$PROJECT_DIR" ]]; then
    print_status "Creating project directory: $PROJECT_DIR"
    mkdir -p "$PROJECT_DIR"
fi

# Create Python virtual environment
print_status "Creating Python virtual environment..."
VENV_DIR="$PROJECT_DIR/venv"
if [[ ! -d "$VENV_DIR" ]]; then
    python3 -m venv "$VENV_DIR"
fi

# Activate virtual environment
print_status "Activating virtual environment..."
source "$VENV_DIR/bin/activate"

# Upgrade pip in virtual environment
python -m pip install --upgrade pip setuptools wheel

# Install Python packages from requirements
print_status "Installing Python packages..."
if [[ -f "requirements.txt" ]]; then
    pip install -r requirements.txt
else
    print_warning "requirements.txt not found, installing packages manually..."
    pip install \
        pyserial>=3.4 \
        gpiozero>=1.6.2 \
        numpy>=1.19.5 \
        matplotlib>=3.3.4 \
        pygame>=2.0.1 \
        psutil>=5.8.0 \
        colorama>=0.4.4
fi

# Configure system settings for optimal performance
print_status "Configuring system settings..."

# Add user to GPIO and serial groups (Pi-specific)
if [[ $IS_PI == true ]]; then
    sudo usermod -a -G gpio,spi,i2c,dialout "$(whoami)"
    print_success "Added user to hardware access groups"
fi

# Configure serial port access
sudo usermod -a -G dialout "$(whoami)"

# Create udev rules for consistent device naming
print_status "Creating udev rules for hardware devices..."
sudo tee /etc/udev/rules.d/99-rpi-control.rules > /dev/null << 'EOF'
# Rules for RPi Control System
# Load cell and serial devices
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ttyUSB_loadcell"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="ttyUSB_loadcell"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ttyUSB_loadcell"

# Give permission to access GPIO and hardware
KERNEL=="gpiomem", GROUP="gpio", MODE="0660"
SUBSYSTEM=="gpio", GROUP="gpio", MODE="0660"
EOF

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Configure audio for alerts
print_status "Configuring audio system..."
if [[ $IS_PI == true ]]; then
    # Set audio output to headphone jack by default
    sudo raspi-config nonint do_audio 1 2>/dev/null || true
    
    # Add user to audio group
    sudo usermod -a -G audio "$(whoami)"
fi

# Create desktop shortcut
print_status "Creating desktop shortcut..."
DESKTOP_DIR="/home/$(whoami)/Desktop"
if [[ -d "$DESKTOP_DIR" ]]; then
    cat > "$DESKTOP_DIR/RPi_Control.desktop" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=RPi Control System
Comment=Load Cell and Motor Control System
Exec=$VENV_DIR/bin/python $PROJECT_DIR/rpi_control.py
Icon=applications-system
Terminal=false
Categories=System;Utility;
StartupNotify=true
EOF
    chmod +x "$DESKTOP_DIR/RPi_Control.desktop"
    print_success "Desktop shortcut created"
fi

# Create systemd service for auto-start (optional)
print_status "Creating systemd service..."

# Update the existing service file with current user and paths
print_status "Configuring service file for current user and paths..."
if [[ -f "rpicontrol.service" ]]; then
    # Create a customized version of the service file
    cp rpicontrol.service rpicontrol.service.backup
    
    # Replace username and paths in service file
    sed "s/User=sigma/User=$(whoami)/g" rpicontrol.service > rpicontrol.service.tmp
    sed "s|/home/sigma/projects/RPi_control|$PROJECT_DIR|g" rpicontrol.service.tmp > rpicontrol.service.configured
    rm rpicontrol.service.tmp
    
    print_success "Service file configured for user: $(whoami)"
    print_success "Service file configured for path: $PROJECT_DIR"
else
    # Create service file if it doesn't exist
    print_status "Creating new service file..."
    cat > rpicontrol.service.configured << EOF
[Unit]
Description=RPi Control GUI
After=graphical-session.target network.target

[Service]
Type=simple
User=$(whoami)
Environment=DISPLAY=:0
ExecStart=/bin/bash -c 'source $VENV_DIR/bin/activate && python $PROJECT_DIR/rpi_control.py'
WorkingDirectory=$PROJECT_DIR
Restart=always
RestartSec=3

[Install]
WantedBy=graphical-session.target
EOF
    print_success "New service file created"
fi

# Install the service for the current user
print_status "Installing systemd user service..."
mkdir -p "/home/$(whoami)/.config/systemd/user"
cp rpicontrol.service.configured "/home/$(whoami)/.config/systemd/user/rpicontrol.service"

# Reload systemd user daemon
systemctl --user daemon-reload
print_success "Systemd user daemon reloaded"

# Ask user if they want to enable auto-boot
print_status "Configuring auto-boot options..."
echo ""
echo "🚀 Auto-Boot Configuration Options:"
echo "1. Enable GUI auto-start (starts when user logs in)"
echo "2. Skip auto-boot configuration (manual start only)"
echo "3. Configure full auto-boot (auto-login + GUI start)"
echo ""
read -p "Choose option (1-3) [default: 2]: " autoboot_choice

case "${autoboot_choice:-2}" in
    "1")
        print_status "Enabling GUI auto-start..."
        systemctl --user enable rpicontrol.service
        sudo loginctl enable-linger "$(whoami)"
        print_success "✅ GUI will auto-start when user logs in"
        print_warning "⚠️  Manual login required (configure auto-login separately if needed)"
        ;;
    "3")
        print_status "Configuring full auto-boot (auto-login + GUI start)..."
        
        # Enable the service
        systemctl --user enable rpicontrol.service
        sudo loginctl enable-linger "$(whoami)"
        
        # Configure auto-login
        print_status "Configuring auto-login..."
        sudo systemctl set-default graphical.target
        sudo mkdir -p /etc/systemd/system/getty@tty1.service.d/
        
        cat << AUTOLOGIN_EOF | sudo tee /etc/systemd/system/getty@tty1.service.d/autologin.conf > /dev/null
[Service]
ExecStart=
ExecStart=-/sbin/agetty --autologin $(whoami) --noclear %I \$TERM
AUTOLOGIN_EOF
        
        print_success "✅ Full auto-boot configured"
        print_success "✅ GUI will start automatically on system boot"
        ;;
    *)
        print_status "Skipping auto-boot configuration"
        print_success "✅ Service installed but not enabled"
        print_warning "⚠️  Use 'systemctl --user enable rpicontrol.service' to enable later"
        ;;
esac

# Create service management script
print_status "Creating service management script..."
cat > "$PROJECT_DIR/manage_service.sh" << 'EOF'
#!/bin/bash
# Service management script for RPi Control System

SERVICE_NAME="rpicontrol.service"

print_usage() {
    echo "Usage: $0 [start|stop|restart|enable|disable|status|logs|install]"
    echo ""
    echo "Commands:"
    echo "  start    - Start the service"
    echo "  stop     - Stop the service"
    echo "  restart  - Restart the service"
    echo "  enable   - Enable auto-start on boot"
    echo "  disable  - Disable auto-start"
    echo "  status   - Show service status"
    echo "  logs     - Show service logs (live)"
    echo "  install  - Install/update service file"
}

check_service_file() {
    if [[ ! -f "$HOME/.config/systemd/user/$SERVICE_NAME" ]]; then
        echo "❌ Service file not found at: $HOME/.config/systemd/user/$SERVICE_NAME"
        echo "Run: $0 install"
        exit 1
    fi
}

case "${1:-status}" in
    "start")
        check_service_file
        systemctl --user start $SERVICE_NAME
        echo "✅ Service started"
        ;;
    "stop")
        check_service_file
        systemctl --user stop $SERVICE_NAME
        echo "✅ Service stopped"
        ;;
    "restart")
        check_service_file
        systemctl --user restart $SERVICE_NAME
        echo "✅ Service restarted"
        ;;
    "enable")
        check_service_file
        systemctl --user enable $SERVICE_NAME
        sudo loginctl enable-linger $USER
        echo "✅ Service enabled for auto-start"
        ;;
    "disable")
        check_service_file
        systemctl --user disable $SERVICE_NAME
        echo "✅ Service disabled"
        ;;
    "status")
        check_service_file
        echo "=== Service Status ==="
        systemctl --user status $SERVICE_NAME --no-pager
        echo ""
        echo "=== Auto-start Status ==="
        if systemctl --user is-enabled $SERVICE_NAME >/dev/null 2>&1; then
            echo "✅ Auto-start: ENABLED"
        else
            echo "❌ Auto-start: DISABLED"
        fi
        ;;
    "logs")
        check_service_file
        echo "=== Service Logs (Press Ctrl+C to exit) ==="
        journalctl --user -u $SERVICE_NAME -f
        ;;
    "install")
        echo "Installing/updating service file..."
        if [[ -f "rpicontrol.service" ]]; then
            mkdir -p "$HOME/.config/systemd/user"
            
            # Update service file with current paths
            sed "s/User=sigma/User=$USER/g" rpicontrol.service | \
            sed "s|/home/sigma/projects/RPi_control|$(pwd)|g" > "$HOME/.config/systemd/user/$SERVICE_NAME"
            
            systemctl --user daemon-reload
            echo "✅ Service file installed/updated"
        else
            echo "❌ rpicontrol.service file not found in current directory"
            exit 1
        fi
        ;;
    *)
        print_usage
        exit 1
        ;;
esac
EOF

chmod +x "$PROJECT_DIR/manage_service.sh"
print_success "Service management script created: $PROJECT_DIR/manage_service.sh"

# Configure log rotation
print_status "Setting up log rotation..."
sudo tee /etc/logrotate.d/rpi-control > /dev/null << 'EOF'
/home/*/rpi_control/data_logs/*.csv {
    daily
    missingok
    rotate 30
    compress
    delaycompress
    notifempty
    copytruncate
}
EOF

# Set up automatic updates (optional)
print_status "Configuring automatic updates..."
cat > "$PROJECT_DIR/update_system.sh" << 'EOF'
#!/bin/bash
# Automatic system update script
cd "$(dirname "$0")"
source venv/bin/activate
pip install --upgrade -r requirements.txt
git pull origin main 2>/dev/null || echo "No git repository found"
EOF
chmod +x "$PROJECT_DIR/update_system.sh"

# Performance optimizations for Raspberry Pi
if [[ $IS_PI == true ]]; then
    print_status "Applying Raspberry Pi performance optimizations..."
    
    # GPU memory split
    if ! grep -q "gpu_mem=" /boot/config.txt; then
        echo "gpu_mem=128" | sudo tee -a /boot/config.txt
    fi
    
    # Disable unnecessary services
    sudo systemctl disable bluetooth 2>/dev/null || true
    sudo systemctl disable hciuart 2>/dev/null || true
    
    # Enable SPI and I2C if needed
    sudo raspi-config nonint do_spi 0 2>/dev/null || true
    sudo raspi-config nonint do_i2c 0 2>/dev/null || true
    
    print_success "Pi-specific optimizations applied"
fi

# Create configuration files
print_status "Creating default configuration files..."

# System configuration
cat > "$PROJECT_DIR/system_config.json" << 'EOF'
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
EOF

# Create data directories
mkdir -p "$PROJECT_DIR/data_logs"
mkdir -p "$PROJECT_DIR/config_backups"

# Set permissions
chmod -R 755 "$PROJECT_DIR"

# Create run script
cat > "$PROJECT_DIR/run.sh" << EOF
#!/bin/bash
cd "$PROJECT_DIR"
source venv/bin/activate
python rpi_control.py
EOF
chmod +x "$PROJECT_DIR/run.sh"

# Final setup
print_status "Performing final setup..."

# Copy main script if it exists
if [[ -f "rpi_control.py" ]]; then
    cp rpi_control.py "$PROJECT_DIR/"
    print_success "Main script copied to project directory"
fi

# Copy functionality check script
if [[ -f "functionality_check.py" ]]; then
    cp functionality_check.py "$PROJECT_DIR/"
    chmod +x "$PROJECT_DIR/functionality_check.py"
    print_success "Functionality check script copied"
fi

# Create system validation script
cat > "$PROJECT_DIR/validate_system.sh" << 'EOF'
#!/bin/bash
# System validation and health check script
cd "$(dirname "$0")"
source venv/bin/activate

echo "Running system functionality check..."
python functionality_check.py

echo ""
echo "Running basic application test..."
timeout 10s python -c "
import sys
sys.path.insert(0, '.')
try:
    # Test basic imports without starting GUI
    from rpi_control import Controller
    print('✅ Main application imports successfully')
    
    # Test configuration loading
    controller = Controller()
    print('✅ Controller initialization successful')
    print('✅ Basic application test PASSED')
except Exception as e:
    print(f'❌ Application test FAILED: {e}')
    sys.exit(1)
" || echo "⚠️  Application test timed out or failed"

echo ""
echo "System validation complete!"
EOF
chmod +x "$PROJECT_DIR/validate_system.sh"

# Run initial functionality check
print_status "Running initial functionality check..."
cd "$PROJECT_DIR"
source venv/bin/activate
if [[ -f "functionality_check.py" ]]; then
    python functionality_check.py || print_warning "Some functionality checks failed - review the output above"
else
    print_warning "Functionality check script not found"
fi

# Deactivate virtual environment
deactivate

print_success "Installation completed successfully!"
echo ""
echo "======================================================"
echo "Installation Summary:"
echo "======================================================"
echo "Project Directory: $PROJECT_DIR"
echo "Virtual Environment: $VENV_DIR"
echo "Desktop Shortcut: $DESKTOP_DIR/RPi_Control.desktop"
echo ""
echo "🔧 Service Management:"
echo "• Manage service: cd $PROJECT_DIR && ./manage_service.sh"
echo "• Service status: ./manage_service.sh status"
echo "• Enable auto-boot: ./manage_service.sh enable"
echo "• View logs: ./manage_service.sh logs"
echo ""
echo "🚀 Starting Options:"
echo "1. Manual: cd $PROJECT_DIR && ./run.sh"
echo "2. Desktop: Double-click the desktop shortcut"
echo "3. Service: ./manage_service.sh start"
echo ""
echo "To validate the system:"
echo "cd $PROJECT_DIR && ./validate_system.sh"
echo ""
if [[ $IS_PI == true ]]; then
    if [[ "${autoboot_choice:-2}" == "3" ]]; then
        print_warning "IMPORTANT: Reboot required for auto-login to take effect:"
        print_warning "sudo reboot"
        print_success "After reboot, the GUI will start automatically!"
    else
        print_warning "IMPORTANT: Please reboot to apply all Pi-specific settings:"
        print_warning "sudo reboot"
        if [[ "${autoboot_choice:-2}" == "1" ]]; then
            print_warning "After reboot, log in to desktop to auto-start the GUI"
        fi
    fi
else
    print_warning "Please log out and back in to apply group membership changes"
    if [[ "${autoboot_choice:-2}" == "1" ]]; then
        print_warning "GUI will auto-start when you log into the desktop"
    fi
fi
echo "======================================================"
