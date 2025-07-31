#!/bin/bash

# Project management script for RPi Force Control
set -euo pipefail  # Exit on error, undefined vars, pipe failures

CURRENT_USER=$(whoami)
PROJECT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SERVICE_NAME="rpicontrol@${CURRENT_USER}.service"
SYSTEMD_USER_DIR="${HOME}/.config/systemd/user"

# Color output functions
print_success() { echo -e "\e[32m$1\e[0m"; }
print_error() { echo -e "\e[31m$1\e[0m"; }
print_warning() { echo -e "\e[33m$1\e[0m"; }
print_status() { echo -e "\e[34m$1\e[0m"; }

# Error handling function
handle_error() {
    local exit_code=$?
    print_error "Command failed with exit code $exit_code"
    return $exit_code
}

show_help() {
    echo "RPi Force Control - Project Management Script"
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  setup           Complete setup for fresh Raspberry Pi (recommended)"
    echo "  setup-service   Setup and install systemd service only"
    echo "  start           Start the service"
    echo "  stop            Stop the service"
    echo "  restart         Restart the service"
    echo "  status          Show service status"
    echo "  logs            Show service logs"
    echo "  logs-follow     Follow service logs in real-time"
    echo "  enable          Enable service to start on boot"
    echo "  disable         Disable service from starting on boot"
    echo "  remove-service  Remove the systemd service"
    echo "  setup-venv      Setup Python virtual environment"
    echo "  install-deps    Install Python dependencies"
    echo "  validate        Validate system configuration"
    echo "  update          Update system and Python packages"
    echo "  backup          Create system backup"
    echo "  restore         Restore from backup"
    echo "  monitor         System monitoring"
    echo "  temp            Check Pi temperature"
    echo "  gpio            Check GPIO status"
    echo "  clean           Clean temporary files"
    echo "  check-boot      Check auto-boot configuration"
    echo "  fix-boot        Fix auto-boot issues"
    echo "  setup-autostart Setup desktop autostart (alternative to service)"
    echo "  debug           Debug service startup issues"
    echo "  run-manual      Run application manually for testing"
    echo "  test-service    Test service startup with detailed output"
    echo "  help            Show this help message"
}

setup_venv() {
    print_status "Setting up Python virtual environment..."
    if [[ ! -d "venv" ]]; then
        if python3 -m venv venv; then
            print_success "Virtual environment created"
        else
            print_error "Failed to create virtual environment"
            return 1
        fi
    else
        print_warning "Virtual environment already exists"
    fi
}

install_deps() {
    print_status "Installing Python dependencies..."
    if [[ -f "requirements.txt" ]]; then
        if [[ -d "venv" ]]; then
            # Temporarily disable exit on error for deactivate
            set +e
            source venv/bin/activate
            if pip install -r requirements.txt; then
                print_success "Dependencies installed"
                deactivate
                set -e
            else
                print_error "Failed to install dependencies"
                deactivate
                set -e
                return 1
            fi
        else
            print_error "Virtual environment not found. Run 'setup-venv' first."
            return 1
        fi
    else
        print_warning "requirements.txt not found"
    fi
}

cleanup_service_files() {
    print_status "Cleaning up duplicate service files..."
    
    # Remove backup service files from user systemd directory
    local files=("rpicontrol.service.backup" "rpicontrol.service.configured")
    for file in "${files[@]}"; do
        if [[ -f "$HOME/.config/systemd/user/$file" ]]; then
            rm -f "$HOME/.config/systemd/user/$file"
            print_success "Removed $file from user directory"
        fi
    done
    
    # Clean up project directory duplicates
    local timestamp
    timestamp=$(date +%Y%m%d_%H%M%S)
    for file in "${files[@]}"; do
        if [[ -f "$PROJECT_DIR/$file" ]] && [[ -f "$PROJECT_DIR/rpicontrol.service" ]]; then
            if diff "$PROJECT_DIR/rpicontrol.service" "$PROJECT_DIR/$file" >/dev/null 2>&1; then
                rm -f "$PROJECT_DIR/$file"
                print_success "Removed duplicate $file"
            else
                mv "$PROJECT_DIR/$file" "$PROJECT_DIR/$file.$timestamp"
                print_warning "Renamed different $file to $file.$timestamp"
            fi
        fi
    done
}

debug_service() {
    print_status "Debugging service startup issues..."
    echo ""
    
    # Check if main script exists
    echo "🔍 Main Application:"
    if [[ -f "rpi_control.py" ]]; then
        print_success "✅ rpi_control.py found"
    else
        print_error "❌ rpi_control.py missing - this is the main issue!"
        print_status "The service is trying to run 'python rpi_control.py' but the file doesn't exist."
        print_status "Please ensure you have the main application file in the project directory."
        return 1
    fi
    
    # Check virtual environment
    echo ""
    echo "🐍 Virtual Environment:"
    if [[ -d "venv" ]]; then
        print_success "✅ venv directory exists"
        if [[ -f "venv/bin/activate" ]]; then
            print_success "✅ activate script exists"
            
            # Test activation
            set +e
            if source venv/bin/activate 2>/dev/null; then
                print_success "✅ Virtual environment can be activated"
                
                # Check Python executable
                if command -v python >/dev/null 2>&1; then
                    print_success "✅ Python available in venv: $(python --version)"
                else
                    print_error "❌ Python not available in venv"
                fi
                
                deactivate 2>/dev/null || true
            else
                print_error "❌ Cannot activate virtual environment"
            fi
            set -e
        else
            print_error "❌ activate script missing"
        fi
    else
        print_error "❌ venv directory missing"
    fi
    
    # Check service file
    echo ""
    echo "🔧 Service Configuration:"
    if [[ -f "${SYSTEMD_USER_DIR}/${SERVICE_NAME}" ]]; then
        print_success "✅ Service file exists"
        echo "Service file content:"
        cat "${SYSTEMD_USER_DIR}/${SERVICE_NAME}"
    else
        print_error "❌ Service file missing"
    fi
    
    # Check service logs
    echo ""
    echo "📋 Recent Service Logs:"
    journalctl --user -u "${SERVICE_NAME}" --no-pager -n 10 2>/dev/null || print_status "No service logs available"
    
    # Check current directory and files
    echo ""
    echo "📁 Current Directory Contents:"
    ls -la "$PROJECT_DIR"
    
    # Check X11 availability
    echo ""
    echo "🖼️  Display Environment:"
    if [[ -n "${DISPLAY:-}" ]]; then
        print_success "✅ DISPLAY set: $DISPLAY"
        if command -v xset >/dev/null 2>&1; then
            if xset q &>/dev/null; then
                print_success "✅ X11 server accessible"
            else
                print_warning "⚠️  X11 server not accessible"
            fi
        else
            print_warning "⚠️  xset command not available"
        fi
    else
        print_warning "⚠️  DISPLAY not set"
    fi
}

run_manual() {
    print_status "Running application manually for testing..."
    
    if [[ ! -f "rpi_control.py" ]]; then
        print_error "❌ rpi_control.py not found!"
        print_status "Cannot run the application without the main script."
        return 1
    fi
    
    if [[ ! -d "venv" ]]; then
        print_error "❌ Virtual environment not found!"
        print_status "Run './manage.sh setup-venv' first."
        return 1
    fi
    
    print_status "Activating virtual environment and running application..."
    cd "$PROJECT_DIR"
    
    # Activate venv and run
    set +e
    source venv/bin/activate
    
    print_status "Running: python rpi_control.py"
    echo "Press Ctrl+C to stop the application"
    echo ""
    
    python rpi_control.py
    local exit_code=$?
    
    deactivate 2>/dev/null || true
    set -e
    
    echo ""
    if [[ $exit_code -eq 0 ]]; then
        print_success "Application ran successfully"
    else
        print_error "Application exited with code: $exit_code"
    fi
}

setup_service() {
    print_status "Setting up systemd service..."
    
    # First check if main script exists
    if [[ ! -f "rpi_control.py" ]]; then
        print_error "❌ rpi_control.py not found!"
        print_status "Cannot create service without the main application script."
        print_status "Please ensure rpi_control.py exists in the project directory."
        return 1
    fi
    
    # Clean up old/duplicate service files first
    cleanup_service_files
    
    # Create systemd user directory if it doesn't exist
    mkdir -p "${SYSTEMD_USER_DIR}"
    
    # Get user ID safely
    local user_id
    user_id=$(id -u)
    
    # Create the service file with dynamic paths - improved for GUI boot
    cat > "${SYSTEMD_USER_DIR}/${SERVICE_NAME}" << EOF
[Unit]
Description=RPi Control GUI - Load Cell and Motor Control System
After=graphical-session.target network.target multi-user.target
Wants=graphical-session.target

[Service]
Type=simple
User=${CURRENT_USER}
Environment=DISPLAY=:0
Environment=XDG_RUNTIME_DIR=/run/user/${user_id}
Environment=PYTHONPATH=${PROJECT_DIR}
Environment=PYTHONUNBUFFERED=1
Environment=QT_X11_NO_MITSHM=1
# Wait for X11 to be ready with timeout and fallback
ExecStartPre=/bin/bash -c 'timeout 30 bash -c "while ! xset q &>/dev/null; do sleep 1; done" || echo "X11 not ready, proceeding anyway"'
ExecStart=/bin/bash -c 'cd ${PROJECT_DIR} && source venv/bin/activate && python rpi_control.py'
WorkingDirectory=${PROJECT_DIR}
Restart=on-failure
RestartSec=15
StartLimitBurst=3
StartLimitIntervalSec=300
StandardOutput=journal
StandardError=journal
KillMode=mixed
TimeoutStartSec=90
TimeoutStopSec=15

[Install]
WantedBy=default.target graphical-session.target
EOF
    
    # Reload systemd and enable the service
    if systemctl --user daemon-reload && systemctl --user enable "${SERVICE_NAME}"; then
        print_success "Service ${SERVICE_NAME} has been created and enabled"
    else
        print_error "Failed to setup service"
        return 1
    fi
}

complete_setup() {
    print_status "=== RPi Force Control - Complete Setup ==="
    print_status "Setting up project for fresh Raspberry Pi..."
    echo ""
    
    # Temporarily disable exit on error for individual steps
    set +e
    
    # Step 1: Setup Python virtual environment
    print_status "Step 1/6: Setting up Python virtual environment..."
    if ! setup_venv; then
        print_error "Failed to setup virtual environment"
        set -e
        return 1
    fi
    echo ""
    
    # Step 2: Install dependencies
    print_status "Step 2/6: Installing Python dependencies..."
    install_deps  # This can fail but we continue
    echo ""
    
    # Step 3: Setup systemd service
    print_status "Step 3/6: Setting up systemd service..."
    if ! setup_service; then
        print_error "Failed to setup service"
        set -e
        return 1
    fi
    echo ""
    
    # Step 4: Enable user lingering for boot startup
    print_status "Step 4/6: Enabling user lingering for boot startup..."
    if sudo loginctl enable-linger "${CURRENT_USER}"; then
        print_success "User lingering enabled - service will start on boot"
    else
        print_warning "Could not enable user lingering. Service may not start on boot."
        print_warning "You can enable it manually with: sudo loginctl enable-linger ${CURRENT_USER}"
    fi
    echo ""
    
    # Step 5: Configure auto-login for GUI
    print_status "Step 5/6: Configuring auto-login for GUI..."
    configure_auto_login
    echo ""
    
    # Step 6: Start the service
    print_status "Step 6/6: Starting the service..."
    if systemctl --user start "${SERVICE_NAME}"; then
        print_success "Service started successfully"
    else
        print_error "Failed to start service"
    fi
    echo ""
    
    set -e  # Re-enable exit on error
    
    print_success "=== Setup Complete! ==="
    print_status "Service Status:"
    systemctl --user status "${SERVICE_NAME}" --no-pager -l || true
    echo ""
    print_success "The service is now running and will start automatically on boot."
    print_status "Use './manage.sh logs-follow' to monitor real-time logs."
    print_status "Use './manage.sh check-boot' to verify auto-boot configuration."
    print_status "Use './manage.sh help' for all available commands."
    echo ""
    print_warning "A reboot is recommended to test auto-boot functionality."
}

configure_auto_login() {
    print_status "Configuring auto-login for GUI access..."
    
    # Set default target to graphical
    sudo systemctl set-default graphical.target
    
    # Configure auto-login
    sudo mkdir -p /etc/systemd/system/getty@tty1.service.d/
    cat << EOF | sudo tee /etc/systemd/system/getty@tty1.service.d/autologin.conf > /dev/null
[Service]
ExecStart=
ExecStart=-/sbin/agetty --autologin ${CURRENT_USER} --noclear %I \$TERM
EOF
    
    print_success "Auto-login configured for user: ${CURRENT_USER}"
}

setup_autostart() {
    print_status "Setting up desktop autostart (alternative to systemd service)..."
    
    # Create autostart directory
    mkdir -p "$HOME/.config/autostart"
    
    # Create autostart desktop entry
    cat > "$HOME/.config/autostart/rpi-control.desktop" << EOF
[Desktop Entry]
Type=Application
Name=RPi Control System
Comment=Load Cell and Motor Control System
Exec=${PROJECT_DIR}/start_gui.sh
Icon=applications-system
Hidden=false
NoDisplay=false
X-GNOME-Autostart-enabled=true
StartupNotify=true
Categories=System;Utility;
EOF
    
    # Create start_gui.sh script
    cat > "${PROJECT_DIR}/start_gui.sh" << EOF
#!/bin/bash
cd "${PROJECT_DIR}"
source venv/bin/activate
python rpi_control.py
EOF
    chmod +x "${PROJECT_DIR}/start_gui.sh"
    
    print_success "Desktop autostart configured"
    print_status "GUI will start automatically when desktop session begins"
}

check_boot_config() {
    print_status "Checking auto-boot configuration..."
    echo ""
    
    local issues=0
    
    # Check service status
    echo "🔧 Systemd Service:"
    if systemctl --user is-enabled "${SERVICE_NAME}" >/dev/null 2>&1; then
        print_success "✅ Service enabled for auto-start"
        if systemctl --user is-active "${SERVICE_NAME}" >/dev/null 2>&1; then
            print_success "✅ Service currently running"
        else
            print_warning "⚠️  Service not currently running"
            ((issues++))
        fi
    else
        print_error "❌ Service not enabled for auto-start"
        ((issues++))
    fi
    
    # Check user lingering
    echo ""
    echo "👤 User Lingering:"
    if sudo loginctl show-user "${CURRENT_USER}" | grep -q "Linger=yes"; then
        print_success "✅ User lingering enabled"
    else
        print_error "❌ User lingering not enabled"
        print_status "Services won't start without login"
        ((issues++))
    fi
    
    # Check auto-login
    echo ""
    echo "🖥️  Auto-Login:"
    if [[ -f /etc/systemd/system/getty@tty1.service.d/autologin.conf ]]; then
        if grep -q "autologin ${CURRENT_USER}" /etc/systemd/system/getty@tty1.service.d/autologin.conf; then
            print_success "✅ Auto-login configured for ${CURRENT_USER}"
        else
            print_warning "⚠️  Auto-login configured for different user"
            ((issues++))
        fi
    else
        print_warning "⚠️  Auto-login not configured (manual login required)"
        ((issues++))
    fi
    
    # Check display target
    echo ""
    echo "🎯 Boot Target:"
    local current_target
    current_target=$(systemctl get-default)
    if [[ "$current_target" == "graphical.target" ]]; then
        print_success "✅ Graphical target set as default"
    else
        print_warning "⚠️  Default target is $current_target (should be graphical.target)"
        ((issues++))
    fi
    
    # Check virtual environment
    echo ""
    echo "🐍 Virtual Environment:"
    if [[ -d "venv" ]] && [[ -f "venv/bin/activate" ]]; then
        print_success "✅ Virtual environment available"
    else
        print_error "❌ Virtual environment missing"
        ((issues++))
    fi
    
    # Check display environment
    echo ""
    echo "🖼️  Display Environment:"
    if [[ -n "${DISPLAY:-}" ]]; then
        print_success "✅ DISPLAY variable set: $DISPLAY"
        if xset q &>/dev/null; then
            print_success "✅ X11 server accessible"
        else
            print_warning "⚠️  X11 server not accessible"
        fi
    else
        print_warning "⚠️  DISPLAY variable not set"
    fi
    
    # Summary
    echo ""
    echo "📋 Summary:"
    if [[ $issues -eq 0 ]]; then
        print_success "✅ Auto-boot configuration looks good!"
        print_status "GUI should start automatically on next boot"
    else
        print_error "❌ Found $issues configuration issues"
        print_status "Run './manage.sh fix-boot' to resolve issues"
    fi
    
    # Show recent logs if service exists
    echo ""
    echo "📋 Recent Service Logs:"
    journalctl --user -u "${SERVICE_NAME}" --no-pager -n 5 2>/dev/null || print_status "No service logs available"
}

fix_boot_issues() {
    print_status "Fixing auto-boot issues..."
    echo ""
    
    # Fix user lingering
    print_status "1. Enabling user lingering..."
    if sudo loginctl enable-linger "${CURRENT_USER}"; then
        print_success "✅ User lingering enabled"
    else
        print_error "❌ Failed to enable user lingering"
    fi
    
    # Fix auto-login
    print_status "2. Configuring auto-login..."
    configure_auto_login
    
    # Fix boot target
    print_status "3. Setting graphical boot target..."
    if sudo systemctl set-default graphical.target; then
        print_success "✅ Graphical target set as default"
    else
        print_error "❌ Failed to set graphical target"
    fi
    
    # Recreate service with improved configuration
    print_status "4. Updating service configuration..."
    setup_service
    
    # Reload and restart service
    print_status "5. Reloading and restarting service..."
    systemctl --user daemon-reload
    systemctl --user restart "${SERVICE_NAME}" || true
    
    print_success "Boot issues fixed!"
    print_warning "Reboot the system to test: sudo reboot"
    echo ""
    print_status "After reboot, check status with: ./manage.sh check-boot"
}

app_validate() {
    cd "$PROJECT_DIR"
    print_status "Validating system configuration..."
    
    local errors=0
    
    # Check main script
    if [[ -f "rpi_control.py" ]]; then
        print_success "✅ Main script found"
    else
        print_error "❌ Main script missing"
        ((errors++))
    fi
    
    # Check virtual environment
    if [[ -d "venv" ]] && [[ -f "venv/bin/activate" ]]; then
        print_success "✅ Virtual environment found"
        
        # Test activation
        set +e
        if source venv/bin/activate 2>/dev/null; then
            print_success "✅ Virtual environment functional"
            deactivate 2>/dev/null || true
        else
            print_error "❌ Virtual environment corrupted"
            ((errors++))
        fi
        set -e
    else
        print_error "❌ Virtual environment missing"
        ((errors++))
    fi
    
    # Check service
    if [[ -f "${SYSTEMD_USER_DIR}/${SERVICE_NAME}" ]]; then
        print_success "✅ Service file found"
    else
        print_error "❌ Service file missing"
        ((errors++))
    fi
    
    # Check Pi hardware
    if [[ -f /proc/device-tree/model ]]; then
        local pi_model
        pi_model=$(cat /proc/device-tree/model 2>/dev/null || echo "Unknown")
        print_success "✅ Pi hardware: $pi_model"
    fi
    
    if [[ $errors -eq 0 ]]; then
        print_success "✅ All validations passed"
    else
        print_error "❌ Found $errors validation errors"
        return 1
    fi
}

system_update() {
    print_status "Updating system..."
    cd "$PROJECT_DIR"
    
    # Update system packages
    if sudo apt update && sudo apt upgrade -y; then
        print_success "System packages updated"
    else
        print_error "Failed to update system packages"
        return 1
    fi
    
    # Update Python packages
    if [[ -d "venv" ]]; then
        set +e
        source venv/bin/activate
        if pip install --upgrade pip setuptools wheel; then
            print_success "Python tools updated"
            if [[ -f "requirements.txt" ]] && pip install --upgrade -r requirements.txt; then
                print_success "Python packages updated"
            fi
        else
            print_error "Failed to update Python packages"
        fi
        deactivate 2>/dev/null || true
        set -e
    fi
    
    print_success "System update completed"
}

system_backup() {
    print_status "Creating backup..."
    cd "$PROJECT_DIR"
    
    local backup_dir="config_backups"
    mkdir -p "$backup_dir"
    
    local timestamp backup_file
    timestamp=$(date +%Y%m%d_%H%M%S)
    backup_file="$backup_dir/backup_$timestamp.tar.gz"
    
    if tar -czf "$backup_file" \
        --exclude=venv \
        --exclude=data_logs \
        --exclude="*.pyc" \
        --exclude=__pycache__ \
        --exclude=config_backups \
        .; then
        print_success "Backup created: $backup_file"
    else
        print_error "Failed to create backup"
        return 1
    fi
}

system_restore() {
    print_status "Available backups:"
    if ! ls -la config_backups/backup_*.tar.gz 2>/dev/null; then
        print_error "No backups found"
        return 1
    fi
    
    echo ""
    local backup_file
    read -r -p "Enter backup filename to restore: " backup_file
    
    if [[ -f "config_backups/$backup_file" ]]; then
        if tar -xzf "config_backups/$backup_file"; then
            print_success "Backup restored: $backup_file"
        else
            print_error "Failed to restore backup"
            return 1
        fi
    else
        print_error "Backup file not found"
        return 1
    fi
}

system_clean() {
    print_status "Cleaning temporary files..."
    cd "$PROJECT_DIR"
    
    # Clean Python cache
    find . -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
    find . -name "*.pyc" -delete 2>/dev/null || true
    
    # Clean logs older than 30 days
    find data_logs -name "*.log" -mtime +30 -delete 2>/dev/null || true
    
    # Clean old backups (keep last 10)
    ls -t config_backups/backup_*.tar.gz 2>/dev/null | tail -n +11 | xargs rm -f 2>/dev/null || true
    
    # Clean up duplicate service files
    cleanup_service_files
    
    print_success "Cleanup completed"
}

system_monitor() {
    print_status "System monitoring (Press Ctrl+C to exit)..."
    
    while true; do
        clear
        echo "=== RPi Control System Monitor ==="
        echo "Time: $(date)"
        echo ""
        
        # Service status
        if systemctl --user is-active $SERVICE_NAME >/dev/null 2>&1; then
            echo "🟢 Service: RUNNING"
        else
            echo "🔴 Service: STOPPED"
        fi
        
        # System resources
        if command -v vcgencmd >/dev/null 2>&1; then
            temp=$(vcgencmd measure_temp 2>/dev/null | cut -d= -f2 || echo "N/A")
            echo "🌡️  Temperature: $temp"
            
            throttled=$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2 || echo "N/A")
            if [[ "$throttled" == "0x0" ]]; then
                echo "⚡ Throttling: None"
            else
                echo "⚠️  Throttling: $throttled"
            fi
        fi
        
        # Memory and CPU
        if command -v free >/dev/null 2>&1; then
            memory=$(free -h | grep "Mem:" | awk '{print $3 "/" $2}')
            echo "💾 Memory: $memory"
        fi
        
        if command -v uptime >/dev/null 2>&1; then
            load=$(uptime | awk -F'load average:' '{print $2}')
            echo "📊 Load: $load"
        fi
        
        echo ""
        echo "Press Ctrl+C to exit"
        sleep 5
    done
}

system_temp() {
    if command -v vcgencmd >/dev/null 2>&1; then
        local temp temp_val
        temp=$(vcgencmd measure_temp)
        print_status "Pi Temperature: $temp"
        
        # Check if overheating (requires bc for float comparison)
        temp_val=$(echo "$temp" | grep -o '[0-9.]*')
        if command -v bc >/dev/null 2>&1 && [[ -n "$temp_val" ]]; then
            if (( $(echo "$temp_val > 80" | bc -l) )); then
                print_error "⚠️  Temperature critical! Consider cooling."
            elif (( $(echo "$temp_val > 70" | bc -l) )); then
                print_warning "⚠️  Temperature high. Monitor cooling."
            else
                print_success "Temperature normal."
            fi
        fi
    else
        print_error "Temperature monitoring not available (not on Pi?)"
    fi
}

system_gpio() {
    if command -v gpio >/dev/null 2>&1; then
        print_status "GPIO Status:"
        gpio readall
    elif [[ -d /sys/class/gpio ]]; then
        print_status "GPIO pins in use:"
        ls /sys/class/gpio/ | grep gpio
    else
        print_error "GPIO monitoring not available"
    fi
}

test_service_start() {
    print_status "Testing service startup..."
    
    # Stop service if running
    systemctl --user stop "${SERVICE_NAME}" 2>/dev/null || true
    
    # Clear any previous logs
    journalctl --user --vacuum-time=1s &>/dev/null || true
    
    print_status "Starting service..."
    if systemctl --user start "${SERVICE_NAME}"; then
        print_success "Service start command succeeded"
        
        # Wait a moment for the service to start
        sleep 3
        
        # Check status
        print_status "Service status:"
        systemctl --user status "${SERVICE_NAME}" --no-pager -l || true
        
        # Check logs
        print_status "Recent logs:"
        journalctl --user -u "${SERVICE_NAME}" --no-pager -n 20 || print_status "No logs available"
        
    else
        print_error "Service start command failed"
        return 1
    fi
}

# Main case statement with improved error handling
case "${1:-help}" in
    setup)
        complete_setup || exit 1
        ;;
    setup-service)
        setup_service || exit 1
        ;;
    start)
        if systemctl --user start "${SERVICE_NAME}"; then
            print_success "Service started"
        else
            print_error "Failed to start service"
            print_status "Run './manage.sh debug' to investigate the issue"
            exit 1
        fi
        ;;
    stop)
        if systemctl --user stop "${SERVICE_NAME}"; then
            print_success "Service stopped"
        else
            print_error "Failed to stop service"
            exit 1
        fi
        ;;
    restart)
        if systemctl --user restart "${SERVICE_NAME}"; then
            print_success "Service restarted"
        else
            print_error "Failed to restart service"
            print_status "Run './manage.sh debug' to investigate the issue"
            exit 1
        fi
        ;;
    status)
        systemctl --user status "${SERVICE_NAME}" || true
        ;;
    logs)
        journalctl --user -u "${SERVICE_NAME}" --no-pager || true
        ;;
    logs-follow)
        journalctl --user -u "${SERVICE_NAME}" -f || true
        ;;
    enable)
        if systemctl --user enable "${SERVICE_NAME}"; then
            print_success "Service enabled for auto-start"
        else
            print_error "Failed to enable service"
            exit 1
        fi
        ;;
    disable)
        if systemctl --user disable "${SERVICE_NAME}"; then
            print_success "Service disabled from auto-start"
        else
            print_error "Failed to disable service"
            exit 1
        fi
        ;;
    remove-service)
        systemctl --user stop "${SERVICE_NAME}" 2>/dev/null || true
        systemctl --user disable "${SERVICE_NAME}" 2>/dev/null || true
        rm -f "${SYSTEMD_USER_DIR}/${SERVICE_NAME}"
        systemctl --user daemon-reload || true
        print_success "Service removed"
        ;;
    setup-venv)
        setup_venv || exit 1
        ;;
    install-deps)
        install_deps || exit 1
        ;;
    validate)
        app_validate || exit 1
        ;;
    update)
        system_update || exit 1
        ;;
    backup)
        system_backup || exit 1
        ;;
    restore)
        system_restore || exit 1
        ;;
    monitor)
        system_monitor || exit 1
        ;;
    temp)
        system_temp || exit 1
        ;;
    gpio)
        system_gpio || exit 1
        ;;
    clean)
        system_clean || exit 1
        ;;
    check-boot)
        check_boot_config
        ;;
    fix-boot)
        fix_boot_issues
        ;;
    setup-autostart)
        setup_autostart
        ;;
    debug)
        debug_service
        ;;
    run-manual)
        run_manual
        ;;
    test-service)
        test_service_start
        ;;
    help|--help|-h|"")
        show_help
        ;;
    *)
        print_error "Unknown command: ${1:-<empty>}"
        echo ""
        show_help
        exit 1
        ;;
esac
