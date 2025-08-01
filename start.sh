#!/bin/bash
# Robust start script for RPi Control GUI with comprehensive error checking

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Color output functions
print_success() { echo -e "\e[32m$1\e[0m"; }
print_error() { echo -e "\e[31m$1\e[0m"; }
print_warning() { echo -e "\e[33m$1\e[0m"; }
print_status() { echo -e "\e[34m$1\e[0m"; }

print_status "🎮 Starting RPi Control GUI..."

# Check if virtual environment exists
if [[ ! -d "venv" ]]; then
    print_error "❌ Virtual environment not found"
    print_status "🔧 Run: ./manage.sh setup-venv"
    exit 1
fi

# Check if virtual environment is functional
if [[ ! -f "venv/bin/activate" ]]; then
    print_error "❌ Virtual environment activate script missing"
    print_status "🔧 Run: ./manage.sh setup-venv"
    exit 1
fi

# Activate virtual environment
print_status "Activating virtual environment..."
set +e  # Don't exit on deactivate errors
source venv/bin/activate
activation_result=$?
set -e

if [[ $activation_result -ne 0 ]]; then
    print_error "❌ Failed to activate virtual environment"
    print_status "🔧 Run: ./manage.sh setup-venv"
    exit 1
fi

# Check if main script exists
if [[ ! -f "rpi_control.py" ]]; then
    print_error "❌ Main script not found: rpi_control.py"
    deactivate 2>/dev/null || true
    exit 1
fi

# Check Python availability in venv
if ! command -v python >/dev/null 2>&1; then
    print_error "❌ Python not available in virtual environment"
    deactivate 2>/dev/null || true
    exit 1
fi

# Check if required packages are installed
print_status "Checking dependencies..."
if ! python -c "import tkinter, serial, numpy, gpiod" 2>/dev/null; then
    print_warning "⚠️  Some dependencies missing - attempting to install..."
    if pip install -r requirements.txt; then
        print_success "✅ Dependencies installed"
    else
        print_error "❌ Failed to install dependencies"
        print_status "🔧 Run: ./manage.sh install-deps"
        deactivate 2>/dev/null || true
        exit 1
    fi
fi

# Check display environment (for GUI)
if [[ -z "${DISPLAY:-}" ]]; then
    print_warning "⚠️  DISPLAY variable not set - GUI may not work"
    print_status "Setting DISPLAY=:0 as fallback..."
    export DISPLAY=:0
fi

# Start the application with error handling
print_success "🚀 Starting application..."
set +e
python rpi_control.py
app_exit_code=$?
set -e

# Deactivate virtual environment
deactivate 2>/dev/null || true

# Report exit status
if [[ $app_exit_code -eq 0 ]]; then
    print_success "✅ Application exited normally"
else
    print_error "❌ Application exited with error code: $app_exit_code"
    exit $app_exit_code
fi
