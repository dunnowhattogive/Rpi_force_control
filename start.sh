#!/bin/bash
# Simple start script for RPi Control GUI

cd "$(dirname "$0")"

# Check if virtual environment exists
if [[ ! -d "venv" ]]; then
    echo "❌ Virtual environment not found"
    echo "🔧 Run: ./manage.sh setup"
    exit 1
fi

# Activate virtual environment
source venv/bin/activate

# Check if main script exists
if [[ ! -f "rpi_control.py" ]]; then
    echo "❌ Main script not found: rpi_control.py"
    exit 1
fi

# Start the application
echo "🎮 Starting RPi Control GUI..."
python rpi_control.py
