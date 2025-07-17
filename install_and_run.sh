#!/bin/bash

set -e

echo "Updating package lists..."
sudo apt-get update

echo "Installing system dependencies..."
sudo apt-get install -y python3 python3-pip python3-tk python3-serial python3-venv

echo "Creating Python virtual environment..."
python3 -m venv venv

echo "Activating virtual environment and installing Python requirements..."
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt

echo "Copying systemd service file..."
sudo cp rpicontrol.service /etc/systemd/system/rpicontrol.service

echo "Enabling and starting rpicontrol service..."
sudo systemctl daemon-reload
sudo systemctl enable rpicontrol.service
sudo systemctl start rpicontrol.service

echo "Setup complete."
echo "You can check the status with: sudo systemctl status rpicontrol.service"
echo "The program will now run on boot. To run it immediately, use:"
echo "  source venv/bin/activate && python rpi_control.py"
