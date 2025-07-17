# rpi_control.py

## Overview

[`rpi_control.py`](rpi_control.py) is a Python script designed to run on a Raspberry Pi for controlling a stepper motor and up to three servo motors, with real-time force feedback via a load cell. It features a Tkinter-based GUI for interactive control and monitoring. The script is intended for use in laboratory or automation setups where precise motor positioning and force regulation are required.

---

## Features

- **Stepper Motor Control:**  
  Move a stepper motor in both directions, with adjustable step count and direction, using GPIO pins.

- **Servo Motor Control:**  
  Independently control up to three servo motors via PWM, setting their angles from 0° to 180°.
  - **Preset Buttons:** Quickly set each servo to 0°, 30°, 60°, 90°, 120°, 150°, or 180° using dedicated buttons in the GUI.

- **Force Feedback:**  
  Continuously read force data from a serial-connected load cell (e.g., via `/dev/ttyUSB0`), and use this feedback to automatically adjust the stepper motor to maintain a target force.

- **Interactive GUI:**  
  - Adjust servo angles with sliders.
  - Use preset buttons for common servo angles.
  - Set the force threshold and tolerance.
  - View real-time force readings.
  - See status feedback (below/within/above threshold) with color-coded indicators.
  - **Enable/Disable Controls:** Checkboxes allow you to enable or disable servo and stepper motor controls individually.
  - **Increase/Decrease Force Threshold:** Buttons to adjust the force threshold value.
  - **Stop Program Button:** Gracefully stops the program when clicked.

- **Threaded Architecture:**  
  The GUI runs in a separate thread, allowing real-time hardware control and feedback without blocking the user interface.

- **PID Force Feedback Control:**  
  The stepper motor is controlled by a PID algorithm. You can tune the proportional (Kp), integral (Ki), and derivative (Kd) gains in the Settings tab for optimal force regulation.

- **Works Without Hardware:**  
  The GUI and all test features work even if no hardware (GPIO, serial, load cell) is connected.  
  Hardware errors are handled gracefully and simulated actions are printed to the console for development and testing.

- **Error Logging:**  
  If the program fails to start (e.g., missing serial port, GPIO errors), the error is written to the system log (`syslog`) for troubleshooting.

---

## Hardware Requirements

- **Raspberry Pi** (with RPi.GPIO and serial support)
- **Stepper Motor** (with driver connected to GPIO pins 16, 20, 21)
- **Up to 3 Servo Motors** (connected to GPIO pins 17, 27, 22 by default)
- **Load Cell** with serial output (connected to `/dev/ttyUSB0` or similar)
- **Python 3** with the following packages:
  - `RPi.GPIO`
  - `pyserial`
  - `tkinter`

---

## Pin Assignments

| Function         | GPIO Pin |
|------------------|----------|
| Stepper DIR      | 20       |
| Stepper STEP     | 21       |
| Stepper ENABLE   | 16       |
| Servo 1 PWM      | 17       |
| Servo 2 PWM      | 27       |
| Servo 3 PWM      | 22       |

Modify these in the script if your hardware uses different pins.

---

## Software Structure

### Classes and Functions

- **`StepperMotor`**  
  Controls the stepper motor via GPIO.  
  - `step(steps, direction)`: Move the motor a given number of steps in the specified direction.
  - `disable()`: Disable the stepper driver.

- **`ServoController`**  
  Controls up to three servos via PWM.  
  - `set_angle(idx, angle)`: Set the angle of servo `idx` (0-based).
  - `cleanup()`: Stop all PWM signals.

- **`read_force(ser)`**  
  Reads a line from the serial port, parses it as a float, and returns the force value in grams.

- **`SharedData`**  
  Holds shared state for communication between the main thread and the GUI (force threshold, tolerance, current force, status).

- **`servo_force_gui(servo_ctrl, shared)`**  
  Launches a Tkinter GUI for interactive control and monitoring.
  - **Servo Presets:** Each servo has preset buttons at 30° intervals (0°, 30°, 60°, 90°, 120°, 150°, 180°) for quick angle selection.

- **`Controller`**  
  Integrates hardware control, force feedback, and GUI threading.  
  Handles enabling/disabling of servo and stepper controls, force threshold adjustment, and graceful shutdown.

- **`App`**  
  Main Tkinter GUI application.  
  Provides checkboxes for enabling/disabling servo and stepper controls, buttons for adjusting force threshold, and a stop button.

- **`main()`**  
  Initializes the application and handles signal-based shutdown.

---

## Usage

### 1. Hardware Setup

- Connect the stepper and servo motors to the specified GPIO pins.
- Connect the load cell to the Raspberry Pi via a USB-to-serial adapter.
- Ensure the Raspberry Pi has the required Python packages installed.

### 2. Install Dependencies

You can use the provided installer script to automatically install all dependencies, set up autostart on boot, and run the program:

```sh
chmod +x install_and_run.sh
./install_and_run.sh
```

This script will:
- Install required system and Python packages
- Install Python dependencies from `requirements.txt`
- Copy the `rpicontrol.service` file to the correct location
- Enable and start the service for autostart on boot

Alternatively, you can install dependencies manually:

```sh
pip3 install -r requirements.txt
```

Or, using apt for system packages:

```sh
sudo apt-get install python3-tk python3-serial python3-rpi.gpio
```

### 3. Run the Script

```sh
python3 rpi_control.py
```

### 4. GUI Controls

- **Servo Sliders:** Adjust the angle of each servo motor in real time.
- **Servo Preset Buttons:** Instantly set each servo to 0°, 30°, 60°, 90°, 120°, 150°, or 180°.
- **Force Threshold Slider:** Set the target force (in grams).
- **Force Display:** Shows the current force measured by the load cell.
- **Status Indicator:**  
  - **Blue:** Below threshold (stepper moves up)
  - **Green:** Within threshold (stepper holds)
  - **Red:** Above threshold (stepper moves down)
- **Enable Servo Control:** Checkbox to enable/disable servo controls.
- **Enable Stepper Control:** Checkbox to enable/disable stepper controls and threshold adjustment.
- **Increase/Decrease Force Threshold:** Buttons to adjust the force threshold value.
- **Increase/Decrease Force:** Buttons to manually increase or decrease the force applied through the stepper motor.
- **PID Parameters:**  
  - **Kp, Ki, Kd fields:** Configure the proportional, integral, and derivative gains for the PID force feedback controller in the Settings tab.  
  - **Apply PID Button:** Apply new PID values at runtime.
- **Stop Program Button:** Gracefully stops the program when clicked.

### 5. Auto-Start on Boot

To run this program automatically on device boot, use the installer script as described above, or follow these manual steps:

1. Copy `rpicontrol.service` to `/etc/systemd/system/`:
   ```sh
   sudo cp rpicontrol.service /etc/systemd/system/rpicontrol.service
   ```
2. Enable and start the service:
   ```sh
   sudo systemctl enable rpicontrol.service
   sudo systemctl start rpicontrol.service
   ```

The service will now start the program automatically on boot.

### 6. Stopping the Program from the GUI

A "Stop Program" button has been added to the GUI.  
Clicking this button will gracefully stop the program.

### 7. Stopping the Program

- Press `Ctrl+C` in the terminal to stop the script safely.  
  The script will disable the motors and clean up GPIO on exit.

---

## Customization

- **Change Serial Port:**  
  Edit the `SERIAL_PORT` variable in `Controller` to match your hardware (e.g., `/dev/ttyUSB1`).

- **Adjust Stepper/Servo Pins:**  
  Modify `DIR_PIN`, `STEP_PIN`, `ENABLE_PIN` in `StepperMotor` and `SERVO_PINS` in `ServoController` as needed.

- **Force Target and Tolerance:**  
  Set initial values in `FORCE_TARGET` and `FORCE_TOLERANCE` in `Controller`.

---

## Example Workflow

1. Start the script.
2. Use the GUI to set servo angles and force threshold.
3. Use preset buttons for quick servo positioning.
4. The system will automatically move the stepper motor to maintain the measured force within the specified threshold and tolerance.
5. Monitor the force and status in real time.
6. Use checkboxes to enable/disable servo or stepper controls as needed.

---

## Troubleshooting

- **No Hardware Connected:**  
  You can run and test the GUI, settings, and tests without any hardware attached.  
  All hardware actions are simulated and printed to the console.

- **Startup Errors:**  
  If the program fails to start (e.g., missing `/dev/ttyUSB0`, GPIO errors), the error is logged to `syslog` for diagnosis.

- **Serial Port Not Found:**  
  If `/dev/ttyUSB0` is not present, the GUI will still run and display simulated force readings.

- **GPIO Errors:**  
  If GPIO cannot be initialized (e.g., not running on a Raspberry Pi), all motor actions are simulated.

---

## File Reference

- [rpi_control.py](rpi_control.py): Main script for motor and force control.
- [requirements.txt](requirements.txt): Python package requirements.
- [install_and_run.sh](install_and_run.sh): Installer script for dependencies and autostart setup.
- [rpicontrol.service](rpicontrol.service): Systemd service file for autostart on boot.

## File Explanations

### rpi_control.py

This is the main Python script and entry point for the RPi Control application.  
It contains:
- The GUI logic (using Tkinter)
- Core functionality for controlling the device, including force reading and feedback
- Automatic adjustment of force:  
  - If the force reading goes below a threshold, the stepper motor turns anti-clockwise to increase force  
  - If the force reading goes above the threshold, the stepper motor turns clockwise to decrease force
- Manual "Increase Force Threshold" and "Decrease Force Threshold" buttons to adjust the threshold value
- Manual "Increase Force" and "Decrease Force" buttons to directly control the stepper motor (these allow you to manually apply or release force through the stepper motor regardless of the threshold)
- Checkboxes to enable/disable servo and stepper motor controls individually
- **Servo and Stepper Motor Pin Selection:** GUI fields and an "Apply Pins" button allow you to set the GPIO pins for all motors at runtime.
- A "Stop Program" button for graceful shutdown

### install_and_run.sh

This shell script automates the installation of all required dependencies, sets up the systemd service for autostart, and can optionally run the main program immediately.

### rpicontrol.service

A systemd service file that ensures `rpi_control.py` runs automatically on boot.

### main.py

This file contains the main application logic, integrating the GUI and hardware control components.  
It initializes the shared data, sets up the controller and GUI, and manages the application lifecycle.

## Platform Support

- **Raspberry Pi:** This program is designed to run on Raspberry Pi devices.
- **LCD Touch Screen:** The GUI is compatible with LCD touch screens connected to the Raspberry Pi, allowing for touch-based interaction.

---

## Using This Project on Non-Linux Systems

This project is primarily designed for Raspberry Pi (Linux) hardware, using GPIO and serial interfaces specific to the Pi.  
For non-Linux users (e.g., Windows, macOS):

- **Direct Use:**  
  Most features (GPIO, RPi.GPIO, systemd, etc.) will not work natively on Windows or macOS, as they require Raspberry Pi hardware and Linux-specific libraries.

- **Development/Testing:**  
  - You can run and test the GUI logic on Windows/macOS by commenting out or mocking hardware-specific code (e.g., RPi.GPIO, serial).
  - Use Python's `unittest.mock` or conditional imports to bypass hardware dependencies for development.

- **Emulation:**  
  - Consider using a Raspberry Pi emulator or a virtual machine running Raspberry Pi OS for more complete testing.
  - Some GPIO libraries offer "mock" modes for development.

- **Deployment:**  
  - For actual hardware control, you must deploy and run the project on a Raspberry Pi running Raspberry Pi OS (Linux).

- **Alternative:**  
  - If you need cross-platform hardware control, consider using USB-based controllers or microcontrollers (e.g., Arduino) that communicate via serial/USB, and adapt the code accordingly.

---

## Testing

The GUI includes a **Tests** tab where you can run both unit and system tests:

- **Unit Tests:**  
  Validate core logic such as servo duty cycle calculation, stepper motor disable, shared data initialization, force reading parsing, and PID parameter assignment.

- **System Tests:**  
  Exercise hardware control methods, manual force control, threshold adjustment, PID logic, and force feedback/automatic control.  
  The PID logic test checks error, output, step calculation, and direction.  
  The force feedback tests simulate force readings below, within, and above the threshold and verify correct stepper motor response.

Test results are displayed in the GUI for easy verification.

- **Simulated Hardware:**  
  All tests and GUI actions work in simulation mode if hardware is not available.

---

## License

This script is provided as-is for educational and research purposes.  
Adapt and modify as needed.#   R p i _ f o r c e _ c o n t r o l 
 