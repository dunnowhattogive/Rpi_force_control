# RPi Control System

A comprehensive Raspberry Pi control system for managing stepper motors, servo motors, and MK10 tension-based load cells with automatic force feedback control.

## Features

### Hardware Support
- **Stepper Motor Control**: NEMA 17 (E21H4N-2.5-900) via Easy Driver controller
- **Servo Control**: Up to 3 SG90 servo motors with PWM control
- **Load Cell Integration**: MK10 tension-based load cell (10kg capacity) with HX711 amplifier
- **Auto-Detection**: Automatic detection of load cell serial ports (USB/UART)
- **Simulation Mode**: Runs without hardware for development/testing

### Control Modes
- **Manual Control**: Direct stepper/servo positioning via GUI
- **Automatic Control**: PID-based force feedback control
- **Preset System**: Configurable angle/step presets for quick positioning
- **Real-time Monitoring**: Live force readings and status indicators

### Software Features
- **Multi-tab GUI**: Organized interface with Main, Settings, and Tests tabs
- **Status Logging**: Timestamped event logging with auto-scroll
- **Configuration Management**: Save/load presets and settings
- **Unit Testing**: Built-in unit and system tests
- **Error Handling**: Graceful fallback to simulation mode

## Hardware Requirements

### Essential Components
- Raspberry Pi 4 (or compatible)
- NEMA 17 Stepper Motor (E21H4N-2.5-900)
- Easy Driver Stepper Controller
- 3x SG90 Servo Motors
- MK10 Tension Load Cell (10kg capacity)
- HX711 Load Cell Amplifier
- USB-to-Serial Adapter (for load cell communication)

### Power Requirements
- 5V 3A PSU (Raspberry Pi)
- 12V 2A PSU (Stepper Motor)
- 5V 2A PSU (Servos and Sensors)

### Wiring Connections

#### Stepper Motor (Easy Driver)
- GPIO 20 → DIR (Direction)
- GPIO 21 → STEP (Step pulse)
- GPIO 16 → ENABLE (Motor enable)
- 12V PSU → Motor power
- Common GND

#### Servo Motors
- GPIO 17 → Servo 1 PWM (Orange wire)
- GPIO 27 → Servo 2 PWM (Orange wire)
- GPIO 22 → Servo 3 PWM (Orange wire)
- 5V PSU → All servo power (Red wires)
- Common GND → All servo ground (Brown wires)

#### Load Cell System
- MK10 Load Cell → HX711 Amplifier
- HX711 → USB-Serial Adapter
- USB-Serial → Raspberry Pi USB port
- 5V PSU → HX711 power
- Common GND

## Installation

### System Dependencies
```bash
sudo apt update
sudo apt install python3 python3-pip python3-venv git

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install Python dependencies
pip install -r requirements.txt
```

### Required Python Packages
```bash
pip install gpiozero pyserial tkinter matplotlib
```

### Hardware Setup
1. Connect all components according to wiring diagram
2. Ensure proper power supply connections
3. Calibrate HX711 amplifier with known weights
4. Test load cell output via serial terminal

## Usage

### Starting the Application
```bash
cd /path/to/RPi_control
source venv/bin/activate
python3 rpi_control.py
```

### GUI Interface

#### Main Tab
- **Live Force Display**: Real-time load cell readings in grams
- **Force Threshold**: Current target force value
- **Force Status Indicator**: Visual status (WITHIN/ABOVE/BELOW range)
- **Servo Controls**: Individual servo angle adjustment with presets
- **Stepper Control**: Manual/automatic mode toggle with quick step presets
- **Status Log**: Timestamped event logging

#### Settings Tab
- **PID Parameters**: Tune proportional, integral, derivative gains
- **Pin Configuration**: Modify GPIO pin assignments
- **Force Threshold**: Set target force values
- **Preset Management**: Configure servo angles and stepper steps
- **Enable/Disable Controls**: Individual component control

#### Tests Tab
- **Unit Tests**: Verify component functionality
- **System Tests**: End-to-end testing with hardware simulation

### Configuration Files

#### Servo Presets (`servo_config.json`)
```json
{
  "servo_presets": [0, 30, 45, 60, 90, 120, 135, 150, 180]
}
```

#### Stepper Presets (`stepper_config.json`)
```json
{
  "stepper_presets": [1, 5, 10, 25, 50, 100]
}
```

### MK10 Load Cell Calibration

#### Prerequisites
- Properly mounted MK10 load cell
- HX711 amplifier correctly wired
- Known calibration weights (1kg, 2kg, 5kg recommended)

#### Calibration Process
1. Connect load cell system
2. Power on and verify serial communication
3. Apply known weights and record HX711 output
4. Calculate calibration factor: `factor = (reading_with_weight - reading_no_weight) / known_weight`
5. Update HX711 firmware/configuration with calibration factor
6. Verify accuracy with multiple test weights

#### Expected Output Format
The HX711 should output calibrated force values in grams via serial:
```
0.00
156.23
345.67
1023.45
```

## API Reference

### Controller Class
```python
controller = Controller()
controller.increase_force()    # Manual stepper increment
controller.decrease_force()    # Manual stepper decrement
controller.set_app(app)        # Set GUI reference for logging
```

### Stepper Motor Control
```python
stepper = StepperMotor()
stepper.step(steps=10, direction=True)   # True=anti-clockwise, False=clockwise
stepper.disable()                        # Disable motor
```

### Servo Control
```python
servo_ctrl = ServoController()
servo_ctrl.set_angle(idx=0, angle=90)    # Set servo 0 to 90 degrees
```

### Load Cell Reading
```python
force = read_force(serial_port)          # Returns force in grams or None
```

## Troubleshooting

### Common Issues

#### Load Cell Not Detected
- **Symptoms**: "No load cell detected (simulation mode)" message
- **Solutions**:
  - Check USB-Serial adapter connection
  - Verify HX711 power supply (5V)
  - Test serial communication manually
  - Ensure proper baud rate (9600)
  - Check device permissions: `sudo usermod -a -G dialout $USER`

#### Stepper Motor Not Moving
- **Symptoms**: "[SIM] Stepper step" messages in log
- **Solutions**:
  - Verify GPIO pin connections
  - Check 12V power supply to Easy Driver
  - Ensure proper wiring (DIR, STEP, ENABLE pins)
  - Test with multimeter for signal presence
  - Run with `sudo` for GPIO access

#### Servo Motors Not Responding
- **Symptoms**: "[SIM] Servo X set to angle Y" messages
- **Solutions**:
  - Check 5V power supply connections
  - Verify PWM signal wires (orange)
  - Ensure common ground connection
  - Test individual servos with simple PWM script

#### GUI Errors
- **Symptoms**: AttributeError or widget-related errors
- **Solutions**:
  - Update tkinter: `sudo apt install python3-tk`
  - Check Python version compatibility (3.8+)
  - Verify all dependencies installed
  - Run from terminal to see full error messages

### GPIO Pin Factory Warnings
These warnings are normal when running without proper GPIO access:
```
PinFactoryFallback: Falling back from lgpio: No module named 'lgpio'
```
Solutions:
- Install GPIO libraries: `sudo apt install python3-lgpio python3-rpi.gpio`
- Run with sudo: `sudo python3 rpi_control.py`
- Add user to gpio group: `sudo usermod -a -G gpio $USER`

### Force Calibration Issues
- **Symptoms**: Incorrect force readings, unstable values
- **Solutions**:
  - Recalibrate HX711 with precision weights
  - Check load cell mounting (proper tension alignment)
  - Verify electrical connections (no loose wires)
  - Shield from electromagnetic interference
  - Use proper gauge wires (18-22 AWG)

## Development

### File Structure
```
RPi_control/
├── rpi_control.py              # Main application
├── generate_connection_diagram.py  # Hardware diagram generator
├── requirements.txt            # Python dependencies
├── README.md                  # This documentation
├── servo_config.json         # Servo preset configuration
├── stepper_config.json       # Stepper preset configuration
└── docs/
    ├── connection_diagrams/   # Generated wiring diagrams
    └── api_documentation.md   # API reference
```

### Adding New Features

#### New Hardware Components
1. Add hardware class in appropriate section
2. Update pin configuration in Settings tab
3. Add control widgets in Main tab
4. Include in unit/system tests
5. Update connection diagram

#### New Control Modes
1. Add mode toggle in GUI
2. Implement control logic in Controller class
3. Add status indicators
4. Include in system tests

### Testing

#### Unit Tests
```bash
python3 -c "from rpi_control import run_unit_tests; print('\n'.join(run_unit_tests()))"
```

#### System Tests
```bash
python3 -c "from rpi_control import *; controller = Controller(); print('\n'.join(run_system_tests(controller)))"
```

## Hardware Connection Diagrams

Generate updated wiring diagrams:
```bash
python3 generate_connection_diagram.py
```

This creates:
- `rpi_control_main_diagram.png` - Complete system overview
- `rpi_control_legend.png` - Component color coding
- `rpi_control_pin_table.png` - Pin assignment reference
- `rpi_control_notes.png` - System notes and safety information
- `rpi_control_complete_documentation.pdf` - All diagrams combined

## Safety Considerations

### Electrical Safety
- Use proper gauge wires for power connections (18-22 AWG minimum)
- Ensure stable power supplies with adequate current capacity
- Implement proper grounding for all components
- Use fuses/circuit breakers for protection
- Avoid mixing AC and DC circuits

### Mechanical Safety
- Properly mount and secure all moving components
- Ensure adequate clearance for stepper motor rotation
- Use appropriate load cell mounting hardware
- Test all mechanical connections before applying full loads
- Implement emergency stop functionality

### Software Safety
- Always test in simulation mode first
- Implement bounds checking for all motor movements
- Use reasonable force thresholds to prevent damage
- Include timeout mechanisms for automatic control
- Log all critical operations for debugging

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Support

For issues and questions:
1. Check the troubleshooting section
2. Review the hardware connection diagrams
3. Test with simulation mode
4. Submit an issue with full error logs and hardware configuration

## Changelog

### Version 2.1.0 (Current)
- Added automatic load cell port detection
- Enhanced error handling and reconnection capability
- Improved connection diagram generation with detailed labeling
- Added comprehensive unit and system testing
- Enhanced documentation with troubleshooting guides
- Optimized GUI performance and responsiveness

### Version 2.0.0
- Added MK10 load cell support with auto-detection
- Implemented PID-based force feedback control
- Enhanced GUI with tabbed interface and status logging
- Added comprehensive testing framework
- Improved error handling and simulation mode
- Generated hardware connection diagrams

### Version 1.0.0
- Initial release with basic stepper and servo control
- Simple GUI interface
- Manual control modes only