# RPi Control System Connection Diagram

## Generating Visual Diagram

To generate a visual connection diagram image, run:

```bash
python generate_connection_diagram.py
```

This will create:
- `rpi_control_connection_diagram.png` - High-resolution image
- `rpi_control_connection_diagram.pdf` - Vector graphics version

**Required dependencies:**
```bash
pip install matplotlib numpy
```

## System Overview

This system integrates a Raspberry Pi with motor control hardware and force sensing for automated positioning and force regulation.

## Hardware Components

### Main Components
- **Raspberry Pi 4/3B+** - Main controller
- **Easy Driver Stepper Motor Driver** - Controls stepper motor
- **NEMA 17 Stepper Motor** - Primary force control actuator
- **3x Servo Motors (SG90 or similar)** - Secondary positioning actuators
- **Load Cell + HX711 Amplifier** - Force measurement
- **USB-to-Serial Adapter** - Load cell data interface

### Power Supplies
- **5V 3A Power Supply** - Raspberry Pi main power
- **12V 2A Power Supply** - Stepper motor power (via Easy Driver)
- **5V 2A Power Supply** - Servo motors and load cell amplifier

## Pin Connections

### Raspberry Pi GPIO Assignments

```
GPIO Pin | Function              | Connection
---------|----------------------|------------------
GPIO 16  | Stepper ENABLE       | Easy Driver ENABLE pin
GPIO 20  | Stepper DIRECTION    | Easy Driver DIR pin  
GPIO 21  | Stepper STEP         | Easy Driver STEP pin
GPIO 17  | Servo 1 PWM          | Servo 1 Signal (Orange)
GPIO 27  | Servo 2 PWM          | Servo 2 Signal (Orange)
GPIO 22  | Servo 3 PWM          | Servo 3 Signal (Orange)
5V       | Servo Power          | Servo VCC (Red)
GND      | Ground               | Servo/Easy Driver GND (Black/Brown)
```

## Connection Diagram

```
                    ┌─────────────────────────────────────────────┐
                    │              RASPBERRY PI 4                │
                    │                                             │
                    │  GPIO 16 (ENABLE) ──────────────────┐      │
                    │  GPIO 20 (DIR)    ──────────────┐   │      │
                    │  GPIO 21 (STEP)   ──────────┐   │   │      │
                    │  GPIO 17 (Servo1) ─────┐    │   │   │      │
                    │  GPIO 27 (Servo2) ──┐  │    │   │   │      │
                    │  GPIO 22 (Servo3) ┐ │  │    │   │   │      │
                    │  5V Power ─────────┼─┼──┼────┼───┼───┼──┐   │
                    │  GND ──────────────┼─┼──┼────┼───┼───┼──┼─┐ │
                    │  USB Port ─────────┼─┼──┼────┼───┼───┼──┼─┼─┤
                    └────────────────────┼─┼──┼────┼───┼───┼──┼─┼─┘
                                         │ │  │    │   │   │  │ │
                    ┌────────────────────┘ │  │    │   │   │  │ │
                    │  ┌────────────────────┘  │    │   │   │  │ │
                    │  │  ┌──────────────────────┘    │   │   │  │ │
                    │  │  │                          │   │   │  │ │
                    ▼  ▼  ▼                          ▼   ▼   ▼  ▼ ▼
            ┌─────────────────────┐            ┌─────────────────────┐
            │    EASY DRIVER      │            │     SERVO MOTORS    │
            │  Stepper Controller │            │                     │
            │                     │            │  ┌─────┐ ┌─────┐    │
            │  VCC ←── 12V PSU    │            │  │SG90 │ │SG90 │    │
            │  GND ←── GND        │            │  │  1  │ │  2  │    │
            │  ENABLE ←── GPIO16  │            │  └─────┘ └─────┘    │
            │  DIR    ←── GPIO20  │            │           ┌─────┐   │
            │  STEP   ←── GPIO21  │            │           │SG90 │   │
            │  A+,A-,B+,B- ──┐    │            │           │  3  │   │
            └─────────────────┼───┘            │           └─────┘   │
                              │                └─────────────────────┘
                              ▼                          ▲
                    ┌─────────────────────┐              │
                    │   NEMA 17 STEPPER   │              │
                    │      MOTOR          │              │
                    │                     │              │
                    │   4-Wire Bipolar    │              │
                    │   Connection        │              │
                    └─────────────────────┘              │
                                                         │
            ┌─────────────────────────────────────────────┘
            │              5V 2A PSU
            │         (Servo Power Supply)
            │
    ┌───────▼──────────────────────────────────────────┐
    │                 USB-SERIAL ADAPTER                │
    │                                                   │
    │  USB ──── To Raspberry Pi USB Port               │
    │  TX/RX ── To Load Cell Amplifier (HX711)         │
    └───────────────────────────────────────────────────┘
                              │
                              ▼
            ┌─────────────────────────────────────────┐
            │         LOAD CELL SYSTEM                │
            │                                         │
            │  ┌─────────────┐    ┌─────────────────┐ │
            │  │   HX711     │    │   LOAD CELL     │ │
            │  │ AMPLIFIER   │    │   (Force Sensor)│ │
            │  │             │    │                 │ │
            │  │ VCC ←── 5V  │    │  Red ─┐         │ │
            │  │ GND ←── GND │    │  Blk ─┼─── HX711│ │
            │  │ DT  ──┐     │    │  Wht ─┤         │ │
            │  │ SCK ──┼──┐  │    │  Grn ─┘         │ │
            │  └───────┼──┼──┘    └─────────────────┘ │
            │          │  │                           │
            │    Serial│Output                        │
            │      ────┴──┴─── To USB-Serial Adapter  │
            └─────────────────────────────────────────┘
```

## Visual Diagram Features

The generated diagram includes:
- **Color-coded components** for easy identification
- **Connection lines with labels** showing signal paths
- **Pin assignment table** for quick reference
- **Power distribution visualization**
- **Component legends and notes**

## Power Distribution

```
12V 2A PSU ────┬──→ Easy Driver VCC (Stepper Motor Power)
               └──→ Easy Driver Logic (if not using 5V logic)

5V 3A PSU ─────────→ Raspberry Pi (via micro-USB or USB-C)

5V 2A PSU ────┬────→ Servo Motor 1 VCC (Red wire)
              ├────→ Servo Motor 2 VCC (Red wire)  
              ├────→ Servo Motor 3 VCC (Red wire)
              └────→ HX711 Load Cell Amplifier VCC

Common GND ───┬────→ Raspberry Pi GND
              ├────→ Easy Driver GND
              ├────→ All Servo Motors GND (Brown/Black wires)
              ├────→ HX711 Amplifier GND
              └────→ All Power Supply GND
```

## Detailed Pin Assignments

### Easy Driver Connections
```
Easy Driver Pin | Connection
----------------|------------------
VCC            | 12V Power Supply (+)
GND            | Common Ground
ENABLE         | RPi GPIO 16
DIRECTION      | RPi GPIO 20  
STEP           | RPi GPIO 21
A+, A-         | Stepper Motor Coil A
B+, B-         | Stepper Motor Coil B
```

### Servo Motor Connections (each servo)
```
Servo Wire | Color        | Connection
-----------|-------------|------------------
Power      | Red         | 5V Power Supply
Ground     | Brown/Black | Common Ground
Signal     | Orange/Yellow| RPi GPIO (17/27/22)
```

### Load Cell System
```
HX711 Pin  | Connection
-----------|------------------
VCC        | 5V Power Supply
GND        | Common Ground
DT (Data)  | USB-Serial TX
SCK (Clock)| USB-Serial RX/Clock
E+, E-     | Load Cell Excitation
A+, A-     | Load Cell Signal
```

## Software Configuration

The GPIO pins are configurable in the software:
- **Main Tab**: Real-time control and monitoring
- **Settings Tab**: Pin configuration panel allows changing GPIO assignments
- **Default Pins**: As specified in the connection diagram above

## Safety Considerations

1. **Power Isolation**: Keep high-current motor power (12V) separate from logic power (5V/3.3V)
2. **Common Ground**: Ensure all components share a common ground reference
3. **Current Limits**: Easy Driver provides current limiting for stepper motor protection
4. **Fusing**: Consider adding fuses on power supply lines for protection
5. **Emergency Stop**: Software includes emergency stop functionality

## Testing Without Hardware

The software includes simulation modes that work without physical hardware connected:
- Stepper motor commands print to console/GUI log
- Servo commands are simulated and logged  
- Force readings return simulated values
- All GUI functionality remains operational

This allows for software development and testing without requiring the complete hardware setup.
