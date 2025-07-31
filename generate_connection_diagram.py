"""
Generate visual connection diagram for RPi Control System
Requires: pip install matplotlib
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyBboxPatch, ConnectionPatch
import numpy as np

def create_connection_diagram():
    # Create maximum size figure with much more space
    fig, ax = plt.subplots(1, 1, figsize=(36, 24))
    ax.set_xlim(0, 36)
    ax.set_ylim(0, 24)
    ax.set_aspect('equal')
    ax.axis('off')
    
    # Colors
    rpi_color = '#228B22'  # Forest Green
    motor_color = '#4169E1'  # Royal Blue
    power_color = '#DC143C'  # Crimson
    sensor_color = '#FF8C00'  # Dark Orange
    
    # Title with maximum space
    ax.text(18, 23, 'RPi Control System Connection Diagram', 
            fontsize=32, fontweight='bold', ha='center')
    
    # Component positions with maximum spacing to avoid any overlaps
    
    # Raspberry Pi - center position
    rpi_pos = (15, 14)
    rpi_size = (6, 4)
    rpi_box = FancyBboxPatch(rpi_pos, rpi_size[0], rpi_size[1], boxstyle="round,pad=0.2", 
                             facecolor=rpi_color, edgecolor='black', linewidth=3)
    ax.add_patch(rpi_box)
    ax.text(rpi_pos[0] + rpi_size[0]/2, rpi_pos[1] + rpi_size[1]/2, 
            'RASPBERRY PI 4', fontsize=18, fontweight='bold', 
            ha='center', va='center', color='white')
    
    # GPIO connection points clearly positioned on edges
    gpio_points = {
        'enable': (rpi_pos[0] + rpi_size[0], rpi_pos[1] + 3),   # Right edge
        'dir': (rpi_pos[0] + rpi_size[0], rpi_pos[1] + 2.5),
        'step': (rpi_pos[0] + rpi_size[0], rpi_pos[1] + 2),
        'servo1': (rpi_pos[0], rpi_pos[1] + 3),                 # Left edge
        'servo2': (rpi_pos[0], rpi_pos[1] + 2.5),
        'servo3': (rpi_pos[0], rpi_pos[1] + 2),
        'power_5v': (rpi_pos[0] + rpi_size[0]/2, rpi_pos[1]),   # Bottom edge
        'usb': (rpi_pos[0] + rpi_size[0]/2, rpi_pos[1] + rpi_size[1])  # Top edge
    }
    
    # Easy Driver - far right with maximum gap
    driver_pos = (26, 15)
    driver_size = (5, 3.5)
    driver_box = FancyBboxPatch(driver_pos, driver_size[0], driver_size[1], boxstyle="round,pad=0.15",
                                facecolor=motor_color, edgecolor='black', linewidth=3)
    ax.add_patch(driver_box)
    ax.text(driver_pos[0] + driver_size[0]/2, driver_pos[1] + driver_size[1]/2 + 0.4, 
            'EASY DRIVER', fontsize=16, fontweight='bold',
            ha='center', va='center', color='white')
    ax.text(driver_pos[0] + driver_size[0]/2, driver_pos[1] + driver_size[1]/2 - 0.4, 
            'Stepper Controller', fontsize=13,
            ha='center', va='center', color='white')
    
    driver_points = {
        'enable': (driver_pos[0], driver_pos[1] + 2.5),          # Left edge
        'dir': (driver_pos[0], driver_pos[1] + 2),
        'step': (driver_pos[0], driver_pos[1] + 1.5),
        'motor_out': (driver_pos[0] + driver_size[0], driver_pos[1] + driver_size[1]/2),  # Right edge
        'power_12v': (driver_pos[0] + driver_size[0]/2, driver_pos[1])  # Bottom edge
    }
    
    # Stepper Motor - far right, below driver
    stepper_pos = (26, 9)
    stepper_size = (5, 3.5)
    stepper_box = FancyBboxPatch(stepper_pos, stepper_size[0], stepper_size[1], boxstyle="round,pad=0.15",
                                 facecolor=motor_color, edgecolor='black', linewidth=3)
    ax.add_patch(stepper_box)
    ax.text(stepper_pos[0] + stepper_size[0]/2, stepper_pos[1] + stepper_size[1]/2 + 0.5, 
            'NEMA 17', fontsize=16, fontweight='bold',
            ha='center', va='center', color='white')
    ax.text(stepper_pos[0] + stepper_size[0]/2, stepper_pos[1] + stepper_size[1]/2, 
            'STEPPER MOTOR', fontsize=14,
            ha='center', va='center', color='white')
    ax.text(stepper_pos[0] + stepper_size[0]/2, stepper_pos[1] + stepper_size[1]/2 - 0.5, 
            '4-Wire Bipolar', fontsize=12,
            ha='center', va='center', color='white')
    
    stepper_points = {
        'motor_in': (stepper_pos[0], stepper_pos[1] + stepper_size[1]/2)  # Left edge
    }
    
    # Servo Motors - far left with maximum vertical spacing
    servo_positions = [(2, 18), (2, 13), (2, 8)]
    servo_size = (4, 2.5)
    servo_points = []
    
    for i, pos in enumerate(servo_positions):
        servo_box = FancyBboxPatch(pos, servo_size[0], servo_size[1], boxstyle="round,pad=0.1",
                                   facecolor=motor_color, edgecolor='black', linewidth=2)
        ax.add_patch(servo_box)
        ax.text(pos[0] + servo_size[0]/2, pos[1] + servo_size[1]/2, 
                f'SG90\nServo {i+1}', fontsize=13, fontweight='bold',
                ha='center', va='center', color='white')
        servo_points.append((pos[0] + servo_size[0], pos[1] + servo_size[1]/2))  # Right edge
    
    # Load Cell System - bottom center with maximum space
    loadcell_pos = (12, 3)
    loadcell_size = (8, 4)
    loadcell_box = FancyBboxPatch(loadcell_pos, loadcell_size[0], loadcell_size[1], boxstyle="round,pad=0.15",
                                  facecolor=sensor_color, edgecolor='black', linewidth=3)
    ax.add_patch(loadcell_box)
    ax.text(loadcell_pos[0] + loadcell_size[0]/2, loadcell_pos[1] + loadcell_size[1]/2 + 0.7, 
            'LOAD CELL SYSTEM', fontsize=16, fontweight='bold',
            ha='center', va='center', color='white')
    ax.text(loadcell_pos[0] + 2, loadcell_pos[1] + loadcell_size[1]/2 - 0.3, 
            'HX711\nAmplifier', fontsize=13, ha='center', va='center', color='white')
    ax.text(loadcell_pos[0] + 6, loadcell_pos[1] + loadcell_size[1]/2 - 0.3, 
            'Load Cell\n(Force Sensor)', fontsize=13, ha='center', va='center', color='white')
    
    loadcell_points = {
        'serial_out': (loadcell_pos[0] + loadcell_size[0], loadcell_pos[1] + loadcell_size[1]/2),  # Right edge
        'power_5v': (loadcell_pos[0] + loadcell_size[0]/2, loadcell_pos[1])  # Bottom edge
    }
    
    # USB-Serial Adapter - bottom right with maximum space
    usb_pos = (23, 3)
    usb_size = (4.5, 3)
    usb_box = FancyBboxPatch(usb_pos, usb_size[0], usb_size[1], boxstyle="round,pad=0.15",
                             facecolor=sensor_color, edgecolor='black', linewidth=3)
    ax.add_patch(usb_box)
    ax.text(usb_pos[0] + usb_size[0]/2, usb_pos[1] + usb_size[1]/2, 
            'USB-Serial\nAdapter', fontsize=14, fontweight='bold',
            ha='center', va='center', color='white')
    
    usb_points = {
        'serial_in': (usb_pos[0], usb_pos[1] + usb_size[1]/2),  # Left edge
        'usb_out': (usb_pos[0] + usb_size[0]/2, usb_pos[1] + usb_size[1])  # Top edge
    }
    
    # Power Supplies - bottom row with maximum spacing
    power_positions = [(4, 0.5), (16, 0.5), (28, 0.5)]
    power_labels = ['5V 3A\nRPi Power', '12V 2A\nStepper Power', '5V 2A\nServo/Sensor Power']
    power_size = (4, 2)
    power_points = {}
    
    for i, (pos, label) in enumerate(zip(power_positions, power_labels)):
        psu_box = FancyBboxPatch(pos, power_size[0], power_size[1], boxstyle="round,pad=0.1",
                                 facecolor=power_color, edgecolor='black', linewidth=2)
        ax.add_patch(psu_box)
        ax.text(pos[0] + power_size[0]/2, pos[1] + power_size[1]/2, 
                label, fontsize=12, fontweight='bold',
                ha='center', va='center', color='white')
        power_points[f'psu_{i}'] = (pos[0] + power_size[0]/2, pos[1] + power_size[1])  # Top edge
    
    # Simple clean connections with no overlaps
    def draw_simple_connection(start, end, label, color='black', style='-'):
        """Draw a simple L-shaped connection to avoid overlaps"""
        start_x, start_y = start
        end_x, end_y = end
        
        # Create L-shaped path with maximum clearance
        if abs(start_x - end_x) > abs(start_y - end_y):
            # Horizontal priority
            mid_point = (end_x, start_y)
            path = [start, mid_point, end]
        else:
            # Vertical priority
            mid_point = (start_x, end_y)
            path = [start, mid_point, end]
        
        # Draw the path
        for i in range(len(path) - 1):
            line = ConnectionPatch(path[i], path[i+1], "data", "data",
                                  arrowstyle="-" if i < len(path)-2 else "->", 
                                  shrinkA=5, shrinkB=5, mutation_scale=20, 
                                  color=color, linewidth=4, linestyle=style)
            ax.add_patch(line)
        
        # Place label with maximum clearance
        if len(path) >= 2:
            if abs(path[1][0] - path[0][0]) > abs(path[1][1] - path[0][1]):
                # Label on horizontal segment
                label_x = (path[0][0] + path[1][0]) / 2
                label_y = path[0][1] + 1
            else:
                # Label on vertical segment
                label_x = path[0][0] + 1.2
                label_y = (path[0][1] + path[1][1]) / 2
            
            ax.text(label_x, label_y, label, fontsize=11, ha='center', va='center',
                    bbox=dict(boxstyle="round,pad=0.5", facecolor='white', alpha=0.95, 
                             edgecolor=color, linewidth=2))
    
    # Draw all connections with maximum clearance
    
    # RPi to Easy Driver connections
    draw_simple_connection(gpio_points['enable'], driver_points['enable'], 
                          'ENABLE\n(GPIO 16)', '#FF4500')
    draw_simple_connection(gpio_points['dir'], driver_points['dir'], 
                          'DIRECTION\n(GPIO 20)', '#FF6500')
    draw_simple_connection(gpio_points['step'], driver_points['step'], 
                          'STEP\n(GPIO 21)', '#FF8500')
    
    # Easy Driver to Stepper Motor with maximum clearance
    driver_out = driver_points['motor_out']
    stepper_in = stepper_points['motor_in']
    intermediate_point = (driver_out[0] + 2, (driver_out[1] + stepper_in[1]) / 2)
    motor_path = [driver_out, intermediate_point, stepper_in]
    
    for i in range(len(motor_path) - 1):
        line = ConnectionPatch(motor_path[i], motor_path[i+1], "data", "data",
                              arrowstyle="-" if i < len(motor_path)-2 else "->", 
                              shrinkA=5, shrinkB=5, mutation_scale=20, 
                              color='#4169E1', linewidth=4)
        ax.add_patch(line)
    
    ax.text(intermediate_point[0] + 1, intermediate_point[1], 'Motor Wires\n(A+,A-,B+,B-)', 
           fontsize=11, ha='center', va='center',
           bbox=dict(boxstyle="round,pad=0.5", facecolor='white', alpha=0.95, 
                    edgecolor='#4169E1', linewidth=2))
    
    # RPi to Servos with maximum clearance
    servo_labels = ['PWM Signal\n(GPIO 17)', 'PWM Signal\n(GPIO 27)', 'PWM Signal\n(GPIO 22)']
    servo_gpio = [gpio_points['servo1'], gpio_points['servo2'], gpio_points['servo3']]
    
    for servo_point, gpio_point, label in zip(servo_points, servo_gpio, servo_labels):
        draw_simple_connection(gpio_point, servo_point, label, '#32CD32')
    
    # USB connections with maximum clearance routing
    usb_start = gpio_points['usb']
    usb_end = usb_points['usb_out']
    usb_intermediate = (usb_end[0], usb_start[1] + 4)  # Route well above RPi
    usb_path = [usb_start, usb_intermediate, usb_end]
    
    for i in range(len(usb_path) - 1):
        line = ConnectionPatch(usb_path[i], usb_path[i+1], "data", "data",
                              arrowstyle="-" if i < len(usb_path)-2 else "->", 
                              shrinkA=5, shrinkB=5, mutation_scale=20, 
                              color='#9370DB', linewidth=4)
        ax.add_patch(line)
    
    ax.text(usb_intermediate[0] - 2, usb_intermediate[1] + 0.5, 'USB Connection', 
           fontsize=11, ha='center', va='center',
           bbox=dict(boxstyle="round,pad=0.5", facecolor='white', alpha=0.95, 
                    edgecolor='#9370DB', linewidth=2))
    
    # USB-Serial to Load Cell
    draw_simple_connection(usb_points['serial_in'], loadcell_points['serial_out'], 
                          'Serial Data\n(TX/RX)', '#9370DB')
    
    # Power connections with maximum clearance
    power_colors = ['#DC143C', '#FF1493', '#FF69B4']
    
    # 5V to RPi
    draw_simple_connection(power_points['psu_0'], gpio_points['power_5v'], 
                          '5V Power\n(RPi)', power_colors[0])
    
    # 12V to Easy Driver
    draw_simple_connection(power_points['psu_1'], driver_points['power_12v'], 
                          '12V Power\n(Stepper)', power_colors[1])
    
    # 5V to Load Cell
    draw_simple_connection(power_points['psu_2'], loadcell_points['power_5v'], 
                          '5V Power\n(Sensors)', power_colors[2])
    
    # 5V to Servos with maximum clearance routing
    for i, servo_point in enumerate(servo_points):
        servo_power_point = (servo_point[0] - 0.5, servo_point[1])
        psu_point = power_points['psu_2']
        # Route with maximum clearance
        intermediate1 = (psu_point[0], 10)  # Much higher to avoid all components
        intermediate2 = (servo_power_point[0], 10)  # Across at safe height
        path = [psu_point, intermediate1, intermediate2, servo_power_point]
        
        for j in range(len(path) - 1):
            line = ConnectionPatch(path[j], path[j+1], "data", "data",
                                  arrowstyle="-" if j < len(path)-2 else "->", 
                                  shrinkA=3, shrinkB=3, mutation_scale=15, 
                                  color=power_colors[2], linewidth=3, linestyle='--')
            ax.add_patch(line)
        
        if i == 1:  # Label only on middle servo connection
            ax.text(intermediate2[0] - 2, 10.5, '5V Power\n(Servos)', 
                   fontsize=11, ha='center', va='center',
                   bbox=dict(boxstyle="round,pad=0.4", facecolor='white', alpha=0.95, 
                            edgecolor=power_colors[2], linewidth=1.5))

    # Simple clean connections function
    def draw_simple_connection(start, end, label, color='black', style='-'):
        """Draw a simple L-shaped connection to avoid overlaps"""
        start_x, start_y = start
        end_x, end_y = end
        
        # Create L-shaped path with maximum clearance
        if abs(start_x - end_x) > abs(start_y - end_y):
            # Horizontal priority
            mid_point = (end_x, start_y)
            path = [start, mid_point, end]
        else:
            # Vertical priority
            mid_point = (start_x, end_y)
            path = [start, mid_point, end]
        
        # Draw the path
        for i in range(len(path) - 1):
            line = ConnectionPatch(path[i], path[i+1], "data", "data",
                                  arrowstyle="-" if i < len(path)-2 else "->", 
                                  shrinkA=5, shrinkB=5, mutation_scale=20, 
                                  color=color, linewidth=4, linestyle=style)
            ax.add_patch(line)
        
        # Place label with maximum clearance
        if len(path) >= 2:
            if abs(path[1][0] - path[0][0]) > abs(path[1][1] - path[0][1]):
                # Label on horizontal segment
                label_x = (path[0][0] + path[1][0]) / 2
                label_y = path[0][1] + 1
            else:
                # Label on vertical segment
                label_x = path[0][0] + 1.2
                label_y = (path[0][1] + path[1][1]) / 2
            
            ax.text(label_x, label_y, label, fontsize=11, ha='center', va='center',
                    bbox=dict(boxstyle="round,pad=0.5", facecolor='white', alpha=0.95, 
                             edgecolor=color, linewidth=2))
    
    # Legend - pinned to top left corner
    legend_x, legend_y = 1, 21.5
    legend_elements = [
        ('Raspberry Pi', rpi_color),
        ('Motors & Drivers', motor_color),
        ('Sensors & Interface', sensor_color),
        ('Power Supplies', power_color)
    ]
    
    # Legend background box
    legend_bg = FancyBboxPatch((legend_x - 0.2, legend_y - 2.5), 6, 3, 
                               boxstyle="round,pad=0.2", facecolor='lightgray', 
                               edgecolor='black', linewidth=2, alpha=0.9)
    ax.add_patch(legend_bg)
    
    ax.text(legend_x, legend_y, 'Legend:', fontsize=16, fontweight='bold', ha='left')
    for i, (label, color) in enumerate(legend_elements):
        y_pos = legend_y - 0.7 - i*0.6
        # Draw colored square
        square = patches.Rectangle((legend_x, y_pos - 0.2), 0.5, 0.5,
                                 facecolor=color, edgecolor='black', linewidth=1)
        ax.add_patch(square)
        # Add label
        ax.text(legend_x + 0.7, y_pos, label, fontsize=13, ha='left', va='center')
    
    # Pin assignment table - pinned to top right corner
    table_data = [
        ['Component', 'Pin/Wire', 'RPi Connection'],
        ['Easy Driver', 'ENABLE', 'GPIO 16'],
        ['Easy Driver', 'DIR', 'GPIO 20'],
        ['Easy Driver', 'STEP', 'GPIO 21'],
        ['Servo 1', 'Signal (Orange)', 'GPIO 17'],
        ['Servo 2', 'Signal (Orange)', 'GPIO 27'],
        ['Servo 3', 'Signal (Orange)', 'GPIO 22'],
        ['All Servos', 'Power (Red)', '5V'],
        ['All Components', 'Ground (Black)', 'Common GND'],
        ['Load Cell', 'Data (Serial)', 'USB Port'],
    ]
    
    # Position table in top right corner
    table_x, table_y = 23, 21.5
    
    # Table background
    table_bg = FancyBboxPatch((table_x - 0.2, table_y - 4.5), 12.5, 5, 
                              boxstyle="round,pad=0.2", facecolor='lightgray', 
                              edgecolor='black', linewidth=2, alpha=0.9)
    ax.add_patch(table_bg)
    
    for i, row in enumerate(table_data):
        for j, cell in enumerate(row):
            cell_color = 'lightblue' if i == 0 else 'white'
            cell_box = patches.Rectangle((table_x + j*3, table_y - i*0.5), 2.9, 0.45,
                                       facecolor=cell_color, edgecolor='black', linewidth=1)
            ax.add_patch(cell_box)
            font_weight = 'bold' if i == 0 else 'normal'
            font_size = 11 if i == 0 else 10
            ax.text(table_x + j*3 + 1.45, table_y - i*0.5 + 0.225, cell,
                   fontsize=font_size, ha='center', va='center', fontweight=font_weight)
    
    ax.text(table_x + 4.35, table_y + 0.8, 'Pin Assignment Reference', 
            fontsize=16, fontweight='bold', ha='center')
    
    # System notes - pinned to bottom left corner  
    notes = [
        "• All components share common ground reference",
        "• GPIO pins are software-configurable via Settings tab", 
        "• Easy Driver provides automatic current limiting protection",
        "• System operates in simulation mode without physical hardware",
        "• Use proper gauge wire for power connections (18-22 AWG)",
        "• Ensure stable 5V/12V power supplies with adequate current capacity"
    ]
    
    notes_x, notes_y = 1, 7
    
    # Notes background
    notes_bg = FancyBboxPatch((notes_x - 0.2, notes_y - 3.5), 16, 4, 
                              boxstyle="round,pad=0.2", facecolor='lightgray', 
                              edgecolor='black', linewidth=2, alpha=0.9)
    ax.add_patch(notes_bg)
    
    ax.text(notes_x, notes_y, 'System Notes:', fontsize=16, fontweight='bold', ha='left')
    for i, note in enumerate(notes):
        ax.text(notes_x, notes_y - 0.5 - i*0.5, note, fontsize=12, ha='left', va='center')
    
    plt.tight_layout()
    return fig

def create_legend_image():
    """Create a separate legend image"""
    fig, ax = plt.subplots(1, 1, figsize=(8, 6))
    ax.set_xlim(0, 8)
    ax.set_ylim(0, 6)
    ax.set_aspect('equal')
    ax.axis('off')
    
    # Colors
    rpi_color = '#228B22'
    motor_color = '#4169E1'
    sensor_color = '#FF8C00'
    power_color = '#DC143C'
    
    legend_elements = [
        ('Raspberry Pi', rpi_color),
        ('Motors & Drivers', motor_color),
        ('Sensors & Interface', sensor_color),
        ('Power Supplies', power_color)
    ]
    
    # Legend title
    ax.text(4, 5.5, 'Component Legend', fontsize=20, fontweight='bold', ha='center')
    
    # Legend items
    for i, (label, color) in enumerate(legend_elements):
        y_pos = 4.5 - i*0.8
        # Draw colored square
        square = patches.Rectangle((1, y_pos - 0.3), 0.6, 0.6,
                                 facecolor=color, edgecolor='black', linewidth=2)
        ax.add_patch(square)
        # Add label
        ax.text(2, y_pos, label, fontsize=16, ha='left', va='center', fontweight='bold')
    
    plt.tight_layout()
    return fig

def create_pin_table_image():
    """Create a separate pin assignment table image"""
    fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 8)
    ax.set_aspect('equal')
    ax.axis('off')
    
    table_data = [
        ['Component', 'Pin/Wire', 'RPi Connection'],
        ['Easy Driver', 'ENABLE', 'GPIO 16'],
        ['Easy Driver', 'DIR', 'GPIO 20'],
        ['Easy Driver', 'STEP', 'GPIO 21'],
        ['Servo 1', 'Signal (Orange)', 'GPIO 17'],
        ['Servo 2', 'Signal (Orange)', 'GPIO 27'],
        ['Servo 3', 'Signal (Orange)', 'GPIO 22'],
        ['All Servos', 'Power (Red)', '5V'],
        ['All Components', 'Ground (Black)', 'Common GND'],
        ['Load Cell', 'Data (Serial)', 'USB Port'],
    ]
    
    # Table title
    ax.text(6, 7.5, 'Pin Assignment Reference', fontsize=20, fontweight='bold', ha='center')
    
    # Table
    table_x, table_y = 1, 6.5
    for i, row in enumerate(table_data):
        for j, cell in enumerate(row):
            cell_color = 'lightblue' if i == 0 else 'white'
            cell_box = patches.Rectangle((table_x + j*3.3, table_y - i*0.6), 3.2, 0.55,
                                       facecolor=cell_color, edgecolor='black', linewidth=1.5)
            ax.add_patch(cell_box)
            font_weight = 'bold' if i == 0 else 'normal'
            font_size = 14 if i == 0 else 12
            ax.text(table_x + j*3.3 + 1.6, table_y - i*0.6 + 0.275, cell,
                   fontsize=font_size, ha='center', va='center', fontweight=font_weight)
    
    plt.tight_layout()
    return fig

def create_notes_image():
    """Create a separate system notes image"""
    fig, ax = plt.subplots(1, 1, figsize=(14, 8))
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 8)
    ax.set_aspect('equal')
    ax.axis('off')
    
    # Notes title
    ax.text(7, 7.5, 'System Notes & Important Information', fontsize=20, fontweight='bold', ha='center')
    
    notes = [
        "• All components share common ground reference",
        "• GPIO pins are software-configurable via Settings tab", 
        "• Easy Driver provides automatic current limiting protection",
        "• System operates in simulation mode without physical hardware",
        "• Use proper gauge wire for power connections (18-22 AWG)",
        "• Ensure stable 5V/12V power supplies with adequate current capacity"
    ]
    
    # Add background box
    notes_bg = FancyBboxPatch((0.5, 1), 13, 5.5, 
                              boxstyle="round,pad=0.3", facecolor='lightyellow', 
                              edgecolor='black', linewidth=2, alpha=0.9)
    ax.add_patch(notes_bg)
    
    # Notes content
    for i, note in enumerate(notes):
        ax.text(1, 6 - i*0.7, note, fontsize=14, ha='left', va='center', fontweight='normal')
    
    plt.tight_layout()
    return fig

def create_main_diagram():
    """Create the main connection diagram without legend, table, or notes"""
    fig, ax = plt.subplots(1, 1, figsize=(60, 45))  # Massive canvas size
    ax.set_xlim(0, 60)
    ax.set_ylim(0, 45)
    ax.set_aspect('equal')
    ax.axis('off')
    
    # Colors
    rpi_color = '#228B22'
    motor_color = '#4169E1'
    power_color = '#DC143C'
    sensor_color = '#FF8C00'
    
    # Title with maximum space
    ax.text(30, 42, 'RPi Control System Connection Diagram', 
            fontsize=18, fontweight='bold', ha='center')
    
    # Component positioning with EXTREME spacing - blocks very far apart
    
    # Power Supplies - top row with massive horizontal spacing
    power_positions = [(5, 37), (27, 37), (49, 37)]
    power_labels = ['5V 3A\nRPi Power', '12V 2A\nStepper Power', '5V 2A\nServo/Sensor Power']
    power_size = (8, 4)
    power_points = {}
    
    for i, (pos, label) in enumerate(zip(power_positions, power_labels)):
        psu_box = FancyBboxPatch(pos, power_size[0], power_size[1], boxstyle="round,pad=0.3",
                                 facecolor=power_color, edgecolor='black', linewidth=3)
        ax.add_patch(psu_box)
        ax.text(pos[0] + power_size[0]/2, pos[1] + power_size[1]/2, 
                label, fontsize=18, fontweight='bold',
                ha='center', va='center', color='white')
        power_points[f'psu_{i}'] = (pos[0] + power_size[0]/2, pos[1])  # Bottom edge
    
    # Ground reference - bottom center with massive vertical spacing
    ground_pos = (26, 3)
    ground_size = (8, 4)
    ground_box = FancyBboxPatch(ground_pos, ground_size[0], ground_size[1], boxstyle="round,pad=0.3",
                                facecolor='black', edgecolor='black', linewidth=3)
    ax.add_patch(ground_box)
    ax.text(ground_pos[0] + ground_size[0]/2, ground_pos[1] + ground_size[1]/2, 
            'COMMON GROUND', fontsize=18, fontweight='bold',
            ha='center', va='center', color='white')
    
    ground_point = (ground_pos[0] + ground_size[0]/2, ground_pos[1] + ground_size[1])
    
    # Sensors - extreme far left with massive spacing
    loadcell_pos = (2, 28)
    loadcell_size = (10, 5)
    loadcell_box = FancyBboxPatch(loadcell_pos, loadcell_size[0], loadcell_size[1], boxstyle="round,pad=0.3",
                                  facecolor=sensor_color, edgecolor='black', linewidth=3)
    ax.add_patch(loadcell_box)
    ax.text(loadcell_pos[0] + loadcell_size[0]/2, loadcell_pos[1] + loadcell_size[1]/2 + 1, 
            'LOAD CELL SYSTEM', fontsize=18, fontweight='bold',
            ha='center', va='center', color='white')
    ax.text(loadcell_pos[0] + 2.5, loadcell_pos[1] + loadcell_size[1]/2 - 0.5, 
            'HX711\nAmplifier', fontsize=18, ha='center', va='center', color='white')
    ax.text(loadcell_pos[0] + 7.5, loadcell_pos[1] + loadcell_size[1]/2 - 0.5, 
            'MK10 Tension\nLoad Cell', fontsize=18, ha='center', va='center', color='white')
    
    loadcell_points = {
        'serial_out': (loadcell_pos[0] + loadcell_size[0], loadcell_pos[1] + loadcell_size[1]/2),
        'power_5v': (loadcell_pos[0] + loadcell_size[0]/2, loadcell_pos[1] + loadcell_size[1]),
        'ground': (loadcell_pos[0] + loadcell_size[0]/2, loadcell_pos[1])
    }
    
    # USB-Serial Adapter - below load cell with massive vertical gap
    usb_pos = (3, 18)
    usb_size = (8, 5)
    usb_box = FancyBboxPatch(usb_pos, usb_size[0], usb_size[1], boxstyle="round,pad=0.3",
                             facecolor=sensor_color, edgecolor='black', linewidth=3)
    ax.add_patch(usb_box)
    ax.text(usb_pos[0] + usb_size[0]/2, usb_pos[1] + usb_size[1]/2, 
            'USB-Serial\nAdapter', fontsize=18, fontweight='bold',
            ha='center', va='center', color='white')
    
    usb_points = {
        'serial_in': (usb_pos[0] + usb_size[0]/2, usb_pos[1] + usb_size[1]),
        'usb_out': (usb_pos[0] + usb_size[0], usb_pos[1] + usb_size[1]/2),
        'ground': (usb_pos[0] + usb_size[0]/2, usb_pos[1])
    }
    
    # Raspberry Pi - center with massive spacing from everything
    rpi_pos = (22, 25)
    rpi_size = (10, 6)
    rpi_box = FancyBboxPatch(rpi_pos, rpi_size[0], rpi_size[1], boxstyle="round,pad=0.4", 
                             facecolor=rpi_color, edgecolor='black', linewidth=4)
    ax.add_patch(rpi_box)
    ax.text(rpi_pos[0] + rpi_size[0]/2, rpi_pos[1] + rpi_size[1]/2, 
            'RASPBERRY PI 4', fontsize=18, fontweight='bold', 
            ha='center', va='center', color='white')
    
    gpio_points = {
        'enable': (rpi_pos[0] + rpi_size[0], rpi_pos[1] + 5),   # Right edge
        'dir': (rpi_pos[0] + rpi_size[0], rpi_pos[1] + 4),
        'step': (rpi_pos[0] + rpi_size[0], rpi_pos[1] + 3),
        'servo1': (rpi_pos[0], rpi_pos[1] + 5.2),              # Left edge
        'servo2': (rpi_pos[0], rpi_pos[1] + 4.2),
        'servo3': (rpi_pos[0], rpi_pos[1] + 3.2),
        'power_5v': (rpi_pos[0] + rpi_size[0]/2, rpi_pos[1] + rpi_size[1]),  # Top edge
        'usb': (rpi_pos[0], rpi_pos[1] + 2.2),                 # Left edge
        'ground': (rpi_pos[0] + rpi_size[0]/2, rpi_pos[1])     # Bottom edge
    }
    
    # Easy Driver - center-right with massive gap from RPi
    driver_pos = (38, 26)
    driver_size = (9, 5)
    driver_box = FancyBboxPatch(driver_pos, driver_size[0], driver_size[1], boxstyle="round,pad=0.3",
                                facecolor=motor_color, edgecolor='black', linewidth=4)
    ax.add_patch(driver_box)
    ax.text(driver_pos[0] + driver_size[0]/2, driver_pos[1] + driver_size[1]/2 + 0.7, 
            'EASY DRIVER', fontsize=18, fontweight='bold',
            ha='center', va='center', color='white')
    ax.text(driver_pos[0] + driver_size[0]/2, driver_pos[1] + driver_size[1]/2 - 0.7, 
            'Stepper Controller', fontsize=18,
            ha='center', va='center', color='white')
    
    driver_points = {
        'enable': (driver_pos[0], driver_pos[1] + 3.8),         # Left edge
        'dir': (driver_pos[0], driver_pos[1] + 2.8),
        'step': (driver_pos[0], driver_pos[1] + 1.8),
        'motor_out': (driver_pos[0] + driver_size[0], driver_pos[1] + driver_size[1]/2),  # Right edge
        'power_12v': (driver_pos[0] + driver_size[0]/2, driver_pos[1] + driver_size[1]),  # Top edge
        'ground': (driver_pos[0] + driver_size[0]/2, driver_pos[1])  # Bottom edge
    }
    
    # Stepper Motor - extreme far right with massive gap
    stepper_pos = (50, 24)
    stepper_size = (9, 7)
    stepper_box = FancyBboxPatch(stepper_pos, stepper_size[0], stepper_size[1], boxstyle="round,pad=0.3",
                                 facecolor=motor_color, edgecolor='black', linewidth=4)
    ax.add_patch(stepper_box)
    ax.text(stepper_pos[0] + stepper_size[0]/2, stepper_pos[1] + stepper_size[1]/2 + 1.2, 
            'E21H4N-2.5-900', fontsize=18, fontweight='bold',
            ha='center', va='center', color='white')
    ax.text(stepper_pos[0] + stepper_size[0]/2, stepper_pos[1] + stepper_size[1]/2, 
            'STEPPER MOTOR', fontsize=18,
            ha='center', va='center', color='white')
    ax.text(stepper_pos[0] + stepper_size[0]/2, stepper_pos[1] + stepper_size[1]/2 - 1.2, 
            '4-Wire Bipolar', fontsize=18,
            ha='center', va='center', color='white')
    
    stepper_points = {
        'motor_in': (stepper_pos[0], stepper_pos[1] + stepper_size[1]/2),  # Left edge
        'ground': (stepper_pos[0] + stepper_size[0]/2, stepper_pos[1])     # Bottom edge
    }
    
    # Servo Motors - extreme right side, massive vertical spacing
    servo_positions = [(50, 36), (50, 15), (50, 8)]
    servo_size = (8, 4)
    servo_points = []
    
    for i, pos in enumerate(servo_positions):
        servo_box = FancyBboxPatch(pos, servo_size[0], servo_size[1], boxstyle="round,pad=0.2",
                                   facecolor=motor_color, edgecolor='black', linewidth=3)
        ax.add_patch(servo_box)
        ax.text(pos[0] + servo_size[0]/2, pos[1] + servo_size[1]/2, 
                f'SG90\nServo {i+1}', fontsize=18, fontweight='bold',
                ha='center', va='center', color='white')
        servo_points.append({
            'signal': (pos[0], pos[1] + servo_size[1]/2),      # Left edge
            'power': (pos[0] + servo_size[0]/2, pos[1] + servo_size[1]),  # Top edge
            'ground': (pos[0] + servo_size[0]/2, pos[1])       # Bottom edge
        })
    
    # Clean connection drawing function with massive clearance
    def draw_clean_connection(start, end, label, color='black', style='-', label_offset=(0, 2)):
        start_x, start_y = start
        end_x, end_y = end
        
        # Create clean routing path with massive clearance
        if abs(start_x - end_x) > abs(start_y - end_y):
            # Horizontal routing
            mid_point = (end_x, start_y)
            path = [start, mid_point, end]
        else:
            # Vertical routing  
            mid_point = (start_x, end_y)
            path = [start, mid_point, end]
        
        # Draw the path with consistent line width
        for i in range(len(path) - 1):
            line = ConnectionPatch(path[i], path[i+1], "data", "data",
                                  arrowstyle="-" if i < len(path)-2 else "->", 
                                  shrinkA=5, shrinkB=5, mutation_scale=30, 
                                  color=color, linewidth=5, linestyle=style)
            ax.add_patch(line)
        
        # Position label with massive clearance
        if len(path) >= 2:
            # Find the best segment for label placement
            if abs(path[1][0] - path[0][0]) > abs(path[1][1] - path[0][1]):
                # Label on horizontal segment
                label_x = (path[0][0] + path[1][0]) / 2
                label_y = path[0][1] + label_offset[1]
            else:
                # Label on vertical segment
                label_x = path[0][0] + label_offset[0]
                label_y = (path[0][1] + path[1][1]) / 2
            
            ax.text(label_x, label_y, label, fontsize=6, ha='center', va='center',
                    bbox=dict(boxstyle="round,pad=0.8", facecolor='white', alpha=0.95, 
                             edgecolor=color, linewidth=3))
    
    # RPi to Easy Driver connections with massive spacing
    draw_clean_connection(gpio_points['enable'], driver_points['enable'], 
                          'ENABLE\n(GPIO 16)', '#FF4500', label_offset=(0, 2.5))
    draw_clean_connection(gpio_points['dir'], driver_points['dir'], 
                          'DIRECTION\n(GPIO 20)', '#FF6500', label_offset=(0, -2.5))
    draw_clean_connection(gpio_points['step'], driver_points['step'], 
                          'STEP\n(GPIO 21)', '#FF8500', label_offset=(0, 2.5))
    
    # Easy Driver to Stepper Motor with massive clearance
    draw_clean_connection(driver_points['motor_out'], stepper_points['motor_in'], 
                          'Motor Wires\n(A+,A-,B+,B-)', '#4169E1', label_offset=(0, 2.5))
    
    # RPi to Servos - routed with massive clearance
    servo_labels = ['PWM Signal\n(GPIO 17)', 'PWM Signal\n(GPIO 27)', 'PWM Signal\n(GPIO 22)']
    servo_gpio = [gpio_points['servo1'], gpio_points['servo2'], gpio_points['servo3']]
    
    for i, (servo_dict, gpio_point, label) in enumerate(zip(servo_points, servo_gpio, servo_labels)):
        # Route with massive clearance around other components
        intermediate1 = (15, gpio_point[1])  # Go far left first
        intermediate2 = (15, servo_dict['signal'][1])  # Then vertically with massive clearance
        path = [gpio_point, intermediate1, intermediate2, servo_dict['signal']]
        
        for j in range(len(path) - 1):
            line = ConnectionPatch(path[j], path[j+1], "data", "data",
                                  arrowstyle="-" if j < len(path)-2 else "->", 
                                  shrinkA=5, shrinkB=5, mutation_scale=30, 
                                  color='#32CD32', linewidth=5)
            ax.add_patch(line)
        
        # Place label on vertical segment with massive clearance
        ax.text(17, (gpio_point[1] + servo_dict['signal'][1]) / 2, label, 
               fontsize=6, ha='left', va='center', rotation=90,
               bbox=dict(boxstyle="round,pad=0.8", facecolor='white', alpha=0.95, 
                        edgecolor='#32CD32', linewidth=3))
    
    # USB connections with massive clearance
    draw_clean_connection(gpio_points['usb'], usb_points['usb_out'], 
                          'USB Connection', '#9370DB', label_offset=(2.5, 0))
    draw_clean_connection(usb_points['serial_in'], loadcell_points['serial_out'], 
                          'Serial Data\n(TX/RX)', '#9370DB', label_offset=(0, 2.5))
    
    # Power connections from top with massive clearance
    power_colors = ['#DC143C', '#FF1493', '#FF69B4']
    
    # 5V to RPi with massive vertical clearance
    draw_clean_connection(power_points['psu_0'], gpio_points['power_5v'], 
                          '5V Power\n(RPi)', power_colors[0], label_offset=(-3, 0))
    
    # 12V to Easy Driver with massive vertical clearance
    draw_clean_connection(power_points['psu_1'], driver_points['power_12v'], 
                          '12V Power\n(Stepper)', power_colors[1], label_offset=(-3, 0))
    
    # 5V to Load Cell with massive clearance
    draw_clean_connection(power_points['psu_2'], loadcell_points['power_5v'], 
                          '5V Power\n(Sensors)', power_colors[2], label_offset=(4, 0))
    
    # 5V to Servos with massive clearance routing
    for i, servo_dict in enumerate(servo_points):
        psu_point = power_points['psu_2']
        servo_power = servo_dict['power']
        
        # Route with massive clearance to avoid all crossings
        intermediate1 = (psu_point[0], 34)  # Go very high from PSU
        intermediate2 = (servo_power[0], 34)  # Across at maximum height
        path = [psu_point, intermediate1, intermediate2, servo_power]
        
        for j in range(len(path) - 1):
            line = ConnectionPatch(path[j], path[j+1], "data", "data",
                                  arrowstyle="-" if j < len(path)-2 else "->", 
                                  shrinkA=5, shrinkB=5, mutation_scale=25, 
                                  color=power_colors[2], linewidth=4, linestyle='--')
            ax.add_patch(line)
        
        if i == 1:  # Label only on middle servo with massive clearance
            ax.text(servo_power[0] - 3, 35, '5V Power\n(Servos)', 
                   fontsize=6, ha='center', va='center',
                   bbox=dict(boxstyle="round,pad=0.8", facecolor='white', alpha=0.95, 
                            edgecolor=power_colors[2], linewidth=3))
    
    # Ground connections with massive clearance and spacing
    ground_connections = [
        (gpio_points['ground'], 'RPi Ground'),
        (driver_points['ground'], 'Driver Ground'),
        (stepper_points['ground'], 'Stepper Ground'),
        (loadcell_points['ground'], 'Sensor Ground'),
        (usb_points['ground'], 'USB Ground')
    ]
    
    for point, label in ground_connections:
        draw_clean_connection(point, ground_point, label, 'black', label_offset=(0, -2))
    
    # Servo ground connections with massive clearance
    for i, servo_dict in enumerate(servo_points):
        draw_clean_connection(servo_dict['ground'], ground_point, 
                             f'Servo {i+1} Ground', 'black', label_offset=(0, -2))
    
    plt.tight_layout()
    return fig

def main():
    """Generate and save all diagram components"""
    print("Generating connection diagrams...")
    
    # Create main diagram
    main_fig = create_main_diagram()
    main_output = 'rpi_control_main_diagram.png'
    main_fig.savefig(main_output, dpi=300, bbox_inches='tight', 
                     facecolor='white', edgecolor='none')
    print(f"Main diagram saved as: {main_output}")
    
    # Save main diagram as PDF
    main_pdf = 'rpi_control_main_diagram.pdf'
    main_fig.savefig(main_pdf, dpi=300, bbox_inches='tight',
                     facecolor='white', edgecolor='none')
    print(f"Main diagram PDF saved as: {main_pdf}")
    
    # Create legend
    legend_fig = create_legend_image()
    legend_output = 'rpi_control_legend.png'
    legend_fig.savefig(legend_output, dpi=300, bbox_inches='tight',
                       facecolor='white', edgecolor='none')
    print(f"Legend saved as: {legend_output}")
    
    # Save legend as PDF
    legend_pdf = 'rpi_control_legend.pdf'
    legend_fig.savefig(legend_pdf, dpi=300, bbox_inches='tight',
                       facecolor='white', edgecolor='none')
    print(f"Legend PDF saved as: {legend_pdf}")
    
    # Create pin table
    table_fig = create_pin_table_image()
    table_output = 'rpi_control_pin_table.png'
    table_fig.savefig(table_output, dpi=300, bbox_inches='tight',
                      facecolor='white', edgecolor='none')
    print(f"Pin table saved as: {table_output}")
    
    # Save pin table as PDF
    table_pdf = 'rpi_control_pin_table.pdf'
    table_fig.savefig(table_pdf, dpi=300, bbox_inches='tight',
                      facecolor='white', edgecolor='none')
    print(f"Pin table PDF saved as: {table_pdf}")
    
    # Create notes
    notes_fig = create_notes_image()
    notes_output = 'rpi_control_notes.png'
    notes_fig.savefig(notes_output, dpi=300, bbox_inches='tight',
                      facecolor='white', edgecolor='none')
    print(f"Notes saved as: {notes_output}")
    
    # Save notes as PDF
    notes_pdf = 'rpi_control_notes.pdf'
    notes_fig.savefig(notes_pdf, dpi=300, bbox_inches='tight',
                      facecolor='white', edgecolor='none')
    print(f"Notes PDF saved as: {notes_pdf}")
    
    # Combine all diagrams into a single multi-page PDF
    from matplotlib.backends.backend_pdf import PdfPages
    
    combined_pdf = 'rpi_control_complete_documentation.pdf'
    with PdfPages(combined_pdf) as pdf:
        # Page 1: Main diagram
        pdf.savefig(main_fig, bbox_inches='tight', facecolor='white', edgecolor='none')
        
        # Page 2: Legend
        pdf.savefig(legend_fig, bbox_inches='tight', facecolor='white', edgecolor='none')
        
        # Page 3: Pin table
        pdf.savefig(table_fig, bbox_inches='tight', facecolor='white', edgecolor='none')
        
        # Page 4: Notes
        pdf.savefig(notes_fig, bbox_inches='tight', facecolor='white', edgecolor='none')
        
        # Add PDF metadata
        d = pdf.infodict()
        d['Title'] = 'RPi Control System Connection Documentation'
        d['Author'] = 'RPi Control System'
        d['Subject'] = 'Hardware Connection Diagrams and Reference'
        d['Keywords'] = 'Raspberry Pi, Stepper Motor, Servo, Load Cell, Hardware Connections'
        d['Creator'] = 'Python matplotlib'
    
    print(f"\nCombined PDF documentation saved as: {combined_pdf}")
    print("This PDF contains all diagrams in order:")
    print("  Page 1: Main Connection Diagram")
    print("  Page 2: Component Legend")
    print("  Page 3: Pin Assignment Table")
    print("  Page 4: System Notes")
    
    print("\nAll diagrams generated successfully!")
    print("PNG files: High-resolution raster images")
    print("PDF files: Vector graphics for scalable printing")
    print("Combined PDF: Complete documentation package")
    
    # Show all plots
    plt.show()

if __name__ == "__main__":
    main()
