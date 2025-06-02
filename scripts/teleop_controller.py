"""
Script for controlling the servos using an xbox controller.
"""

import rclpy
from geometry_msgs.msg import Vector3
import time
import socket
import threading
from evdev import InputDevice, categorize, ecodes, list_devices

class State:
    def __init__(self):
        self.axis_x = 0.0
        self.axis_y = 0.0
        self.pump_speed = 0.0
        self.quit_flag = False
        
        # Servo limits
        self.pan_min = 60.0
        self.pan_max = 140.0
        self.tilt_min = 30.0
        self.tilt_max = 100.0
        
        # Movement restrictions
        self.allow_pan_left = True
        self.allow_pan_right = True
        self.allow_tilt_up = True
        self.allow_tilt_down = True

def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
    finally:
        s.close()
    return local_ip

def udp_discovery_server(event):
    UDP_IP = "0.0.0.0"
    UDP_PORT = 9999
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print("UDP discovery server started on port 9999")

    while True:
        data, addr = sock.recvfrom(1024)
        if data == b"WHERE_IS_MY_ROBOT_OVERLORD":
            local_ip = get_local_ip()
            sock.sendto(local_ip.encode(), addr)
            print(f"Sent IP {local_ip} to {addr}")
            event.set()

def controller_handler(state, controller_device):
    for event in controller_device.read_loop():
        if event.type == ecodes.EV_ABS:
            if event.code == ecodes.ABS_X:
                state.axis_x = event.value / 32768.0
            elif event.code == ecodes.ABS_Y:
                state.axis_y = event.value / 32768.0
            elif event.code == ecodes.ABS_RZ:  # Right trigger for pump speed
                state.pump_speed = event.value / 255.0  # Normalize to 0.0-1.0
        elif event.type == ecodes.EV_KEY and event.code == ecodes.BTN_A and event.value == 1:
            state.quit_flag = True

def main():
    # UDP discovery server
    discovery_event = threading.Event()
    discovery_thread = threading.Thread(target=udp_discovery_server, args=(discovery_event,), daemon=True)
    discovery_thread.start()
    print("Waiting for discovery message from ESP32...")
    discovery_event.wait()
    print("Discovery complete, starting teleoperation in 2 seconds...")
    time.sleep(2)

    # Initialize ROS2
    rclpy.init()
    node = rclpy.create_node('servo_teleop')
    publisher = node.create_publisher(Vector3, '/servo_positions', 10)

    # Find controller device
    devices = [InputDevice(path) for path in list_devices()]
    controller_device = None
    for device in devices:
        if 'controller' in device.name.lower() or 'xbox' in device.name.lower() or 'x-box' in device.name.lower() or 'pad' in device.name.lower():
            controller_device = device
            break
    if controller_device is None:
        print("No controller device found. Please check permissions or device availability.")
        exit(1)
    print(f"Using controller: {controller_device.name}")

    state = State()

    # Start controller event handler thread
    controller_thread = threading.Thread(target=controller_handler, args=(state, controller_device), daemon=True)
    controller_thread.start()

    # Main teleoperation loop
    print("Teleoperation started. Use left joystick to control servos. Use right trigger for pump speed. Press A to quit.")
    while not state.quit_flag:
        # Get current values from state
        axis_x = state.axis_x
        axis_y = state.axis_y
        pump_speed = state.pump_speed

        # Apply deadzone
        if abs(axis_x) < 0.1:
            axis_x = 0.0
        if abs(axis_y) < 0.1:
            axis_y = 0.0

        # Apply directional restrictions
        if not state.allow_pan_left and axis_x < 0:
            axis_x = 0
        if not state.allow_pan_right and axis_x > 0:
            axis_x = 0
        if not state.allow_tilt_up and axis_y < 0:
            axis_y = 0
        if not state.allow_tilt_down and axis_y > 0:
            axis_y = 0

        pan = 90 + (axis_x * 90)
        tilt = 90 + (axis_y * 90)

        # Apply range limits
        pan = max(state.pan_min, min(state.pan_max, pan))
        tilt = max(state.tilt_min, min(state.tilt_max, tilt))
        msg = Vector3()
        msg.x = tilt
        msg.y = pan
        msg.z = pump_speed  # Set pump speed
        publisher.publish(msg)
        print(f"Published: pan={pan:.2f}, tilt={tilt:.2f}, pump={pump_speed:.2f}")
        time.sleep(0.1)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()