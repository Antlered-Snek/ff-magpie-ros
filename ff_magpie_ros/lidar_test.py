#!/usr/bin/env python3

import serial
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LD06USBBridge(Node):
    def __init__(self):
        super().__init__('ld06_usb_bridge')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        
        # Adjust serial port path if necessary (e.g., /dev/ttyUSB0 or /dev/ttyACM0)
        self.port = '/dev/ttyACM0'
        self.baud = 921600
        self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
        
        self.get_logger().info(f"Connected to ESP32 on {self.port} at {self.baud} baud.")
        self.timer = self.create_timer(0.001, self.read_serial_loop)

    def read_serial_loop(self):
        # Look for header bytes 0xAA, 0x55
        if self.ser.read() == b'\xAA':
            if self.ser.read() == b'\x55':
                scan_data = bytearray()
                expected_bytes = 360 * 6  # 360 points * (4 bytes float + 2 bytes uint16)
                
                while len(scan_data) < expected_bytes:
                    chunk = self.ser.read(expected_bytes - len(scan_data))
                    if not chunk:
                        return
                    scan_data.extend(chunk)

                # Unpack points
                ranges = [float('inf')] * 360
                for i in range(360):
                    offset = i * 6
                    angle = struct.unpack('f', scan_data[offset:offset+4])[0]
                    dist_mm = struct.unpack('H', scan_data[offset+4:offset+6])[0]
                    
                    if dist_mm > 0:
                        # Convert mm to meters and map to nearest index
                        idx = int(round(angle)) % 360
                        ranges[idx] = dist_mm / 1000.0

                # Publish LaserScan message
                scan_msg = LaserScan()
                scan_msg.header.stamp = self.get_clock().now().to_msg()
                scan_msg.header.frame_id = 'laser_frame'
                scan_msg.angle_min = 0.0
                scan_msg.angle_max = 2.0 * np.pi
                scan_msg.angle_increment = (2.0 * np.pi) / 360.0
                scan_msg.scan_time = 0.1
                scan_msg.range_min = 0.02
                scan_msg.range_max = 12.0
                scan_msg.ranges = ranges
                
                self.publisher_.publish(scan_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LD06USBBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()