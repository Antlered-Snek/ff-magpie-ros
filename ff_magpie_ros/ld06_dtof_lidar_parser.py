#!/usr/bin/env python3

"""Publish LD06 DTOF scans received over a configurable serial connection."""

import math
import struct
import time

import serial
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan



# ANSI Colors
RED		= "\033[91m"
BLUE	= "\033[94m"
MAGENTA	= "\033[95m"
CYAN	= "\033[96m"
YELLOW	= "\033[93m"
GREEN	= "\033[92m"
RESET	= "\033[0m"


# Configs
PACKET_HEADER = 0x54
PACKET_VERSION_LENGTH = 0x2C
POINTS_PER_PACKET = 12
POINT_FREQUENCY_HZ = 4500
PACKET_SIZE = 47
SCAN_SIZE = 360
RANGE_MIN = 0.02
RANGE_MAX = 12.0
SERIAL_RETRY_INTERVAL = 2.0


# Functions and Dataclasses
@dataclass(frozen=True)
class LD06Packet:
	speed_dps: int
	start_angle_deg: float
	end_angle_deg: float
	timestamp_ms: int
	points: tuple[tuple[int, int], ...]


def ld06_crc8(data: bytes) -> int:
	"""Calculate the LD06 CRC-8 using polynomial 0x4D and an initial value of 0."""
	crc = 0
	for value in data:
		crc ^= value
		for _ in range(8):
			if crc & 0x80:
				crc = ((crc << 1) ^ 0x4D) & 0xFF
			else:
				crc = (crc << 1) & 0xFF
	return crc


def parse_ld06_packet(data: bytes) -> LD06Packet | None:
	"""Decode a complete, CRC-checked LD06 packet; return None if invalid."""

	# Return if there's something wrong with the message
	is_wrong_packet_size:bool = (len(data)  != PACKET_SIZE)
	is_wrong_head_byte0	:bool = (data[0] != PACKET_HEADER)
	is_wrong_vers_byte1	:bool= (data[1] != PACKET_VERSION_LENGTH)
	is_corrupted		:bool= (ld06_crc8(data[:-1]) != data[-1])
	if (is_wrong_packet_size or is_wrong_head_byte0 or is_wrong_vers_byte1 or is_corrupted): return None

	# Unpack data
	speed_dps, start_angle = struct.unpack_from('<HH', data, 2)
	end_angle, timestamp_ms = struct.unpack_from('<HH', data, 42)
	if start_angle >= 36000 or end_angle >= 36000:
		return None
	angle_span = (end_angle - start_angle) % 36000
	maximum_angle_span = (
		speed_dps * POINTS_PER_PACKET / POINT_FREQUENCY_HZ * 1.5 * 100
	)
	if angle_span > maximum_angle_span:
		return None

	# Get points
	points = tuple(
		struct.unpack_from('<HB', data, 6 + point_index * 3)
		for point_index in range(POINTS_PER_PACKET)
	)

	# Return Confirmed Clean Packet
	return LD06Packet(
		speed_dps=speed_dps,
		start_angle_deg=start_angle / 100.0,
		end_angle_deg=end_angle / 100.0,
		timestamp_ms=timestamp_ms,
		points=points,
	)


# ROS Node
class LD06DTOFLidarParser(Node):
	"""Read LD06 DTOF data from USB or GPIO UART and publish LaserScan."""

	def __init__(self) -> None:
		super().__init__('ld06_dtof_lidar_parser')
		
		# Declaring Serial Parameters
		self.declare_parameter('serial_port', '/dev/ttyACM0')
		self.declare_parameter('serial_baud', 230400)
		
		# Getting Serial Parameters as properties
		self.port = str(self.get_parameter('serial_port').value)
		self.baud = int(self.get_parameter('serial_baud').value)
		
		# Make sure serial parameters exist
		if not self.port:	raise ValueError('serial_port must not be empty')
		if self.baud <= 0:	raise ValueError('serial_baud must be greater than zero')

		# The device may be connected after startup, so connection attempts happen
		# from the timer instead of failing node construction.
		self.ser: serial.Serial | None = None
		self._last_connection_attempt = -SERIAL_RETRY_INTERVAL
		self._serial_is_connected = False

		# Private Lidar Variables
		self._serial_buffer = bytearray()
		self._ranges = [float('inf')] * SCAN_SIZE
		self._last_angle_deg: float | None = None
		self._has_seen_wrap = False
		
		# ROS Communication
		self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
		self.timer = self.create_timer(0.1, self.read_serial_loop)

		# Debug Display
		self.get_logger().info(f'Connecting to serial device {self.port}: ')

	def _connect_serial(self) -> bool:
		"""Open the serial device, retrying later if it is not available yet."""
		now = time.monotonic()
		if now - self._last_connection_attempt < SERIAL_RETRY_INTERVAL:
			return False
		self._last_connection_attempt = now

		try:
			self.ser = serial.Serial(self.port, self.baud, timeout=0)
		except (serial.SerialException, OSError) as error:
			if (self._serial_is_connected):
				self._serial_is_connected = False
				self.get_logger().warn(f'{error}\nSerial device {self.port} disconnected. Reconnecting:...')
			else:
				self.get_logger().info(f"{RED}Waiting for {self.port} ...{RESET}")
			return False

		self._serial_is_connected = True
		self.get_logger().info(f'Connected to serial device {self.port} at {self.baud} baud.')
		return True

	def read_serial_loop(self) -> None:
		"""Read and validate LD06 packets, publishing each completed revolution."""
		if self.ser is None and not self._connect_serial():
			return
		if self.ser is None:
			return

		try:
			bytes_available = self.ser.in_waiting
			if bytes_available == 0:
				return
			self._serial_buffer.extend(self.ser.read(bytes_available))
		except (serial.SerialException, OSError) as error:
			self.get_logger().warning(
				f'Lost serial connection to {self.port}: {error}; '
				f'retrying in {SERIAL_RETRY_INTERVAL:g} seconds.'
			)
			self.ser.close()
			self.ser = None
			self._last_connection_attempt = time.monotonic()
			self._serial_buffer.clear()
			self._ranges = [float('inf')] * SCAN_SIZE
			self._last_angle_deg = None
			self._has_seen_wrap = False
			return

		# Constant Booleans
		header_has_negative_index	:bool = lambda: (header_index < 0)
		header_is_none_zero			:bool = lambda: (header_index > 0)
		buffer_only_contains_header	:bool = lambda: (len(self._serial_buffer) < 2)
		byte_is_not_version_length	:bool = lambda: (self._serial_buffer[1] != PACKET_VERSION_LENGTH)
		buffer_is_shorter_than_msg	:bool = lambda: (len(self._serial_buffer) < PACKET_SIZE)

		# Iterate through the serial buffer in FIFO order.
		while True:
			# Find the oldest packet header and discard bytes before it.
			header_index = self._serial_buffer.find(bytes((PACKET_HEADER,)))
			if header_has_negative_index():
				self._serial_buffer.clear()
				return
			if header_is_none_zero():
				del self._serial_buffer[:header_index]	# deletes data from before the selected header index

			# Verify version length (byte 1) of seleted header
			if buffer_only_contains_header():
				return
			if byte_is_not_version_length():
				del self._serial_buffer[0]				# deletes selected header if byte 1 isn't a version_length
				continue
			if buffer_is_shorter_than_msg():
				return

			# Validate and process one packet.
			packet = parse_ld06_packet(bytes(self._serial_buffer[:PACKET_SIZE]))
			if packet is None:
				del self._serial_buffer[0]				# deletes selected header if parsed data is corrupted
				continue

			# Delete the entire serial buffer and processes the packet
			del self._serial_buffer[:PACKET_SIZE]
			self._process_packet(packet)

	def _process_packet(self, packet: LD06Packet) -> None:
		"""Add a packet's interpolated measurements to the current scan."""
		angle_span = (packet.end_angle_deg - packet.start_angle_deg) % 360.0
		for point_index, point in enumerate(packet.points):
			distance_mm = point[0]
			angle_deg = ( packet.start_angle_deg + angle_span * point_index / (POINTS_PER_PACKET - 1) ) % 360.0

			is_wrapping: bool = (self._last_angle_deg is not None) and (self._last_angle_deg - angle_deg > 180.0)
			if is_wrapping:
				if self._has_seen_wrap:
					self._publish_scan(packet.speed_dps)
				self._ranges = [float('inf')] * SCAN_SIZE
				self._has_seen_wrap = True

			if distance_mm > 0:
				distance_m = distance_mm / 1000.0
				if RANGE_MIN <= distance_m <= RANGE_MAX:
					self._ranges[int(round(angle_deg)) % SCAN_SIZE] = distance_m
			self._last_angle_deg = angle_deg

	def _publish_scan(self, speed_dps: int) -> None:
		"""Publish the completed 360-degree scan."""
		scan_msg = LaserScan()
		scan_msg.header.stamp = self.get_clock().now().to_msg()
		scan_msg.header.frame_id = 'laser_frame'
		scan_msg.angle_min = 0.0
		scan_msg.angle_max = math.radians(SCAN_SIZE - 1)
		scan_msg.angle_increment = (2.0 * math.pi) / SCAN_SIZE
		scan_msg.scan_time = 360.0 / speed_dps if speed_dps > 0 else 0.1
		scan_msg.time_increment = scan_msg.scan_time / SCAN_SIZE
		scan_msg.range_min = RANGE_MIN
		scan_msg.range_max = RANGE_MAX
		scan_msg.ranges = self._ranges.copy()
		self.publisher_.publish(scan_msg)


def main(args: list[str] | None = None) -> None:
	rclpy.init(args=args)
	node = LD06DTOFLidarParser()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		if node.ser is not None:
			node.ser.close()
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
