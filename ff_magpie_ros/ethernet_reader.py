#!/usr/bin/env python3

"""ROS 2 bridge for ff-magpie-cam UDP telemetry and settings."""

import csv
import json
import math
import socket
import threading
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ANSI Colors
RED		= "\033[91m"
BLUE	= "\033[94m"
MAGENTA	= "\033[95m"
CYAN	= "\033[96m"
YELLOW	= "\033[93m"
GREEN	= "\033[92m"
RESET	= "\033[0m"



# Functions and Dataclasses
@dataclass
class TrackedDetections:
	first_seen: datetime
	last_seen: datetime
	id: int = 0
	x: float = 0.0
	y: float = 0.0
	conf_avg: float = 0.0
	conf_highest: float = 0.0
	_observation_count: int = field(default=0, repr=False)
	_confidence_sum: float = field(default=0.0, repr=False)
	_weighted_x_sum: float = field(default=0.0, repr=False)
	_weighted_y_sum: float = field(default=0.0, repr=False)
	_coordinate_weight_sum: float = field(default=0.0, repr=False)


def detection_map_coordinates(
	distance_m: float,
	angle_x_deg: float,
	angle_y_deg: float,
	robot_pose: tuple[float, float, float],
) -> tuple[float, float]:
	"""Project a camera detection into map coordinates using the SLAM robot pose."""
	robot_x, robot_y, robot_yaw = robot_pose
	angle_x = math.radians(angle_x_deg)
	angle_y = math.radians(angle_y_deg)
	horizontal_distance = distance_m * math.cos(angle_y)

	# Camera-right maps to negative base_link y in the ROS base_link convention.
	base_x = horizontal_distance * math.cos(angle_x)
	base_y = -horizontal_distance * math.sin(angle_x)
	map_x = robot_x + math.cos(robot_yaw) * base_x - math.sin(robot_yaw) * base_y
	map_y = robot_y + math.sin(robot_yaw) * base_x + math.cos(robot_yaw) * base_y
	return map_x, map_y


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
	"""Extract planar yaw from a pose quaternion."""
	return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def format_telemetry(payload: dict[str, Any]) -> str:
	"""Turn one telemetry packet into readable terminal text."""
	detections = payload.get('detections', [])
	fps = payload.get('fps')
	sequence = payload.get('sequence')
	summary = f'seq={sequence} fps={fps}'

	if not detections:
		return f'{summary} detections=none'

	lines = [summary]
	for detection in detections:
		center = detection.get('center', {})
		lines.append(
			'  '
			f"id={detection.get('id')} "
			f"class={detection.get('class_name')} "
			f"conf={detection.get('confidence')} "
			f"distance={detection.get('distance_m')}m "
			f"angle=({detection.get('angle_x_deg')},"
			f"{detection.get('angle_y_deg')})deg "
			f"center=({center.get('x')},{center.get('y')})"
		)
	return '\n'.join(lines)


def parse_command(line: str) -> Optional[dict[str, Any]]:
	"""Convert a text command into an ff-magpie-cam UDP payload."""
	parts = line.strip().split(maxsplit=1)
	if not parts:
		return None

	command = parts[0].lower()
	value = parts[1].strip() if len(parts) == 2 else ''

	if command == 'height':
		try:
			return {'type': 'camera_settings', 'height_m': float(value)}
		except ValueError:
			return None
	if command == 'angle':
		try:
			return {'type': 'camera_settings', 'angle_deg': float(value)}
		except ValueError:
			return None
	if command in ('calibration', 'start_calibration'):
		if value.lower() not in ('on', 'off', 'start', 'true', 'false'):
			return None
		return {
			'type': 'start_calibration',
			'start_calibration': value.lower() in ('on', 'start', 'true'),
		}
	return None


def save_to_csv(
	dataset: dict[int, TrackedDetections],
	output_dir: str,
	file_prefix: str,
) -> Path:
	"""Write tracked detections to a timestamped CSV in the configured directory."""
	if not output_dir.strip():
		raise ValueError('output_dir must not be empty')
	if not file_prefix.strip() or Path(file_prefix).name != file_prefix:
		raise ValueError('file_prefix must be a non-empty filename prefix')

	directory = Path(output_dir).expanduser()
	directory.mkdir(parents=True, exist_ok=True)
	timestamp = datetime.now(timezone.utc)
	filename = f'{file_prefix}_{timestamp.strftime("%Y%m%dT%H%M%S_%fZ")}.csv'
	csv_path = directory / filename

	fieldnames = (
		'id',
		'first_seen',
		'last_seen',
		'x',
		'y',
		'conf_avg',
		'conf_highest',
	)
	with csv_path.open('w', newline='', encoding='utf-8') as csv_file:
		writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
		writer.writeheader()
		for detection_id in sorted(dataset):
			detection = dataset[detection_id]
			writer.writerow({
				'id': detection.id,
				'first_seen': detection.first_seen.isoformat(),
				'last_seen': detection.last_seen.isoformat(),
				'x': detection.x,
				'y': detection.y,
				'conf_avg': detection.conf_avg,
				'conf_highest': detection.conf_highest,
			})

	return csv_path


# ROS Node
class EthernetReader(Node):
	"""Receive camera telemetry and send camera settings over UDP."""

	def __init__(self) -> None:
		super().__init__('ethernet_reader')

		# Declaring Network Parameters
		self.declare_parameter('listen_host', '0.0.0.0')
		self.declare_parameter('listen_port', 9000)
		self.declare_parameter('command_port', 9001)
		self.declare_parameter('target_ip', '')

		# Declaring Dataset Storage Parameters
		self.declare_parameter('output_dir', '')
		self.declare_parameter('file_prefix', '')
		
		# Getting Network Parameters as properties
		self.listen_host	= str(self.get_parameter('listen_host').get_parameter_value().string_value)
		self.listen_port	= int(self.get_parameter('listen_port').get_parameter_value().integer_value)
		self.command_port	= int(self.get_parameter('command_port').get_parameter_value().integer_value)
		target_ip = str(self.get_parameter('target_ip').get_parameter_value().string_value)

		# Getting Dataset Storage Parameters as properties
		self.csv_output_dir  = str(self.get_parameter('output_dir').get_parameter_value().string_value)
		self.csv_file_prefix = str(self.get_parameter('file_prefix').get_parameter_value().string_value)

		# Other properties
		self.camera_address: Optional[tuple[str, int]] = ((target_ip, self.command_port) if target_ip else None)
		self.stop_event = threading.Event()
		self.payload_lock = threading.Lock()
		self.pending_payloads: list[tuple[dict[str, Any], str, int]] = []
		self.dataset: dict[int, TrackedDetections] = {}
		self.robot_pose: Optional[tuple[float, float, float]] = None

		# ROS Communication
		self.telemetry_publisher = self.create_publisher(String, 'camera/telemetry', 10)
		self.command_subscription = self.create_subscription(String, 'camera/command', self.command_callback, 10)
		self.pose_subscription = self.create_subscription(PoseWithCovarianceStamped, '/slam_toolbox/pose', self.pose_callback, 10,)
		self.publish_timer = self.create_timer(0.02, self.process_pending_telemetry)
		self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

		# Thread for receiving high-speed UDP
		self.receive_thread = threading.Thread(
			target=self.receive_telemetry,
			name='ethernet-telemetry-receiver',
			daemon=True,
		)
		self.receive_thread.start()

		# Debug Display
		self.get_logger().info(f'Listening for telemetry on UDP {self.listen_host}:{self.listen_port}')
		if self.camera_address:
			self.get_logger().info(f'Command target set to {self.camera_address[0]}:{self.camera_address[1]}')

		# Commands
		self.get_logger().info( f"\n\n{YELLOW}Send command to MaixCAM 2:{RESET}\n" \
			f"{CYAN}ros2 topic pub --once /camera/command std_msgs/msg/String {YELLOW}<DATA>{RESET}\n" \
			f"{YELLOW}         <DATA> => \"{{data: \'command value\'}}\"{RESET}\n\n"
			
			f"{GREEN}         Avaliable Commands:{RESET}\n" \
			f"{GREEN}           - height{RESET}\n" \
			f"{GREEN}           - angle{RESET}\n" \
			f"{GREEN}           - target_ip{RESET}\n\n" \
			
			f"{GREEN}          Special Commands:{RESET}\n" \
			f"{GREEN}           - calibration start{RESET}\n" \
			f"{GREEN}           - quit{RESET}\n" \
			f"{GREEN}           - exit{RESET}\n"
		)


	def receive_telemetry(self) -> None:
		"""Receive UDP packets until the node shuts down."""
		with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receive_socket:
			receive_socket.bind((self.listen_host, self.listen_port))
			receive_socket.settimeout(0.5)

			# While still active
			while not self.stop_event.is_set():

				# Getting Address
				try:
					data, address = receive_socket.recvfrom(65535)
				except socket.timeout:
					continue
				except OSError as error:
					self.get_logger().error(f'Receive error: {error}')
					continue

				# Getting JSON payload
				try:
					payload = json.loads(data.decode('utf-8'))
				except (UnicodeDecodeError, json.JSONDecodeError) as error:
					self.get_logger().warning(f'Invalid telemetry from {address[0]}:{address[1]}: {error}')
					continue

				# Make sure payload is a dictionary instance
				if not isinstance(payload, dict):
					self.get_logger().warning(f'Invalid telemetry from {address[0]}:{address[1]}: expected a JSON object')
					continue

				# Make sure there's something on that IP address
				if self.camera_address is None:
					self.camera_address = (address[0], self.command_port)
					self.get_logger().info(f'Command target set from telemetry to {address[0]}:{self.command_port}')

				# Storing the previous JSON Payload in a memory-safe manner
				with self.payload_lock:
					self.pending_payloads.append((payload, address[0], address[1]))

	def pose_callback(self, message: PoseWithCovarianceStamped) -> None:
		"""Keep the latest robot pose published by slam_toolbox."""
		position = message.pose.pose.position
		orientation = message.pose.pose.orientation
		self.robot_pose = (
			position.x,
			position.y,
			quaternion_yaw(orientation.x, orientation.y, orientation.z, orientation.w),
		)

	def store_telemetry(self, payload: dict[str, Any]) -> None:
		"""Project detections and update the per-ID confidence aggregates."""
		
		# Make sure Pose is actually populated
		if self.robot_pose is None:
			self.get_logger().warning('Cannot store detections before receiving /slam_toolbox/pose')
			return

		# For every detection in payload
		for detection in payload.get('detections', []):

			# Make sure detection is a dictionary
			if not isinstance(detection, dict):
				self.get_logger().warning('Skipping telemetry detection that is not an object')
				continue

			try:
				# Get values
				detection_id	= int(detection['id'])
				distance		= float(detection['distance_m'])
				angle_x			= float(detection['angle_x_deg'])
				angle_y			= float(detection['angle_y_deg'])
				confidence		= float(detection['confidence'])

				# Error Check
				value_is_not_finite: bool = not all(math.isfinite(value) for value in (distance, angle_x, angle_y, confidence))
				if value_is_not_finite:
					raise ValueError('detection values must be finite')
				if (distance < 0 or confidence < 0):
					raise ValueError('distance and confidence must be non-negative')
			except (KeyError, TypeError, ValueError) as error:
				self.get_logger().warning(f'Skipping invalid telemetry detection: {error}')
				continue

			# Local polar coordinates to global cartesian coordinates
			x, y = detection_map_coordinates(distance, angle_x, angle_y, self.robot_pose)
			detection['x'] = x
			detection['y'] = y

			# Create new dateset entry if ID is unique
			stored = self.dataset.get(detection_id)
			if stored is None:
				first_seen = datetime.now(timezone.utc)
				stored = TrackedDetections(
					id=detection_id,
					first_seen=first_seen,
					last_seen=first_seen,
				)
				self.dataset[detection_id] = stored
			else:
				stored.last_seen = datetime.now(timezone.utc)

			# Averaging Confidence
			stored._observation_count += 1
			stored._confidence_sum += confidence
			stored.conf_avg = stored._confidence_sum / stored._observation_count
			stored.conf_highest = max(stored.conf_highest, confidence)

			# Averaging coordinates weighted by confidence
			stored._weighted_x_sum += x * confidence
			stored._weighted_y_sum += y * confidence
			stored._coordinate_weight_sum += confidence
			if stored._coordinate_weight_sum > 0:
				stored.x = stored._weighted_x_sum / stored._coordinate_weight_sum
				stored.y = stored._weighted_y_sum / stored._coordinate_weight_sum
			else:
				stored.x = (stored.x * (stored._observation_count - 1) + x) / stored._observation_count
				stored.y = (stored.y * (stored._observation_count - 1) + y) / stored._observation_count


	def process_pending_telemetry(self) -> None:
		"""Publish received packets from the ROS executor thread."""
		with self.payload_lock:
			pending = self.pending_payloads
			self.pending_payloads = []

		# For every payload in pending_payloads
		for payload, sender_ip, sender_port in pending:
			self.store_telemetry(payload)
			message = String()
			message.data = json.dumps(payload, separators=(',', ':'))
			self.telemetry_publisher.publish(message)
			self.get_logger().info(f'[{sender_ip}:{sender_port}] {format_telemetry(payload)}')


	def command_callback(self, message: String) -> None:
		"""Update camera variables via commands received on the camera/command topic"""
		line = message.data.strip()
		parts = line.split(maxsplit=1)
		if not parts:
			return

		command = parts[0].lower()
		value = parts[1].strip() if len(parts) == 2 else ''

		# Execute command for changing targets
		if command == 'target':
			if not value:
				self.get_logger().warning('Usage: target <camera-ip>')
				return
			self.camera_address = (value, self.command_port)
			self.get_logger().info(f'Command target set to {value}:{self.command_port}')
			return

		# Execute command for exiting
		if command in ("quit", "exit"):
			self.get_logger().info('Shutdown requested; saving tracked detections.')
			rclpy.try_shutdown(context=self.context)
			return
		
		# Log warning and safely return if no camera ip address
		if self.camera_address is None:
			self.get_logger().warning('Set a target first with: target <camera-ip>')
			return

		# Turn Line into proper UDP Payload
		payload = parse_command(line)
		if payload is None:
			self.get_logger().warning( 'Unknown or invalid command. Usage: height <meters>, angle <degrees>, or calibration <on|off|start>')
			return

		# Send commands to MaixCAM2 through UDP
		try:
			self.command_socket.sendto(json.dumps(payload, separators=(',', ':')).encode('utf-8'),self.camera_address,)
			self.get_logger().info(f'Sent {payload} to {self.camera_address[0]}:{self.camera_address[1]}')
		except OSError as error:
			self.get_logger().error(f'Send error: {error}')

	def close(self) -> None:
		"""Stop the UDP receiver and close the command socket."""
		self.stop_event.set()
		self.command_socket.close()


def main(args: Optional[list[str]] = None) -> None:
	rclpy.init(args=args)
	node = EthernetReader()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		try:
			node.close()
		finally:
			try:
				csv_path = save_to_csv(node.dataset, node.csv_output_dir, node.csv_file_prefix)
				node.get_logger().info(f'Saved tracked detections to {csv_path}')
			finally:
				node.destroy_node()
				if rclpy.ok():
					rclpy.shutdown()


if __name__ == '__main__':
	main()
