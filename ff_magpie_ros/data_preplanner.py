#!/usr/bin/env python3

"""Merge tracked detections into planning sites and a normalized prize grid."""

import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import tempfile
import time

from ff_magpie_ros.ethernet_reader import TRACKED_DETECTION_CSV_FIELDS

from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.ndimage import gaussian_filter
from std_msgs.msg import String


# Functions and Dataclassess
@dataclass
class DetectionRecord:
	"""A tracked or filtered detection site."""

	id: int  # noqa: A003
	first_seen: datetime
	last_seen: datetime
	frame_id: str
	x: float
	y: float
	conf_avg: float
	conf_highest: float
	observation_count: int = 1

	@property
	def prize(self) -> float:
		"""Use average detection confidence as this site's planning reward."""
		return self.conf_avg


@dataclass(frozen=True)
class PrizeHistogram:
	"""Normalized density grid and its ROS map geometry."""

	density: np.ndarray
	origin_x: float
	origin_y: float
	resolution: float
	frame_id: str

	@property
	def width(self) -> int:
		return int(self.density.shape[1])

	@property
	def height(self) -> int:
		return int(self.density.shape[0])


def _parse_timestamp(value: str, column: str) -> datetime:
	"""Parse ISO timestamps emitted by the telemetry recorder."""
	try:
		timestamp = datetime.fromisoformat(value.replace('Z', '+00:00'))
	except ValueError as error:
		raise ValueError(f'invalid {column} timestamp {value!r}') from error
	if timestamp.tzinfo is None:
		timestamp = timestamp.replace(tzinfo=timezone.utc)
	return timestamp.astimezone(timezone.utc)


def read_detection_csv(csv_path: Path) -> list[DetectionRecord]:
	"""Read tracked or filtered rows using the telemetry CSV schema."""
	records: list[DetectionRecord] = []
	with csv_path.open(newline='', encoding='utf-8') as csv_file:
		reader = csv.DictReader(csv_file)
		required_fields = set(TRACKED_DETECTION_CSV_FIELDS)

		# Error Check if CSV is corrupted
		missing_fields = required_fields.difference(reader.fieldnames or [])
		if missing_fields:
			raise ValueError(f'{csv_path} is missing CSV columns: {", ".join(sorted(missing_fields))}')

		# For every entry
		for line_number, row in enumerate(reader, start=2):
			try:
				# Get record entry
				record = DetectionRecord(
					id=int(row['id']),
					first_seen=_parse_timestamp(row['first_seen'], 'first_seen'),
					last_seen=_parse_timestamp(row['last_seen'], 'last_seen'),
					frame_id=row['frame_id'].strip(),
					x=float(row['x']),
					y=float(row['y']),
					conf_avg=float(row['conf_avg']),
					conf_highest=float(row['conf_highest']),
					observation_count=int(row['observation_count']),
				)
				numeric_values = (
					record.x,
					record.y,
					record.conf_avg,
					record.conf_highest,
				)
				if (
					(not record.frame_id)
					or (not all(math.isfinite(value) for value in numeric_values))
					or (not 0 <= record.conf_avg <= 1)
					or (not 0 <= record.conf_highest <= 1)
					or (record.observation_count < 1)
				):
					raise ValueError(f'{csv_path}:{line_number}: coordinates, frame, confidence, or count are invalid')
			except (KeyError, TypeError, ValueError) as error:
				raise ValueError(f'{csv_path}:{line_number}: invalid detection row: {error}') from error
			records.append(record)
	return records


def merge_detection_group(records: list[DetectionRecord]) -> DetectionRecord:
	"""
		Deduplication. Combine entries that are too close together, as they are likely duplicates.
		Retaining the ID of its oldest detection.
	"""

	# Make sure that all coordinates are relative to the same frame
	if not records:
		raise ValueError('cannot merge an empty detection group')
	frames = {record.frame_id for record in records}
	if len(frames) != 1:
		raise ValueError('cannot merge detections from different coordinate frames')

	# Calculating necessary variables, Fallback way to calculate weights to prevent a ZeroDivision Error
	confidence_weights = [record.conf_avg * record.observation_count for record in records]
	if sum(confidence_weights) == 0:
		coordinate_weights: list[float] = [float(record.observation_count) for record in records]
	else:
		coordinate_weights = confidence_weights
	total_coordinate_weight = sum(coordinate_weights)
	oldest = min(records, key=lambda record: (record.first_seen, record.id))
	total_observations = sum(record.observation_count for record in records)

	# Return the single merged record from the list of records
	return DetectionRecord(
		id=oldest.id,
		first_seen=min(record.first_seen for record in records),
		last_seen=max(record.last_seen for record in records),
		frame_id=oldest.frame_id,
		x=sum(
			record.x * weight
			for record, weight in zip(records, coordinate_weights)
		) / total_coordinate_weight,
		y=sum(
			record.y * weight
			for record, weight in zip(records, coordinate_weights)
		) / total_coordinate_weight,
		conf_avg=sum(
			record.conf_avg * record.observation_count for record in records
		) / total_observations,
		conf_highest=max(record.conf_highest for record in records),
		observation_count=total_observations,
	)


def merge_close_detections(records: list[DetectionRecord], tolerance_m: float) -> list[DetectionRecord]:
	"""
		Checks every entry within an input list to see if they are too close to each other.
		If they are too close, calls merge_detection_group to perform deduplication.
	"""
	if tolerance_m < 0:
		raise ValueError('tolerance_m must not be negative')
	ordered_records = sorted(records, key=lambda record: (record.first_seen, record.id))
	groups: list[list[DetectionRecord]] = []

	"""Elaboration:
		> "group" is a list of DetectionRecord entries that are too close to each other
		> "groups" is the list of separate groups
		> "possible_groups" is a list of possible groups that a record entry could be a part of. The closest group gets chosen in the end
	"""

	# Check every entry/record in a single csv
	for record in ordered_records:
		possible_groups: list[tuple[float, list[DetectionRecord]]] = []	# Float is distance from group center

		for group in groups:
			if group[0].frame_id != record.frame_id:
				continue
			
			# List of distances of a record from every single member of a group
			distances = [
				math.hypot(record.x - member.x, record.y - member.y)
				for member in group
			]

			# Checks if the record's distance is within tolerance from every single group member
			if all(distance <= tolerance_m for distance in distances):
				center_x = sum(member.x for member in group) / len(group)
				center_y = sum(member.y for member in group) / len(group)
				possible_groups.append((
					math.hypot(record.x - center_x, record.y - center_y),
					group,
				))
				# If so, appends to the possible
		
		# if there are possible groups then append the record into the group that its closest to
		if possible_groups:
			_, closest_group = min(possible_groups, key=lambda item: item[0])
			closest_group.append(record)
		# else append as an entirely new group
		else:
			groups.append([record])

	deduplicated_merged_list = [merge_detection_group(group) for group in groups]
	return deduplicated_merged_list


def build_prize_histogram(
	records: list[DetectionRecord],
	resolution: float,
	sigma_m: float,
	max_cells: int,
) -> PrizeHistogram:
	"""Builds a 2D Histogram out of filtered_detections, applies a Gaussian Filter, and normalizes the total density"""
	
	# Error check
	if not records:
		raise ValueError('cannot build a prize histogram without detections')
	if resolution <= 0 or sigma_m <= 0 or max_cells <= 0:
		raise ValueError('resolution, sigma_m, and max_cells must be positive')
	frames = {record.frame_id for record in records}
	if len(frames) != 1:
		raise ValueError('all histogram detections must use the same coordinate frame')
	weights = np.asarray([record.prize for record in records], dtype=np.float64)
	if (
		not np.all(np.isfinite(weights))
		or np.any(weights < 0)
		or weights.sum() <= 0
	):
		raise ValueError('detection prizes must have a positive finite sum')

	# Creating the Grid
	padding = max(4.0 * sigma_m, resolution)
	min_x = min(record.x for record in records) - padding
	min_y = min(record.y for record in records) - padding
	max_x = max(record.x for record in records) + padding
	max_y = max(record.y for record in records) + padding
	origin_x = math.floor(min_x / resolution) * resolution
	origin_y = math.floor(min_y / resolution) * resolution
	width = max(1, math.ceil((max_x - origin_x) / resolution))
	height = max(1, math.ceil((max_y - origin_y) / resolution))
	if width * height > max_cells:
		raise ValueError(f'histogram would contain {width * height} cells, above limit {max_cells}')

	# Creating the 2D Histogram with Numpy and applying Gaussian Filter
	x_edges = origin_x + np.arange(width + 1, dtype=np.float64) * resolution
	y_edges = origin_y + np.arange(height + 1, dtype=np.float64) * resolution
	histogram, _, _ = np.histogram2d(
		[record.y for record in records],
		[record.x for record in records],
		bins=(y_edges, x_edges),
		weights=weights,
	)
	density = gaussian_filter(
		histogram,
		sigma=sigma_m / resolution,
		mode='constant',
		truncate=4.0,
	)

	# Normalizing density to total 1.0
	total_mass = float(density.sum())
	if not math.isfinite(total_mass) or total_mass <= 0:
		raise ValueError('smoothed prize histogram has no finite mass')
	density /= total_mass

	return PrizeHistogram(
		density=density,
		origin_x=origin_x,
		origin_y=origin_y,
		resolution=resolution,
		frame_id=next(iter(frames)),
	)


def write_filtered_csv(records: list[DetectionRecord], csv_path: Path) -> None:
	"""Write filtered sites using the tracked-detection CSV schema."""
	with csv_path.open('w', newline='', encoding='utf-8') as csv_file:
		writer = csv.DictWriter(
			csv_file,
			fieldnames=TRACKED_DETECTION_CSV_FIELDS,
		)
		writer.writeheader()
		for record in sorted(records, key=lambda item: item.id):
			writer.writerow({
				'id': record.id,
				'first_seen': record.first_seen.isoformat(),
				'last_seen': record.last_seen.isoformat(),
				'frame_id': record.frame_id,
				'x': record.x,
				'y': record.y,
				'conf_avg': record.conf_avg,
				'conf_highest': record.conf_highest,
				'observation_count': record.observation_count,
			})


def save_prize_histogram(grid: PrizeHistogram, output_path: Path) -> None:
	"""Save normalized density and map geometry in a compressed archive."""
	output_path.parent.mkdir(parents=True, exist_ok=True)
	with tempfile.NamedTemporaryFile(
		mode='wb',
		dir=output_path.parent,
		prefix=f'.{output_path.name}.',
		suffix='.tmp',
		delete=False,
	) as temporary_file:
		temporary_path = Path(temporary_file.name)
		np.savez_compressed(
			temporary_file,
			density=grid.density.astype(np.float32),
			origin_x=np.float64(grid.origin_x),
			origin_y=np.float64(grid.origin_y),
			resolution=np.float64(grid.resolution),
			frame_id=np.asarray(grid.frame_id),
		)
	os.replace(temporary_path, output_path)






# Node
class DataPreplanner(Node):
	"""Process tracked detection CSVs only when commanded."""

	def __init__(self) -> None:
		super().__init__('data_preplanner')
		
		# Declaring Histogram Parameters
		self.declare_parameter('tolerance_m', 0.02)
		self.declare_parameter('histogram_resolution_m', 0.10)
		self.declare_parameter('gaussian_sigma_m', 0.10)
		self.declare_parameter('max_histogram_cells', 4_000_000)

		# Declaring Dataset Storage Parameters
		self.declare_parameter('tracked_dir', '')
		self.declare_parameter('tracked_file_prefix', 'tracked_detections_run')
		self.declare_parameter('filtered_dir', '')
		self.declare_parameter('filtered_file_prefix', 'filtered_detections_run')
		self.declare_parameter('histogram_file_prefix', 'detection_prize_histogram')

		# Declaring ROS COmmunication Parameters
		self.declare_parameter('command_topic', '/data_preplanner/command')
		self.declare_parameter('histogram_topic', '/data_preplanner/prize_grid')

		# TODO: Get the topics out to a string
		
		# Getting Historgram Parameters as properties
		self.histogram_file_prefix = str(self.get_parameter('histogram_file_prefix').value)
		self.tolerance_m = float(self.get_parameter('tolerance_m').value)
		self.histogram_resolution_m = float(self.get_parameter('histogram_resolution_m').value)
		self.gaussian_sigma_m = float(self.get_parameter('gaussian_sigma_m').value)
		self.max_histogram_cells = int(self.get_parameter('max_histogram_cells').value)		

		# Getting Dataset Storage Parameters as properties
		self.tracked_dir = Path(str(self.get_parameter('tracked_dir').value)).expanduser()
		self.tracked_file_prefix = str(self.get_parameter('tracked_file_prefix').value)
		self.filtered_dir = Path(str(self.get_parameter('filtered_dir').value)).expanduser()
		self.filtered_file_prefix = str(self.get_parameter('filtered_file_prefix').value)

		# Getting ROS Communication Parameters as strings
		histogram_topic	:str = str(self.get_parameter('histogram_topic').value)
		command_topic	:str = str(self.get_parameter('command_topic').value)

		# ROS COmmunication
		self.histogram_publisher = self.create_publisher(OccupancyGrid, histogram_topic, 1)
		self.command_subscription = self.create_subscription(String, command_topic, self.command_callback,10,)

		# Debug Info
		self.get_logger().info(f'Waiting for the "process" command on {command_topic}')

	def command_callback(self, message: String) -> None:
		"""
		Runs data-preparation ONLY when receiving the process command.
		a ROS communication-centric wrapper for the process_datasets() method.
		"""

		command = message.data.strip().lower()
		if command != 'process':
			self.get_logger().warning(f'Ignoring unknown command {message.data!r}; expected "process".')
			return
		try:
			self.process_datasets()		# Actual Processing happens here
		except (OSError, ValueError, csv.Error) as error:
			self.get_logger().error(f'Data preplanning failed: {error}')

	def _latest_filtered_file(self) -> Path | None:
		pattern = f'{self.filtered_file_prefix}_*.csv'
		matching_files = sorted(self.filtered_dir.glob(pattern))
		return matching_files[-1] if matching_files else None

	def _tracked_files(self) -> list[Path]:
		pattern = f'{self.tracked_file_prefix}_*.csv'
		return sorted(self.tracked_dir.glob(pattern))

	def _save_filtered_snapshot(self, records: list[DetectionRecord]) -> Path:
		timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
		output_path = self.filtered_dir / f'{self.filtered_file_prefix}_{timestamp}.csv'
		self.filtered_dir.mkdir(parents=True, exist_ok=True)
		with tempfile.NamedTemporaryFile(
			mode='w',
			newline='',
			encoding='utf-8',
			dir=self.filtered_dir,
			prefix=f'.{output_path.name}.',
			suffix='.tmp',
			delete=False,
		) as temporary_file:
			temporary_path = Path(temporary_file.name)
		try:
			write_filtered_csv(records, temporary_path)
			os.replace(temporary_path, output_path)
		finally:
			temporary_path.unlink(missing_ok=True)
		return output_path

	def _load_new_records(
		self,
		existing_records: list[DetectionRecord],
	) -> list[DetectionRecord]:
		represented_ids = {record.id for record in existing_records}
		new_records: list[DetectionRecord] = []

		for tracked_file in self._tracked_files():
			file_records = read_detection_csv(tracked_file)
			pending_records: list[DetectionRecord] = []
			for record in file_records:
				if record.id in represented_ids:
					continue
				known_records = existing_records + new_records
				# Merged IDs aren't stored, so recognize them by site proximity.
				if any(
					record.frame_id == known.frame_id
					and math.hypot(
						record.x - known.x,
						record.y - known.y,
					) <= self.tolerance_m
					for known in known_records
				):
					represented_ids.add(record.id)
					continue
				pending_records.append(record)
				represented_ids.add(record.id)
			if not pending_records:
				continue

			merged_records = merge_close_detections(
				pending_records,
				self.tolerance_m,
			)
			new_records.extend(merged_records)

		return new_records

	def _publish_histogram(self, grid: PrizeHistogram) -> None:
		"""Tailored for RViZ2 Visualization"""
		message = OccupancyGrid()
		message.header.stamp = self.get_clock().now().to_msg()
		message.header.frame_id = grid.frame_id
		message.info.resolution = grid.resolution
		message.info.width = grid.width
		message.info.height = grid.height
		message.info.origin.position.x = grid.origin_x
		message.info.origin.position.y = grid.origin_y
		message.info.origin.orientation.w = 1.0
		if grid.density.max() > 0:
			relative_prizes = np.rint(
				grid.density / grid.density.max() * 100
			).astype(np.int8)
			message.data = relative_prizes.ravel(order='C').tolist()
		else:
			raise ValueError('cannot publish an empty prize histogram')
		self.histogram_publisher.publish(message)

	def process_datasets(self) -> None:
		"""
		Orchestrator of the entire data-preparation pipeline:
		- Deduplicating tarcked_detections into filtered_detections
		- 2D Histogram with Gaussian Blur in the form of am .npz file
		- Publishing said histogram to RViZ
		"""
		self.get_logger().info('[Finished 0/4]: Starting data-preparation...')
		start_time = time.perf_counter()

		latest_filtered = self._latest_filtered_file()
		filtered_records = (read_detection_csv(latest_filtered) if latest_filtered else [])
		new_records = self._load_new_records(filtered_records)
		if new_records:
			frames = {
				record.frame_id
				for record in filtered_records + new_records
			}
			if len(frames) != 1:
				raise ValueError('tracked and filtered detections must share one map/odom frame')
			filtered_records.extend(new_records)
			filtered_path = self._save_filtered_snapshot(filtered_records)
			self.get_logger().info(f'[Finished 1/4]: Added {len(new_records)} filtered sites to {filtered_path}')
		elif not filtered_records:
			self.get_logger().warning(f'No tracked CSV data found in {self.tracked_dir} and no filtered sites exist.')
			return
		else:
			self.get_logger().info('[Finished 1/4]: No new records to deduplicate')

		# Creating 2D Histogram with Gaussian Blur
		grid = build_prize_histogram(
			filtered_records,
			self.histogram_resolution_m,
			self.gaussian_sigma_m,
			self.max_histogram_cells,
		)
		self.get_logger().info('[Finished 2/4]: Histogram Created')
		
		# Saving and Publishing Histogram
		timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
		histogram_path = self.filtered_dir / f'{self.histogram_file_prefix}_{timestamp}.npz'
		save_prize_histogram(grid, histogram_path)
		self._publish_histogram(grid)
		self.get_logger().info(f'[Finished 3/4]: Published prize grid ({grid.width}x{grid.height}), and saved {histogram_path}')

		end_time = time.perf_counter() - start_time
		self.get_logger().info(f'[Finished 4/4] Data preparation done within {end_time:.2f}s')


def main(args: list[str] | None = None) -> None:
	rclpy.init(args=args)
	node = DataPreplanner()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()


if __name__ == '__main__':
	main()
