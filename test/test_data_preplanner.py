# Copyright 2025 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import csv
from datetime import datetime, timedelta, timezone
from pathlib import Path
from types import SimpleNamespace
from typing import cast

from ff_magpie_ros.data_preplanner import (
    build_prize_histogram,
    DataPreplanner,
    DetectionRecord,
    merge_close_detections,
)
from ff_magpie_ros.ethernet_reader import TRACKED_DETECTION_CSV_FIELDS

from nav_msgs.msg import OccupancyGrid

import numpy as np

import pytest


def make_record(
    detection_id: int,
    x: float,
    y: float,
    confidence: float,
    first_seen: datetime,
    observation_count: int = 1,
) -> DetectionRecord:
    return DetectionRecord(
        id=detection_id,
        first_seen=first_seen,
        last_seen=first_seen + timedelta(seconds=1),
        frame_id='map',
        x=x,
        y=y,
        conf_avg=confidence,
        conf_highest=confidence,
        observation_count=observation_count,
    )


def test_close_sites_merge_weighted_and_keep_oldest_id() -> None:
    first_seen = datetime(2026, 11, 25, tzinfo=timezone.utc)
    records = [
        make_record(9, 0.0, 0.0, 0.5, first_seen, observation_count=2),
        make_record(
            4,
            0.01,
            0.0,
            1.0,
            first_seen + timedelta(seconds=2),
        ),
    ]

    merged = merge_close_detections(records, tolerance_m=0.02)

    assert len(merged) == 1
    assert merged[0].id == 9
    assert merged[0].observation_count == 3
    assert merged[0].x == pytest.approx(0.005)
    assert merged[0].conf_avg == (0.5 * 2 + 1.0) / 3
    assert merged[0].prize == merged[0].conf_avg


def test_proximity_groups_do_not_chain_beyond_tolerance() -> None:
    first_seen = datetime(2026, 11, 25, tzinfo=timezone.utc)
    records = [
        make_record(1, 0.00, 0.0, 0.8, first_seen),
        make_record(2, 0.01, 0.0, 0.8, first_seen + timedelta(seconds=1)),
        make_record(3, 0.02, 0.0, 0.8, first_seen + timedelta(seconds=2)),
    ]

    merged = merge_close_detections(records, tolerance_m=0.015)

    assert len(merged) == 2
    assert [record.id for record in merged] == [1, 3]


def test_prize_histogram_is_smoothed_normalized_and_frame_anchored() -> None:
    timestamp = datetime(2026, 11, 25, tzinfo=timezone.utc)
    grid = build_prize_histogram(
        [
            make_record(1, 1.0, 2.0, 0.75, timestamp),
            make_record(2, 1.5, 2.0, 0.25, timestamp),
        ],
        resolution=0.1,
        sigma_m=0.1,
        max_cells=10_000,
    )

    assert grid.frame_id == 'map'
    assert grid.origin_x < 1.0
    assert grid.origin_y < 2.0
    assert grid.density.shape == (grid.height, grid.width)
    assert np.isclose(grid.density.sum(), 1.0)
    assert np.count_nonzero(grid.density) > 2


def test_process_command_merges_new_csvs_and_publishes_once(
    tmp_path: Path,
    monkeypatch,
) -> None:
    tracked_dir = tmp_path / 'tracked'
    filtered_dir = tmp_path / 'filtered'
    tracked_dir.mkdir()
    timestamp = datetime(2026, 11, 25, tzinfo=timezone.utc)
    tracked_csv = tracked_dir / 'tracked_detections_run_001.csv'
    rows = [
        {
            'id': 8,
            'first_seen': timestamp.isoformat(),
            'last_seen': timestamp.isoformat(),
            'frame_id': 'map',
            'x': '1.0',
            'y': '2.0',
            'conf_avg': '0.5',
            'conf_highest': '0.8',
            'observation_count': '4',
        },
        {
            'id': 3,
            'first_seen': (timestamp + timedelta(seconds=1)).isoformat(),
            'last_seen': (timestamp + timedelta(seconds=1)).isoformat(),
            'frame_id': 'map',
            'x': '1.01',
            'y': '2.0',
            'conf_avg': '1.0',
            'conf_highest': '1.0',
            'observation_count': '2',
        },
        {
            'id': 6,
            'first_seen': timestamp.isoformat(),
            'last_seen': timestamp.isoformat(),
            'frame_id': 'map',
            'x': '3.0',
            'y': '4.0',
            'conf_avg': '0.6',
            'conf_highest': '0.6',
            'observation_count': '1',
        },
    ]
    with tracked_csv.open('w', newline='', encoding='utf-8') as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=TRACKED_DETECTION_CSV_FIELDS,
        )
        writer.writeheader()
        writer.writerows(rows)

    published_messages: list[OccupancyGrid] = []
    node = DataPreplanner.__new__(DataPreplanner)
    node.tracked_dir = tracked_dir
    node.tracked_file_prefix = 'tracked_detections_run'
    node.filtered_dir = filtered_dir
    node.filtered_file_prefix = 'filtered_detections_run'
    node.histogram_file_prefix = 'prizes'
    node.tolerance_m = 0.02
    node.histogram_resolution_m = 0.1
    node.gaussian_sigma_m = 0.1
    node.max_histogram_cells = 100_000
    setattr(
        node,
        'histogram_publisher',
        SimpleNamespace(publish=published_messages.append),
    )
    logger = SimpleNamespace(info=lambda message: None)
    monkeypatch.setattr(DataPreplanner, 'get_logger', lambda self: logger)
    monkeypatch.setattr(
        DataPreplanner,
        'get_clock',
        lambda self: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: object())
        ),
    )

    DataPreplanner.process_datasets(cast(DataPreplanner, node))

    filtered_files = list(filtered_dir.glob('filtered_detections_run_*.csv'))
    histogram_files = list(filtered_dir.glob('prizes_*.npz'))
    assert len(filtered_files) == 1
    assert len(histogram_files) == 1
    with filtered_files[0].open(newline='', encoding='utf-8') as csv_file:
        reader = csv.DictReader(csv_file)
        assert tuple(reader.fieldnames or ()) == TRACKED_DETECTION_CSV_FIELDS
        filtered_rows = list(reader)
    assert [row['id'] for row in filtered_rows] == ['6', '8']
    assert filtered_rows[1]['observation_count'] == '6'
    assert len(published_messages) == 1
    grid_message = published_messages[0]
    assert grid_message.header.frame_id == 'map'
    grid_cells = grid_message.info.width * grid_message.info.height
    assert grid_cells == len(grid_message.data)
    with np.load(histogram_files[0]) as histogram:
        assert histogram['frame_id'].item() == 'map'
        assert np.isclose(histogram['density'].sum(), 1.0, atol=1e-6)

    DataPreplanner.process_datasets(cast(DataPreplanner, node))
    assert len(list(filtered_dir.glob('filtered_detections_run_*.csv'))) == 1
    assert len(published_messages) == 2
