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
from datetime import datetime, timezone
import math
from pathlib import Path
from types import SimpleNamespace
from typing import cast

from ff_magpie_ros.ethernet_reader import (
    detection_map_coordinates,
    EthernetReader,
    save_tracked_detections,
    TrackedDetections,
)

import pytest

from std_msgs.msg import String


def test_detection_map_coordinates_applies_robot_pose() -> None:
    x, y = detection_map_coordinates(2.0, 0.0, 0.0, (1.0, 2.0, math.pi / 2))

    assert x == pytest.approx(1.0)
    assert y == pytest.approx(4.0)


def test_store_telemetry_aggregates_same_id_by_confidence() -> None:
    reader = EthernetReader.__new__(EthernetReader)
    reader.robot_pose = (1.0, 2.0, 0.0)
    reader.robot_pose_frame = 'map'
    reader.dataset = {}
    payloads = [
        {
            'detections': [{
                'id': 7,
                'distance_m': 1.0,
                'angle_x_deg': 0.0,
                'angle_y_deg': 0.0,
                'confidence': 0.25,
            }],
        },
        {
            'detections': [{
                'id': 7,
                'distance_m': 3.0,
                'angle_x_deg': 0.0,
                'angle_y_deg': 0.0,
                'confidence': 0.75,
            }],
        },
    ]

    for payload in payloads:
        EthernetReader.store_telemetry(reader, payload)

    stored = reader.dataset[7]
    assert stored.x == pytest.approx(3.5)
    assert stored.y == pytest.approx(2.0)
    assert stored.conf_avg == pytest.approx(0.5)
    assert stored.conf_highest == pytest.approx(0.75)
    assert stored.frame_id == 'map'
    assert payloads[0]['detections'][0]['frame_id'] == 'map'
    assert stored.first_seen.tzinfo == timezone.utc
    assert stored.last_seen >= stored.first_seen
    assert payloads[0]['detections'][0]['x'] == pytest.approx(2.0)
    assert payloads[0]['detections'][0]['y'] == pytest.approx(2.0)


def test_save_to_csv_writes_timestamped_dataset(tmp_path: Path) -> None:
    timestamp = datetime(2026, 11, 25, 4, 0, tzinfo=timezone.utc)
    dataset = {
        7: TrackedDetections(
            id=7,
            first_seen=timestamp,
            last_seen=timestamp,
            frame_id='map',
            x=1.5,
            y=2.5,
            conf_avg=0.7,
            conf_highest=0.9,
        ),
    }

    csv_path = save_tracked_detections(dataset, str(tmp_path), 'detections')

    assert csv_path.parent == tmp_path
    assert csv_path.name.startswith('detections_')
    with csv_path.open(newline='', encoding='utf-8') as csv_file:
        rows = list(csv.DictReader(csv_file))
    assert rows == [{
        'id': '7',
        'first_seen': timestamp.isoformat(),
        'last_seen': timestamp.isoformat(),
        'frame_id': 'map',
        'x': '1.5',
        'y': '2.5',
        'conf_avg': '0.7',
        'conf_highest': '0.9',
        'observation_count': '1',
    }]


@pytest.mark.parametrize('command', ['quit', 'exit'])
def test_exit_command_requests_context_shutdown(
    monkeypatch,
    command: str,
) -> None:
    context = object()
    reader = SimpleNamespace(
        context=context,
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
    )
    shutdown_calls: list[object] = []
    monkeypatch.setattr(
        'ff_magpie_ros.ethernet_reader.rclpy.try_shutdown',
        lambda context: shutdown_calls.append(context),
    )

    EthernetReader.command_callback(
        cast(EthernetReader, reader),
        cast(String, SimpleNamespace(data=command)),
    )

    assert shutdown_calls == [context]
