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

import struct
from types import SimpleNamespace
from typing import cast

from ff_magpie_ros.ld06_dtof_lidar_parser import (
    ld06_crc8,
    LD06DTOFLidarParser,
    parse_ld06_packet,
)

import serial


def build_packet() -> bytes:
    packet = bytearray(47)
    packet[0:2] = b'\x54\x2C'
    struct.pack_into('<HH', packet, 2, 3600, 1000)
    for point_index in range(12):
        struct.pack_into(
            '<HB',
            packet,
            6 + point_index * 3,
            1000 + point_index,
            point_index,
        )
    struct.pack_into('<HH', packet, 42, 2100, 1234)
    packet[-1] = ld06_crc8(packet[:-1])
    return bytes(packet)


class FakeSerial:
    """Small in-memory serial input for parser tests."""

    def __init__(self, data: bytes) -> None:
        self.data = bytearray(data)

    @property
    def in_waiting(self) -> int:
        return len(self.data)

    def read(self, size: int) -> bytes:
        result = bytes(self.data[:size])
        del self.data[:size]
        return result


def test_ld06_crc_and_packet_fields() -> None:
    packet = build_packet()

    assert ld06_crc8(b'\x01') == 0x4D
    decoded = parse_ld06_packet(packet)
    assert decoded is not None
    assert decoded.speed_dps == 3600
    assert decoded.start_angle_deg == 10.0
    assert decoded.end_angle_deg == 21.0
    assert decoded.timestamp_ms == 1234
    assert decoded.points[0] == (1000, 0)
    assert decoded.points[-1] == (1011, 11)


def test_bad_crc_is_rejected() -> None:
    packet = bytearray(build_packet())
    packet[-1] ^= 0xFF

    assert parse_ld06_packet(bytes(packet)) is None


def test_implausible_packet_angle_span_is_rejected() -> None:
    packet = bytearray(build_packet())
    struct.pack_into('<H', packet, 42, 5000)
    packet[-1] = ld06_crc8(packet[:-1])

    assert parse_ld06_packet(bytes(packet)) is None


def test_read_serial_loop_keeps_partial_packet_until_complete() -> None:
    packet = build_packet()
    reader = LD06DTOFLidarParser.__new__(LD06DTOFLidarParser)
    fake_serial = FakeSerial(packet[:15])
    reader.ser = cast(serial.Serial, fake_serial)
    reader._serial_buffer = bytearray()
    reader._ranges = [float('inf')] * 360
    reader._last_angle_deg = None
    reader._has_seen_wrap = False

    reader.read_serial_loop()
    assert reader._last_angle_deg is None

    fake_serial.data.extend(packet[15:])
    reader.read_serial_loop()
    assert reader._serial_buffer == bytearray()
    assert reader._last_angle_deg == 21.0
    assert reader._ranges[10] == 1.0


def test_read_serial_loop_resynchronizes_after_bad_packet() -> None:
    bad_packet = bytearray(build_packet())
    bad_packet[-1] ^= 0xFF
    reader = LD06DTOFLidarParser.__new__(LD06DTOFLidarParser)
    fake_serial = FakeSerial(b'\xAA\x55' + bad_packet + build_packet())
    reader.ser = cast(serial.Serial, fake_serial)
    reader._serial_buffer = bytearray()
    reader._ranges = [float('inf')] * 360
    reader._last_angle_deg = None
    reader._has_seen_wrap = False

    reader.read_serial_loop()

    assert reader._serial_buffer == bytearray()
    assert reader._last_angle_deg == 21.0


def test_missing_serial_device_is_retried_without_raising(monkeypatch) -> None:
    status_messages: list[str] = []
    reader = LD06DTOFLidarParser.__new__(LD06DTOFLidarParser)
    reader.port = '/dev/ttyACM0'
    reader.baud = 230400
    reader.ser = None
    reader._last_connection_attempt = -2.0
    reader._serial_is_connected = False
    reader._serial_buffer = bytearray()
    reader._ranges = [float('inf')] * 360
    reader._last_angle_deg = None
    reader._has_seen_wrap = False
    monkeypatch.setattr(
        LD06DTOFLidarParser,
        'get_logger',
        lambda self: SimpleNamespace(
            warning=status_messages.append,
            info=status_messages.append,
        ),
    )
    clock_values = iter((10.0, 11.0, 12.0))
    monkeypatch.setattr(
        'ff_magpie_ros.ld06_dtof_lidar_parser.time.monotonic',
        lambda: next(clock_values),
    )
    serial_attempts = []

    def open_serial(*args, **kwargs):
        serial_attempts.append((args, kwargs))
        if len(serial_attempts) == 1:
            raise FileNotFoundError('/dev/ttyACM0')
        return FakeSerial(b'')

    monkeypatch.setattr(serial, 'Serial', open_serial)

    reader.read_serial_loop()
    assert len(serial_attempts) == 1
    reader.read_serial_loop()
    reader.read_serial_loop()
    assert len(serial_attempts) == 2
    assert reader.ser is not None
    assert any('Waiting for /dev/ttyACM0' in msg for msg in status_messages)
