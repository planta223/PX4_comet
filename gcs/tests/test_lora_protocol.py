import csv
import io
import queue
import struct
import types
import unittest

import config
import lora
import shared_state
import stats


_PAYLOAD_STRUCT = struct.Struct("<5BIiii6h2B")


def make_telemetry(
    sequence=1,
    flight_state=1,
    status_flags=0x17,
    fault_flags=0,
    time_ms=123456,
    latitude_raw=375_123_456,
    longitude_raw=1271_234_567,
    altitude_raw=-1234,
    velocity_n_raw=125,
    velocity_e_raw=-25,
    velocity_d_raw=31,
    roll_raw=-1025,
    pitch_raw=250,
    yaw_raw=17999,
    ack_sequence=9,
    ack_result=0,
):
    payload = _PAYLOAD_STRUCT.pack(
        config.LORA_TELEMETRY_HEADER,
        sequence,
        flight_state,
        status_flags,
        fault_flags,
        time_ms,
        latitude_raw,
        longitude_raw,
        altitude_raw,
        velocity_n_raw,
        velocity_e_raw,
        velocity_d_raw,
        roll_raw,
        pitch_raw,
        yaw_raw,
        ack_sequence,
        ack_result,
    )
    return payload + bytes([lora._crc8(payload)])


class _CsvState:
    def __init__(self):
        self.csv_file = io.StringIO()
        self.csv_writer = csv.writer(self.csv_file)


class _FakeSerial:
    def __init__(self, pending=b"", responses=None):
        self._buffer = bytearray(pending)
        self._responses = list(responses or [])
        self.writes = []

    @property
    def in_waiting(self):
        return len(self._buffer)

    def write(self, data):
        self.writes.append(bytes(data))
        index = len(self.writes) - 1
        if index < len(self._responses):
            self._buffer.extend(self._responses[index])

    def read(self, count):
        if not self._buffer:
            return b""
        data = bytes(self._buffer[:count])
        del self._buffer[:count]
        return data


def _command_context(timeout=0.01, retries=2):
    cfg = types.SimpleNamespace(**{
        name: getattr(config, name) for name in dir(config) if name.isupper()
    })
    cfg.LORA_ACK_TIMEOUT_SEC = timeout
    cfg.LORA_COMMAND_RETRY_COUNT = retries
    telemetry_stats = {
        "rate": stats.RateStats(10),
        "seq_loss": stats.SeqLossTracker(),
        "pos": stats.Vec3RollingStats(10),
        "last_csv_flush": 0.0,
        "last_telemetry_sequence": None,
        "last_ack_sequence": None,
        "last_ack_result": cfg.LORA_ACK_RESULT_NONE,
    }
    return (
        cfg,
        queue.Queue(),
        queue.Queue(),
        _CsvState(),
        shared_state.SharedOrigin(),
        telemetry_stats,
    )


class LoraProtocolTests(unittest.TestCase):
    def test_struct_size_and_signed_scaled_fields(self):
        frame = make_telemetry(fault_flags=0x12)
        self.assertEqual(len(frame), 36)
        self.assertEqual(lora._TELEMETRY_STRUCT.size, config.LORA_TELEMETRY_LENGTH)

        parsed = lora.parse_lora_packet(frame)
        self.assertEqual(parsed["sequence"], 1)
        self.assertEqual(parsed["flight_state"], "ASCENT")
        self.assertEqual(parsed["status_flags"], 0x17)
        self.assertEqual(parsed["fault_flags"], 0x12)
        self.assertAlmostEqual(parsed["latitude"], 37.5123456)
        self.assertAlmostEqual(parsed["longitude"], 127.1234567)
        self.assertAlmostEqual(parsed["altitude"], -12.34)
        self.assertAlmostEqual(parsed["velocity_n"], 12.5)
        self.assertAlmostEqual(parsed["velocity_e"], -2.5)
        self.assertAlmostEqual(parsed["velocity_d"], 3.1)
        self.assertAlmostEqual(parsed["roll"], -10.25)
        self.assertAlmostEqual(parsed["pitch"], 2.5)
        self.assertAlmostEqual(parsed["yaw"], 179.99)
        self.assertEqual(parsed["ack_sequence"], 9)
        self.assertEqual(parsed["ack_result_name"], "OK")

    def test_parser_crc_noise_concatenation_fragmentation_and_wrap(self):
        parser = lora._FrameParser(config)
        bad = bytearray(make_telemetry(sequence=10))
        bad[20] ^= 0x40
        frame_255 = make_telemetry(sequence=255)
        frame_0 = make_telemetry(sequence=0)

        self.assertEqual(parser.feed(b"noise" + bytes(bad) + frame_255[:7]), [])
        frames = parser.feed(frame_255[7:] + frame_0)
        self.assertEqual([lora.parse_lora_packet(f)["sequence"] for f in frames], [255, 0])
        self.assertGreaterEqual(parser.crc_fail_count, 1)

    def test_parse_rejects_incomplete_or_corrupt_frame(self):
        frame = make_telemetry()
        with self.assertRaises(ValueError):
            lora.parse_lora_packet(frame[:-1])
        corrupt = bytearray(frame)
        corrupt[-1] ^= 1
        with self.assertRaises(ValueError):
            lora.parse_lora_packet(corrupt)

    def test_invalid_coordinate_sentinel_and_altitude_saturation(self):
        parsed = lora.parse_lora_packet(make_telemetry(
            status_flags=0x11,
            latitude_raw=0,
            longitude_raw=0,
            altitude_raw=-2_147_483_648,
        ))
        self.assertFalse(parsed["position_valid"])
        self.assertFalse(parsed["altitude_valid"])

    def test_embedded_ack_ignores_pre_send_buffer_and_processes_telemetry(self):
        ctx = _command_context()
        cfg, trajectory_q, status_q, csv_state, origin, telemetry_stats = ctx
        stale = make_telemetry(sequence=40, status_flags=0x10, ack_sequence=7, ack_result=0)
        no_ack = make_telemetry(
            sequence=41, status_flags=0x10,
            ack_sequence=0, ack_result=config.LORA_ACK_RESULT_NONE,
        )
        final = make_telemetry(sequence=42, status_flags=0x10, ack_sequence=7, ack_result=0)
        serial = _FakeSerial(pending=stale, responses=[no_ack + final])
        parser = lora._FrameParser(cfg)

        success, result, attempts = lora._send_command_and_wait_ack(
            serial, parser, "LOCK_ALL", 7, cfg,
            trajectory_q, status_q, csv_state, origin, telemetry_stats,
        )

        self.assertTrue(success)
        self.assertEqual(result, "OK")
        self.assertEqual(attempts, 1)
        self.assertEqual(trajectory_q.qsize(), 3)

    def test_retry_reuses_identical_sequence(self):
        ctx = _command_context(timeout=0.001, retries=2)
        cfg, trajectory_q, status_q, csv_state, origin, telemetry_stats = ctx
        ack1 = make_telemetry(sequence=1, status_flags=0x10, ack_sequence=255, ack_result=0)
        ack2 = make_telemetry(sequence=2, status_flags=0x10, ack_sequence=255, ack_result=0)
        serial = _FakeSerial(responses=[b"", ack1 + ack2])

        success, _, attempts = lora._send_command_and_wait_ack(
            serial, lora._FrameParser(cfg), "RELEASE_ALL", 255, cfg,
            trajectory_q, status_q, csv_state, origin, telemetry_stats,
        )
        self.assertTrue(success)
        self.assertEqual(attempts, 2)
        self.assertEqual(len(serial.writes), 2)
        self.assertEqual(serial.writes[0], serial.writes[1])
        self.assertEqual(serial.writes[0][1], 255)


if __name__ == "__main__":
    unittest.main()
