import unittest
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from toilet_bluetooth_interface import BleFrameParser, FRAME_START_BYTE, make_frame


class BleFramingParserTests(unittest.TestCase):
    def test_single_full_frame_in_one_chunk(self):
        parser = BleFrameParser()
        payload = b"hello world"
        frames = parser.append(make_frame(payload))
        self.assertEqual(frames, [payload])
        self.assertEqual(parser.frames_parsed, 1)

    def test_one_frame_split_across_many_chunks(self):
        parser = BleFrameParser()
        payload = b"abcdefghijklmnopqrstuvwxyz0123456789"
        frame = make_frame(payload)
        out = []
        for byte in frame:
            out.extend(parser.append(bytes([byte])))
        self.assertEqual(out, [payload])
        self.assertEqual(parser.frames_parsed, 1)

    def test_multiple_frames_in_one_chunk(self):
        parser = BleFrameParser()
        p1 = b"frame-1"
        p2 = b"frame-2-longer"
        combined = make_frame(p1) + make_frame(p2)
        frames = parser.append(combined)
        self.assertEqual(frames, [p1, p2])
        self.assertEqual(parser.frames_parsed, 2)

    def test_noise_before_start_byte_resync(self):
        parser = BleFrameParser()
        payload = b"clean-payload"
        chunk = b"\x00\x01\x02junk" + make_frame(payload)
        frames = parser.append(chunk)
        self.assertEqual(frames, [payload])
        self.assertGreater(parser.bytes_dropped_resync, 0)

    def test_truncated_frame_waits_for_next_chunk(self):
        parser = BleFrameParser()
        payload = b"needs-two-callbacks"
        frame = make_frame(payload)
        first = frame[:5]
        second = frame[5:]
        self.assertEqual(parser.append(first), [])
        self.assertEqual(parser.append(second), [payload])

    def test_make_frame_rejects_payload_over_uint16(self):
        with self.assertRaises(ValueError):
            make_frame(b"a" * 65536)

    def test_parser_recovers_after_malformed_length(self):
        parser = BleFrameParser(max_payload=32)
        malformed = bytes([FRAME_START_BYTE, 0x40, 0x00]) + b"xxxx"
        good_payload = b"ok"
        frames = parser.append(malformed + make_frame(good_payload))
        self.assertEqual(frames, [good_payload])
        self.assertEqual(parser.malformed_frame_count, 1)


if __name__ == "__main__":
    unittest.main()
