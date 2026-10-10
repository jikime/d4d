import base64
import json
import os
from pathlib import Path
import socket
import struct
import threading
import tempfile
import time
import unittest
from unittest.mock import patch

import httpx

from d4d_web.board_wifi import BoardFrames, BoardWifiService
from d4d_web.hardware import HardwareError
from d4d_web.viewer_bridge import ViewerProcess, ViewerReader, VIEWER, find_viewer


class BoardFramesTests(unittest.TestCase):
    def test_binary_chunks_preserve_embedded_newlines_and_interleaved_telemetry(self):
        image = b'\xff\xd8\n{"type":"fc","armed":true}\nSNAP_END\n\xff\xd9'
        split = 9
        wire = (f"SNAPB {len(image)} 320 240\nSNAPC {split}\n".encode() + image[:split]
                + b'\n{"type":"fc","armed":false}\n' + f"SNAPC {len(image)-split}\n".encode()
                + image[split:] + b"\nSNAP_END\n")
        for chunk_size in (1, 7, 10000):
            parser, events = BoardFrames(), []
            for start in range(0, len(wire), chunk_size):
                events += parser.feed(wire[start:start + chunk_size])
            self.assertEqual(events, [("telemetry", {"type": "fc", "armed": False}), ("image", (image, 320, 240))])
            self.assertEqual(parser.errors, 0)

    def test_bad_binary_frame_recovers_at_the_next_frame(self):
        parser = BoardFrames()
        # 선언된 사진보다 큰 조각은 버리되 그 원본 바이트가 JSON으로 처리되지 않는다.
        invalid = b'{"type":"fc","armed":true}\n'
        events = parser.feed(b"SNAPB 4 320 240\n" + f"SNAPC {len(invalid)}\n".encode() + invalid
                             + b"\nSNAP_END\nSNAPB 4 320 240\nSNAPC 4\n\xff\xd8\xff\xd9\nSNAP_END\n")
        self.assertEqual(events, [("image", (b"\xff\xd8\xff\xd9", 320, 240))])
        self.assertGreater(parser.errors, 0)
        for invalid_header in (b"SNAPB 500001 320 240\n", b"SNAPC 999999\n", b"SNAPC -1\n", b"SNAPB 4 0 240\n"):
            self.assertEqual(parser.feed(invalid_header), [])
        self.assertIsNone(parser.header)
        self.assertEqual(parser.chunk_remaining, 0)

    def test_split_utf8_and_interleaved_json_do_not_corrupt_photo(self):
        image = b"\xff\xd8camera-data\xff\xd9"
        encoded = base64.b64encode(image)
        wire = (f"SNAP_BEGIN {len(image)} 320 240 {len(encoded)}\n".encode()
                + encoded[:8] + b"\n" + json.dumps({"type": "wifi", "sta_ssid": "훈련장"}, ensure_ascii=False).encode()
                + b"\n" + encoded[8:] + b"\nSNAP_END\n")
        parser, events = BoardFrames(), []
        for start in range(0, len(wire), 7):
            events += parser.feed(wire[start:start + 7])
        self.assertEqual(events, [("telemetry", {"type": "wifi", "sta_ssid": "훈련장"}), ("image", (image, 320, 240))])
        self.assertEqual(parser.errors, 0)

    def test_truncated_oversize_and_invalid_photos_are_rejected(self):
        parser = BoardFrames()
        for wire in (b"SNAP_BEGIN 500001 320 240 666668\nSNAP_END\n", b"SNAP_BEGIN 8 320 240 12\nAAAA\nSNAP_END\n",
                     b"SNAP_BEGIN 3 320 240 4\nYWJj\nSNAP_END\n"):
            self.assertEqual(parser.feed(wire), [])
        self.assertGreater(parser.errors, 0)
        parser.feed(b"x" * (parser.MAX_LINE + 1))
        self.assertEqual(parser.buffer, b"")
        self.assertEqual(parser.feed(b'{"type":"fc","armed":false}\n')[0][0], "telemetry")

    def test_nonfinite_json_is_not_published_as_a_number(self):
        events = BoardFrames().feed(b'{"type":"flow","height_cm":NaN}\n')
        self.assertIsNone(events[0][1]["height_cm"])


class BoardWifiTests(unittest.TestCase):
    def test_missing_attitude_does_not_refresh_the_last_valid_sample(self):
        service = BoardWifiService(now=lambda: 11)
        service._record({"type": "approach", "att": [3.5, -2.0]}, 10)
        service._record({"type": "approach", "att": [None, None]}, 10.5)
        sample = service.snapshot()["latest"]["attitude_rp"]
        self.assertEqual(sample["data"], {"roll_deg": 3.5, "pitch_deg": -2.0})
        self.assertEqual(sample["age_ms"], 1000)

    def test_serialized_attitude_units_and_freshness(self):
        now = [10.0]
        service = BoardWifiService(now=lambda: now[0])
        service.state = "connected"
        service._record({"type": "fcmsp", "cmd": 108, "ok": True, "error": False,
                         "hex": struct.pack("<3h", 123, -45, 271).hex()}, now[0])
        now[0] += .4
        entry = service.snapshot()["latest"]["attitude"]
        self.assertEqual(entry["data"], {"roll_deg": 12.3, "pitch_deg": -4.5, "yaw_deg": 271})
        self.assertAlmostEqual(entry["age_ms"], 400)
        service._record({"type": "fcmsp", "cmd": 108, "ok": False, "error": False, "hex": "00"}, now[0])
        self.assertEqual(service.snapshot()["latest"]["attitude"]["at_ms"], 10000)

    def test_private_address_and_command_allowlist(self):
        self.assertEqual(BoardWifiService.validate_host("192.168.0.151"), "192.168.0.151")
        for address in ("8.8.8.8", "169.254.169.254", "127.0.0.1:3333", "0.0.0.0", "host\nmsp on"):
            with self.assertRaises(HardwareError):
                BoardWifiService.validate_host(address)
        service = BoardWifiService()
        for command in (b"msp on\n", b"fcmsp 200\n", b"arm\n", b"simh 100\n", b"approach on\n"):
            with self.assertRaises(ValueError):
                service._send(command)

    def test_old_firmware_does_not_automatically_request_snapshots(self):
        service = BoardWifiService(now=lambda: 10)
        for build, allowed in (("TAGVISION-20261010-34", False), ("unknown", False), ("TAGVISION-20261011-36", True)):
            service._record({"type": "info", "build": build, "camera": True}, 10)
            self.assertEqual(service._camera_allowed(10), allowed)
        self.assertFalse(service._camera_allowed(14))

    def test_streaming_is_only_enabled_for_supported_firmware(self):
        service = BoardWifiService()
        for build, supported in (("TAGVISION-20261011-36", False), ("unknown", False), ("TAGVISION-20261011-38", True)):
            service._record({"type": "info", "build": build, "camera": True}, 10)
            self.assertEqual(service._stream_supported(), supported)

    def test_supported_board_streams_binary_photos_without_one_shot_requests(self):
        server = socket.socket()
        server.bind(("127.0.0.1", 0))
        server.listen(1)
        commands, done = [], threading.Event()

        def board():
            conn, _ = server.accept()
            with conn:
                conn.settimeout(.1)
                buf = b""
                while not done.is_set():
                    try:
                        data = conn.recv(8192)
                        if not data:
                            break
                        buf += data
                    except socket.timeout:
                        continue
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        commands.append(line + b"\n")
                        if line == b"info":
                            conn.sendall(b'{"type":"info","build":"TAGVISION-20261011-38","camera":true,"sim":false}\n')
                        elif line == b"snap stream 200":
                            conn.sendall(b"SNAPB 4 320 240\nSNAPC 4\n\xff\xd8\xff\xd9\nSNAP_END\n")

        worker = threading.Thread(target=board)
        worker.start()
        service = BoardWifiService(connector=lambda _address, timeout: socket.create_connection(server.getsockname(), timeout), viewer_finder=lambda _: None)
        try:
            service.connect("127.0.0.1")
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline and service.snapshot()["frame"]["sequence"] == 0:
                time.sleep(.01)
            self.assertEqual(service.camera_frame(), b"\xff\xd8\xff\xd9")
            self.assertIn(b"snap stream 200\n", commands)
            self.assertNotIn(b"snap small\n", commands)
            self.assertTrue(set(commands).issubset(BoardWifiService.READ_COMMANDS))
        finally:
            done.set()
            service.disconnect()
            worker.join(timeout=2)
            server.close()

    def test_expired_or_disconnected_camera_is_not_served(self):
        now = [0.0]
        service = BoardWifiService(now=lambda: now[0])
        service.state, service.image = "connected", b"jpeg"
        service.frame["at"] = 0
        self.assertEqual(service.camera_frame(), b"jpeg")
        now[0] = 11
        self.assertIsNone(service.camera_frame())
        service.frame["at"] = 11
        service.state = "disconnected"
        self.assertIsNone(service.camera_frame())

    def test_socket_receives_real_protocol_and_only_sends_queries(self):
        server = socket.socket()
        server.bind(("127.0.0.1", 0))
        server.listen(1)
        commands = []
        done = threading.Event()

        def board():
            conn, _ = server.accept()
            with conn:
                conn.settimeout(.1)
                buf = b""
                while not done.is_set():
                    try:
                        data = conn.recv(8192)
                        if not data:
                            break
                        buf += data
                    except socket.timeout:
                        continue
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        commands.append(line + b"\n")
                        if line == b"info":
                            conn.sendall(b'{"type":"info","build":"TAGVISION-20261011-36","camera":true,"sim":false}\n')
                        elif line == b"fcmsp 108":
                            conn.sendall(b'{"type":"fcmsp","cmd":108,"ok":true,"error":false,"hex":"640000005a00"}\n')
                        elif line == b"snap small":
                            conn.sendall(b"SNAP_BEGIN 4 320 240 8\n/9j/2Q==\nSNAP_END\n")

        worker = threading.Thread(target=board)
        worker.start()
        service = BoardWifiService(connector=lambda _address, timeout: socket.create_connection(server.getsockname(), timeout))
        try:
            service.connect("127.0.0.1")
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                snap = service.snapshot()
                if "attitude" in snap["latest"] and snap["frame"]["sequence"] > 0:
                    break
                time.sleep(.02)
            self.assertEqual(snap["state"], "connected")
            self.assertIn("attitude", snap["latest"])
            self.assertGreater(snap["frame"]["sequence"], 0)
            self.assertTrue(set(commands).issubset(BoardWifiService.READ_COMMANDS))
            same_session = service.connect("127.0.0.1")["session"]
            self.assertEqual(same_session, snap["session"])
        finally:
            done.set()
            service.disconnect()
            worker.join(2)
            server.close()
        self.assertEqual(service.snapshot()["state"], "disconnected")


class ViewerBridgeTests(unittest.TestCase):
    def test_discovers_only_this_projects_viewer_for_the_same_board(self):
        listing = f"12 python /other/cam_view.py --wifi 192.168.0.151\n13 python {VIEWER} --wifi 192.168.0.151 --ap-log /tmp/run.jsonl --dump /tmp/camera.jpg\n"
        with patch("d4d_web.viewer_bridge.subprocess.run") as run:
            run.return_value.stdout = listing
            viewer = find_viewer("192.168.0.151")
            self.assertEqual(viewer, ViewerProcess(13, Path("/tmp/run.jsonl"), Path("/tmp/camera.jpg")))
            self.assertIsNone(find_viewer("192.168.0.152"))

    def test_partial_log_lines_age_and_truncation(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "telemetry.jsonl"
            path.write_text('{"t":1,"ap":{"type":"approach","att":[2,3]}}\n{"t":2,"fc":', encoding="utf-8")
            os.utime(path, (time.time()-5, time.time()-5))
            reader = ViewerReader(ViewerProcess(os.getpid(), path, None))
            events = reader.telemetry(100)
            self.assertLessEqual(events[0][1], 95)
            self.assertEqual(reader.telemetry(101), [])
            with path.open("a") as f:
                f.write('{"type":"fc","armed":false}}\n')
            self.assertEqual(reader.telemetry(102)[0][0]["type"], "fc")
            path.write_text('{"t":0,"ap":{"type":"approach"}}\n')
            self.assertEqual(reader.telemetry(103)[0][0]["type"], "approach")
            self.assertEqual(reader.generation, 2)

    def test_overlay_rewrites_do_not_make_an_old_photo_fresh(self):
        from PIL import Image
        with tempfile.TemporaryDirectory() as tmp:
            photo = Path(tmp) / "camera.jpg"
            Image.new("RGB", (320, 240)).save(photo)
            reader = ViewerReader(ViewerProcess(os.getpid(), None, photo))
            reader.frame_at = 50
            frame = reader.camera(100)
            self.assertEqual(frame[3], 50)
            self.assertIsNone(reader.camera(101))
            Image.new("RGB", (320, 240), (12, 34, 56)).save(photo)
            self.assertEqual(reader.camera(102)[3], 50)

    def test_existing_viewer_is_shared_without_opening_or_writing_a_socket(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "telemetry.jsonl"
            path.write_text('{"t":1,"ap":{"type":"approach","att":[2,3],"height_valid":true,"height_cm":37}}\n')
            viewer = ViewerProcess(os.getpid(), path, None)
            connector = unittest.mock.Mock(side_effect=AssertionError("기존 연결을 빼앗으면 안 됨"))
            service = BoardWifiService(connector=connector, viewer_finder=lambda _: viewer)
            try:
                service.connect("192.168.0.151")
                deadline = time.monotonic()+2
                while time.monotonic()<deadline and service.snapshot()["state"]!="connected":
                    time.sleep(.01)
                snap = service.snapshot()
                self.assertEqual(snap["source"], "camera_viewer")
                self.assertEqual(snap["latest"]["approach"]["data"]["height_cm"], 37)
                connector.assert_not_called()
            finally:
                service.disconnect()


class BoardApiTests(unittest.IsolatedAsyncioTestCase):
    async def test_api_reads_do_not_open_hardware_and_cross_site_requests_are_rejected(self):
        from d4d_web.api import app
        service = BoardWifiService()
        with patch("d4d_web.board_api.board_wifi", service), patch.object(service, "connect", return_value=service.snapshot()) as connect:
            async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://localhost") as client:
                response = await client.get("/api/hardware/wifi/status")
                self.assertEqual(response.status_code, 200)
                connect.assert_not_called()
                response = await client.post("/api/hardware/wifi/connect", json={"host": "192.168.0.151"}, headers={"Origin": "https://evil.example"})
                self.assertEqual(response.status_code, 403)
                connect.assert_not_called()
                response = await client.post("/api/hardware/wifi/connect", json={"host": "192.168.0.151", "command": "arm"})
                self.assertEqual(response.status_code, 422)
                response = await client.post("/api/hardware/wifi/connect", json={"host": "192.168.0.151"}, headers={"Origin": "http://127.0.0.1:5173"})
                self.assertEqual(response.status_code, 200)
                connect.assert_called_once_with("192.168.0.151")
                self.assertEqual((await client.get("/api/hardware/wifi/camera.jpg")).status_code, 503)


if __name__ == "__main__":
    unittest.main()
