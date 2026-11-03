import copy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from bringup.camera_destination import CameraDestination


def stream(name="Front camera"):
    return {"video_and_stream": {
        "name": name,
        "video_source": {"Local": {"device_path": "/dev/video2"}},
        "stream_information": {
            "endpoints": ["udp://192.168.2.138:5600"],
            "configuration": {"type": "video", "encode": "H264", "width": 1280,
                              "height": 720, "frame_interval": {"numerator": 1, "denominator": 30}},
            "extended_configuration": {"thermal": False, "disable_mavlink": False},
        },
    }}


class CameraDestinationTests(unittest.TestCase):
    def setUp(self):
        self.manager = CameraDestination("http://192.168.2.2:6020")
        self.streams = [stream()]
        self.writes = []
        self.fail_post = False
        self.timeout_after_post = False
        self.manager.local_ip = lambda: "192.168.2.1"
        self.manager.backup = lambda original: "/tmp/test-backup.json"
        self.remembered = None
        self.manager.previous_endpoint = lambda name: self.remembered
        self.manager.remember_endpoint = self.remember_endpoint
        self.manager.request = self.request

    def remember_endpoint(self, name, endpoint):
        self.remembered = endpoint

    def request(self, path, method="GET", payload=None):
        if method == "GET":
            return copy.deepcopy(self.streams)
        self.writes.append((method, copy.deepcopy(payload)))
        if method == "DELETE":
            self.streams = []
        elif method == "POST":
            if self.fail_post:
                self.fail_post = False
                raise RuntimeError("Failed to create stream")
            restored = stream(payload["name"])
            restored["video_and_stream"]["stream_information"] = copy.deepcopy(payload["stream_information"])
            self.streams = [restored]
            if self.timeout_after_post:
                self.timeout_after_post = False
                raise TimeoutError("Reply lost after successful POST")

    def test_preserves_viewers_and_settings_and_is_idempotent(self):
        original = copy.deepcopy(self.streams[0])
        endpoint, changed, _ = self.manager.ensure()
        self.assertTrue(changed)
        expected = original["video_and_stream"]["stream_information"]
        expected["endpoints"].append(endpoint)
        self.assertEqual(self.streams[0]["video_and_stream"]["stream_information"], expected)
        self.assertFalse(self.manager.ensure()[1])
        self.assertEqual(len(self.writes), 2)

    def test_new_route_adds_new_address(self):
        self.manager.ensure()
        self.manager.local_ip = lambda: "192.168.2.42"
        self.assertEqual(self.manager.ensure()[0], "udp://192.168.2.42:5600")
        self.assertEqual(self.streams[0]["video_and_stream"]["stream_information"]["endpoints"],
                         ["udp://192.168.2.138:5600", "udp://192.168.2.42:5600"])

    def test_jpeg_registration_preserves_encoding_and_resolution(self):
        config = self.streams[0]["video_and_stream"]["stream_information"]["configuration"]
        config.update(encode="MJPG", width=640, height=480)
        expected = copy.deepcopy(config)
        self.manager.ensure()
        selected = self.manager.select(self.streams)
        self.assertEqual(selected["video_and_stream"]["stream_information"]["configuration"], expected)
        self.assertIn("rtpjpegdepay ! jpegdec", self.manager.receiver_pipeline(selected))
        self.assertFalse(self.manager.ensure()[1])

    def test_receiver_matches_codec(self):
        self.assertIn("rtph264depay", self.manager.receiver_pipeline(stream()))
        unsupported = stream()
        unsupported["video_and_stream"]["stream_information"]["configuration"]["encode"] = "YUYV"
        with self.assertRaises(ValueError):
            self.manager.receiver_pipeline(unsupported)
        with self.assertRaises(RuntimeError):
            self.manager.select([unsupported])

    def test_failed_restart_restores_original(self):
        original = copy.deepcopy(self.streams)
        self.fail_post = True
        with self.assertRaisesRegex(RuntimeError, "recovery copy"):
            self.manager.ensure()
        self.assertEqual(self.streams, original)

    def test_lost_reply_does_not_destroy_successful_update(self):
        self.timeout_after_post = True
        with self.assertRaises(RuntimeError):
            self.manager.ensure()
        self.assertFalse(self.manager.ensure()[1])
        self.assertEqual(len(self.writes), 2)

    def test_multiple_cameras_require_selection(self):
        self.streams.append(stream("Rear camera"))
        with self.assertRaisesRegex(RuntimeError, "camera_stream_name"):
            self.manager.ensure()
        self.assertEqual(self.writes, [])
        self.manager.stream_name = "Rear camera"
        self.assertEqual(self.manager.select(self.streams)["video_and_stream"]["name"], "Rear camera")

    def test_offline_vehicle_is_not_modified(self):
        with patch.object(self.manager, "request", side_effect=TimeoutError):
            with self.assertRaises(TimeoutError):
                self.manager.ensure()
        self.assertEqual(self.writes, [])

    def test_backup_failure_prevents_deletion(self):
        with patch.object(self.manager, "backup", side_effect=OSError("Disk full")):
            with self.assertRaises(OSError):
                self.manager.ensure()
        self.assertEqual(self.writes, [])

    def test_pending_recovery_after_outage(self):
        original = self.manager.payload(self.streams[0])
        self.manager.pending_restore = original
        self.streams = []
        self.manager.ensure()
        self.assertEqual(self.writes[0], ("POST", original))
        self.assertIsNone(self.manager.pending_restore)

    def test_managed_address_survives_helper_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch("bringup.camera_destination.Path.home", return_value=Path(directory)):
                first = CameraDestination("http://192.168.2.2:6020")
                first.remember_endpoint("Front camera", "udp://192.168.2.1:5600")
                restarted = CameraDestination("http://192.168.2.2:6020")
                self.assertEqual(restarted.previous_endpoint("Front camera"), "udp://192.168.2.1:5600")
                self.assertIsNone(restarted.previous_endpoint("Rear camera"))


if __name__ == "__main__":
    unittest.main()
