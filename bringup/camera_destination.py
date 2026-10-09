"""Register this ROS receiver with BlueOS's MAVLink Camera Manager."""

import argparse
import copy
import hashlib
import json
from pathlib import Path
import socket
import tempfile
import time
from urllib.parse import urlencode, urlsplit
from urllib.request import Request, build_opener, ProxyHandler


class CameraDestination:
    def __init__(self, api_url, stream_name="", port=5600, timeout=3.0):
        self.api_url = api_url.rstrip("/")
        parsed = urlsplit(self.api_url)
        if parsed.scheme not in ("http", "https") or not parsed.hostname:
            raise ValueError("camera_blueos_url must be an HTTP(S) camera-manager URL")
        if not 1 <= port <= 65535:
            raise ValueError("camera_udp_port must be between 1 and 65535")
        self.host = parsed.hostname
        self.api_port = parsed.port or (443 if parsed.scheme == "https" else 80)
        self.stream_name = stream_name
        self.port = port
        self.timeout = timeout
        # Vehicle-local traffic must not use a desktop HTTP proxy.
        self.opener = build_opener(ProxyHandler({}))
        self.pending_restore = None

    def request(self, path, method="GET", payload=None):
        body = None if payload is None else json.dumps(payload).encode()
        req = Request(self.api_url + path, data=body, method=method,
                      headers={"Content-Type": "application/json"})
        with self.opener.open(req, timeout=self.timeout) as response:
            data = response.read()
            return json.loads(data) if data else None

    def local_ip(self):
        # The kernel chooses the interface/address used to reach this vehicle.
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect((self.host, self.api_port))
            address = sock.getsockname()[0]
        if address.startswith("127.") or address == "0.0.0.0":
            raise RuntimeError("BlueOS route does not select a vehicle-facing IPv4 address")
        return address

    @staticmethod
    def payload(stream):
        video = stream["video_and_stream"]
        source = video["video_source"].get("Local", {}).get("device_path")
        if not source:
            raise RuntimeError("Automatic registration currently requires a local BlueOS camera")
        return {"name": video["name"], "source": source,
                "stream_information": copy.deepcopy(video["stream_information"])}

    def select(self, streams):
        if self.stream_name:
            candidates = [s for s in streams
                          if s["video_and_stream"]["name"] == self.stream_name]
        else:
            candidates = [s for s in streams
                          if s["video_and_stream"]["stream_information"]
                          .get("configuration", {}).get("encode") == "H264"
                          and "Local" in s["video_and_stream"]["video_source"]]
        if len(candidates) != 1:
            raise RuntimeError("Expected one H264 camera stream; found "
                               f"{len(candidates)}. Set camera_stream_name to select one.")
        selected = candidates[0]
        if selected["video_and_stream"]["stream_information"]["configuration"].get("encode") != "H264":
            raise RuntimeError("Selected stream is not H264, as required by the default receiver")
        return selected

    def backup(self, payload):
        directory = Path.home() / ".ros" / "camera_destination_backups"
        directory.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(mode="w", prefix="stream-", suffix=".json",
                                         dir=directory, delete=False) as out:
            json.dump({"api_url": self.api_url, "stream": payload}, out, indent=2)
            return out.name

    def state_path(self, name):
        key = hashlib.sha256((self.api_url + "\n" + name).encode()).hexdigest()[:24]
        return Path.home() / ".ros" / "camera_destination_backups" / f"managed-{key}.json"

    def previous_endpoint(self, name):
        try:
            return json.loads(self.state_path(name).read_text())["endpoint"]
        except FileNotFoundError:
            return None

    def remember_endpoint(self, name, endpoint):
        path = self.state_path(name)
        path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, delete=False) as out:
            json.dump({"endpoint": endpoint}, out)
        Path(out.name).replace(path)

    def ensure(self):
        streams = self.request("/streams")
        # Recover after a failed restart when the vehicle becomes reachable again.
        if self.pending_restore is not None:
            original = self.pending_restore
            if not any(s["video_and_stream"]["name"] == original["name"] for s in streams):
                self.request("/streams", "POST", original)
                streams = self.request("/streams")
            self.pending_restore = None
        selected = self.select(streams)
        original = self.payload(selected)
        endpoint = f"udp://{self.local_ip()}:{self.port}"
        previous = self.previous_endpoint(original["name"])
        endpoints = original["stream_information"]["endpoints"]
        remove_previous = previous and previous != endpoint and previous in endpoints
        if endpoint in endpoints and not remove_previous:
            self.remember_endpoint(original["name"], endpoint)
            return endpoint, False, None
        updated = copy.deepcopy(original)
        if remove_previous:
            updated["stream_information"]["endpoints"].remove(previous)
        if endpoint not in updated["stream_information"]["endpoints"]:
            updated["stream_information"]["endpoints"].append(endpoint)
        backup = self.backup(original)
        # Recheck just before the restart to avoid overwriting a concurrent UI edit.
        current = self.payload(self.select(self.request("/streams")))
        if current != original:
            raise RuntimeError("Camera settings changed during registration; retrying")
        self.pending_restore = original
        try:
            self.request("/delete_stream?" + urlencode({"name": original["name"]}), "DELETE")
            self.request("/streams", "POST", updated)
            actual = self.payload(self.select(self.request("/streams")))
            if endpoint not in actual["stream_information"]["endpoints"]:
                raise RuntimeError("BlueOS did not retain the requested destination")
        except Exception as error:
            # A timed-out POST might have succeeded: never delete that stream.
            try:
                remaining = self.request("/streams")
                if not any(s["video_and_stream"]["name"] == original["name"] for s in remaining):
                    self.request("/streams", "POST", original)
                self.pending_restore = None
            except Exception:
                pass  # Keep recovery pending for the next retry.
            raise RuntimeError(f"Camera registration failed; recovery copy: {backup}: {error}") from error
        self.pending_restore = None
        self.remember_endpoint(original["name"], endpoint)
        return endpoint, True, backup


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--api-url", default="http://192.168.2.2:6020")
    parser.add_argument("--stream-name", default="")
    parser.add_argument("--port", type=int, default=5600)
    parser.add_argument("--image-topic", default="/alpha/image_raw")
    args = parser.parse_args()
    manager = CameraDestination(args.api_url, args.stream_name, args.port)
    rclpy.init(args=[])
    node = Node("camera_destination_discovery")
    last_frame = None
    last_status = None
    started = time.monotonic()

    def received(msg):
        nonlocal last_frame
        if last_frame is None or time.monotonic() - last_frame > 15:
            node.get_logger().info(f"Camera frames received on {args.image_topic}: "
                                   f"{msg.width}x{msg.height} {msg.encoding}")
        last_frame = time.monotonic()

    def check():
        nonlocal last_status
        try:
            endpoint, changed, backup = manager.ensure()
            status = f"Camera destination registered: {endpoint}"
            if changed:
                node.get_logger().info(f"Added {endpoint}; previous settings saved to {backup}")
            if status != last_status:
                node.get_logger().info(status)
            if time.monotonic() - (last_frame or started) > 20:
                node.get_logger().warning(f"Destination registered but no recent frames on {args.image_topic}")
        except Exception as error:
            status = str(error)
            if status != last_status:
                node.get_logger().warning(f"Camera destination discovery will retry: {status}")
        last_status = status

    node.create_subscription(Image, args.image_topic, received, qos_profile_sensor_data)
    node.create_timer(10.0, check)
    try:
        check()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
