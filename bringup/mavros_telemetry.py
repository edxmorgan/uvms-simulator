"""Maintain requested telemetry rates; never sends actuator or arming commands."""

import math
import time

import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class MavrosTelemetry(Node):
    def __init__(self, **kwargs):
        super().__init__('mavros_telemetry', **kwargs)
        namespace = self.declare_parameter('mavros_namespace', '/mavros').value.rstrip('/')
        attitude = float(self.declare_parameter('attitude_rate_hz', 200.0).value)
        pressure = float(self.declare_parameter('pressure_rate_hz', 10.0).value)
        self.refresh_s = float(self.declare_parameter('refresh_seconds', 60.0).value)
        if not all(math.isfinite(x) and x > 0 for x in (attitude, pressure, self.refresh_s)):
            raise ValueError('Telemetry rates and refresh_seconds must be finite and positive')
        self.rates = [(30, attitude), (137, pressure)]
        self.client = self.create_client(MessageInterval, namespace + '/set_message_interval')
        self.connected = False
        self.state_received = 0.0
        self.pending = None
        self.pending_since = 0.0
        self.index = 0
        self.next_attempt = 0.0
        self.subscription = self.create_subscription(
            State, namespace + '/state', self.on_state, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0, self.tick)
        self.get_logger().info('Waiting for MAVROS connection to request attitude/pressure telemetry rates')

    def reset_requests(self):
        if self.pending is not None:
            self.client.remove_pending_request(self.pending)
            self.pending.cancel()
        self.pending = None
        self.index = 0
        self.next_attempt = 0.0

    def on_state(self, msg):
        connected = bool(msg.connected)
        if connected != self.connected:
            self.reset_requests()
        self.connected = connected
        self.state_received = time.monotonic()

    def tick(self):
        now = time.monotonic()
        if (not self.connected or now - self.state_received > 5.0
                or not self.client.service_is_ready()):
            self.reset_requests()
            return
        if self.pending is not None:
            if self.pending.done():
                try:
                    accepted = self.pending.result().success
                except Exception:
                    accepted = False
                self.pending = None
                if accepted:
                    self.index += 1
                    if self.index == len(self.rates):
                        self.get_logger().info(
                            f'Telemetry requests accepted: attitude={self.rates[0][1]:g} Hz, '
                            f'pressure={self.rates[1][1]:g} Hz (delivered rates depend on firmware)')
                        self.next_attempt = now + self.refresh_s
                else:
                    self.get_logger().warning('Telemetry rate request rejected; retrying in 3 seconds')
                    self.next_attempt = now + 3.0
                return
            if now - self.pending_since > 5.0:
                self.client.remove_pending_request(self.pending)
                self.pending.cancel()
                self.pending = None
                self.next_attempt = now + 3.0
                self.get_logger().warning('Telemetry rate request timed out; retrying in 3 seconds')
            return
        if now < self.next_attempt:
            return
        # Refresh also covers a fast autopilot/MAVROS restart whose disconnect
        # event was missed. Never modify persistent autopilot SRx parameters.
        if self.index == len(self.rates):
            self.index = 0
        mid, rate = self.rates[self.index]
        self.pending = self.client.call_async(
            MessageInterval.Request(message_id=mid, message_rate=rate))
        self.pending_since = now


def main():
    rclpy.init()
    node = MavrosTelemetry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.reset_requests()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
