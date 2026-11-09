"""Rate-request lifecycle checks without connecting to vehicle hardware."""

from unittest.mock import Mock, patch

from rclpy.task import Future
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval

from bringup.mavros_telemetry import MavrosTelemetry


def helper():
    node = Mock()
    node.rates = [(30, 200.0), (137, 10.0), (27, 200.0)]
    node.refresh_s = 60.0
    node.connected = True
    node.state_received = 100.0
    node.pending = None
    node.pending_since = 0.0
    node.index = 0
    node.next_attempt = 0.0
    node.client.service_is_ready.return_value = True
    node.client.call_async.side_effect = lambda request: Future()
    node.reset_requests.side_effect = lambda: MavrosTelemetry.reset_requests(node)
    return node


def tick(node, now=100.0):
    with patch('bringup.mavros_telemetry.time.monotonic', return_value=now):
        MavrosTelemetry.tick(node)


def test_rates_and_refresh():
    node = helper()
    for index in range(3):
        tick(node)
        request = node.client.call_async.call_args.args[0]
        assert (request.message_id, request.message_rate) == node.rates[index]
        node.pending.set_result(MessageInterval.Response(success=True))
        tick(node)
    tick(node)
    assert node.client.call_async.call_count == 3
    node.state_received = 160.0
    tick(node, 160.0)
    assert node.client.call_async.call_count == 4
    assert node.client.call_async.call_args.args[0].message_id == 30


def test_rejection_and_timeout_retry_same_request():
    node = helper()
    tick(node)
    node.pending.set_result(MessageInterval.Response(success=False))
    tick(node)
    tick(node, 102.0)
    assert node.client.call_async.call_count == 1
    tick(node, 103.0)
    pending = node.pending
    node.state_received = 109.0
    tick(node, 109.0)
    assert pending.cancelled()
    assert node.index == 0
    node.state_received = 112.0
    tick(node, 112.0)
    assert node.client.call_async.call_count == 3


def test_disconnect_and_reconnect():
    node = helper()
    tick(node)
    pending = node.pending
    MavrosTelemetry.on_state(node, State(connected=False))
    assert pending.cancelled()
    tick(node)
    assert node.client.call_async.call_count == 1
    with patch('bringup.mavros_telemetry.time.monotonic', return_value=101.0):
        MavrosTelemetry.on_state(node, State(connected=True))
    tick(node, 101.0)
    assert node.client.call_async.call_count == 2
    assert node.client.call_async.call_args.args[0].message_id == 30


def test_missing_service_or_stale_state_does_not_send():
    node = helper()
    node.client.service_is_ready.return_value = False
    tick(node)
    node.client.service_is_ready.return_value = True
    tick(node, 106.0)
    node.client.call_async.assert_not_called()
