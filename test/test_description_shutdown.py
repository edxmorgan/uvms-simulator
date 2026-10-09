from unittest.mock import Mock

import pytest
from rclpy.executors import ExternalShutdownException
from bringup import robot_description_publisher as publisher


@pytest.mark.parametrize('exception', [KeyboardInterrupt, ExternalShutdownException])
@pytest.mark.parametrize('still_ok', [True, False])
def test_shutdown_only_once(monkeypatch, exception, still_ok):
    node = Mock()
    ros = Mock()
    ros.spin.side_effect = exception()
    ros.ok.return_value = still_ok
    monkeypatch.setattr(publisher, 'rclpy', ros)
    monkeypatch.setattr(publisher, 'RobotDescriptionPublisher', lambda: node)
    publisher.main()
    node.destroy_node.assert_called_once()
    assert ros.shutdown.call_count == int(still_ok)
