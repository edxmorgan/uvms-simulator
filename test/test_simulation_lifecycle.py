"""Mixed deployments pause and release only their simulated backends."""
from itertools import product
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bringup.simulation_lifecycle import simulated_components, startup_paused
from bringup.sim_reset_coordinator import SimResetCoordinator


@pytest.mark.parametrize('vehicle,arm', list(product([False, True], repeat=2)))
def test_backend_capabilities_and_release(vehicle, arm):
    capabilities = simulated_components(['robot_real_', 'robot_1_'],
        vehicle_hardware=vehicle, manipulator_hardware=arm)
    assert capabilities['robot_1_'] == ['manipulator', 'vehicle']
    expected = ([] if arm else ['manipulator']) + ([] if vehicle else ['vehicle'])
    assert capabilities['robot_real_'] == expected
    calls = []
    logger = SimpleNamespace(info=lambda _: None, error=lambda _: None)
    fake = SimpleNamespace(_simulated_components=capabilities,
        _release_manipulator_clients={'robot_real_': 'manipulator'},
        _release_vehicle_clients={'robot_real_': 'vehicle'},
        get_logger=lambda: logger,
        _call_trigger=lambda client, name: (calls.append(client) is None, 'ok'))
    response = SimResetCoordinator._handle_release(fake, 'robot_real_', SimpleNamespace())
    assert response.success
    assert calls == expected


@pytest.mark.parametrize('task', ['interactive', 'manual', 'coverage'])
def test_startup_policy(task):
    assert startup_paused(task, 'auto') == (task == 'interactive')
    assert startup_paused(task, 'false') is False
    if task == 'interactive':
        assert startup_paused(task, 'true') is True
    else:
        with pytest.raises(ValueError):
            startup_paused(task, 'true')


def test_invalid_option_rejected():
    with pytest.raises(ValueError):
        startup_paused('interactive', 'typo')
