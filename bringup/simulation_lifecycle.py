"""Deployment capabilities and startup policy; independent of dynamics parameters."""
COMPONENTS = ('manipulator', 'vehicle')


def simulated_components(prefixes, *, vehicle_hardware, manipulator_hardware):
    """Match the hardware selection used to instantiate the robot description."""
    physical = {'vehicle': vehicle_hardware, 'manipulator': manipulator_hardware}
    return {prefix: [part for part in COMPONENTS
                     if not (prefix == 'robot_real_' and physical[part])]
            for prefix in prefixes}


def startup_paused(task, option):
    if option not in ('auto', 'true', 'false'):
        raise ValueError('sim_startup_pause must be auto, true or false')
    paused = task == 'interactive' if option == 'auto' else option == 'true'
    if paused and task != 'interactive':
        raise ValueError('sim_startup_pause requires interactive execution')
    return paused
