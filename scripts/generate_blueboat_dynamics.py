#!/usr/bin/env python3
"""Generate and check BlueBoat's planar CasADi dynamics shared library.

Run with the same Python/CasADi environment used by the ROS workspace:
    python3 generate_blueboat_dynamics.py --output-dir /path/to/casadi_lib

Function: BlueBoatNext_reg(state12, wrench6, parameters33, dt, external6)
Output:   next_state12, in the existing vehicle interface's state order.

State: [x, y, z, roll, pitch, yaw, u, v, w, p, q, r].
Wrenches are body-frame [Fx, Fy, Fz, Tx, Ty, Tz] in the internal NED convention.
Parameter order matches pack_vehicle_dynamics() in the hardware adapter.

The model has diagonal effective inertia, linear/quadratic damping, and planar
Coriolis coupling. It preserves the input height and constrains roll, pitch,
w, p, and q to zero. The caller supplies the allocated, actuator-limited wrench;
this function adds external surge/sway/yaw loads, then integrates the motion.

Contract for the caller: finite inputs; positive m_x_du, m_y_dv, i_z_n_dr;
nonpositive active damping coefficients; zero current; 0 <= dt <= 1 second.
The existing C++ profile/state validation must continue to enforce these rules.
This generator does not read or bake a dynamics profile into the library.
"""

import argparse
import math
import shutil
import subprocess
import tempfile
from pathlib import Path

import casadi as ca
import numpy as np


FUNCTION_NAME = "BlueBoatNext_reg"
LIBRARY_NAME = "libBlueBoat_xnext.so"
ACTIVE = [0, 1, 5, 6, 7, 11]
PARAMETER_NAMES = (
    "m_x_du", "m_y_dv", "m_z_dw", "mz_g_x_dq", "mz_g_y_dp",
    "mz_g_k_dv", "mz_g_m_du", "i_x_k_dp", "i_y_m_dq", "i_z_n_dr",
    "weight", "buoyancy", "x_g_weight_minus_x_b_buoyancy",
    "y_g_weight_minus_y_b_buoyancy", "z_g_weight_minus_z_b_buoyancy",
    "x_u", "y_v", "z_w", "k_p", "m_q", "n_r",
    "x_uu", "y_vv", "z_ww", "k_pp", "m_qq", "n_rr",
    "current_velocity_0", "current_velocity_1", "current_velocity_2",
    "current_velocity_3", "current_velocity_4", "current_velocity_5",
)
P = {name: index for index, name in enumerate(PARAMETER_NAMES)}


def build_model():
    """Use CasADi's fixed-step RK integrator, as in Dory's model generator."""
    state = ca.SX.sym("state", 12)
    wrench = ca.SX.sym("wrench", 6)
    parameters = ca.SX.sym("parameters", len(PARAMETER_NAMES))
    dt = ca.SX.sym("dt")
    external = ca.SX.sym("external_wrench", 6)
    planar = ca.SX.sym("planar", 6)
    yaw, u, v, r = planar[2], planar[3], planar[4], planar[5]

    mx = parameters[P["m_x_du"]]
    my = parameters[P["m_y_dv"]]
    iz = parameters[P["i_z_n_dr"]]
    load = wrench + external
    drag_x = (parameters[P["x_u"]] + parameters[P["x_uu"]] * ca.fabs(u)) * u
    drag_y = (parameters[P["y_v"]] + parameters[P["y_vv"]] * ca.fabs(v)) * v
    drag_n = (parameters[P["n_r"]] + parameters[P["n_rr"]] * ca.fabs(r)) * r

    # Negative hydrodynamic coefficients already give resisting forces here.
    rate = ca.vertcat(
        ca.cos(yaw) * u - ca.sin(yaw) * v,
        ca.sin(yaw) * u + ca.cos(yaw) * v,
        r,
        (load[0] + my * v * r + drag_x) / mx,
        (load[1] - mx * u * r + drag_y) / my,
        (load[5] + (mx - my) * u * v + drag_n) / iz,
    )

    # Integrate over normalized time [0, 1]; dt scales the physical ODE.
    # Thirty RK4 elements follow the published Dory generator's configuration.
    integrator_parameters = ca.vertcat(wrench, parameters, dt, external)
    integrate = ca.integrator(
        "blueboat_rk", "rk",
        {"x": planar, "p": integrator_parameters, "ode": dt * rate},
        0.0, 1.0,
        {"simplify": True, "number_of_finite_elements": 30},
    )
    result = integrate(x0=state[ACTIVE], p=integrator_parameters)["xf"]
    next_state = ca.vertcat(
        result[0], result[1], state[2], 0, 0, result[2],
        result[3], result[4], 0, 0, 0, result[5],
    )
    return ca.Function(
        FUNCTION_NAME, [state, wrench, parameters, dt, external], [next_state],
        ["state", "wrench", "parameters", "dt", "external_wrench"],
        ["next_state"],
    )


def reference_step(state, wrench, parameters, dt, external):
    """Independent numeric copy of the existing C++ RK4 motion calculation."""
    s = np.asarray(state, dtype=float)[ACTIVE].copy()
    mx, my, iz = parameters[0], parameters[1], parameters[9]
    load = np.asarray(wrench) + np.asarray(external)

    def rate(a):
        yaw, u, v, r = a[2:]
        return np.array([
            math.cos(yaw) * u - math.sin(yaw) * v,
            math.sin(yaw) * u + math.cos(yaw) * v,
            r,
            (load[0] + my * v * r + (parameters[15] + parameters[21] * abs(u)) * u) / mx,
            (load[1] - mx * u * r + (parameters[16] + parameters[22] * abs(v)) * v) / my,
            (load[5] + (mx - my) * u * v + (parameters[20] + parameters[26] * abs(r)) * r) / iz,
        ])

    steps = max(1, math.ceil(dt / 0.01))
    h = dt / steps
    for _ in range(steps):
        k1 = rate(s)
        k2 = rate(s + 0.5 * h * k1)
        k3 = rate(s + 0.5 * h * k2)
        k4 = rate(s + h * k3)
        s += h * (k1 + 2 * k2 + 2 * k3 + k4) / 6
    output = np.zeros(12)
    output[ACTIVE] = s
    output[2] = state[2]
    return output


def check_model(symbolic, compiled):
    """Check compiled evaluation, analytic motion, constraints, and coupling."""
    expected_shapes = [(12, 1), (6, 1), (33, 1), (1, 1), (6, 1)]
    if compiled.n_in() != 5 or compiled.n_out() != 1:
        raise RuntimeError("Unexpected generated function input/output count")
    if [compiled.size_in(i) for i in range(5)] != expected_shapes:
        raise RuntimeError("Generated input layout does not match the vehicle adapter")
    if compiled.size_out(0) != (12, 1):
        raise RuntimeError("Generated state output must be 12-by-1")

    parameters = np.zeros(33)
    parameters[[0, 1, 9, 15, 16, 20]] = [20, 30, 4, -10, -15, -2]
    zeros = np.zeros(6)

    def evaluate(state, wrench=zeros, dt=0.01, external=zeros, p=parameters):
        return np.asarray(compiled(state, wrench, p, dt, external)).reshape(12)

    # Both translation and rotation have closed-form solutions for these cases.
    for label, index, command_index, command in [
        ("forward", 6, 0, 10), ("reverse", 6, 0, -10), ("yaw", 11, 5, 2)
    ]:
        state = np.zeros(12)
        wrench = np.zeros(6)
        wrench[command_index] = command
        for _ in range(300):
            state = evaluate(state, wrench)
        steady = command / (10 if index == 6 else 2)
        expected_velocity = steady * (1 - math.exp(-0.5 * 3))
        expected_position = steady * (3 - (1 - math.exp(-0.5 * 3)) / 0.5)
        np.testing.assert_allclose(state[index], expected_velocity, rtol=0, atol=1e-9)
        np.testing.assert_allclose(state[0 if index == 6 else 5], expected_position, rtol=0, atol=1e-9)

    state = np.zeros(12)
    state[6] = 1
    for _ in range(300):
        state = evaluate(state)
    np.testing.assert_allclose(state[6], math.exp(-1.5), rtol=0, atol=1e-9)

    state = np.zeros(12)
    state[[2, 3, 4, 8, 9, 10]] = [0.17, 0.2, -0.3, 0.4, 0.5, -0.6]
    projected = evaluate(state, dt=0)
    np.testing.assert_array_equal(projected[[3, 4, 8, 9, 10]], np.zeros(5))
    np.testing.assert_equal(projected[2], 0.17)

    rng = np.random.default_rng(87)
    max_reference_error = 0.0
    for i in range(100):
        state = rng.uniform(-0.5, 0.5, 12)
        wrench = rng.uniform(-5, 5, 6)
        external = rng.uniform(-0.2, 0.2, 6)
        p = parameters.copy()
        p[[21, 22, 26]] = [-1, -2, -0.3]
        dt = [0, 0.004, 0.01, 0.02, 0.05][i % 5]
        actual = evaluate(state, wrench, dt, external, p)
        expression = np.asarray(symbolic(state, wrench, p, dt, external)).reshape(12)
        reference = reference_step(state, wrench, p, dt, external)
        np.testing.assert_allclose(actual, expression, rtol=1e-12, atol=1e-12)
        np.testing.assert_allclose(actual, reference, rtol=0, atol=1e-8)
        max_reference_error = max(max_reference_error, float(np.max(np.abs(actual - reference))))

    # With no damping or input, the planar Coriolis terms conserve kinetic energy.
    p = parameters.copy()
    p[[15, 16, 20]] = 0
    state = np.zeros(12)
    state[[6, 7, 11]] = [0.6, 0.2, 0.3]

    def energy(a):
        return 0.5 * (p[0] * a[6] ** 2 + p[1] * a[7] ** 2 + p[9] * a[11] ** 2)

    initial_energy = energy(state)
    for _ in range(100):
        state = evaluate(state, p=p)
    np.testing.assert_allclose(energy(state), initial_energy, rtol=1e-10, atol=1e-10)
    print("PASS: forward, reverse, yaw, coasting, planar constraints, and Coriolis energy check")
    print(f"PASS: 100 compiled/reference cases; maximum state difference {max_reference_error:.3e}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True,
                        help="Destination for the tested libBlueBoat_xnext.so")
    parser.add_argument("--cc", default="cc", help="C compiler executable (default: cc)")
    args = parser.parse_args()
    compiler = shutil.which(args.cc)
    if compiler is None:
        parser.error(f"C compiler not found: {args.cc}")

    print(f"Python CasADi: {ca.__version__}")
    model = build_model()
    with tempfile.TemporaryDirectory(prefix="blueboat_codegen_") as temporary:
        work = Path(temporary)
        generator = ca.CodeGenerator("blueboat_xnext.c")
        generator.add(model)
        generator.generate(str(work) + "/")
        library = work / LIBRARY_NAME
        subprocess.run([
            compiler, "-O3", "-std=c99", "-fPIC", "-shared",
            str(work / "blueboat_xnext.c"), "-o", str(library), "-lm",
        ], check=True)
        compiled = ca.external(FUNCTION_NAME, str(library))
        check_model(model, compiled)
        output = args.output_dir.expanduser().resolve()
        output.mkdir(parents=True, exist_ok=True)
        destination = output / LIBRARY_NAME
        shutil.copy2(library, destination)

    print(f"Created: {destination}")
    print("Function: BlueBoatNext_reg(state12, wrench6, parameters33, dt, external6) -> next_state12")
    print("The ROS hardware adapter still needs to be connected to this function.")


if __name__ == "__main__":
    main()