Arm simulator and augmented disturbance EKF
===========================================

The four arm coordinates use the plant's nominal rigid-body, gravity, friction,
payload and lock equations. The gripper retains its separate scalar filter.
The augmented state is position (4), velocity (4), and external joint torque (4),
in radians, rad/s and Nm. Applied actuator torque is after current conversion
and saturation; injected external load goes to the plant only.

Timing and integration
----------------------

Each posterior uses the preceding completed write's input, lock mask, contact
flags and elapsed time. Repeated reads do not assimilate observations twice.
Reset and pause preserve this sequence.

``arm_time_integration.hpp`` divides long intervals into chunks no longer than
10 ms, also respecting the plant's lock-decay stability bound. It integrates
all elapsed time, propagates covariance between chunks, and corrects once at
the actual endpoint. There are no synthetic intermediate measurements. Gaps
above five seconds or excessive work require reset. Catch-up integration cannot
prevent operating-system stalls or reconstruct commands never executed.

The plant retains its 30-element RK4 discretization and gripper equation,
using a shared reduced mass solve instead of the original repeated inverses.
The original binary remains available as a numerical regression oracle. The EKF uses four RK4
elements per chunk for free motion and analytic decay for locked coordinates.
Free accelerations use a reduced mass-matrix solve with locked accelerations
prescribed as -alpha times velocity, avoiding cancellation. Automatic
differentiation covers states/inputs, not the 81 fixed parameters. Covariance
correction uses Joseph form. Original Q/R and controller gains are unchanged.

Constraints and observability
-----------------------------

Artificial lock hysteresis uses applied actuator torque: unlock above 0.10 Nm,
lock below 0.05 Nm, unless disabled by ``arm_joint_locks:=false``. A hidden load
on a locked joint cannot be determined from its fixed position. Its disturbance
mean is retained, uncertainty continues to grow, and its observable flag is
false. This is intentional and does not imply zero load.

Joint limits come from Xacro. The plant applies its existing unilateral clamp
at the completed write endpoint. Actual contact censors that coordinate's free
motion observation; reaction is not interpreted as external load. Free-coordinate
evidence and correlated covariance are preserved. Controller admission is
conservative during contact. Numerical validity does not imply observability.

The arm GPIO publishes the complete 4-by-4 posterior disturbance covariance as
``estimated_disturbance.covariance.i.j`` in edcb coordinate order, in Nm squared.
The existing per-joint variances remain available. Equivalent tool-wrench
inference uses the observable principal submatrix, including its correlations;
locked/contact rows are still excluded. Vehicle-arm cross-covariance is not
available, so their fusion explicitly retains an independence approximation.
Posterior covariance is model-dependent, not a calibrated accuracy guarantee.

The hardware adapter uses the same generated filter with fresh, bounded-skew
sample timing and contact guards. It remains observation-only. Simulation tests
do not validate physical torque calibration or mechanical locking assumptions.

Reproduction
------------

The canonical generator lives in Floating-KinDyn-Graph's existing
``usage/joint_ekf.ipynb``. Run that notebook from its ``usage`` directory.
The augmented arm EKF cells generate the functions and shared library into
``build/arm_disturbance_ekf`` by default; set ``ARM_EKF_OUTPUT_DIR`` before
running the notebook to choose another output directory.

Copy the generated ``libmDisturbanceEKF_next.so`` into this package's
``casadi_lib`` and rebuild. Install the generated API and C++ adapter together.
Run ``test_arm_disturbance_ekf`` and ``test_hardware_arm_observer``. These cover
lock combinations, derivatives, contact/release, known loads, reset sequencing,
sample freshness and extended integration gaps.
