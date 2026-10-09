# Real-arm serial traffic budget

The real Reach hardware interface caps feedback polling at 20 cycles/s and
command transmission at 50 batches/s. The ROS controller-manager rate remains
unchanged. Requested polling above 20 Hz is clamped with an effective-rate log;
nonpositive polling is rejected. Simulation-only hardware is unchanged.

At 115200 baud with 8N1 framing, each direction carries 11520 bytes/s.
Each polling cycle requests three values on five joints: 15 seven-byte requests,
with fifteen ten-byte float replies. At 20 Hz this uses 2100 B/s outgoing and
3000 B/s incoming. Five ten-byte commands at 50 Hz use 2500 B/s outgoing.
Even if all five zero-current commands require their existing zero-velocity
follow-up, regular outgoing traffic is bounded to 7100 B/s, leaving about 38%
for other packets. This budget covers this interface's regular traffic, not
arbitrary additional clients or firmware-specific extra messages.

The command limiter uses monotonic host time. Calls inside the 20-ms interval
send nothing; the next batch reads the latest command interfaces, without a
software queue or catch-up burst. Standby/disable lifecycle commands are not
rate-limited by this gate. Normal zero commands can wait until the next batch;
this software path is not an emergency stop. Serial writes are serialized
across polling, command, and heartbeat threads, and partial writes/EINTR are
handled without interleaving packet bytes.

Restart the entire ROS hardware launch after rebuilding, not only the interactive
controller. Expect a startup message reporting effective polling=20 Hz and
command batches <=50 Hz. Validate actual command/current-feedback lag in one
supervised recording before collecting experiments. Link-budget tests do not
establish the achieved latency, physical reset convergence, or tracking stability.
The archived older datasets used different communication settings; report that
protocol change when comparing results.

## Command ownership and feedback safety

The real hardware now validates command-interface claims in prepare, without
changing actuator state. Perform applies only the affected joint changes,
sends zero current/velocity on transitions, and clears previous current/effort
commands. Stopping a claim disables that joint's host output. Switching unrelated
vehicle controllers no longer puts all arm joints into effort mode.

Only effort and current interfaces are supported by this real-hardware claim
handler. Position, velocity, acceleration and computed_effort command claims
are rejected explicitly. Native position/velocity operation needs a separately
validated firmware-limit configuration; it must not be inferred from exported
interfaces. No firmware limits or firmware tuning are changed here.

Actual valid serial packets timestamp position, velocity and current separately
using the monotonic clock. ROS publications/read cycles cannot refresh these
timestamps. Claimed joints must have all three signals received within 0.5 s.
During the first 0.5 s after hardware activation, missing feedback suppresses
motion commands. Thereafter missing/stale feedback sends zero current/velocity
to all joints, latches a fault and returns hardware ERROR to controller_manager.
It does not automatically resume motion on recovery; hardware reactivation is
required. This is a software safeguard, not a replacement for the hardware stop.
Non-finite effort/current commands also send zero instead of retaining the
previous command. Torque/current conversion arguments are local to each call,
not shared between hardware read and write paths.

## Limits of the diagnosis

The latest pre-change reset requested about -0.40 Nm on E, with roughly -36 mA
reported current. E remained near 3.300 rad. This is consistent with the loaded
torque/current map below its configured static-compensation threshold; it is
not evidence that E's configured position limit blocked the target. Earlier
resets also showed little E motion at larger reported currents, so the lower
current alone does not explain all observed stalls. Do not increase gains or
current caps on that basis.

The current driver subscribes to position, velocity, current and heartbeat,
not a decoded actuator fault/firmware-limit report. ROS controller diagnostics
cannot establish physical actuator health. An on-site, supervised actuator
check remains necessary if E does not move normally after these software fixes.
