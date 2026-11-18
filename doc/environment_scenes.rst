Environment Scenes
==================

Select the initial environment with ``environment_scene``::

   ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py \
     use_manipulator_hardware:=false use_vehicle_hardware:=false \
     sim_robot_count:=1 task:=interactive environment_scene:=pipeline

During an interactive simulation, choose a scene directly from the robot's
**Environment** menu. ``default`` is displayed as **Terrain**.

=====================  ==================================
Scene                  Content
=====================  ==================================
default                Terrain and shipwreck
pipeline               Two pipeline sections
pipe_tank              Wave tank, containers and pipes
lake                   Charging structures and towers
ancient_ship           Wooden shipwreck
modern_ship            Metal shipwreck
tower                  Multipart obstacle tower
turbines               Turbine assemblies and anodes
wind_farm              Three turbine assemblies
=====================  ==================================

Scene switching
---------------

Switching replaces the shared environment in interactive simulation, hardware,
and mixed launches. Old plans are cancelled and feedback control retains the
current pose while geometry is prepared off the control callback thread. Robot
poses, estimators, control authorization and simulation pause state are preserved.
Use interactive feedback control rather than joystick or command replay during
a scene change. Resetting a simulated robot remains a separate explicit action. If a transition
fails, new scene-dependent requests remain blocked; correct the reported problem
and select a scene again. Existing feedback control continues.

The service ``/environment_scene/command`` accepts ``switch`` with a scene name,
or ``status``. Acceptance is not completion: wait for ``state=ready`` on
``/environment_scene/status``. Replaying recorded scene status does not switch scenes.

Adding a scene
--------------

Place bundles in ``description/scenes/<name>``, or pass an absolute bundle path.
A bundle's ``world.xacro`` defines ``scene_environment(world_frame,
asset_directory)``, static ``bathymetry_`` links, and a ``world_bottom`` floor
reference. Use metres and consistent visual/collision geometry.

Robots and lighting in the shared scene
---------------------------------------

The composite robot description supplies robot geometry, articulated link
ownership, and the Lumen emitter frames. Ownership follows the joint tree:
arm and gripper base links belong to the vehicle, not separate robots.
Camera rendering uses physical simulation TF; planning uses operational TF.
These are deliberately different state sources for the same scene entities.

One obstacle publisher composes scripted obstacles and robot sphere proxies on
``/dynamic_obstacles``. Each record carries ``source``, ``owner_robot``, sample
``stamp`` and ``velocity_valid``. Missing robot poses are listed explicitly in
``unavailable_robot_owners``; they are not silently treated as free space.
Editable profiles and ``/obstacle_scene`` contain only scripted obstacles.
Clearing or replacing those obstacles never removes robot entries.

Vehicle planning requests include ``robot_prefix``. Planning and trajectory
checks exclude that owner explicitly on each query. Other
robots use the same obstacle representation and query path as scripted objects.
Planning uses an admitted snapshot with fixed poses and sample ages during the
solve. Execution monitors use live snapshots; stale or unavailable data blocks
clearance checks. Valid TF-derived robot velocities support constant-velocity
prediction, not coordinated multi-robot trajectory prediction.

Contact visualization uses detailed robot meshes for terrain/scripted contacts
and one sphere pair for each inter-robot contact, without self-proxy collisions.
MCAP records the composed snapshot. Replay editing filters generated robot
entries so a recorded robot pose cannot become an editable obstacle or command.

The PyVista camera renderer attaches four lamps to each robot's ``lumen_*_link``
frames. Their mounts are defined in ``description/xacro/urdf.xacro``. A missing
emitter transform disables its lamp until the transform is available. Scene
replacement recreates the lights. All scene mesh actors receive scene lighting,
including robots and dynamic obstacles. Forward lamps illuminate surfaces in
their beams; they do not guarantee illumination of their own vehicle behind them.
Visuals preserve embedded mesh colors unless an explicit inline or named URDF
material supplies a color. This applies to robot and environment meshes alike.
PyVista uses the PR lighting settings: lamp intensity 10, cone angle 67.5 degrees,
exponent 5, and attenuation (1, 0.1, 0.05). Scripted obstacle material lighting
uses ambient/diffuse/specular coefficients (0.35, 0.65, 0.12); URDF visuals and
water retain (0, 0.25, 0.1). Missing emitter poses
set lamp intensity to zero while retaining the lighting pass, preventing VTK's
no-active-light rendering from displaying bright, unlit surfaces.

These lights affect simulated camera images, not RViz's independent RobotModel
renderer. The Open3D backend retains its existing lighting setup.

Robot navigation spheres are also published on ``/dynamic_voxels_cloud``.
Voxels consume the composed snapshot's geometry and pose directly; they do not
reconstruct robot proxies from TF. This global cloud shows all robots, while
planning excludes its own sphere. The fixed radius is shared with launch/spawn
validation and the navigation approximation; current-pose resizing is not enabled.
Detailed camera meshes remain tied to the same robot identities and physical TF;
collision-only robot records are not rendered as opaque spheres over the robots.

Camera source validity and lighting scope
-----------------------------------------

Camera robot geometry, lamps, and camera poses use physical simulation TF only.
A missing, future-dated, or older-than-``physical_feedback_timeout`` (default
0.5 s) robot transform hides that geometry and extinguishes its lamp. A stale
camera pose produces no new image. Static environment transforms do not expire.
Scripted obstacle visuals require a current composed snapshot; source age plus
elapsed reception time must remain within ``obstacle_snapshot_timeout`` (default
1 s). Expired or invalid snapshots hide their visuals until valid data return.
This preserves the distinction between physical camera state and operational
collision estimates without silently rendering unavailable state as current.

The proximity-light behavior is supported and regression-tested in PyVista.
Open3D remains selectable, but explicitly warns that its backend lighting is not
photometrically equivalent and does not implement the robot proximity lamps.
Lamp intensity, cone, exponent, and attenuation are centralized display settings,
not calibrated physical photometry. Underwater haze and tint are image effects,
not a water optical-path model. Embedded material base colors are sampled to
vertices; metallic, roughness, normal-map, and embedded-alpha effects are not
preserved. Explicit URDF and obstacle RGBA opacity is honored, including zero.

Dynamic motion supervision
--------------------------

``dynamic_replanning_enabled`` controls automatic non-preemptive replanning.
The replanner requests a replacement route when the remaining trajectory enters
the configured proactive clearance margin. It does not hold motion or
perform braking previews and automatic recovery.

Collision checks apply to the planner's path. Generated trajectories are not
collision-checked again before activation, so smoothing, tracking error and
obstacle motion can consume the planned clearance. Optional replanning uses
trajectory previews to request replacements without interrupting execution.
Operator stops and feedback-fault handling remain active.
