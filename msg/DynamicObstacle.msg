# Dynamic obstacle state shared by planning, visualization, and simulated cameras.

uint8 GEOMETRY_NONE=0
uint8 GEOMETRY_SPHERE=1
uint8 GEOMETRY_BOX=2
uint8 GEOMETRY_CYLINDER=3
uint8 GEOMETRY_MESH=4

string id

# Pose and twist are expressed in the frame_id of DynamicObstacleArray.header.
geometry_msgs/Pose pose
geometry_msgs/Twist twist

# Collision geometry used by planners and safety monitors.
# dimensions:
#   sphere:   [radius]
#   box:      [x, y, z]
#   cylinder: [radius, height]
#   mesh:     unused; provide collision_mesh_resource and collision_mesh_scale.
uint8 collision_type
float64[] collision_dimensions
# Mesh vertices are in obstacle-local coordinates, transformed by pose.
# No implicit fallback to visual geometry or a bounding proxy.
string collision_mesh_resource
float64[] collision_mesh_scale

# Visual geometry used by RViz and simulated camera renderers.
# If visual_type is GEOMETRY_MESH, visual_mesh_resource should be a package:// or file:// URI.
uint8 visual_type
float64[] visual_dimensions
string visual_mesh_resource
std_msgs/ColorRGBA color
