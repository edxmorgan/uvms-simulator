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
#   mesh:     optional; collision falls back to a bounding sphere/box if provided.
uint8 collision_type
float64[] collision_dimensions

# Visual geometry used by RViz and simulated camera renderers.
# If visual_type is GEOMETRY_MESH, visual_mesh_resource should be a package:// or file:// URI.
uint8 visual_type
float64[] visual_dimensions
string visual_mesh_resource
std_msgs/ColorRGBA color
