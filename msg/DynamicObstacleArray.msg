# Dynamic obstacle snapshot shared by planning, visualization, and simulated cameras.

std_msgs/Header header
DynamicObstacle[] obstacles

# Robot owners expected in the scene whose pose is currently missing/invalid/stale.
# Consumers may exclude their own owner, but must not treat missing others as free space.
string[] unavailable_robot_owners
