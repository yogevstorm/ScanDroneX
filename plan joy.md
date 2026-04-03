# goal
Create a new joy teleop package that receives joystick data from the joy topic and publishes command messages to the /simple_drone/cmd_vel topic.

## Q&A

Existing code: There's already a teleop_joystick.py in sjtu_drone_control and a joy_teleop.yaml config. Should this be a brand new standalone package, or an enhancement/replacement of what already exists?
a: brand new standalone package
Package type: Should this be a Python ROS 2 package, or C++ (or either)?
a: c++ ros2
Joystick mapping: Which joystick axes/buttons should map to which velocity components (linear x/y/z, angular z)? Do you have a specific controller in mind (e.g., PS4, Xbox, generic)?
a: axes[1] - linear.x, axes[0] - linear.y, axes[3] - linear.z, axes[2] - angular.z
Velocity limits: Should the node apply any scaling/clamping to the output velocities, or just pass through raw axis values?
a: do linearization and clamping. for value 1.0 in the axes do some maximum velocity
Mode switching: Should any buttons toggle between modes (e.g., enable/disable, takeoff/land, speed boost)?
a: yea button[0] - takeoff, button[1] - land
Topic names: The goal mentions /simple_drone/cmd_vel — is this the only output topic, or should it also handle takeoff/land/emergency commands on separate topics?
a: also publish takeoff/land commands