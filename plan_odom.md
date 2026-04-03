# goal
I have two odometry sources: one derived from drone command velocities published on the /odom_cmd_vel topic, and another obtained from LiDAR data published on the /odom_rf2o topic. I want to fuse these two odometry sources using a Kalman filter to produce a single, unified odometry output on the /odom_kalman topic, and broadcast the corresponding odom_kalman frame.

## Q&A
What are the message types on /odom_cmd_vel and /odom_rf2o? Both nav_msgs/Odometry?
a: Both nav_msgs/Odometry
What fields do you actually want to fuse — pose (x, y, yaw), twist (vx, vy, vyaw), or both?
a: both
Do you have any covariance information set in either source, or should we define it manually?
a: define it manually
About the filter design:
4. What state vector do you want? Minimal (x, y, yaw) or full 6-DOF? Is the drone operating in 2D or 3D?
a: Minimal (x, y, yaw)
5. Do you want a simple linear Kalman filter, or an Extended Kalman Filter (EKF)? (EKF is more appropriate if the motion model is nonlinear, e.g. heading-dependent)
a: EKF
6. Which source do you trust more — command velocities (/odom_cmd_vel) or LiDAR odometry (/odom_rf2o)? This affects the process/measurement noise tuning.
a: well most of the time i trust more the LiDAR odometry. but in long corridors I will trust more in the command velocities.
About the output:
7. Should /odom_kalman publish at a fixed rate, or be triggered on incoming messages?
a: fixed rate
8. Should the odom_kalman TF frame be broadcast relative to odom or map?
a: well the chile frame is base_link and the parent frame is the odom_kalman
About the codebase:
9. Is there existing ROS node infrastructure (package, CMakeLists, etc.) I should add this into, or should I create a new node from scratch?
a: create a new node from scratch
10. What language — Python or C++?
a: C++
