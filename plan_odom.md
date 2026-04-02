# goal
I can control the drone using cmd_vel commands. The goal is to use these commands to generate odometry for the drone and broadcast an odometry frame named odom_cmd_vel.


## Q&A
1. Frame names clarification (Q5)
In ROS odometry convention, frame_id is the fixed reference frame and child_frame_id is the moving robot frame. Based on your answer, did you mean:

frame_id = odom_cmd_vel (the new odometry reference frame)
child_frame_id = base_link (the robot body)
...and a TF broadcast of odom_cmd_vel → base_link? Or did you literally mean the reversed order?
a: the reversed order

2. Launch file integration (Q7 — unanswered)
Should the node be added to an existing launch file (e.g., scandronex_sim.launch.py), or just built standalone for you to wire in manually?
a: built as standalone