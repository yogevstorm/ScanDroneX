#!/usr/bin/env python3
"""
Generate a Gazebo Harmonic SDF world file from an ASCII maze map.

Usage:
    python3 generate_maze.py            # writes maze.world next to this script
    python3 generate_maze.py my.world   # writes to a custom path

Edit the MAZE list below to design your world:
    '#'  = wall block
    ' '  = open corridor
    'S'  = drone start position (open floor; exact XY coordinates printed at end)

The maze is centred at the world origin so the drone spawns at (0, 0, 0.3).
"""

import sys
import os

# ---------------------------------------------------------------------------
# Maze layout — edit this to design your world.
# All strings must be the same length.
# ---------------------------------------------------------------------------
# MAZE = [
#     "###########",   # outer top wall
#     "#S   #    #",   # Room 1 (top-left) | Room 2 (top-right)
#     "#         #",   # doorway: Room 1 <-> Room 2  (col 5 open)
#     "#    #    #",
#     "#    #    #",
#     "## #### ###",   # wall row: doorway Room 1->3 (col 2) | doorway Room 2->4 (col 7)
#     "#    #    #",   # Room 3 (mid-left)  | Room 4 (mid-right)
#     "#         #",   # doorway: Room 3 <-> Room 4
#     "#    #    #",
#     "#    #    #",
#     "## #### ###",   # wall row: doorway Room 3->5 (col 2) | doorway Room 4->6 (col 7)
#     "#    #    #",   # Room 5 (bot-left)  | Room 6 (bot-right)
#     "#         #",   # doorway: Room 5 <-> Room 6
#     "#    #    #",
#     "#    #    #",
#     "###########",   # outer bottom wall
# ]

MAZE = [
    "#################",   # outer top wall
    "#   S     #     #",   # Room 1 (top-left) | Room 2 (top-right)
    "#         #     #",
    "#         #     #",
    "#         #     #",
    "### ########## ##",   # wall: door at col 3 (Room 1) | door at col 14 (Room 2)
    "#               #",   # long horizontal corridor — drone starts at left end
    "##### ######## ##",   # wall: door at col 5 (Room 3) | door at col 16 (Room 4)
    "#         #     #",   # Room 3 (bot-left) | Room 4 (bot-right)
    "#         #     #",
    "#         #     #",
    "#         #     #",
    "#################",   # outer bottom wall
]

CELL   = 2.0   # metres per cell (corridor width)
HEIGHT = 5.0   # wall height in metres

# ---------------------------------------------------------------------------


def _wall_model(wall_id: int, x: float, y: float) -> str:
    return f"""
    <model name="wall_{wall_id}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {HEIGHT / 2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{CELL:.3f} {CELL:.3f} {HEIGHT:.3f}</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{CELL:.3f} {CELL:.3f} {HEIGHT:.3f}</size></box>
          </geometry>
          <material>
            <ambient>0.45 0.45 0.45 1</ambient>
            <diffuse>0.65 0.65 0.65 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def generate_sdf(maze: list) -> tuple:
    rows = len(maze)
    cols = max(len(row) for row in maze)

    # Centre maze at world origin
    offset_x = -(cols * CELL) / 2.0 + CELL / 2.0
    offset_y =  (rows * CELL) / 2.0 - CELL / 2.0

    wall_blocks = []
    start_world = (0.0, 0.0)
    wall_id = 0

    for r, row in enumerate(maze):
        for c, ch in enumerate(row):
            wx = offset_x + c * CELL
            wy = offset_y - r * CELL

            if ch == 'S':
                start_world = (wx, wy)
            elif ch == '#':
                wall_blocks.append(_wall_model(wall_id, wx, wy))
                wall_id += 1

    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="maze">

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system"
            name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-contact-system"
            name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>200 200</size></plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>200 200</size></plane>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
{"".join(wall_blocks)}

  </world>
</sdf>
"""
    return sdf, start_world


if __name__ == "__main__":
    out_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "maze.world"
    )

    sdf, (sx, sy) = generate_sdf(MAZE)

    with open(out_path, "w") as f:
        f.write(sdf)

    rows = len(MAZE)
    cols = max(len(r) for r in MAZE)
    wall_count = sum(ch == "#" for row in MAZE for ch in row)

    print(f"World written : {out_path}")
    print(f"Maze size     : {rows} rows x {cols} cols  ({rows * CELL:.1f} m x {cols * CELL:.1f} m)")
    print(f"Cell size     : {CELL} m   Wall height: {HEIGHT} m   Walls: {wall_count}")
    print(f"Drone start   : x={sx:.3f}  y={sy:.3f}  z=0.300")
