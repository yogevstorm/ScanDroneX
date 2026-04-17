# Goal

I want to add autonomous scanning functionality. To do this, I want to create a new layer in the navigation package called ScanLayer, following a structure similar to the existing layers.

## Step 1

Create a node for this layer called ScanNode.cpp.

ScanNode.cpp should publish a /goal_pose that points to a location in the unknown area of the map — that is, a green pixel with a value of -1.

## Step 2

The drone should move toward the published goal pose. However, the path may become blocked because the map is being built dynamically.

When the path is blocked, ScanNode should publish the same /goal_pose again, and continue doing so repeatedly until the path becomes available.

To detect whether the path is blocked, use the function FindBlockedEndNode from BehaviorPlanner.cpp.

It may also be useful to publish an additional indication of whether the path is currently blocked or not.

If you want, I can also turn this into:

a more technical design note
a GitHub/Jira task
a developer-ready implementation spec