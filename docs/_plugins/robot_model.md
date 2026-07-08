---
title: "Robot Model"
description: "Renders a URDF robot model as a top-down 2D projection in the map scene. Supports box, cylinder, and sphere primitives as well as mesh geometry with diffuse textures and Lambert shading."
image: ""
parameters:
  - name: "Source"
    description: "Where to read the robot description from: **Topic** (subscribes to a `std_msgs/String` topic, typically `/robot_description`, using a transient-local QoS to receive the last-published value immediately) or **File** (loads a URDF directly from a local file path)"
  - name: "Topic / File"
    description: "The topic name or absolute file path, depending on the Source selection"
  - name: "Alpha"
    description: "Overall opacity of the rendered model (0.0 - 1.0)"
---
