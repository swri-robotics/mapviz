---
title: "GPS"
description: "Projects [gps_common::GPSFix](http://docs.ros.org/kinetic/api/gps_common/html/msg/GPSFix.html) message data into the scene."
image: ""
parameters:
  - name: Topic
    description: The GPS topic
  - name: Color
    description: The color of the GPS data
  - name: Draw Style
    description: (lines | points | arrows)
  - name: Static Arrow Sizes
    description: If checked, draw arrows the same size regardless of zoom level; slider adjusts size
  - name: Position Tolerance
    description: Distance threshold for adding new GPS points to visualization
  - name: Buffer Size
    description: Size of circular buffer of GPS points
  - name: Show Laps
    description: If checked, multiple loops of GPS coordinates will have different colors
---
