---
title: "Disparity"
description: "Overlays a [sensor_msgs::DisparityImage](http://docs.ros.org/api/stereo_msgs/html/msg/DisparityImage.html) onto the display using the ''jet'' color map."
image: "disparity.png"
parameters:
  - name: Topic
    description: The disparity topic name
  - name: Anchor
    description: (top left | top center | top right | center left | center | center right | bottom left | bottom center | bottom right)
  - name: Offset X
    description: Display offset from the left
  - name: Offset Y
    description: Display offset from the top
  - name: Width
    description: Display width
  - name: Height
    description: Display height
  - name: Units
    description: (pixels | percent of window)
---
