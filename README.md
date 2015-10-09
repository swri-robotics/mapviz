Mapviz
======

Mapviz is a [ROS](http://www.ros.org/) based visualization tool with a plug-in system similar to [RVIZ](http://wiki.ros.org/rviz) focused on visualizing 2D data.

![](https://github.com/swri-robotics/mapviz/wiki/mapviz.png)

Build Status
------------

Package | Indigo (Saucy) | Indigo (Trusty) | Jade (Trusty) | Jade (Utopic) | Jade (Vivid)
------- | -------------- | --------------- | ------------- | ------------- | ------------
mapviz (32-bit) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-mapviz_binarydeb_saucy_i386)](http://jenkins.ros.org/job/ros-indigo-mapviz_binarydeb_saucy_i386/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-mapviz_binarydeb_trusty_i386)](http://jenkins.ros.org/job/ros-indigo-mapviz_binarydeb_trusty_i386/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-jade-mapviz_binarydeb_trusty_i386)](http://jenkins.ros.org/job/ros-jade-mapviz_binarydeb_trusty_i386/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-jade-mapviz_binarydeb_utopic_i386)](http://jenkins.ros.org/job/ros-jade-mapviz_binarydeb_utopic_i386/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-jade-mapviz_binarydeb_vivid_i386)](http://jenkins.ros.org/job/ros-jade-mapviz_binarydeb_vivid_i386/) |
mapviz (64-bit) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-mapviz_binarydeb_saucy_amd64)](http://jenkins.ros.org/job/ros-indigo-mapviz_binarydeb_saucy_amd64/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-mapviz_binarydeb_trusty_amd64)](http://jenkins.ros.org/job/ros-indigo-mapviz_binarydeb_trusty_amd64/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-jade-mapviz_binarydeb_trusty_amd64)](http://jenkins.ros.org/job/ros-jade-mapviz_binarydeb_trusty_amd64/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-jade-mapviz_binarydeb_utopic_amd64)](http://jenkins.ros.org/job/ros-jade-mapviz_binarydeb_utopic_amd64/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-jade-mapviz_binarydeb_vivid_amd64)](http://jenkins.ros.org/job/ros-jade-mapviz_binarydeb_vivid_amd64/) |
mapviz_plugins (32-bit) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-mapviz-plugins_binarydeb_saucy_i386)](http://jenkins.ros.org/job/ros-indigo-mapviz-plugins_binarydeb_saucy_i386/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-mapviz-plugins_binarydeb_trusty_i386)](http://jenkins.ros.org/job/ros-indigo-mapviz-plugins_binarydeb_trusty_i386/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-jade-mapviz-plugins_binarydeb_trusty_i386)](http://jenkins.ros.org/job/ros-jade-mapviz-plugins_binarydeb_trusty_i386/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-jade-mapviz-plugins_binarydeb_utopic_i386)](http://jenkins.ros.org/job/ros-jade-mapviz-plugins_binarydeb_utopic_i386/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-jade-mapviz-plugins_binarydeb_vivid_i386)](http://jenkins.ros.org/job/ros-jade-mapviz-plugins_binarydeb_vivid_i386/) |
mapviz_plugins (64-bit) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-mapviz-plugins_binarydeb_saucy_amd64)](http://jenkins.ros.org/job/ros-indigo-mapviz-plugins_binarydeb_saucy_amd64/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-mapviz-plugins_binarydeb_trusty_amd64)](http://jenkins.ros.org/job/ros-indigo-mapviz-plugins_binarydeb_trusty_amd64/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-jade-mapviz-plugins_binarydeb_trusty_amd64)](http://jenkins.ros.org/job/ros-jade-mapviz-plugins_binarydeb_trusty_amd64/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-jade-mapviz-plugins_binarydeb_utopic_amd64)](http://jenkins.ros.org/job/ros-jade-mapviz-plugins_binarydeb_utopic_amd64/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-jade-mapviz-plugins_binarydeb_vivid_amd64)](http://jenkins.ros.org/job/ros-jade-mapviz-plugins_binarydeb_vivid_amd64/) |
multires_image (32-bit) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-multires-image_binarydeb_saucy_i386)](http://jenkins.ros.org/job/ros-indigo-multires-image_binarydeb_saucy_i386/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-multires-image_binarydeb_trusty_i386)](http://jenkins.ros.org/job/ros-indigo-multires-image_binarydeb_trusty_i386/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-jade-multires-image_binarydeb_trusty_i386)](http://jenkins.ros.org/job/ros-jade-multires-image_binarydeb_trusty_i386/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-jade-multires-image_binarydeb_utopic_i386)](http://jenkins.ros.org/job/ros-jade-multires-image_binarydeb_utopic_i386/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-jade-multires-image_binarydeb_vivid_i386)](http://jenkins.ros.org/job/ros-jade-multires-image_binarydeb_vivid_i386/) |
multires_image (64-bit) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-multires-image_binarydeb_saucy_amd64)](http://jenkins.ros.org/job/ros-indigo-multires-image_binarydeb_saucy_amd64/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-multires-image_binarydeb_trusty_amd64)](http://jenkins.ros.org/job/ros-indigo-multires-image_binarydeb_trusty_amd64/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-jade-multires-image_binarydeb_trusty_amd64)](http://jenkins.ros.org/job/ros-jade-multires-image_binarydeb_trusty_amd64/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-jade-multires-image_binarydeb_utopic_amd64)](http://jenkins.ros.org/job/ros-jade-multires-image_binarydeb_utopic_amd64/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-jade-multires-image_binarydeb_vivid_amd64)](http://jenkins.ros.org/job/ros-jade-multires-image_binarydeb_vivid_amd64/) |
tile_map (32-bit) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-tile-map_binarydeb_saucy_i386)](http://jenkins.ros.org/job/ros-indigo-tile-map_binarydeb_saucy_i386/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-tile-map_binarydeb_trusty_i386)](http://jenkins.ros.org/job/ros-indigo-tile-map_binarydeb_trusty_i386/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-jade-tile-map_binarydeb_trusty_i386)](http://jenkins.ros.org/job/ros-jade-tile-map_binarydeb_trusty_i386/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-jade-tile-map_binarydeb_utopic_i386)](http://jenkins.ros.org/job/ros-jade-tile-map_binarydeb_utopic_i386/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-jade-tile-map_binarydeb_vivid_i386)](http://jenkins.ros.org/job/ros-jade-tile-map_binarydeb_vivid_i386/) |
tile_map (64-bit) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-tile-map_binarydeb_saucy_amd64)](http://jenkins.ros.org/job/ros-indigo-tile-map_binarydeb_saucy_amd64/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-tile-map_binarydeb_trusty_amd64)](http://jenkins.ros.org/job/ros-indigo-tile-map_binarydeb_trusty_amd64/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-jade-tile-map_binarydeb_trusty_amd64)](http://jenkins.ros.org/job/ros-jade-tile-map_binarydeb_trusty_amd64/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-jade-tile-map_binarydeb_utopic_amd64)](http://jenkins.ros.org/job/ros-jade-tile-map_binarydeb_utopic_amd64/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-jade-tile-map_binarydeb_vivid_amd64)](http://jenkins.ros.org/job/ros-jade-tile-map_binarydeb_vivid_amd64/) |

Installation (ROS Indigo, Jade)
------------

In ROS Indigo, you can install mapviz using apt-get from the ROS apt repository. This is the recommended installation method for ROS Indigo and Jade.

    sudo apt-get install ros-$ROS_DISTRO-mapviz


Building From Source (ROS Indigo, Jade)
------------

These directions assume you have already set up a catkin workspace. See [this tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) on the ROS Wiki for help setting up a catkin workspace.

### Checking out the source code (wstool)

If you're using wstool, add this repository to your wstool workspace:

    wstool set mapviz --git https://github.com/swri-robotics/mapviz.git -v $ROS_DISTRO-devel

### Checking out the source code (git)

If you're not using wstool, you can check out the repositories with git:

    git clone https://github.com/swri-robotics/mapviz.git --branch $ROS_DISTRO-devel

### Installing dependencies and building

Install all of the dependencies using rosdep by running the following command from the root of your catkin workspace:

    rosdep install --from-paths src --ignore-src

Build the workspace with catkin_make:

    catkin_make

Building From Source (ROS Groovy, Hydro)
------------

These directions assume you have already set up a catkin workspace. See [this tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) on the ROS Wiki for help setting up a catkin workspace.

### Checking out the source code (wstool)

If you're using wstool, add this repository to your wstool workspace:

    wstool set mapviz --git https://github.com/swri-robotics/mapviz.git -v $ROS_DISTRO-devel

### Checking out the source code (git)

If you're not using wstool, you can check out the repositories with git:

    git clone https://github.com/swri-robotics/mapviz.git --branch $ROS_DISTRO-devel

### Installing dependencies and building

Install all of the dependencies manually using these two commands:

    sudo apt-get install ros-$ROS_DISTRO-tf ros-$ROS_DISTRO-common-msgs ros-$ROS_DISTRO-plugin-lib \
      ros-$ROS_DISTRO-gps-umd ros-$ROS_DISTRO-nodelet ros-$ROS_DISTRO-cv-bridge \
      ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-angles ros-$ROS_DISTRO-camera-calibration-parsers \
      ros-$ROS_DISTRO-image-transport

    sudo apt-get install yaml-cpp libboost-random-dev libblas-dev liblapack-dev libproj-dev \
      libglew-dev freeglut3-dev libxmu-dev libgeos++-dev qt4-dev-tools

Build the workspace with catkin_make:

    catkin_make

Plug-ins
--------

### Disparity

Overlays a [sensor_msgs::DisparityImage](http://docs.ros.org/api/stereo_msgs/html/msg/DisparityImage.html) onto the display using the ''jet'' color map.

![](https://github.com/swri-robotics/mapviz/wiki/disparity.png)

**Parameters**
* Topic: The disparity topic name
* Anchor: (top left | top center | top right | center left | center | center right | bottom left | bottom center | bottom right)
* Offset X: Display offset from the left
* Offset Y: Display offset from the top
* Width: Display width
* Height: Display height
* Units: (pixels | percent of window)

### GPS

Projects [sensor_msgs::NavSatFix](http://http://docs.ros.org/jade/api/sensor_msgs/html/msg/NavSatFix.html) message data into the scene.

**Parameters**
 * Topic: The GPS topic
 * Color: The color of the GPS data
 * Draw Style: (lines | points | arrows)
 * Position Tolerance: Distance threshold for adding new GPS points to visualization
 * Buffer Size: Size of circular buffer of GPS points

### Grid
Projects a 2D grid into the scene.

**Parameters**
 * Frame: Coordinate frame of the grid
 * Color: Color of the grid
 * Alpha: Alpha transparency of the grid
 * X: X offset of the grid from the specified coordinate frame origin
 * Y: Y offset of the grid from the specified coordinate frame origin
 * Size: Size of each grid cell
 * Rows: Number of grid rows
 * Columns: Number of grid columns

### Image

Overlays a [sensor_msgs::Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html) onto the display.

**Parameters**
* Topic: The image topic name
* Anchor: (top left | top center | top right | center left | center | center right | bottom left | bottom center | bottom right)
* Offset X: Display offset from the left
* Offset Y: Display offset from the top
* Width: Display width
* Height: Display height
* Units: (pixels | percent of window)

### LaserScan 

Projects a [sensor_msgs::LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html) message into the scene.

* Topic: The laser scan topic name
* Min Color: The color associated with minimum return intensity
* Max Color: The color associated with maximum return intensity
* Min Intesity: Minimum intensity value
* Max Intensity: Maximum intensity value
* Point Size: Display size of laser scan points in pixels
* Buffer Size: Size of circular buffer of laser scan messages points

### Marker
Projects a [visualization_msgs::Marker](http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html) or [visualization_msgs::MarkerArray](http://docs.ros.org/api/visualization_msgs/html/msg/MarkerArray.html) into the scene.

[Markers](http://wiki.ros.org/rviz/DisplayTypes/Marker) are the most flexible display type and more or less mirror the [OpenGL primitives](https://www.opengl.org/wiki/Primitive).

**Parameters**
* Topic: The marker topic

### Multi-res Image
Projects a geo-referenced multi-resolution image tile map into the scene.  The concept is the same as the Google Maps style pan/zoom satellite imagery. 

![](https://github.com/swri-robotics/mapviz/wiki/multires2.png)

**Parameters**
* Geo File: Path to the geo-referenced map tiles.

A custom format is currently used to store the map tiles and geo-reference. The geo-file has the following format:

    image_path: "."        # The relative path to the map tiles
    image_width: 29184     # The full pixel width of the map
    image_height: 15872    # The full pixel height of the map
    tile_size: 512         # The pixel size of the individual tiles
    
    datum: "wgs84"         # Datum is currently ignored
    projection: "utm"      # (utm|wgs84)
 
                           # At least 2 tie points are required for 
                           # scale, and 3 for orientation.
    tiepoints:             #   [pixel x, pixel y, geo x, geo y]
     - point: [4799, 209, 535674.5, 3258382.5]
     - point: [2336, 15596, 535329.5, 3256198.5]
     - point: [26925, 15457, 538837.5, 3256233.5]
     - point: [29133, 84, 539142.5, 3258416.5]

The map tiles are stored in directories for each resolution starting with layer0, the full resolution.  In subsequent layers the resolution is halved until the entire map fits within a single tile.  

Tiles are named using the following format: 

    tile%05dx%05d.png % (row, column)

### Odometry

Projects [nav_msgs::Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html) message data into the scene.

**Parameters**
 * Topic: The odometry topic
 * Color: The color of the odometry data
 * Draw Style: (lines | points | arrows)
 * Show Covariance: Draw covariance ellipse around latest data
 * Position Tolerance: Distance threshold for adding new odometry points to visualization
 * Buffer Size: Size of circular buffer of odometry points

### Path

Projects [nav_msgs::Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html) message data into the scene.

**Parameters**
 * Topic: The path topic

### Robot Image

Projects an image loaded from file into the scene to represent the robot platform.

**Parameters**
 * Image File:  Path to the image file
 * Frame: Frame to tie the image to
 * Width: The physical width represented by the image
 * Height: The physical height represented by the image

### Textured Marker

Projects marti_visualization_msgs::TexturedMarker and marti_visualization_msgs::TexturedMarkerArray message data into the scene.

Textured markers follow the same general approach as traditional markers, but can be used to texture dense image data onto a quad which is projected into the scene.

**Parameters**
 * Topic: The textured marker topic

### Tile Map
Projects a geo-referenced multi-resolution image tile map into the scene.  Data is automatically streamed from [OpenMapQuest](http://open.mapquest.com/) (satellite and roads) or [Stamen Design] (http://maps.stamen.com/) (terrain, watercolor, and toner).  Custom or local map servers can also be specified.  Map data is cached to disk which enables some limited use completely offline.

<img src="https://github.com/swri-robotics/mapviz/wiki/satellite.png" width="200" height="200" />
<img src="https://github.com/swri-robotics/mapviz/wiki/roads.png" width="200" height="200" />
<img src="https://github.com/swri-robotics/mapviz/wiki/terrain.png" width="200" height="200" />
<img src="https://github.com/swri-robotics/mapviz/wiki/watercolor.png" width="200" height="200" />
<img src="https://github.com/swri-robotics/mapviz/wiki/toner.png" width="200" height="200" />

**Parameters**
 * Source: The source of the tile data.

### TF Frame

Projects [Tf](http://wiki.ros.org/tf) data into the scene similar to the Odometry plug-in.

**Parameters**
 * Frame: The Tf frame
 * Color: The color of the Tf data
 * Draw Style: (lines | points | arrows)
 * Position Tolerance: Distance threshold for adding new Tf points to visualization
 * Buffer Size: Size of circular buffer of Tf points
