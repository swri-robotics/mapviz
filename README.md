Mapviz
======

Build Status
------
Item | **Humble** | **Jazzy** | **Kilted** | **Lyrical** | **Rolling**
:--- | :---: | :---: | :---: | :---: | :---:
Branch | [`humble`](https://github.com/swri-robotics/mapviz/tree/ros2-devel) | [`jazzy`](https://github.com/swri-robotics/mapviz/tree/ros2-devel) | [`kilted`](https://github.com/swri-robotics/mapviz/tree/ros2-devel) | [`lyrical`](https://github.com/swri-robotics/mapviz/tree/ros2-devel) | [`rolling`](https://github.com/swri-robotics/mapviz/tree/ros2-devel)
Build status | [![CI](https://github.com/swri-robotics/mapviz/actions/workflows/humble.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/mapviz/blob/ros2-devel/.github/workflows/humble.yml) <br /> [![ROS2 Build Farm](http://build.ros2.org/buildStatus/icon?job=Hdev__mapviz__ubuntu_jammy_amd64)](https://build.ros2.org/job/Hdev__mapviz__ubuntu_jammy_amd64/) | [![CI](https://github.com/swri-robotics/mapviz/actions/workflows/jazzy.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/mapviz/blob/ros2-devel/.github/workflows/jazzy.yml) <br /> [![ROS2 Build Farm](http://build.ros2.org/buildStatus/icon?job=Jdev__mapviz__ubuntu_noble_amd64)](https://build.ros2.org/job/Jdev__mapviz__ubuntu_noble_amd64/) | [![CI](https://github.com/swri-robotics/mapviz/actions/workflows/kilted.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/mapviz/blob/ros2-devel/.github/workflows/kilted.yml) <br /> [![ROS2 Build Farm](http://build.ros2.org/buildStatus/icon?job=Kdev__mapviz__ubuntu_noble_amd64)](https://build.ros2.org/job/Kdev__mapviz__ubuntu_noble_amd64/) | [![CI](https://github.com/swri-robotics/mapviz/actions/workflows/lyrical.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/mapviz/blob/ros2-devel/.github/workflows/lyrical.yml) <br /> [![ROS2 Build Farm](http://build.ros2.org/buildStatus/icon?job=Ldev__mapviz__ubuntu_resolute_amd64)](https://build.ros2.org/job/Ldev__mapviz__ubuntu_resolute_amd64/) | [![CI](https://github.com/swri-robotics/mapviz/actions/workflows/rolling.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/mapviz/blob/ros2-devel/.github/workflows/rolling.yml) <br /> [![ROS2 Build Farm](http://build.ros2.org/buildStatus/icon?job=Rdev__mapviz__ubuntu_resolute_amd64)](https://build.ros2.org/job/Rdev__mapviz__ubuntu_resolute_amd64/)
`mapviz` | [Released](https://index.ros.org/p/mapviz/#humble) | [Released](https://index.ros.org/p/mapviz/#jazzy) | [Released](https://index.ros.org/p/mapviz/#kilted) | [Released](https://index.ros.org/p/mapviz/#lyrical) | [Released](https://index.ros.org/p/mapviz/#rolling)
`mapviz_interfaces` | [Released](https://index.ros.org/p/mapviz_interfaces/#humble) | [Released](https://index.ros.org/p/mapviz_interfaces/#jazzy) | [Released](https://index.ros.org/p/mapviz_interfaces/#kilted) | [Released](https://index.ros.org/p/mapviz_interfaces/#lyrical) | [Released](https://index.ros.org/p/mapviz_interfaces/#rolling)
`mapviz_plugins` | [Released](https://index.ros.org/p/mapviz_plugins/#humble) | [Released](https://index.ros.org/p/mapviz_plugins/#jazzy) | [Released](https://index.ros.org/p/mapviz_plugins/#kilted) | [Released](https://index.ros.org/p/mapviz_plugins/#lyrical) | [Released](https://index.ros.org/p/mapviz_plugins/#rolling)
`multires_image` | [Released](https://index.ros.org/p/multires_image/#humble) | [Released](https://index.ros.org/p/multires_image/#jazzy) | [Released](https://index.ros.org/p/multires_image/#kilted) | [Released](https://index.ros.org/p/multires_image/#lyrical) | [Released](https://index.ros.org/p/multires_image/#rolling)
`tile_map` | [Released](https://index.ros.org/p/tile_map/#humble) | [Released](https://index.ros.org/p/tile_map/#jazzy) | [Released](https://index.ros.org/p/tile_map/#kilted) | [Released](https://index.ros.org/p/tile_map/#lyrical) | [Released](https://index.ros.org/p/tile_map/#rolling)

Mapviz is a [ROS](http://www.ros.org/) based visualization tool with a plug-in system similar to [RVIZ](http://wiki.ros.org/rviz) focused on visualizing 2D data.

![](https://github.com/swri-robotics/mapviz/wiki/mapviz.png)

v4.0.0 Special Note
-----
v4.0.0 introduced several breaking changes.

- Many header files had their extensions changed from `.h` to `.hpp` to conform to ROS conventions.
- The UI and ROS callbacks were split into separate threads. This significantly impacts the plugin architecture. Please see the documentation for instructions on how to safely apply these changes and for examples of plugins that use the new architecture.

Usage
-----

[View the documentation](https://swri-robotics.github.io/mapviz/) for usage information.
