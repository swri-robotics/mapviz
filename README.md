Mapviz
======

Mapviz is a [ROS](http://www.ros.org/) based visualization tool with a plug-in system similar to [RVIZ](http://wiki.ros.org/rviz) focused on visualizing 2D data.

![](https://github.com/swri-robotics/mapviz/wiki/mapviz.png)

Usage
-----

[View the documentation](https://swri-robotics.github.io/mapviz/) for usage information.

v4.0.0 Special Note
-----
v4.0.0 introduced several breaking changes.

- Many header files had their extensions changed from `.h` to `.hpp` to conform to ROS conventions.
- The UI and ROS callbacks were split into separate threads. This significantly impacts the plugin architecture. Please see the documentation for instructions on how to safely apply these changes and for examples of plugins that use the new architecture.

Build Status
------

### Branches

Item | **Humble** | **Jazzy** | **Kilted** | **Lyrical** | **Rolling**
:--- | :---: | :---: | :---: | :---: | :---:
Branch | [`humble`](https://github.com/swri-robotics/mapviz/tree/ros2-devel) | [`jazzy`](https://github.com/swri-robotics/mapviz/tree/ros2-devel) | [`kilted`](https://github.com/swri-robotics/mapviz/tree/ros2-devel) | [`lyrical`](https://github.com/swri-robotics/mapviz/tree/ros2-devel) | [`rolling`](https://github.com/swri-robotics/mapviz/tree/ros2-devel)

### Released Versions

Item | **Humble** | **Jazzy** | **Kilted** | **Lyrical** | **Rolling**
:--- | :---: | :---: | :---: | :---: | :---:
Version | [![ROS2 Humble](https://img.shields.io/ros/v/humble/mapviz.svg?style=flat-square)](https://index.ros.org/r/mapviz/#humble) | [![ROS2 Jazzy](https://img.shields.io/ros/v/jazzy/mapviz.svg?style=flat-square)](https://index.ros.org/r/mapviz/#jazzy) | [![ROS2 Kilted](https://img.shields.io/ros/v/kilted/mapviz.svg?style=flat-square)](https://index.ros.org/r/mapviz/#kilted) | [![ROS2 Lyrical](https://img.shields.io/ros/v/lyrical/mapviz.svg?style=flat-square)](https://index.ros.org/r/mapviz/#lyrical) | [![ROS2 Rolling](https://img.shields.io/ros/v/rolling/mapviz.svg?style=flat-square)](https://index.ros.org/r/mapviz/#rolling)

### CI

Item | **Humble** | **Jazzy** | **Kilted** | **Lyrical** | **Rolling**
:--- | :---: | :---: | :---: | :---: | :---:
GitHub Actions | [![CI](https://github.com/swri-robotics/mapviz/actions/workflows/humble.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/mapviz/blob/ros2-devel/.github/workflows/humble.yml) | [![CI](https://github.com/swri-robotics/mapviz/actions/workflows/jazzy.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/mapviz/blob/ros2-devel/.github/workflows/jazzy.yml) | [![CI](https://github.com/swri-robotics/mapviz/actions/workflows/kilted.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/mapviz/blob/ros2-devel/.github/workflows/kilted.yml) | [![CI](https://github.com/swri-robotics/mapviz/actions/workflows/lyrical.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/mapviz/blob/ros2-devel/.github/workflows/lyrical.yml) | [![CI](https://github.com/swri-robotics/mapviz/actions/workflows/rolling.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/mapviz/blob/ros2-devel/.github/workflows/rolling.yml)

### amd64 dev

Item | **Humble** | **Jazzy** | **Kilted** | **Lyrical** | **Rolling**
:--- | :---: | :---: | :---: | :---: | :---:
[`mapviz`](https://github.com/swri-robotics/mapviz) | [![dev](https://build.ros2.org/buildStatus/icon?job=Hdev__mapviz__ubuntu_jammy_amd64)](https://build.ros2.org/job/Hdev__mapviz__ubuntu_jammy_amd64/) | [![dev](https://build.ros2.org/buildStatus/icon?job=Jdev__mapviz__ubuntu_noble_amd64)](https://build.ros2.org/job/Jdev__mapviz__ubuntu_noble_amd64/) | [![dev](https://build.ros2.org/buildStatus/icon?job=Kdev__mapviz__ubuntu_noble_amd64)](https://build.ros2.org/job/Kdev__mapviz__ubuntu_noble_amd64/) | [![dev](https://build.ros2.org/buildStatus/icon?job=Ldev__mapviz__ubuntu_resolute_amd64)](https://build.ros2.org/job/Ldev__mapviz__ubuntu_resolute_amd64/) | [![dev](https://build.ros2.org/buildStatus/icon?job=Rdev__mapviz__ubuntu_resolute_amd64)](https://build.ros2.org/job/Rdev__mapviz__ubuntu_resolute_amd64/)

### amd64 bin

Item | **Humble** | **Jazzy** | **Kilted** | **Lyrical** | **Rolling**
:--- | :---: | :---: | :---: | :---: | :---:
[`mapviz`](https://index.ros.org/p/mapviz/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__mapviz__ubuntu_jammy_amd64__binary)](https://build.ros2.org/job/Hbin_uJ64__mapviz__ubuntu_jammy_amd64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__mapviz__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Jbin_uN64__mapviz__ubuntu_noble_amd64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Kbin_uN64__mapviz__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Kbin_uN64__mapviz__ubuntu_noble_amd64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Lbin_uR64__mapviz__ubuntu_resolute_amd64__binary)](https://build.ros2.org/job/Lbin_uR64__mapviz__ubuntu_resolute_amd64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Rbin_uR64__mapviz__ubuntu_resolute_amd64__binary)](https://build.ros2.org/job/Rbin_uR64__mapviz__ubuntu_resolute_amd64__binary/)
[`mapviz_interfaces`](https://index.ros.org/p/mapviz_interfaces/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__mapviz_interfaces__ubuntu_jammy_amd64__binary)](https://build.ros2.org/job/Hbin_uJ64__mapviz_interfaces__ubuntu_jammy_amd64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__mapviz_interfaces__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Jbin_uN64__mapviz_interfaces__ubuntu_noble_amd64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Kbin_uN64__mapviz_interfaces__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Kbin_uN64__mapviz_interfaces__ubuntu_noble_amd64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Lbin_uR64__mapviz_interfaces__ubuntu_resolute_amd64__binary)](https://build.ros2.org/job/Lbin_uR64__mapviz_interfaces__ubuntu_resolute_amd64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Rbin_uR64__mapviz_interfaces__ubuntu_resolute_amd64__binary)](https://build.ros2.org/job/Rbin_uR64__mapviz_interfaces__ubuntu_resolute_amd64__binary/)
[`mapviz_plugins`](https://index.ros.org/p/mapviz_plugins/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__mapviz_plugins__ubuntu_jammy_amd64__binary)](https://build.ros2.org/job/Hbin_uJ64__mapviz_plugins__ubuntu_jammy_amd64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__mapviz_plugins__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Jbin_uN64__mapviz_plugins__ubuntu_noble_amd64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Kbin_uN64__mapviz_plugins__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Kbin_uN64__mapviz_plugins__ubuntu_noble_amd64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Lbin_uR64__mapviz_plugins__ubuntu_resolute_amd64__binary)](https://build.ros2.org/job/Lbin_uR64__mapviz_plugins__ubuntu_resolute_amd64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Rbin_uR64__mapviz_plugins__ubuntu_resolute_amd64__binary)](https://build.ros2.org/job/Rbin_uR64__mapviz_plugins__ubuntu_resolute_amd64__binary/)
[`multires_image`](https://index.ros.org/p/multires_image/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__multires_image__ubuntu_jammy_amd64__binary)](https://build.ros2.org/job/Hbin_uJ64__multires_image__ubuntu_jammy_amd64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__multires_image__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Jbin_uN64__multires_image__ubuntu_noble_amd64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Kbin_uN64__multires_image__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Kbin_uN64__multires_image__ubuntu_noble_amd64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Lbin_uR64__multires_image__ubuntu_resolute_amd64__binary)](https://build.ros2.org/job/Lbin_uR64__multires_image__ubuntu_resolute_amd64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Rbin_uR64__multires_image__ubuntu_resolute_amd64__binary)](https://build.ros2.org/job/Rbin_uR64__multires_image__ubuntu_resolute_amd64__binary/)
[`tile_map`](https://index.ros.org/p/tile_map/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__tile_map__ubuntu_jammy_amd64__binary)](https://build.ros2.org/job/Hbin_uJ64__tile_map__ubuntu_jammy_amd64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__tile_map__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Jbin_uN64__tile_map__ubuntu_noble_amd64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Kbin_uN64__tile_map__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Kbin_uN64__tile_map__ubuntu_noble_amd64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Lbin_uR64__tile_map__ubuntu_resolute_amd64__binary)](https://build.ros2.org/job/Lbin_uR64__tile_map__ubuntu_resolute_amd64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Rbin_uR64__tile_map__ubuntu_resolute_amd64__binary)](https://build.ros2.org/job/Rbin_uR64__tile_map__ubuntu_resolute_amd64__binary/)

### arm64 bin

Item | **Humble** | **Jazzy** | **Kilted** | **Lyrical** | **Rolling**
:--- | :---: | :---: | :---: | :---: | :---:
[`mapviz`](https://index.ros.org/p/mapviz/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Hbin_ujv8_uJv8__mapviz__ubuntu_jammy_arm64__binary)](https://build.ros2.org/job/Hbin_ujv8_uJv8__mapviz__ubuntu_jammy_arm64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Jbin_unv8_uNv8__mapviz__ubuntu_noble_arm64__binary)](https://build.ros2.org/job/Jbin_unv8_uNv8__mapviz__ubuntu_noble_arm64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Kbin_unv8_uNv8__mapviz__ubuntu_noble_arm64__binary)](https://build.ros2.org/job/Kbin_unv8_uNv8__mapviz__ubuntu_noble_arm64__binary/) | not built | not built
[`mapviz_interfaces`](https://index.ros.org/p/mapviz_interfaces/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Hbin_ujv8_uJv8__mapviz_interfaces__ubuntu_jammy_arm64__binary)](https://build.ros2.org/job/Hbin_ujv8_uJv8__mapviz_interfaces__ubuntu_jammy_arm64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Jbin_unv8_uNv8__mapviz_interfaces__ubuntu_noble_arm64__binary)](https://build.ros2.org/job/Jbin_unv8_uNv8__mapviz_interfaces__ubuntu_noble_arm64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Kbin_unv8_uNv8__mapviz_interfaces__ubuntu_noble_arm64__binary)](https://build.ros2.org/job/Kbin_unv8_uNv8__mapviz_interfaces__ubuntu_noble_arm64__binary/) | not built | not built
[`mapviz_plugins`](https://index.ros.org/p/mapviz_plugins/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Hbin_ujv8_uJv8__mapviz_plugins__ubuntu_jammy_arm64__binary)](https://build.ros2.org/job/Hbin_ujv8_uJv8__mapviz_plugins__ubuntu_jammy_arm64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Jbin_unv8_uNv8__mapviz_plugins__ubuntu_noble_arm64__binary)](https://build.ros2.org/job/Jbin_unv8_uNv8__mapviz_plugins__ubuntu_noble_arm64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Kbin_unv8_uNv8__mapviz_plugins__ubuntu_noble_arm64__binary)](https://build.ros2.org/job/Kbin_unv8_uNv8__mapviz_plugins__ubuntu_noble_arm64__binary/) | not built | not built
[`multires_image`](https://index.ros.org/p/multires_image/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Hbin_ujv8_uJv8__multires_image__ubuntu_jammy_arm64__binary)](https://build.ros2.org/job/Hbin_ujv8_uJv8__multires_image__ubuntu_jammy_arm64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Jbin_unv8_uNv8__multires_image__ubuntu_noble_arm64__binary)](https://build.ros2.org/job/Jbin_unv8_uNv8__multires_image__ubuntu_noble_arm64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Kbin_unv8_uNv8__multires_image__ubuntu_noble_arm64__binary)](https://build.ros2.org/job/Kbin_unv8_uNv8__multires_image__ubuntu_noble_arm64__binary/) | not built | not built
[`tile_map`](https://index.ros.org/p/tile_map/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Hbin_ujv8_uJv8__tile_map__ubuntu_jammy_arm64__binary)](https://build.ros2.org/job/Hbin_ujv8_uJv8__tile_map__ubuntu_jammy_arm64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Jbin_unv8_uNv8__tile_map__ubuntu_noble_arm64__binary)](https://build.ros2.org/job/Jbin_unv8_uNv8__tile_map__ubuntu_noble_arm64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Kbin_unv8_uNv8__tile_map__ubuntu_noble_arm64__binary)](https://build.ros2.org/job/Kbin_unv8_uNv8__tile_map__ubuntu_noble_arm64__binary/) | not built | not built
