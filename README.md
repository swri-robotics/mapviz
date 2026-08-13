Mapviz
======
| Humble | Jazzy | Kilted | Lyrical | Rolling |
| :--- | :--- | :--- | :--- | :--- |
| [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mapviz__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mapviz__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mapviz__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mapviz__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Kbin_uN64__mapviz__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mapviz__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Lbin_uR64__mapviz__ubuntu_resolute_amd64__binary/badge/icon)](https://build.ros2.org/job/Lbin_uR64__mapviz__ubuntu_resolute_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uR64__mapviz__ubuntu_resolute_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uR64__mapviz__ubuntu_resolute_amd64__binary/)

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
