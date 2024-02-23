---
layout: default
title: Home
nav_order: 1
description: "Mapviz is a highly customizable ROS-based visualization tool focused on large-scale 2D data."
---

# We can show you the world
{: .fs-9}

Mapviz is a highly customizable ROS-based visualization tool focused on large-scale 2D data, with a plugin system for extreme extensibility.
{: .fs-6 .fw-300}

![]({{ site.baseurl }}/assets/images/mapviz.png)

[Get started now](#getting-started){: .btn .btn-blue .fs-5 .mb-4 .mb-md-0 .mr-2 } [View it on GitHub](https://github.com/swri-robotics/mapviz){: .btn .text-blue-000 .bg-grey-lt-000 .fs-5 .mb-4 .mb-md-0 }

---

## Getting Started

The easiest way to install Mapviz is using `apt-get`:

```
$ sudo apt-get install ros-$ROS_DISTRO-mapviz \
                       ros-$ROS_DISTRO-mapviz-plugins \
                       ros-$ROS_DISTRO-tile-map \
                       ros-$ROS_DISTRO-multires-image
```

### Building From Source

These directions assume you have already set up a `colcon` workspace. If not, see [this tutorial](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) on the ROS 2 Wiki to set one up.

1.  Check out the source
    *   Using `vcstool`
        If you're using `vcstool`, add this repository to your `vcstool` workspace:

        ```bash
        $ cat mapviz.repos
        # example repos file to import
        repositories:
          vcstool:
            type: git
            url: https://github.com/swri-robotics/mapviz.git
            version: ros2-devel
        $ vcs import < mapviz.repos
        ```

    *   Using `git`
        If you're not using vcstool, you can check out the repository directly with `git`:

        ```bash
        $ git clone https://github.com/swri-robotics/mapviz.git
        ```

2.  Install dependencies

    Install all of the dependencies using `rosdep` by running the following command from the root of your `colcon` workspace:

    ```bash
    $ rosdep install --from-paths src --ignore-src
    ```

3.  Build the workspace:

    ```bash
    $ colcon build
    ```
