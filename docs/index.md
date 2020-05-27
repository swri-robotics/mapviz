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

These directions assume you have already set up a `catkin` workspace. If not, see [this tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) on the ROS Wiki to set one up.

1.  Check out the source
    *   Using `wstool`
        If you're using `wstool`, add this repository to your `wstool` workspace:

        ```
        $ wstool set mapviz --git https://github.com/swri-robotics/mapviz.git
        $ wstool update mapviz
        ```

    *   Using `git`
        If you're not using wstool, you can check out the repository directly with `git`:

        ```
        $ git clone https://github.com/swri-robotics/mapviz.git
        ```

2.  Install dependencies

    Install all of the dependencies using `rosdep` by running the following command from the root of your `catkin` workspace:

    ```
    $ rosdep install --from-paths src --ignore-src
    ```

3.  Build the workspace:

    ```
    $ catkin_make
    ```
