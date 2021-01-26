# follow_waypoints_mapviz_plugin 

A package that will add buttons to control the [follow_waypoints node](http://wiki.ros.org/follow_waypoints)

#### To add a waypoint to the queue: 

1. Publish a PoseStamped to the `addpose` topic (`/initialpose` by default).  This can be done by the `D Pose Estimate` button in rviz or mapviz (move_base plugin).
2. Publish a PoseStamped to a different `addpose` topic (changing the topic name in the follow_waypoints launchfile or when launching the node) 

```
roslaunch follow_waypoints follow_waypoints.launch addpose_topic:=/selected_pose
```

#### To view the current set of waypoints:

```
rostopic echo /waypoints
```

#### To load the previously saved path:

```
rostopic pub /start_journey std_msgs/Empty -1
```

#### To execute waypoint following:

```
rostopic pub /path_ready std_msgs/Empty -1
```

#### To clear the waypoint queue send a "path reset" message:

```
rostopic pub /path_reset std_msgs/Empty -1
```

#### To load the previously save path:

```
rostopic pub /start_journey std_msgs/Empty -1
```

#### OR, you can do all of the above with mapviz plugins:

```
directions go here.
```

