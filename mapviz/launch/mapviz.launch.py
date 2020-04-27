import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="mapviz",
            node_executable="mapviz",
            node_name="mapviz",
        ),
        launch_ros.actions.Node(
            package="swri_transform_util",
            node_executable="initialize_origin.py",
            node_name="initialize_origin",
            parameters=[
                {"name": "local_xy_frame", "value": "map"},
                {"name": "local_xy_origin", "value": "swri"},
                {"name": "local_xy_origins", "value": [
                    {"name": "swri",
                        "latitude": 29.45196669,
                        "longitude": -98.61370577,
                        "altitude": 233.719,
                        "heading": 0.0},
                    {"name": "back_40",
                        "latitude": 29.447507,
                        "longitude": -98.629367,
                        "altitude": 200.0,
                        "heading": 0.0}
                ]}
            ]
        ),
        launch_ros.actions.Node(
            package="tf2",
            node_executable="static_transform_publisher",
            node_name="swri_transform",
            arguments="0 0 0 0 0 0 /map /origin 100"
        )
    ])
