# Copyright (c) 2020, Southwest Research Institute® (SwRI®)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Southwest Research Institute® (SwRI®) nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    mapviz_node = launch_ros.actions.Node(
        package='mapviz',
        executable='mapviz',
        name='mapviz',
        on_exit=launch.actions.Shutdown(),
    )

    return launch.LaunchDescription([
        mapviz_node,
        launch_ros.actions.Node(
            package='swri_transform_util',
            executable='initialize_origin.py',
            name='initialize_origin',
            parameters=[
                {'local_xy_frame': 'map'},
                {'local_xy_origin': 'swri'},
                {'local_xy_origins': """[
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
                ]"""},
            ]
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='swri_transform',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'origin']
        )
    ])
