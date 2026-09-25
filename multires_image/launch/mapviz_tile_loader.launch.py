#!/usr/bin/env python3

# Copyright (c) 2016, Southwest Research Institute® (SwRI®)
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

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# initialize_origin.py reads local_xy_origins as a YAML string and parses it
# itself.  Passing it as an explicit str (rather than through the XML launch
# frontend, which would parse it into an unsupported list-of-dicts parameter)
# is why this launch file is written in Python.
LOCAL_XY_ORIGINS = """
[{ name: swri,
   latitude: 29.45196669,
   longitude: -98.61370577,
   altitude: 233.719,
   heading: 0.0},

 { name: back_40,
   latitude: 29.447507,
   longitude: -98.629367,
   altitude: 200.0,
   heading: 0.0}]
"""


def generate_launch_description():
    initialize_origin = Node(
        package='swri_transform_util',
        executable='initialize_origin.py',
        name='initialize_origin',
        parameters=[{
            'local_xy_frame': 'far_field',
            'local_xy_origin': 'swri',
            'local_xy_origins': ParameterValue(LOCAL_XY_ORIGINS, value_type=str),
        }],
    )

    # mapviz assigns itself an anonymous node name at startup.
    mapviz = Node(
        package='mapviz',
        executable='mapviz',
    )

    tile_loader = Node(
        package='multires_image',
        executable='mapviz_tile_loader',
        name='mapviz_tile_loader',
        parameters=[{
            'base_directory': os.path.join(os.path.expanduser('~'), '.map_tiles'),
            'draw_order': 1,
            # Select the tileset from the local-XY origin (a PoseStamped on
            # /local_xy_origin) rather than a GPSFix.
            'use_local_xy': True,
        }],
        remappings=[
            ('add_mapviz_display', 'mapviz/add_mapviz_display'),
        ],
    )

    return LaunchDescription([initialize_origin, mapviz, tile_loader])
