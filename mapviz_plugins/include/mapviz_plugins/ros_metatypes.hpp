// *****************************************************************************
//
// Copyright (c) 2026, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#ifndef MAPVIZ_PLUGINS__ROS_METATYPES_HPP_
#define MAPVIZ_PLUGINS__ROS_METATYPES_HPP_

// Qt metatype declarations for the ROS message shared_ptrs that plugins carry
// through queued signal emissions (ROS spin thread -> GUI thread).  They are
// centralized here because AUTOMOC compiles every moc file in a single
// translation unit, so a type declared in two plugin headers would be a
// redefinition.

#include <QMetaType>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <gps_msgs/msg/gps_fix.hpp>
#include <marti_nav_msgs/msg/route.hpp>
#include <marti_nav_msgs/msg/route_position.hpp>

Q_DECLARE_METATYPE(geometry_msgs::msg::Pose::ConstSharedPtr)
Q_DECLARE_METATYPE(geometry_msgs::msg::PoseStamped::ConstSharedPtr)
Q_DECLARE_METATYPE(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr)
Q_DECLARE_METATYPE(nav_msgs::msg::OccupancyGrid::ConstSharedPtr)
Q_DECLARE_METATYPE(nav_msgs::msg::Odometry::ConstSharedPtr)
Q_DECLARE_METATYPE(nav_msgs::msg::Path::ConstSharedPtr)
Q_DECLARE_METATYPE(sensor_msgs::msg::Image::ConstSharedPtr)
Q_DECLARE_METATYPE(sensor_msgs::msg::Imu::ConstSharedPtr)
Q_DECLARE_METATYPE(sensor_msgs::msg::LaserScan::ConstSharedPtr)
Q_DECLARE_METATYPE(sensor_msgs::msg::NavSatFix::ConstSharedPtr)
Q_DECLARE_METATYPE(std_msgs::msg::String::ConstSharedPtr)
Q_DECLARE_METATYPE(stereo_msgs::msg::DisparityImage::ConstSharedPtr)
Q_DECLARE_METATYPE(visualization_msgs::msg::Marker::ConstSharedPtr)
Q_DECLARE_METATYPE(visualization_msgs::msg::MarkerArray::ConstSharedPtr)

Q_DECLARE_METATYPE(gps_msgs::msg::GPSFix::ConstSharedPtr)
Q_DECLARE_METATYPE(marti_nav_msgs::msg::Route::ConstSharedPtr)
Q_DECLARE_METATYPE(marti_nav_msgs::msg::RoutePosition::ConstSharedPtr)

#endif  // MAPVIZ_PLUGINS__ROS_METATYPES_HPP_
