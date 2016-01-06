^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mapviz_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2016-01-06)
------------------
* Enables the possibility to load a one-layer tile set
* Sorts topic, plug-in, and frame lists in selection dialogs.
* Fixes tf plug-in update.
* Contributors: Marc Alban, Vincent Rousseau

0.1.1 (2015-11-17)
------------------
* Extensions for geo files (PR `#262 <https://github.com/swri-robotics/mapviz/issues/262>`_)
* Adds a plugin to visualize laser scans.
  Display features are based on the laserscan plugin for rviz:
  * Points can be colored by range, or x/y/z axis
  * Points can be colored by interpolation between two colors or rainbow coloring
* Adds a plugin to visualize sensor_msgs/NavSatFix msgs, based on the old GPSFix plugin
* Contributors: Claudio Bandera, Ed Venator, Vincent Rousseau

0.1.0 (2015-09-29)
------------------
* Removes gps plugin, since gps_common is not in ROS Jade. See issue 
  `#238 <https://github.com/swri-robotics/mapviz/issues/238>`_.
* Contributors: Edward Venator

0.0.3 (2015-09-28)
------------------

0.0.2 (2015-09-27)
------------------
* Adds missing qt4_opengl dependency

0.0.1 (2015-09-27)
------------------
* Renames all marti_common packages that were renamed.
  (See http://github.com/swri-robotics/marti_common/issues/231)
* Fixes catkin_lint problems that could prevent installation.
* Exports the mapviz_plugins library
* Adds find_package(OpenCV REQUIRED) to cmake config
* adds icon to gps plug-in
* includes yaml_util header in mapviz plug-in base class
* adds gps_common dependency
* Sets the point orientation properly based on the GPSFix track.
* Converts incoming GPSFix points to the local XY frame as they arrive.
* Changes the GPS plugin to always transform from the local XY frame.
* Adds a plugin to display GPSFix data.
* Fixes a few instances where "multires" was typoed as "mutlires".
* updates cmake version to squash the CMP0003 warning
* removes dependencies on build_tools
* switches format 2 package definition
* Updates marker_plugin to correctly handle marker orientation.
* adds color selection for path visualization
* display preview icon next to plug-in names
* sets the z component of path points to 0 before transforming to avoid uninitialized values
* fixes missing organization in license text
* fixes for GLEW/GL include order
* catkinize mapviz
* changes license to BSD
* adds license and readme files
* Contributors: Edward Venator, Elliot Johnson, Marc Alban, P. J. Reed
