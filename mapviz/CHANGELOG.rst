^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mapviz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2016-01-06)
------------------
* Show full path when recording screenshots/movies.
* Fixes a bug in plug-in sorting.
* Sorts topic, plug-in, and frame lists in selection dialogs.
* Contributors: Elliot Johnson, Marc Alban

0.1.1 (2015-11-17)
------------------
* Fixes mapviz launch file frame param
* Marks single argument constructors explicit.
* Contributors: Edward Venator, Marc Alban, Vincent Rousseau

0.1.0 (2015-09-29)
------------------

0.0.3 (2015-09-28)
------------------
* Fixing casting issues that prevented compilation on 32-bit systems.
* Contributors: P. J. Reed

0.0.2 (2015-09-27)
------------------
* Adds missing qt4_opengl dependency

0.0.1 (2015-09-27)
------------------
* Renames all marti_common packages that were renamed.
  (See http://github.com/swri-robotics/marti_common/issues/231)
* Adds missing dependencies in mapviz package.xml.
* Fixes catkin_lint problems that could prevent installation.
* Formats package files
* Cleans up dependencies
* Adds required rosdeps
* Saving/loading config files to the ROS_WORKSPACE directory.
* fixes lint issues
* Makes mapviz show a warning dialog instead of crash when it fails to load a plugin.
* includes yaml_util header in mapviz plug-in base class
* Handles loading old config files that still reference "mutlires_image".
* Adds an RQT plugin version of Mapviz.
* updates cmake version to squash the CMP0003 warning
* uses correct operator when combining quaternions
* adds option for rotating camera 90 degrees
* throttles log msgs
* removes dependencies on build_tools
* uses format 2 package definition
* allows plug-in selection with double-clicks
* displays file name in window title
* displays preview icon next to plug-in names
* fixes issue with coordinates displayed on status bar
* fixes missing organization in license text
* Adds tooltips describing the various mapviz widgets
* fixes GLEW/GL include order
* catkinizes mapviz
* changes license to BSD
* adds license and readme files
* Contributors: Ed Venator, Edward Venator, Jerry Towler, Marc Alban, P. J. Reed
