^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package multires_image
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.5 (2016-05-20)
------------------
* Add helper node to automatically add relevant multires_image display to mapviz based on a GPS message.
* Implement service for adding and modifying mapviz displays.
* Fix for `#339 <https://github.com/swri-robotics/mapviz/issues/339>`_; explicitly depending on OpenCV 2
* Contributors: Marc Alban, P. J. Reed

0.0.4 (2016-01-06)
------------------
* Uses file extension from geo file
* Enables the possibility to load one-layer tile set
* Contributors: Vincent Rousseau

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
* Cleans up dependencies
* Adds find_package(OpenCV REQUIRED) to cmake config
* fixes lint issues
* updates cmake version to squash the CMP0003 warning
* removes dependencies on build_tools
* uses format 2 package definition
* fix missing organization in license text
* exports the multires_image library
* catkinizes mapviz
* changes license to BSD
* adds license and readme files
* Contributors: Ed Venator, Edward Venator, Jerry Towler, Marc Alban, P. J. Reed
