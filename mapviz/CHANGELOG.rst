^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mapviz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2019-02-20)
------------------

1.0.1 (2019-01-25)
------------------

1.0.0 (2019-01-23)
------------------
* Sharing tf_manager\_ between main app and plugins (`#555 <https://github.com/swri-robotics/mapviz/issues/555>`_)
* Contributors: Davide Faconti

0.3.0 (2018-11-16)
------------------
* Merge all -devel branches into a single master branch
* Add function to lock canvas movement (`#596 <https://github.com/swri-robotics/mapviz/issues/596>`_)
* Contributors: P. J. Reed

0.2.6 (2018-07-31)
------------------

0.2.5 (2018-04-12)
------------------
* Add clear history functionality.
* New plugin to send commands to move_base
* improve text contrast (`#550 <https://github.com/swri-robotics/mapviz/issues/550>`_)
* Glew warning fixed (`#539 <https://github.com/swri-robotics/mapviz/issues/539>`_)
* remove copy and paste of Print...
* add context menu to config_item (`#526 <https://github.com/swri-robotics/mapviz/issues/526>`_)
* Merge pull request `#523 <https://github.com/swri-robotics/mapviz/issues/523>`_ from matt-attack/add-keyboard-input-support-kinetic
* Add keyboard input support for plugins
* update to use non deprecated pluginlib macro
* Fix the "File" menu actions (`#513 <https://github.com/swri-robotics/mapviz/issues/513>`_)
* Merge pull request `#481 <https://github.com/swri-robotics/mapviz/issues/481>`_ from pjreed/threaded-video-recording-kinetic
* Move video recording into its own thread
* Contributors: Davide Faconti, Marc Alban, Matthew Bries, Mikael Arguedas, P. J. Reed

0.2.4 (2017-08-11)
------------------
* Add basic profiling to mapviz.
* Handle GL canvas transforms with an invalid target frame
* Add parameter to disable auto-saving functionality.
* Contributors: Elliot Johnson, Marc Alban, P. J. Reed

0.2.3 (2016-12-10)
------------------
* Fix mapviz kinetic build. (`#456 <https://github.com/evenator/mapviz/issues/456>`_)
  Add a missing rosdep dependency on libxi-dev.
* Contributors: Edward Venator

0.2.2 (2016-12-07)
------------------
* Migrated OpenCV to 3.1 (default in Kinetic)
* Contributors: Brian Holt

0.2.1 (2016-10-23)
------------------
* Add a GUI for controlling the Image Transport (`#432 <https://github.com/swri-robotics/mapviz/issues/432>`_)
  This will add a sub-menu under the "View" menu that will:
  - List all available image transports
  - Indicate which one is currently the default
  - Allow the user to choose which one will be used for new ImageTransport subscriptions
  - Save and restore this setting to Mapviz's config file
  - Cause any `image` plugins using the default transport to resubscribe
  In addition, the image plugin now has a menu that can be used to change the
  transport for that specific plugin so that it is different from the default.
  Fixes `#430 <https://github.com/swri-robotics/mapviz/issues/430>`_
  Conflicts:
  mapviz/package.xml
* Fix icon colors for point drawing plugins (`#433 <https://github.com/swri-robotics/mapviz/issues/433>`_)
  This was probably broken back when all of these were refactored to have a
  single base class.  It looks like the member variable that holds the color
  used to draw the icon was never actually being updated.
  Fixes `#426 <https://github.com/swri-robotics/mapviz/issues/426>`_
* Remove unnecessary include
* Fix warnings in mapviz.
  Fix several reorder and signed comparison warnings in the mapviz
  package.
* Giving `tile_map` an interface overhaul
  MapQuest has turned off their public API for map tiles, so this plugin needed some work.  I have:
  - Removed the MapQuest sources
  - Made the interface for adding new sources more powerful
  - Overhauled how sources are saved and loaded under the hood
  - Added a button to reset the current tile cache
  Resolves `#402 <https://github.com/swri-robotics/mapviz/issues/402>`_
  Conflicts:
  tile_map/CMakeLists.txt
* Adding a dialog for selecting services by type
  This dialog is similar to the ones for listing topics or TF frames, but it is
  a little different under the hood.  Notably:
  - It relies on the rosapi node in order to be able to search for services
  - Since searching is done via a service call, ROS communication is handled
  on a separate thread that will not block the GUI
  - Unlike topics, only searching for a single service type is supported
  Conflicts:
  mapviz/package.xml
* Adding a way for plugin config widgets to resize
  - Adding an event plugins can emit to indicate their geometry has changed
  - Modifying the PCL2 plugin to use it as an example
  Fixes `#393 <https://github.com/swri-robotics/mapviz/issues/393>`_
* Adding a button to reset the location and zoom level
  This adds an icon on the right side of Mapviz's status bar tthat will reset
  the view to the default zoom level and center it on the origin of the target
  frame.
  Resolves `#371 <https://github.com/swri-robotics/mapviz/issues/371>`_
* Contributors: Ed Venator, Marc Alban, P. J. Reed

0.2.0 (2016-06-23)
------------------
* Update mapviz to qt5
* Adding a Q_OBJECT declaration to MapvizPlugin
* Adding signals for various plugin events
  The MapvizPlugin class will now emit signals when any of the following settings change:
  - Draw Order
  - Target Frame
  - Use Latest Transforms
  - Visibility
  Note that the signals will only be emitted if the setting actually *changes*, not
  if it is somehow set to the same value that it was previously.
* Contributors: Ed Venator, P. J. Reed

0.1.3 (2016-05-20)
------------------
* Implement mapviz plug-in for calling the marti_nav_msgs::PlanRoute service.
* Adding an explicit dependency on pkg-config to package.xml (`#355 <https://github.com/swri-robotics/mapviz/issues/355>`_)
* Add _gencpp dependency to mapviz targets.
  This commit adds the _gencpp target to mapviz targets to ensure that
  the AddMapvizDisplay service is built before the targets.
* Make compiler flags specific to each target.
* Implement service for adding and modifying mapviz displays.
* Fix for `#339 <https://github.com/swri-robotics/mapviz/issues/339>`_; explicitly depending on OpenCV 2
* Fix for `#336 <https://github.com/swri-robotics/mapviz/issues/336>`_; Qt event handler exceptions shouldn't crash Mapviz
* Fixing blending for GL drawing
  The call to QGLWidget::beginNativePainting has a side effect of clearing
  GL settings related to blending and depth testing, and that was causing
  alpha transparency to not work right for plugins.  I fixed it by manually
  re-enabling those settings every time beginNativePainting is called.
* Fix for `#319 <https://github.com/swri-robotics/mapviz/issues/319>`_
  Previously, the MapCanvas::MapGlCoordToFixedFrame function relied on
  the transform\_ member variable being set, but it is not set if the
  target frame is <none>.  Instead it will now use the qtransform\_
  variable, which is always initialized for the purpose of QPainters.
* Saving & restoring all matrices and attribs
* Moving QPainter drawing back to being after GL
  I had switched the order while debugging things and forgot to set it
  back to the way it originally was.
* Removing a leftover debug print
* Fixing `#317 <https://github.com/swri-robotics/mapviz/issues/317>`_
  First, the model view matrix needs to be saved and restored around
  QPainter operations because Qt clears several GL variables.  Also, the
  image plugin needed to explicitly call glMatrixMode(GL_PROJECTION);
  it does a few operations on the projection matrix and was just assuming
  that was the current matrix mode.  Also, I added a function that plugins
  need to override if they want to do QPainter operations; this will
  eliminate unnecessary overhead for plugins that do not.
* Removing extraneous calls to MapCanvas::update()
  Now that update() is being called automatically at a rate of 50 Hz,
  the explicit calls in many locations are unnecessary.  It was also
  possible for it to be called in some of these locations from a
  non-main thread, which is invalid and could cause crashes.
* Adding the ability to toggle anti-aliasing
  Now there's a checkbox under the "View" menu that will toggle whether
  anti-aliasing is applied to the canvas.  In some situations this will
  make the display look much prettier at a slight performance cost.
* Cleaning up documentation.
* Merging QPainter/anti-aliasing fixes into jade-devel
  This is the same as the old version of this change, except updated
  to the most recent version of Mapviz.
* Fixing a compile error
* Fix for `#298 <https://github.com/swri-robotics/mapviz/issues/298>`_; right-click + drag will now zoom
* Update map canvas at a fixed rate.
  This update adds a timer to the map canvas to repaint at a fixed rate.
  The default rate is 50 Hz, but there is a method to change it (not
  exposed to the UI at the moment).  50Hz was chosen because it is fast
  enough to give smooth animations and we almost always are running
  mapviz with at least one plugin triggering updates from a 50Hz topic.
* Update mapviz.launch file to also launch anonymously.
* Initialize mapviz as an anonymous node.
* This commit adds a class called SelectFrameDialog that plugins can use
  to present the user with a dialog to choose a TF frame. The dialog
  sorts the frames by name and provides an edit box that the user can
  use to filter the frames to a specific substring.
* Fixing an issue that could cause the click publisher plugin's publisher to not be initialized after it's first added.
* Adding a plugin that, when a user clicks on a point, will publish that point's coordinates to a topic.
* Adding color button widget and updating plugins.
  This commit adds a subclass of QPushButton called ColorButton that
  implements a widget for displaying and selecting colors.  We've been
  doing this manually everywhere with duplicated code.  This is a simple
  abstraction but allows us to elminate a lot of duplication, especially
  in plugins that have multiple color selections.
* Remove debugging messages from SelectFrameDialog.
  These were accidentally left in during initial development.
* Add documentation for the SelectTopicDialog.
* Adds SelecTopicDialog to mapviz.
  This commit adds the SelectTopicDialog that can be used in plugins to
  provide the user with a dialog to select topics.  Typically we have
  done this with a lot of duplicated code across all the plugins.  This
  commit also updates the plugins in mapviz_plugins to use the new
  dialog.
  The new dialog provides several benefits:
  - Reduced code duplication
  - Simplifies writing new plugins
  - Common behavior between all plugins
  - Topics sorted by name
  - User can filter topics by substring
  - Continuously checks the master for new topics while the dialog is open.
* Contributors: Elliot Johnson, Marc Alban, P. J. Reed

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
