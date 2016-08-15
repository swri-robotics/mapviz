^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mapviz_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.6 (2016-08-14)
------------------
* Fixes signed comparison warnings in mapviz_plugins
* Adds a way for plugin config widgets to resize

  - Adding an event plugins can emit to indicate their geometry has changed
  - Modifying the PCL2 plugin to use it as an example
  
  Fixes `#393 <https://github.com/swri-robotics/mapviz/issues/393>`_
* Sets default values for uninitialized variables
  Resolves `#372 <https://github.com/swri-robotics/mapviz/issues/372>`_
* Creates and implements an abstract class for drawing point paths
  Updates gps, navsat, odometry, path, and tf_frame plugins to use the
  abstract point drawing class.
* Adds the draw laps functionality, which allows the user to change the color
  of the path as it passes a base point for ease of visibility. This option
  is implemented for the gps and odometry plugins.
* Implements support for the ARROW marker type
  Resolves `#365 <https://github.com/swri-robotics/mapviz/issues/365>`_
* Ensures that Mapviz won't subscribe to empty topic names. This involved
  cleaning up the code for handling subscriptions for all topics. The behavior
  is now:
  
  - All input is trimmed before processing
  - If a topic name is empty, the old subscriber will be shut down and will not subscribe to the empty topic
  
  Resolves `#327 <https://github.com/swri-robotics/mapviz/issues/327>`_
* Fixes a crash in the PointCloud2 plugin
* Caches transformed pointclouds to improve performance
* PointCloud2 plugin "Color Transformer" combo box now properly saves its value
* Return "false" if no other code handles the mouse event
  Fixes `#360 <https://github.com/swri-robotics/mapviz/issues/360>`_
* Contributors: Ed Venator, Marc Alban, P. J. Reed

0.0.5 (2016-05-20)
------------------
* Implement mapviz plug-in for calling the marti_nav_msgs::PlanRoute service.
* Migrate route plugin to use swri_route_util
  This change migrates the mapviz route plugin to use swri_route_util to
  get consistent behavior with route transforms and route position
  interpolation.  As part of this change, the route is now transformed
  with each draw so that it will correctly move around if the transform
  between the fixed frame and the route frame is not constant.
* Add support for mono8 textured markers.
* Implement service for adding and modifying mapviz displays.
* Store and restore position and size of attitude indicator plug-in.
* Fix grid plug-in to correctly display when loading from a config file.
* Changing some "unsigned long"s to "size_t"s.
* Storing source frames individually for plugins w/ buffers
* Fix for `#265 <https://github.com/swri-robotics/mapviz/issues/265>`_; message source frames don't update
  Several plugins were storing the source frames of messages received when
  they first received a message but never updating them, so subsequent
  messages in different frames would be rendered incorrectly.
* Fix for `#339 <https://github.com/swri-robotics/mapviz/issues/339>`_; explicitly depending on OpenCV 2
* Fix route position search
  The route position search would ignore a matching point unless it is
  already transformed, which means that only points that have already been
  searched and missed would be transformed.
  The new logic looks first for the match, then transforms as necessary.
  Unmatched points are ignored.
* Guard against repeated transforms
  A point should only be transformed once, because the mapviz transforms
  are set outside the plugins; `TransformPoint` will now only transform
  un-transformed points.
* Remove unused variable
  prev_position\_ is set, but never actually used.
* Adds route plugin with routeposition marker attachment.
* fixing a variable name
* fixed hidden image/disparity startup issue & issue where mapviz would crash when image/disparity plugins subscribed to empty messages
* Added functionality to subscribe/unsubscribe when hidden to the image plugin and the disparity plugin
* fixed bug with projection
* Adds attitude indicator
* Also updating the disparity plugin
* Fixing `#317 <https://github.com/swri-robotics/mapviz/issues/317>`_
  First, the model view matrix needs to be saved and restored around
  QPainter operations because Qt clears several GL variables.  Also, the
  image plugin needed to explicitly call glMatrixMode(GL_PROJECTION);
  it does a few operations on the projection matrix and was just assuming
  that was the current matrix mode.  Also, I added a function that plugins
  need to override if they want to do QPainter operations; this will
  eliminate unnecessary overhead for plugins that do not.
* Declaring types for Qt signal/slot use properly
* Fixing some typos
* Doing GL drawing on the main thread for `#313 <https://github.com/swri-robotics/mapviz/issues/313>`_
* A plugin for displaying std_msgs/Strings
* Marker plugin will use a QPainter to draw text
  I modified the Marker plugin so that it will use a QPainter to draw
  text labels rather than OpenGL commands.  This doesn't really add any
  functional benefit; it's meant to serve as an example of how to use
  the QPainter.
* Making the Image plugin use image_transport.
  The image_transport package provides support for transparently
  subscribing and publishing to topics using low-bandwidth compressed
  formats; if the publisher supports it, this will cause the Image
  plugin to consume far less bandwidth than before.
* Fixing warnings and cleaning up formatting
* updated mapviz_plugins.xml
* Update map canvas at a fixed rate.
  This update adds a timer to the map canvas to repaint at a fixed rate.
  The default rate is 50 Hz, but there is a method to change it (not
  exposed to the UI at the moment).  50Hz was chosen because it is fast
  enough to give smooth animations and we almost always are running
  mapviz with at least one plugin triggering updates from a 50Hz topic.
* Handle cases where marker topic changes message types.
  This commit makes a better effort to properly support cases where a
  marker topic changes between Marker and MarkerArray during runtime.
* Use ROS' shapeshifter to handle marker/marker arrays.
* This commit adds a class called SelectFrameDialog that plugins can use
  to present the user with a dialog to choose a TF frame. The dialog
  sorts the frames by name and provides an edit box that the user can
  use to filter the frames to a specific substring.
* Indigo compatibility.
  Fixing swri_transform_util and swri_yaml_util API changes from
  Hydro to Indigo.
* Also filtering out clicks that are held for too long.
* Adding a check to prevent the click event from firing if the user is dragging the mouse.
* Fixing an issue that could cause the click publisher plugin's publisher to not be initialized after it's first added.
* Removing some code I had added for debugging.
* Adding a plugin that, when a user clicks on a point, will publish that point's coordinates to a topic.
* Adding color button widget and updating plugins.
  This commit adds a subclass of QPushButton called ColorButton that
  implements a widget for displaying and selecting colors.  We've been
  doing this manually everywhere with duplicated code.  This is a simple
  abstraction but allows us to elminate a lot of duplication, especially
  in plugins that have multiple color selections.
* Adds SelectTopicDialog to mapviz.
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
* Contributors: Edward Venator, Elliot Johnson, Jerry Towler, Marc Alban, Nicholas Alton, P. J. Reed

0.0.4 (2016-01-06)
------------------
* Fixes bad package names in includes.
* Backports navsat plug-in from jade.
* Sorts topic, plug-in, and frame lists in selection dialogs.
* Fixes tf plug-in update.
* Adds a plugin to visualize laser scans based on the laserscan plugin for rviz:
  * Points can be colored by range, or x/y/z axis
  * Points can be colored by interpolation between two colors or rainbow coloring
* Enables the possibility to load one layer tile set
* Contributors: Edward Venator, Marc Alban, P. J. Reed, Vincent Rousseau

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
