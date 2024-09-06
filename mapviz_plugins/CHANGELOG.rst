^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mapviz_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.3 (2024-09-06)
------------------
* Adding rotation to image_plugin (`#682 <https://github.com/swri-robotics/mapviz/issues/682>`_)
  * Adding rotation to image_plugin
  ---------
  Co-authored-by: David Anthony <djanthony@gmail.com>
* fixed button connect (`#807 <https://github.com/swri-robotics/mapviz/issues/807>`_)
* Restore and refactor use_latest_transforms option for point drawing plugins like Odometry. (`#705 <https://github.com/swri-robotics/mapviz/issues/705>`_)
* Contributors: Anthony Jiang, Marc Alban, TobinHall

1.4.2 (2022-12-13)
------------------
* Merge pull request `#716 <https://github.com/swri-robotics/mapviz/issues/716>`_ from swri-robotics/pose_publisher
  Pose publisher
* Merge pull request `#752 <https://github.com/swri-robotics/mapviz/issues/752>`_ from matt-attack/add-stuff
  Add bus features
* Update dates
* Add bus features
* Merge pull request `#734 <https://github.com/swri-robotics/mapviz/issues/734>`_ from matt-attack/improve-textured-marker
  Fix Issues With Textured Marker Plugin
* Make sure textured marker can handle delayed transforms and use marker alpha values
* Merge pull request `#715 <https://github.com/swri-robotics/mapviz/issues/715>`_ from swri-robotics/posearray
  Add visualizer for PoseArray
* Merge pull request `#718 <https://github.com/swri-robotics/mapviz/issues/718>`_ from agyoungs/fix-text-marker-disable
  Allow text markers to be enabled/disabled via their namespace using the marker plugin checkboxes
* Merge pull request `#721 <https://github.com/swri-robotics/mapviz/issues/721>`_ from rjb0026/720-allow-building-with-c++14-for-version-greater-than-kinetic
  720 allow building with c++14 for version greater than kinetic
* Conditionally setting c++ standard per target.
* Removed explicit setting of compile flags for c++ standard version and replaced with conditional for building with C++11 in kinetic for mapviz and mapviz_plugins.
* Allow text markers to be enabled/disabled via their namespace using the marker plugin checkboxes
* Add visualizer for PoseArray
* Contributors: Alex Youngs, David Anthony, Kevin Nickels, Matthew, Matthew Bries, rjb0026

1.4.1 (2020-10-22)
------------------
* Constrain the minimum line and point marker sizes to be 1 pixel wide. (`#704 <https://github.com/swri-robotics/mapviz/issues/704>`_)
* Fix compile warning in ROS Noetic (`#706 <https://github.com/swri-robotics/mapviz/issues/706>`_)
* Contributors: Marc Alban, P. J. Reed

1.4.0 (2020-07-28)
------------------
* Support ROS Noetic (`#696 <https://github.com/swri-robotics/mapviz/issues/696>`_)
* Add Visualization for marti_nav_msgs::TrackedObjectArray and marti_nav_msgs::ObstacleArray (`#701 <https://github.com/swri-robotics/mapviz/issues/701>`_)
* Merge pull request `#689 <https://github.com/swri-robotics/mapviz/issues/689>`_ from mdgrogan/update-distance-on-move-point
* Use higher precision in the coordinate picker for wgs84 (`#692 <https://github.com/swri-robotics/mapviz/issues/692>`_)
* Clear the namespace list after hitting the clear button. (`#691 <https://github.com/swri-robotics/mapviz/issues/691>`_)
* Update the displayed distance continuously while moving a point.
* Contributors: Matt Grogan, Matthew, P. J. Reed

1.3.0 (2020-05-13)
------------------
* Fix plan route visibility (`#680 <https://github.com/swri-robotics/mapviz/issues/680>`_)
* Disable coordinate picker and measuring plugin when hidden (`#678 <https://github.com/swri-robotics/mapviz/issues/678>`_)
* Increase odom queue size so messages dont drop (`#676 <https://github.com/swri-robotics/mapviz/issues/676>`_)
* Add float plugin (`#665 <https://github.com/swri-robotics/mapviz/issues/665>`_)
* Make string plugin also support string stamped (`#666 <https://github.com/swri-robotics/mapviz/issues/666>`_)
* Don't draw arrow for invalid orientation (`#673 <https://github.com/swri-robotics/mapviz/issues/673>`_)
* Remove OpenGL warning (`#667 <https://github.com/swri-robotics/mapviz/issues/667>`_)
* Change for loop to while loop in marker plugin so that iterators aren't getting double incremented on erases. (`#663 <https://github.com/swri-robotics/mapviz/issues/663>`_)
* Add ability to show all covariances (`#659 <https://github.com/swri-robotics/mapviz/issues/659>`_)
* Add ability to use relative paths for images in the robot image plugin (`#658 <https://github.com/swri-robotics/mapviz/issues/658>`_)
* Resolves segfault issue on ClearHistory() function call (`#651 <https://github.com/swri-robotics/mapviz/issues/651>`_)
* Default show background and measuring text to true (`#657 <https://github.com/swri-robotics/mapviz/issues/657>`_)
* Contributors: Daniel D'Souza, Matthew, Matthew Grogan, jgassaway, jreyes512, knickels

1.2.0 (2019-09-04)
------------------
* Add text to measuring plugin (`#640 <https://github.com/swri-robotics/mapviz/issues/640>`_)
* Add mapviz plug-in for PoseStamped messages. (`#641 <https://github.com/swri-robotics/mapviz/issues/641>`_)
* Fix occupancy grid to load color scheme from configuration. (`#642 <https://github.com/swri-robotics/mapviz/issues/642>`_)
* Restore GL_UNPACK_ALIGNMENT to 4 to prevent corruption of Qt font rendering. (`#643 <https://github.com/swri-robotics/mapviz/issues/643>`_)
* Add ability to show and hide markers by namespace (`#636 <https://github.com/swri-robotics/mapviz/issues/636>`_)
* Fixed layout of MeasuringPlugin (`#633 <https://github.com/swri-robotics/mapviz/issues/633>`_)
* Fixed marker plugin to use swri_transform_util to ensure wgs84 markers work properly (`#635 <https://github.com/swri-robotics/mapviz/issues/635>`_)
* Contributors: Arkady Shapkin, Marc Alban, Matthew, Matthew Grogan, agyoungs

1.1.1 (2019-05-17)
------------------
* Textured Marker Adjustments (`#611 <https://github.com/swri-robotics/mapviz/issues/611>`_, `#616 <https://github.com/swri-robotics/mapviz/issues/616>`_) (`#625 <https://github.com/swri-robotics/mapviz/issues/625>`_)
* fixed issue `#623 <https://github.com/swri-robotics/mapviz/issues/623>`_ by updating UI field to read "Draw Style:" (`#624 <https://github.com/swri-robotics/mapviz/issues/624>`_)
* Contributors: mattrich37

1.1.0 (2019-02-20)
------------------
* Improve MarkerPlugin (`#603 <https://github.com/swri-robotics/mapviz/issues/603>`_)
  * Improved performance of MarkerPlugin::handleMarker()
  * Support Text marker alpha channel
  * Don't use QColor for glColor4f
  * Use marker namespace and id as markers map key
* [mapviz_plugins/attitude_indicator] Minor refactoring and redundant logging removed (`#617 <https://github.com/swri-robotics/mapviz/issues/617>`_)
* 606 sequential measuring (`#607 <https://github.com/swri-robotics/mapviz/issues/607>`_)
  * Moved distance calculation to trigger on release of mouse. Added case to prevent distance calculation while moving map.
  * Added vertices and lines between measurement points. Left click to add point. Right click to delete. Added color selection for points and clear button to ui.
  * Added cumulative distance measurements from multiple points
  * Fixed individual and cumulative distance measurements. Changed it to only measure distance between points and not from fixed origin and first point
  * Moved distance calculation into separate function which is called when deleting a point, adding a point, or rearranging points.
* Add image size check to textured marker plugin to prevent crashes. (`#613 <https://github.com/swri-robotics/mapviz/issues/613>`_)
  * Add image size check to textured marker plugin to prevent crashes.
* Fixed typo in string (`#608 <https://github.com/swri-robotics/mapviz/issues/608>`_)
* 605 add reset button marker (`#609 <https://github.com/swri-robotics/mapviz/issues/609>`_)
  * Added Clear all marker buttons, added case for clear all support to markers
* Contributors: Arkady Shapkin, Matthew, jbdaniel18

1.0.1 (2019-01-25)
------------------
* Use shared tf manager in measuring_plugin (`#604 <https://github.com/swri-robotics/mapviz/issues/604>`_)
* Contributors: jgassaway

1.0.0 (2019-01-23)
------------------
* Sharing tf_manager\_ between main app and plugins (`#555 <https://github.com/swri-robotics/mapviz/issues/555>`_)
* Fix potential segfault in pointcloud plug-in. (`#602 <https://github.com/swri-robotics/mapviz/issues/602>`_)
* Add Measuring Plugin (`#598 <https://github.com/swri-robotics/mapviz/issues/598>`_)
* Contributors: Davide Faconti, Marc Alban, Matthew

0.3.0 (2018-11-16)
------------------
* Merge all -devel branches into a single master branch
* Don't transform laser scans twice (`#544 <https://github.com/swri-robotics/mapviz/issues/544>`_)
* Improving point_drawing plugins and bug fix of tf_plugin (`#557 <https://github.com/swri-robotics/mapviz/issues/557>`_)
* OpenGL rendering of PointClouds  (2X speedup) (`#558 <https://github.com/swri-robotics/mapviz/issues/558>`_)
* Occupancy grid (new plugin) (`#568 <https://github.com/swri-robotics/mapviz/issues/568>`_)
* Bug fix in image plugin (`#563 <https://github.com/swri-robotics/mapviz/issues/563>`_)
* Fix Indigo build, clean up warnings (`#597 <https://github.com/swri-robotics/mapviz/issues/597>`_)
* Create Coordinate Picker plugin (`#593 <https://github.com/swri-robotics/mapviz/issues/593>`_)
* Contributors: Davide Faconti, Ed Venator, Edward Venator, Elliot Johnson, Jerry Towler, Marc Alban, Matthew, Matthew Bries, Mikael Arguedas, Neal Seegmiller, Nicholas Alton, P. J. Reed, Vincent Rousseau

0.2.6 (2018-07-31)
------------------
* Fix timestamp interval (`#588 <https://github.com/swri-robotics/mapviz/issues/588>`_)
* Update path_plugin.cpp (`#586 <https://github.com/swri-robotics/mapviz/issues/586>`_)
* Replace depcreated plugin macro with newer version
* Contributors: Matthew, P. J. Reed, camjaws

0.2.5 (2018-04-12)
------------------
* Add clear history functionality.
* Add support for newlines in text marker plugin (`#572 <https://github.com/swri-robotics/mapviz/issues/572>`_)
* New plugin to send commands to move_base
* Glew warning fixed (`#539 <https://github.com/swri-robotics/mapviz/issues/539>`_)
* Added "keep image ratio" to Image plugin (`#543 <https://github.com/swri-robotics/mapviz/issues/543>`_)
* Remove copy and paste of Print...
* PointCloud2 speed improvement (`#531 <https://github.com/swri-robotics/mapviz/issues/531>`_)
* Dead code removed (`#535 <https://github.com/swri-robotics/mapviz/issues/535>`_)
* Ratio added to robot_image_plugin (`#530 <https://github.com/swri-robotics/mapviz/issues/530>`_)
* Speed up improvement in LaserScan and PointCloud2 (`#525 <https://github.com/swri-robotics/mapviz/issues/525>`_)
* Re-add GPSFix plugin to kinetic-devel (`#519 <https://github.com/swri-robotics/mapviz/issues/519>`_)
* Add support for unpacking rgb8 in pointcloud2s
* Use non-deprecated pluginlib macro
* Add plug-in for drawing and publishing a polygon.
* change the signal that triggers AlphaEdited + minor changes (`#514 <https://github.com/swri-robotics/mapviz/issues/514>`_)
* Added timestamp display to odometry for kinetic
* Contributors: Davide Faconti, Marc Alban, Matthew Bries, Mikael Arguedas, P. J. Reed, jgassaway

0.2.4 (2017-08-11)
------------------
* Add /wgs84 frame to point click publisher when available.
* Transform cube and arrow markers properly
* Contributors: Marc Alban, P. J. Reed

0.2.3 (2016-12-10)
------------------
* Delete markers that have expired and remove error message. (`#454 <https://github.com/evenator/mapviz/issues/454>`_)
* Fix segfault in pointcloud2 plug-in when pointcloud is empty. (`#450 <https://github.com/evenator/mapviz/issues/450>`_)
* Initialize buffer size variable. (`#447 <https://github.com/evenator/mapviz/issues/447>`_)
* Contributors: Marc Alban

0.2.2 (2016-12-07)
------------------
* Migrated OpenCV to 3.1 (default in Kinetic)
* General code cleanup of mapviz_plugins
  This doesn't change any functionality; it's just cleaning up code.  Notably, this will:
  - Fix all warnings (notably lots of ones about type casting)
  - Move all .ui files to their own directory
  - Remove unused variables
  - Remove commented-out code
  - Make spacing and indentation consistent
  - Make brace style consistent
* Contributors: Brian Holt, Marc Alban, P. J. Reed

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
* Add option to not scale arrows with zoom level
  This adds a checkbox to all of the plugins that can draw a series of
  coordinates as arrows; i. e., the NavSatFix, Odometry, and TF Frame
  plugins.  This checkbox will control whether the arrows are drawn at a fixed
  size regardless of zoom level or whether they are scaled with the zoom level.
  Resolves `#414 <https://github.com/swri-robotics/mapviz/issues/414>`_
* Fix signed comparison warnings in mapviz_plugins
* Adding a way for plugin config widgets to resize
  - Adding an event plugins can emit to indicate their geometry has changed
  - Modifying the PCL2 plugin to use it as an example
  Fixes `#393 <https://github.com/swri-robotics/mapviz/issues/393>`_
* Adding default values for uninitialized variables
  Resolves `#372 <https://github.com/swri-robotics/mapviz/issues/372>`_
* Creates and implements an abstract class for drawing point paths
  Updates gps,navsat,odometry,path, and tf_frame plugins to use the
  abstract point drawing class. Also adds the draw laps functionality
  which will change the color of the path as it passes a base point for
  ease of visibility, currently implemented on gps and odometry plugins.
  Conflicts:
  mapviz_plugins/CMakeLists.txt
  mapviz_plugins/include/mapviz_plugins/gps_plugin.h
  mapviz_plugins/src/gps_config.ui
  mapviz_plugins/src/gps_plugin.cpp
* Ensuring that Mapviz won't subscribe to empty topic names (`#379 <https://github.com/swri-robotics/mapviz/issues/379>`_)
  Clean up and made more consistent the code for handling subscriptions for all topics.
  The behavior is now:
  - All input is trimmed before processing
  - If a topic name is empty, the old subscriber will be shut down and will not subscribe to the empty topic
  Resolves `#327 <https://github.com/swri-robotics/mapviz/issues/327>`_
* Fixing some typos in documentation.
* Implementing support for the ARROW marker type
  Resolves `#365 <https://github.com/swri-robotics/mapviz/issues/365>`_
* Contributors: Ed Venator, Marc Alban, P. J. Reed

0.2.0 (2016-06-23)
------------------
* Update Qt to version 5
* Fixing a crash in the PointCloud2 plugin
  Also sneaking in a few more changes:
  - Caching transformed clouds to improve performance
  - Properly saving the value of the "Color Transformer" combo box
* Returning "false" if no other code handles the mouse event
  Fixes `#360 <https://github.com/swri-robotics/mapviz/issues/360>`_
* Contributors: Ed Venator, P. J. Reed

0.1.3 (2016-05-20)
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
* Adding attitude indicator plugin.
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
* GPS plugin snuck back into CMakeLists.txt
* A plugin for displaying std_msgs/Strings
* Marker plugin will use a QPainter to draw text
  I modified the Marker plugin so that it will use a QPainter to draw
  text labels rather than OpenGL commands.  This doesn't really add any
  functional benefit; it's meant to serve as an example of how to use
  the QPainter.
* Fixing warnings and cleaning up formatting
* updated mapviz_plugins.xml
* add pointcloud2 plugin
* Update map canvas at a fixed rate.
  This update adds a timer to the map canvas to repaint at a fixed rate.
  The default rate is 50 Hz, but there is a method to change it (not
  exposed to the UI at the moment).  50Hz was chosen because it is fast
  enough to give smooth animations and we almost always are running
  mapviz with at least one plugin triggering updates from a 50Hz topic.
* Making the Image plugin use image_transport.
  The image_transport package provides support for transparently
  subscribing and publishing to topics using low-bandwidth compressed
  formats; if the publisher supports it, this will cause the Image
  plugin to consume far less bandwidth than before.
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
* Contributors: Elliot Johnson, Jerry Towler, Marc Alban, Nicholas Alton, P. J. Reed

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
