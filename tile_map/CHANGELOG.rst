^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tile_map
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.1.1 (2015-11-17)
------------------
* Mark single argument constructors explicit.
* Contributors: Marc Alban

0.1.0 (2015-09-29)
------------------

0.0.3 (2015-09-28)
------------------

0.0.2 (2015-09-27)
------------------

0.0.1 (2015-09-27)
------------------
* Adds missing qt-opengl dependency to tile_map.
* Renames all marti_common packages that were renamed.
  (See http://github.com/swri-robotics/marti_common/issues/231)
* Fixes catkin_lint problems that could prevent installation.
* updates cmake version to squash the CMP0003 warning
* removes dependencies on build_tools
* uses format 2 package definition
* implements subdivision of map tiles at the highest zoom levels to correctly warp map to the canvas coordinate system
* only transform tile map when the transform changes
* fixes related to merging catkin branch into tile_map and building on Ubuntu 12.04
* initial working implementation of tile map plugin
* Contributors: Ed Venator, Edward Venator, Marc Alban, P. J. Reed
