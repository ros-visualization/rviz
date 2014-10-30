^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz
^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.4 (2014-10-30)
-------------------
* Fixed stereo support for custom projection matrices
* Fixed read off end of array in triangle_list_marker
* Add dependency on opengl
  rviz calls find_package(OpenGL), so it should have a direct dependency
  on OpenGL. This matters on ARM, where the other packages that rviz
  depends on use OpenGL.ES, and don't provide a transitive dependency on
  OpenGL.
* Update map via QT signal instead of in ros thread
  Resolved issues when running RViz in rqt where the incomingMap callback
  is not issued from RViz's main QThread causing a crash in Ogre. Map
  updates are now handled by emitting a signal to update the map from the
  callback thread.
* fix rainbow color, see `#813 <https://github.com/ros-visualization/rviz/issues/813>`_
* Added TF listener as parameter to constructors of VisualizationManager and FrameManager
* Fix add by topic for Marker and MarkerArray
* Fixed map plugin to only show when active
* stereo: restore camera after rendering (Avoids a segfault)
* fix stereo eye separation
* fix ogre includes
* Contributors: Acorn Pooley, Alex Bencz, Austin, Austin Hendrix, Ben Charrow, Dave Hershberger, Jonathan Bohren, Kei Okada, William Woodall, ZdenekM, v4hn

1.11.3 (2014-06-26)
-------------------
* remove explicit dependency on urdfdom
  urdfdom is provided via urdf and catkin_* CMake variables.
  The current setup was unbalanced anyways because along with urdfdom, urdfdom_headers should have been being depended on and used.
  This precipitated from urdfdom's rosdep key changing as it became a system dependency in Indigo.
* Add ability to delete all markers in Marker plugin
* fix hidden cursor bug
  On some systems loading a pixmap from an svg file can fail.  On these machines
  an empty cursor results, meaning the cursor is invisible inside Rviz.  This
  works around the problem by using an arrow cursor when the desired cursor
  pixmap canot be loaded.
* Install rviz to the global bin
* Added display for sensor_msgs/RelativeHumidity
* Contributors: Acorn Pooley, Adam Leeper, Chad Rockey, Dave Coleman, William Woodall, hersh, trainman419

1.11.2 (2014-05-13)
-------------------
* Fix an issue with rendering laser scans: `#762 <https://github.com/ros-visualization/rviz/issues/762>`_
* Fix an issue with using boost::signal instead of boost::signal2 with tf
  tf recently moved to boost::signal2, so the effort display needed to be updated too
  I made it so that it would conditionally use boost::signal2 if the tf version is greater than or equal to 1.11.3
  I also fixed some compiler warnings in this code
  closes `#700 <https://github.com/ros-visualization/rviz/issues/700>`_
* Contributors: Vincent Rabaud, William Woodall

1.11.1 (2014-05-01)
-------------------
* fix fragment reference in point_cloud_box.material
  Closes `#759 <https://github.com/ros-visualization/rviz/issues/759>`_
* upgrade ogre model meshs with the OgreMeshUpgrader from ogre 1.9
* Changed TF listener to use a dedicated thread.
* Speed up point cloud rendering by caching some computations and using proper loop iterations
* Fixed rendering of mesh resource type markers with respect to texture rendering and color tinting
* Fix segfault on exit for OSX
* Fix memory leak in BillboardLine destructor (material not being destroyed correctly)
* Fix disabling of groups (`#709 <https://github.com/ros-visualization/rviz/issues/709>`_)
  This was broken with commit 5897285, which reverted the changes in
  commit c6dacb1, but rather than only removing the change concerning
  the read-only attribute, commented out the entire check, including
  the ``parent_->getDisableChildren()`` call (which existed prior to
  commit 5897285).
* Add missing libraries to rviz link step, fixes OS X build.
* fix failing sip bindings when path contains spaces
* EffortDisplay: Added a check to avoid segfaults when receiving a joint state without efforts
* Contributors: Dirk Thomas, Hans Gaiser, Jordan Brindza, Mike Purvis, Mirko, Siegfried-A. Gevatter Pujals, Timm Linder, Vincent Rabaud, William Woodall

1.11.0 (2014-03-04)
-------------------
* fixing problems with urdfdom_headers 0.3.0
* Contributors: William Woodall

1.10.14 (2014-03-04)
--------------------
* Fixed a bug in tutorials caused by uninitialized ros::Time here.
* Contributors: Dave Hershberger, William Woodall

1.10.13 (2014-02-26)
--------------------
* Use assimp-dev as a `build_depend` and leave assimp as the `run_depend`
* Contributors: Scott K Logan, William Woodall

1.10.12 (2014-02-25)
--------------------
* Shiboken is now disabled when a version which would segfault is detected (fix `#728 <https://github.com/ros-visualization/rviz/issues/728>`_)
* Eigen is now found using the FindEigen.cmake from the `cmake_modules` package.
* Added support for rendering rviz in stereo.
  For more information see this commit: https://github.com/ros-visualization/rviz/commit/9cfaf78e2ae8d34e4481de19568b353964846842
* Added a "Queue Size" option for the Range display type.
* Added Ogre-1.10 compatibility
  This allows rviz to compile (and work) against Ogre 1.10 (currently
  the latest version of ogre).
  It also still works with earlier versions of Ogre (tested with Ogre
  1.7.4 as installed via debs on Ubuntu 12.04).
* Now includes ogre without OGRE prefix
  This is necessary to find Ogre files in the right place with
  compatibility between Ogre < 1.9 and Ogre >= 1.9.
  This is also necessary when 2 versions of Ogre are installed on the
  build machine.
* RVIZ doesn't use __connection_header from incoming messages, but only uses ros::MessageEvent's
* Better feature detection for assimp version
  The unified headers were introduced in Assimp 2.0.1150, so checking for Assimp 3.0.0 is not quite the best solution.
  See https://github.com/assimp/assimp/commit/6fa251c2f2e7a142bb861227dce0c26362927fbc
* Contributors: Acorn Pooley, Benjamin Chrétien, Dave Hershberger, Kevin Watts, Scott K Logan, Siegfried-A. Gevatter Pujals, Tully Foote, William Woodall, hersh

1.10.11 (2014-01-26)
--------------------
* Fixed in selection_manager which allows interactive markers to work with orthographic cameras views
* Add support for yamlcpp 0.5 with backwards compatibility with yamlcpp 0.3
* Fixed message type for Polygon display. The polygon display type actually subscribes to PolygonStamped.
* Contributors: Austin, Ken Tossell, Max Schwarz, William Woodall

1.10.10 (2013-12-22)
--------------------
* Fixed a severe memory leak with markers and marker arrays: `#704 <https://github.com/ros-visualization/rviz/issues/704>`_ and `#695 <https://github.com/ros-visualization/rviz/issues/695>`_
* Contributors: David Gossow, Vincent Rabaud

1.10.6 (2013-09-03)
-------------------
* Added a new method for adding displays, by topic as opposed to by type.
* Added new exception handling for loading mesh files which have no content.

1.10.5 (2013-08-28 03:50)
-------------------------
* Removed run_dep on the media_export package
* All previous history is not curated, see the commit `history <https://github.com/ros-visualization/rviz/commits/hydro-devel>`.
