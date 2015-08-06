^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz
^^^^^^^^^^^^^^^^^^^^^^^^^^

1.10.20 (2015-08-05)
--------------------
* Force and Torque can now be scaled separately in the Wrench display: `#862 <https://github.com/ros-visualization/rviz/issues/862>`_
* Fixed a bug in the Wrench display: `#883 <https://github.com/ros-visualization/rviz/issues/883>`_
* Improved error checking when loading ascii stl files.
* Suppressing some new CMake warnings by setting cmake policies.
* Re-enable most all of the tests.
* Check if position and orientation of links of robots contain NaNs when updating pose of robot links.
* Contributors: Carlos Agüero, Gustavo N Goretkin, Jonathan Bohren, Kei Okada, Ryohei Ueda, William Woodall, louise

1.10.19 (2015-02-13)
--------------------
* Fixed a mesh memory leak in ogre_helpers/mesh_shape.h/.cpp
  This fixes a memory leak which is caused due to no meshes ever being destroyed without removing the mesh from the mesh manager.
  This gets really bad when drawing meshes with 50K triangles at 10Hz, resulting in a leak rate @ ~60MB/sec.
* Map via QT signal instead of in ros thread
  Resolved issues when running RViz in rqt where the incoming Map callback is not issued from RViz's main QThread causing a crash in Ogre.
  Map updates are now handled by emitting a signal to update the map from the callback thread.
* Fixed read off end of array in triangle_list_marker.
* Fix add by topic for Marker and MarkerArray
* Contributors: Alex Bencz, Ben Charrow, Dave Hershberger, Jonathan Bohren, William Woodall

1.10.18 (2014-07-29)
--------------------
* Backport fix from Indigo for a warning which fails on the farm.
  Achieved by cherry-picking commit ``12423d1969da5669e6cb0ee698a3483a78ef38dd`` from Indigo.
* Contributors: William Woodall

1.10.17 (2014-07-29)
--------------------
* Fix a bug where the map was shown even if the map plugin was not active
* Fixed stereo: restore camera after rendering
  Avoids a segfault
* fix stereo eye separation
* fix ogre includes
* Added python headers for python bindings
  Adding ``${PYTHON_INCLUDE_DIRS}`` to the sip python bindings.
  Needed at least on OSX/10.9 to compile successfully.
* Fix screenshot dialog. Fixes `#783 <https://github.com/ros-visualization/rviz/issues/783>`_
* fix hidden cursor bug
  On some systems loading a pixmap from an svg file can fail.  On these machines
  an empty cursor results, meaning the cursor is invisible inside Rviz.  This
  works around the problem by using an arrow cursor when the desired cursor
  pixmap canot be loaded.
* rviz is now installed to the global bin folder which is on the PATH
* Added partial support for shading of triagular meshes
* Added display for sensor_msgs/RelativeHumidity
* Fix disabling of groups (`#709 <https://github.com/ros-visualization/rviz/issues/709>`_)
  This was broken with commit 5897285, which reverted the changes in
  commit c6dacb1, but rather than only removing the change concerning
  the read-only attribute, commented out the entire check, including
  the ``parent_->getDisableChildren()`` call (which existed prior to
  commit 5897285).
* Contributors: Acorn Pooley, Chad Rockey, Dave Hershberger, Nikolaus Demmel, Siegfried-A. Gevatter Pujals, William Woodall, hersh, trainman419, v4hn

1.10.16 (2014-05-13)
--------------------
* Fix an issue with rendering laser scans: `#762 <https://github.com/ros-visualization/rviz/issues/762>`_
* Contributors: Vincent Rabaud, William Woodall

1.10.15 (2014-05-01)
--------------------
* Forward ported #707
  Update frame_manager.cpp
  Changed TF listener to use a dedicated thread.
* Fix segfault on exit for OSX
* Fixed rendering of mesh resource type markers with respect to texture rendering and color tinting
* Fix memory leak in BillboardLine destructor (material not being destroyed correctly)
* EffortDisplay: Added a check to avoid segfaults when receiving a joint state without efforts
* Speed up point cloud rendering
  this is mostly caching some computations and using proper loop iterations
* Contributors: Hans Gaiser, Jordan Brindza, Mirko, Timm Linder, Vincent Rabaud, William Woodall

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
