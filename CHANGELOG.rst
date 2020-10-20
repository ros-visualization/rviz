^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz
^^^^^^^^^^^^^^^^^^^^^^^^^^

1.14.3 (2020-10-20)
-------------------
* Fixed linking issue on Debian Buster: missing ~ScrewDisplay
* Contributors: Robert Haschke

1.14.2 (2020-10-19)
-------------------
* Merged melodic-devel improvements
  * [fix]   SplitterHandle: Consider scrollbar width. Fixes `#1545 <https://github.com/ros-visualization/rviz/issues/1545>`_.
  * [fix]   Handle InvalidNameException when loading robot description
  * [fix]   WrenchVisual: Add missing initialization of ``hide_small_values_``
  * [fix]   Fixup `#1519 <https://github.com/ros-visualization/rviz/issues/1519>`_: Correctly (and efficiently) handle 3-byte pixel formats
  * [maint] Adapt to clang-format-10
* [fix]     Selectively install font definition file, suppressing Ogre runtime warning
* [fix]     Suppress cmake warning from libassimp
* [feature] Generic ScrewDisplay for TwistStamped, AccelStamped, and WrenchStamped
* [maint]   Find OGRE based on cmake config and fallback to pkg-config
* [maint]   Correctly link against libassimp for version >= 5.0
* Contributors: Chris Lalancette, Markus Vieth, Robert Haschke, Sean Yen, Wolf Vollprecht

1.14.1 (2020-06-24)
-------------------
* Merged melodic-devel improvements
  * [feature] Make the goal pose tool magenta (`#1520 <https://github.com/ros-visualization/rviz/issues/1520>`_)
  * [bugfix]  Fix memory access in case of 3-byte pixel formats (`#1519 <https://github.com/ros-visualization/rviz/issues/1519>`_)
  * [bugfix]  PropertyTree: set custom SelectionModel only with valid model (`#1504 <https://github.com/ros-visualization/rviz/issues/1504>`_)
* [bugfix] Fix initial pose and goal pose tools (`#1510 <https://github.com/ros-visualization/rviz/issues/1510>`_)
* [bugfix] Fix cutoff in LaserScanDisplay (`#1512 <https://github.com/ros-visualization/rviz/issues/1512>`_)
* [maint]  Added test/send_point_cloud_2.py (`#1514 <https://github.com/ros-visualization/rviz/issues/1514>`_)
* [maint]  Fix (or silence) warnings of newer gcc / clang
* [maint]  clang-format-10
* PropertyTree: set custom SelectionModel only with valid model (`#1504 <https://github.com/ros-visualization/rviz/issues/1504>`_)
* Contributors: Evan Flynn, G.Doisy, Ivor Wanders, Michael Görner, Robert Haschke

1.14.0 (2020-05-11)
-------------------
* [feature] Added scale and color properties to Marker selection panel (`#1436 <https://github.com/ros-visualization/rviz/issues/1436>`_)
* [feature] `queue-size` property to `MessageFilterDisplay` (`#1428 <https://github.com/ros-visualization/rviz/issues/1428>`_)
* [feature] Added ability to set Orbit camera's field of view (`#1362 <https://github.com/ros-visualization/rviz/issues/1362>`_)
* [feature] Added ability to set alpha of Axes and TF displays (`#1357 <https://github.com/ros-visualization/rviz/issues/1357>`_)
* [maint] Finalize `tf2` migration (`#1497 <https://github.com/ros-visualization/rviz/issues/1497>`_)
  Support for `tf1` is dropped. The old `getTFClient()` needs to be replaced by `getTF2BufferPtr()`.
* [maint] clang-tidy fixes
* [maint] Modified Python API to allow import rviz.srv (for service messages provided by rviz)
  In your python scripts, replace `import rviz` with: `from rviz import bindings as rviz`
* [maint] Dropped support for outdated upstream libraries
  * OGRE < 1.9
  * boost::filesystem < 3
  * assimp <= 2.0
  * Qt < 5
* [maint] API cleanup + fix of several warnings (requiring ABI changes)
  - provide virtual constructors were required
  - fix signed/unsigned API params
  - Removed redundant forward of _notifyCurrentCamera()
  - Renamed `add_tool_action_` -> `toolbar_separator_`
  - MarkerSelection: fixed constness of methods
  - Removed deprecated `STLLoader`
* [maint] Ogre 1.9 ... 1.12+ compatibility (Merge pull request `#1434 <https://github.com/ros-visualization/rviz/issues/1434>`_)
  - Ogre compatibility header `ogre_helpers/compatibility.h` to mitigate API differences
  - Avoid referring to anonymous scene nodes by name
* [maint] Prefer vendor-specific OpenGL library
* [maint] Changed semantics of saving
  - When `is_read_only_` is false, the property doesn't need to be saved as its value is computed
  - When `save_` is false, the property and all its children are not saved
* Contributors: Alex Spitzer, Fan Jiang, Hans Gaiser, Maarten de Vries, Micho Radovnikovich, Robert Haschke, Sean Yen, Simon Schmeisser, dkaznacheev

1.13.12 (2020-05-03)
--------------------
* [maint] clang-format (`#1502 <https://github.com/ros-visualization/rviz/issues/1502>`_)
* [maint] Modernize python tests + examples
* [maint] Fix clang compiler warnings
* [maint] clang-tidy
* [maint] Require C++11
* Contributors: Robert Haschke

1.13.11 (2020-04-27)
--------------------
* [feature] Provide load_config and save_config ROS services
* [maint]   clang-tidy fixes (`#1494 <https://github.com/ros-visualization/rviz/issues/1494>`_)
* Contributors: Robert B Anderson, Robert Haschke

1.13.9 (2020-03-31)
-------------------
* Fixup  Sphere being off center (`#1487 <https://github.com/ros-visualization/rviz/issues/1487>`_)
* Revert "Switch libogre-dev to build_depend (`#1482 <https://github.com/ros-visualization/rviz/issues/1482>`_)"
* Configure yaml-cpp include directory (`#1483 <https://github.com/ros-visualization/rviz/issues/1483>`_)
* Contributors: Peter Lehner, Robert Haschke, Wolfgang Merkt

1.13.8 (2020-03-29)
-------------------
* [feature]     Forward focus in PropertyTree to value field (`#1480 <https://github.com/ros-visualization/rviz/issues/1480>`_)
* [feature]     Smooth meshes for sphere, cylinder, and cone (`#1463 <https://github.com/ros-visualization/rviz/issues/1463>`_)
* [fix]         Handle invalid floats in points of arrow marker (`#1471 <https://github.com/ros-visualization/rviz/issues/1471>`_)
* [fix]         Catch exceptions when creating publishers in tools (`#1467 <https://github.com/ros-visualization/rviz/issues/1467>`_)
* [maintanence] Modernize setup.py and cmake  (`#1481 <https://github.com/ros-visualization/rviz/issues/1481>`_)
* [maintanence] Switch libogre-dev to build_depend (`#1482 <https://github.com/ros-visualization/rviz/issues/1482>`_)
* Contributors: Alejandro Hernández Cordero, Michael Görner, Robert Haschke, Simon Schmeisser, Wolfgang Merkt

1.13.7 (2019-12-18)
-------------------
* [fix] Fix segfault when removing displays on presence of DisplayGroupVisibilityProperty
* [fix] CameraDisplay: don't call getCameraInfoTopic() for empty topic
* [fix] MarkerDisplay: clear old markers on topic change (`#1455 <https://github.com/ros-visualization/rviz/issues/1455>`_)
* [maintanence] Fix various warnings
* [maintanence] Support python3 for python bindings (`#1454 <https://github.com/ros-visualization/rviz/issues/1454>`_)
* Contributors: Mike Purvis, Robert Haschke

1.13.6 (2019-11-25)
-------------------
* [fix] Memory leak in rviz::Robot
* [fix] assimp importer: repair invalid normals (`#1452 <https://github.com/ros-visualization/rviz/issues/1452>`_)
* [fix] Fixup cmake file issues
  * Remove redundant include_directories()
  * Generate export headers in devel space
  * Use cmake find_package(yaml-cpp) (`#1445 <https://github.com/ros-visualization/rviz/issues/1445>`_)
* [fix] Gracefully ignore invalid floats (nans) in:
  * LineStripMarker (`#1440 <https://github.com/ros-visualization/rviz/issues/1440>`_)
  * EffortDisplay (`#1437 <https://github.com/ros-visualization/rviz/issues/1437>`_)
* [fix] MovableText: correctly rotate AABB
* [fix] Correctly delete old marker if its type changed
* [maintanence] MovableText:
  * Simplify scaling (there was a scaling by factor 0.5 in getWorldTransforms(), requiring a scaling of 2.0 in _setupGeometry())
  * Simplify _setupGeometry()
* [maintanence] ignore catkin_lint errors/warnings
* [maintanence] Properties: inform model about changed data
* Contributors: Antoine Hoarau, Michael Görner, Robert Haschke

1.13.5 (2019-09-29)
-------------------
* [fix] ImageDisplay/CameraDisplay: fix status reporting / report frame issues (`#1425 <https://github.com/ros-visualization/rviz/issues/1425>`_)
* [fix] Fix `#1422 <https://github.com/ros-visualization/rviz/issues/1422>`_: update recent config to actual filename
* [fix] ImageDisplayBase/MarkerDisplay: increase subscriber queue size with filter queue size
* [fix] Enable mouse tracking for RenderPanel (`#1433 <https://github.com/ros-visualization/rviz/issues/1433>`_)
* Contributors: Robert Haschke, Simon Schmeisser

1.13.4 (2019-09-20)
-------------------
* [feature] Enable `rviz -d my_config` as shortcut for ~/.rviz/my_config.rviz (`#1422 <https://github.com/ros-visualization/rviz/issues/1422>`_)
* [feature] Added --fullscreen cmdline option (`#1413 <https://github.com/ros-visualization/rviz/issues/1413>`_)
* [feature] RobotModelDisplay: Poll robot_description at 1Hz until found
* [feature] Use assimp to load STL files (`#1365 <https://github.com/ros-visualization/rviz/issues/1365>`_)
* [feature] Added frame-aligned view controller (`#1405 <https://github.com/ros-visualization/rviz/issues/1405>`_)
* [feature] Systematically check parameters of incoming marker messages (`#1275 <https://github.com/ros-visualization/rviz/issues/1275>`_, `#1400 <https://github.com/ros-visualization/rviz/issues/1400>`_)
* [feature] MarkerDisplay: clear statuses on disable()
* [feature] Increase zoom range in orbit view controllers (`#1373 <https://github.com/ros-visualization/rviz/issues/1373>`_)
* [feature] Add shortcut "C" to the Publish Point tool (`#1321 <https://github.com/ros-visualization/rviz/issues/1321>`_)
* [fix] ESC shortcut to leave fullscreen prevents canceling tools from working (`#1257 <https://github.com/ros-visualization/rviz/issues/1257>`_)
* [fix] Fixed color tinting of mesh markers (`#1424 <https://github.com/ros-visualization/rviz/issues/1424>`_)
* [fix] Fixed several issues with Camera and Image display (`#1409 <https://github.com/ros-visualization/rviz/issues/1409>`_)
* [fix] Fixed several issues with view controllers, particularly FPS (`#1407 <https://github.com/ros-visualization/rviz/issues/1407>`_)
* [fix] Fixed (small) memory leak in TFDisplay (`#1408 <https://github.com/ros-visualization/rviz/issues/1408>`_)
* [fix] Fixed memory leak in PointCloud display in case of constant /clock (`#1412 <https://github.com/ros-visualization/rviz/issues/1412>`_)
* [fix] Fixed memory leak in MapDisplay (`#1406 <https://github.com/ros-visualization/rviz/issues/1406>`_)
* [fix] Properly resize RenderWidget on high-DPI displays (`#1263 <https://github.com/ros-visualization/rviz/issues/1263>`_)
* [fix] Fixed opacity of Collada meshes (`#1387 <https://github.com/ros-visualization/rviz/issues/1387>`_)
* [fix] Enable non-ascii chars for MovableText (`#1374 <https://github.com/ros-visualization/rviz/issues/1374>`_)
* [fix] Ignore nan and inf values when normalizing images (`#1378 <https://github.com/ros-visualization/rviz/issues/1378>`_)
* [fix] MovableText: consider full translation vector (`#1375 <https://github.com/ros-visualization/rviz/issues/1375>`_)
* [maintanence] ToolManager: simplify key handling code
* [maintanence] Move sip bindings' build directory into the build space. (`#1360 <https://github.com/ros-visualization/rviz/issues/1360>`_)
* [maintanence] Use std random generators for portability on Windows (`#1356 <https://github.com/ros-visualization/rviz/issues/1356>`_)
* [api] public CovarianceVisual (`#1410 <https://github.com/ros-visualization/rviz/issues/1410>`_)
* [api] Properly override FailedDisplay::save (`#1402 <https://github.com/ros-visualization/rviz/issues/1402>`_)
* [api] Removed extra wrapper MarkerArrayDisplay::handleMarkerArray (`#1401 <https://github.com/ros-visualization/rviz/issues/1401>`_)
* Contributors: Robert Haschke, Daiki Maekawa, Jasper, Jeremie Deray, Mike Purvis, Ryan Lober, Sean Yen, Simon Schmeisser, Victor Lamoine, chapulina

1.13.3 (2019-03-16)
-------------------
* [fix] Fixed build issue on Debian Stretch, using tinyxml2 4.x only (`#1354 <https://github.com/ros-visualization/rviz/issues/1354>`_)
* Contributors: Robert Haschke

1.13.2 (2019-03-09)
-------------------
* [feature] InitialPoseTool: allowed to configure initial std deviation (`#1255 <https://github.com/ros-visualization/rviz/issues/1255>`_)
* [feature] Allowed OGRE_PLUGIN_PATH to be defined at cmake config time (`#1274 <https://github.com/ros-visualization/rviz/issues/1274>`_)
* [feature] Improved color support for themes (`#1319 <https://github.com/ros-visualization/rviz/issues/1319>`_)
* [feature] Various adaptions to support native Windows builds
  * Removed support for yaml-cpp < 0.5, which is outdated since Ubuntu Trusty / ROS Indigo.
  * Added DLL symbol exports (`#1335 <https://github.com/ros-visualization/rviz/issues/1335>`_)
* [feature] EffortDisplay: fixed joint display hierarchy (`#1323 <https://github.com/ros-visualization/rviz/issues/1323>`_)
* [feature] Configurable tool button style (`#1309 <https://github.com/ros-visualization/rviz/issues/1309>`_)
* [feature] Publicly exposed VisualizationFrame::addPanelByName() (`#1303 <https://github.com/ros-visualization/rviz/issues/1303>`_)
* [fix] Consider orientation of poses Path messages (`#1246 <https://github.com/ros-visualization/rviz/issues/1246>`_)
* [fix] Don't hide the toolbar when pressing Esc (`#1256 <https://github.com/ros-visualization/rviz/issues/1256>`_)
* [fix] Gracefully handle invalid DISPLAY variable (`#1282 <https://github.com/ros-visualization/rviz/issues/1282>`_)
* [fix] Panels were sometimes hidden on startup (`#1348 <https://github.com/ros-visualization/rviz/issues/1348>`_)
* [fix] Clear statuses in RobotModelDisplay when (re)loading a model (`#1296 <https://github.com/ros-visualization/rviz/issues/1296>`_)
* [fix] Fixed XML parser / Collada loader (`#1341 <https://github.com/ros-visualization/rviz/issues/1341>`_ / `#1317 <https://github.com/ros-visualization/rviz/issues/1317>`_)
* [fix] Fixed memory leak in material handling, simplified code (`#1294 <https://github.com/ros-visualization/rviz/issues/1294>`_)
* [fix] Removed ICC profiles in PNG images to silent libpng errors (`#1304 <https://github.com/ros-visualization/rviz/issues/1304>`_)
* [fix] Fixed memory leaks (`#1310 <https://github.com/ros-visualization/rviz/issues/1310>`_)
* [maintenance] Code cleanup, fixed warnings and catkin issues (`#1245 <https://github.com/ros-visualization/rviz/issues/1245>`_, `#1346 <https://github.com/ros-visualization/rviz/issues/1346>`_, `#1295 <https://github.com/ros-visualization/rviz/issues/1295>`_)
* [maintenance] Removed deprecated boost-signals1 (and thus support for tf < 1.11.3) (`#1344 <https://github.com/ros-visualization/rviz/issues/1344>`_)
* [maintenance] Updated wiki URLs (`#1315 <https://github.com/ros-visualization/rviz/issues/1315>`_)
* Contributors: Alexander Rössler, Chris Ratliff, G.A. vd. Hoorn, Joseph Duchesne, Rein Appeldoorn, Robert Haschke, Sean Yen, Sean Yen, Victor Lamoine, W. Nicholas Greene, d-walsh, ipa-fez, mutronics

1.13.1 (2018-05-10)
-------------------
* Added API option to disable keyboard focus being set on mouse move (`#1174 <https://github.com/ros-visualization/rviz/issues/1174>`_)
* Deprecated tf API's in favor of new tf2 API alternatives (`#1236 <https://github.com/ros-visualization/rviz/issues/1236>`_)
* Added a boolean property to the wrench visualization to make hiding small forces/torques optional (`#1196 <https://github.com/ros-visualization/rviz/issues/1196>`_)
* Converted all of rviz to tinyxml2 from tinyxml, partially to avoid newly deprecated interfaces in urdf (`#1237 <https://github.com/ros-visualization/rviz/issues/1237>`_)
* Added TF Prefix to effort plugin (`#1213 <https://github.com/ros-visualization/rviz/issues/1213>`_)
* Contributors: Antoine Hoarau, Simon Schmeisser, William Woodall, jgueldenstein

1.13.0 (2018-04-27)
-------------------
* Created a new Preferences window, and added PromptSaveOnExit option (`#1216 <https://github.com/ros-visualization/rviz/issues/1216>`_)
* Allowed classes inheriting from image display access to more state (`#1221 <https://github.com/ros-visualization/rviz/issues/1221>`_)
* Updated additional include statement to use new pluginlib and class_loader headers (`#1231 <https://github.com/ros-visualization/rviz/issues/1231>`_)
* Fixed crash when robot model not loaded before processing JointState msg (`#1229 <https://github.com/ros-visualization/rviz/issues/1229>`_)
* Contributors: William Woodall, dhood, daiz, ahoarau, MasterEric

1.12.16 (2018-04-26)
--------------------
* Fixed use of LineSpacing, horizontal alignment and AABB calculation in MovableText (`#1200 <https://github.com/ros-visualization/rviz/issues/1200>`_)
* Disable dock widget text eliding (`#1168 <https://github.com/ros-visualization/rviz/issues/1168>`_)
* Updated include statements to use new pluginlib and class_loader headers (`#1217 <https://github.com/ros-visualization/rviz/issues/1217>`_)
* Updated camera_display plugin to take roi in cameraInfo into consideration (`#1158 <https://github.com/ros-visualization/rviz/issues/1158>`_)
* Fixed bug where help.html wasn't being installed (`#1218 <https://github.com/ros-visualization/rviz/issues/1218>`_)
* Fixed compiler warning due to mismached new/delete in MapDisplay Swatch (`#1211 <https://github.com/ros-visualization/rviz/issues/1211>`_)
* Factored out marker creation from ROS msg into new createMarker() (`#1183 <https://github.com/ros-visualization/rviz/issues/1183>`_)
* Fixed crash if display-config parameter was fewer than 4 characters (`#1189 <https://github.com/ros-visualization/rviz/issues/1189>`_)
* Contributors: Daniel Seifert, Johannes Meyer, Mikael Arguedas, Robert Haschke, Tomáš Černík, Victor Lamoine, dhood, ecazaubon

1.12.15 (2018-01-05)
--------------------
* Fixed Ogre crashes from invalid quaternions by normalizing them so they no longer need to be rejected. (`#1179 <https://github.com/ros-visualization/rviz/issues/1179>`_)
* Restored processing of ROS messages containing invalid quaternions. (`#1182 <https://github.com/ros-visualization/rviz/issues/1182>`_)
  Unnormalized quaternions in messages will generate warnings; previously they were rejected.
  Publishers of invalid quaternions should be updated to publish valid quaternions, as rviz will reject invalid quaternions in the future.
* Contributors: Robert Haschke, dhood

1.12.14 (2017-12-19)
--------------------
* Added global option to disable default light (`#1146 <https://github.com/ros-visualization/rviz/issues/1146>`_)
* Added more checks for invalid quaternion normalization before displaying (`#1167 <https://github.com/ros-visualization/rviz/issues/1167>`_)
* Added MONO8 transformer for point cloud plugin (`#1145 <https://github.com/ros-visualization/rviz/issues/1145>`_)
* Fixed crash when unchecking options of "triangle list" markers `#1163 <https://github.com/ros-visualization/rviz/issues/1163>`_ (`#1164 <https://github.com/ros-visualization/rviz/issues/1164>`_)
* Added CMake definition to prevent collision of "check" macro on OS X (`#1165 <https://github.com/ros-visualization/rviz/issues/1165>`_)
* Added copyright notice for icons and graphics (`#1155 <https://github.com/ros-visualization/rviz/issues/1155>`_)
* Contributors: David Gossow, Kentaro Wada, Lucas Walter, Mike Purvis, Stefan Fabian, Terry Welsh

1.12.13 (2017-08-21)
--------------------
* Fixed an issue which caused mesh markers to appear white where previously they were not (`#1132 <https://github.com/ros-visualization/rviz/issues/1132>`_)
* Contributors: William Woodall

1.12.12 (2017-08-21)
--------------------
* Added check for odometry quaternion normalization before displaying (`#1139 <https://github.com/ros-visualization/rviz/issues/1139>`_)
* Improve point cloud rendering performance (`#1122 <https://github.com/ros-visualization/rviz/issues/1122>`_)
* Replaced Arial font with Liberation Sans (`#1141 <https://github.com/ros-visualization/rviz/issues/1141>`_)
* Contributors: Simon Harst, Thomas, William Woodall

1.12.11 (2017-08-02)
--------------------
* Added dhood as maintainer (`#1131 <https://github.com/ros-visualization/rviz/issues/1131>`_)
* Fixed finding and linking of tinyxml (`#1130 <https://github.com/ros-visualization/rviz/issues/1130>`_)
* Changed to only update window title if necessary (`#1124 <https://github.com/ros-visualization/rviz/issues/1124>`_)
* Added option to invert Z axis for orbit-based view controllers (`#1128 <https://github.com/ros-visualization/rviz/issues/1128>`_)
* Fixed visualization of collada markers with texture (`#1084 <https://github.com/ros-visualization/rviz/issues/1084>`_) (`#1129 <https://github.com/ros-visualization/rviz/issues/1129>`_)
* Fixed bug where Ogre::ItemIdentityException occurred while loading mesh (`#1105 <https://github.com/ros-visualization/rviz/issues/1105>`_)
* Fixed bug caused by combination of Qt and Boost (`#1114 <https://github.com/ros-visualization/rviz/issues/1114>`_)
* Fixed bug with map_display where it ignored resolution changes in OccupancyGrid maps (`#1115 <https://github.com/ros-visualization/rviz/issues/1115>`_)
* Fixed bug where keyboard shortcuts sometimes didn't work (`#1117 <https://github.com/ros-visualization/rviz/issues/1117>`_)
* Contributors: 1r0b1n0, Adam Allevato, Adrian Böckenkamp, Kartik Mohta, Michael Görner, Mikael Arguedas, William Woodall, dhood, gerkey

1.12.10 (2017-06-05)
--------------------
* Fix debian jessie compiler error (`#1111 <https://github.com/ros-visualization/rviz/issues/1111>`_)
* Contributors: William Woodall

1.12.9 (2017-06-05)
-------------------
* Fix variable name (`#1104 <https://github.com/ros-visualization/rviz/issues/1104>`_)
  Somehow, variable names got out of sync. Lines 370 and 371 refer to "parameters" but it is "params" everywhere else.
* Contributors: genemerewether

1.12.8 (2017-05-07)
-------------------
* Fixed bug where generated material names were not unique (`#1102 <https://github.com/ros-visualization/rviz/issues/1102>`_)
  * This was a regression of `#1079 <https://github.com/ros-visualization/rviz/issues/1079>`_
* Contributors: Maarten de Vries

1.12.7 (2017-05-05)
-------------------
* Fix render system regression (`#1101 <https://github.com/ros-visualization/rviz/issues/1101>`_)
  * Also updated the render system code to follow latest recommendations for integrating Qt5 and Ogre3D, see: http://www.ogre3d.org/tikiwiki/tiki-index.php?page=Integrating+Ogre+into+QT5
  * Restored conditional code for Qt5 versus Qt4, which fixed `#1100 <https://github.com/ros-visualization/rviz/issues/1100>`_
* Imported several updates to the covariance related displays (`#1099 <https://github.com/ros-visualization/rviz/issues/1099>`_)
  * Added offset to covariance properties
  * Refactored CovarianceVisual
  * Fixed tolerance test at angular discontinuity
  * Renamed PoseWithCovarianceDisplay::Shape enums
* Contributors: Ellon Paiva Mendes, William Woodall

1.12.6 (2017-05-02)
-------------------
* Added and updated displays to visualize covariance matrices (`#1096 <https://github.com/ros-visualization/rviz/issues/1096>`_)
  * Added display for PoseWithCovariance.
  * Update OdometryDisplay to optionally show covariances.
* Fixed regression in previous release which was a type error that happened with newer versions of urdf (`#1098 <https://github.com/ros-visualization/rviz/issues/1098>`_)
* Contributors: William Woodall

1.12.5 (2017-05-01)
-------------------
* Renamed duplicated pass_depth.vert in nogp program to avoid Ogre 1.10 runtime error (`#1063 <https://github.com/ros-visualization/rviz/issues/1063>`_)
* Fixed some handling of Window ID's for OS X and ogre 1.9 (`#1093 <https://github.com/ros-visualization/rviz/issues/1093>`_)
* Added support for maps larger than video memory using swatches (`#1095 <https://github.com/ros-visualization/rviz/issues/1095>`_)
* Added fullscreen option (f11) (`#1017 <https://github.com/ros-visualization/rviz/issues/1017>`_)
* Added an option to transform map based on header timestamp (`#1066 <https://github.com/ros-visualization/rviz/issues/1066>`_)
* Now updates the display if empty a pointcloud2 message is recieved (`#1073 <https://github.com/ros-visualization/rviz/issues/1073>`_)
  Previously the old point cloud would continue to be rendered.
* Now correctly scales the render panel on high resolution displays (`#1078 <https://github.com/ros-visualization/rviz/issues/1078>`_)
* Added support for multiple materials in a single link of a robot model (`#1079 <https://github.com/ros-visualization/rviz/issues/1079>`_)
* Now includes missing headers necessary for ogre 1.10 (`#1092 <https://github.com/ros-visualization/rviz/issues/1092>`_)
* Fixed duplicate property name for Path colors which caused it to not be restored from saved configs (`#1089 <https://github.com/ros-visualization/rviz/issues/1089>`_)
  See issue `#1087 <https://github.com/ros-visualization/rviz/issues/1087>`_.
* Contributors: Hidde Wieringa, Kei Okada, Maarten de Vries, Phil Osteen, Timo Röhling, Tom Moore, William Woodall, axelschroth

1.12.4 (2016-10-27)
-------------------
* Restored "Use ``urdf::*ShredPtr`` instead of ``boost::shared_ptr``" (`#1064 <https://github.com/ros-visualization/rviz/issues/1064>`_)
  Now supports ``urdfdom`` 0.3 and 0.4 through a compatibility header in ``urdf``.
* You can now visualize joint axis and display type and limits (`#1029 <https://github.com/ros-visualization/rviz/issues/1029>`_)
* Contributors: Lucas Walter, Robert Haschke, William Woodall

1.12.3 (2016-10-19)
-------------------
* Revert "Use ``urdf::*ShredPtr`` instead of ``boost::shared_ptr``" (`#1060 <https://github.com/ros-visualization/rviz/issues/1060>`_)
* Contributors: William Woodall

1.12.2 (2016-10-18)
-------------------
* Paths can now be rendered as 3D arrows or pose markers (`#1059 <https://github.com/ros-visualization/rviz/issues/1059>`_)
* Allow float edits to work with different Locales (`#1043 <https://github.com/ros-visualization/rviz/issues/1043>`_)
* Now check for a valid root link before walking the robot model (`#1041 <https://github.com/ros-visualization/rviz/issues/1041>`_)
* Added close() signal to Tool class (`#1051 <https://github.com/ros-visualization/rviz/issues/1051>`_)
* Fix double free in display dialog (`#1053 <https://github.com/ros-visualization/rviz/issues/1053>`_)
* Tweak focal shape size marker depending on focal distance (`#1021 <https://github.com/ros-visualization/rviz/issues/1021>`_)
* Support 3D arrows and axes for visualizing PoseArrays (`#1022 <https://github.com/ros-visualization/rviz/issues/1022>`_)
* Use ``urdf::*ShredPtr`` instead of ``boost::shared_ptr`` (`#1044 <https://github.com/ros-visualization/rviz/issues/1044>`_)
* Fixed two valgrind-reported issues (`#1027 <https://github.com/ros-visualization/rviz/issues/1027>`_)
  * in ~RenderPanel()
  * in VisualizationManager(): initialization order
* Added option to disable the RViz splash-screen (`#1024 <https://github.com/ros-visualization/rviz/issues/1024>`_)
* Fix compile error due to the user-defined string literals feature (`#1010 <https://github.com/ros-visualization/rviz/issues/1010>`_)
* Fixed some Qt5 related build issues (`#1008 <https://github.com/ros-visualization/rviz/issues/1008>`_)
* Removed dependency on OpenCV (`#1009 <https://github.com/ros-visualization/rviz/issues/1009>`_)
* Contributors: 1r0b1n0, Atsushi Watanabe, Blake Anderson, Jochen Sprickerhof, Kartik Mohta, Maarten de Vries, Michael Görner, Robert Haschke, Victor Lamoine, Víctor Mayoral Vilches, William Woodall

1.12.1 (2016-04-20)
-------------------
* Updated the ``plugin_description.xml`` to reflect the new default plugin library name, see: `#1004 <https://github.com/ros-visualization/rviz/issues/1004>`_
* Contributors: William Woodall

1.12.0 (2016-04-11)
-------------------
* Qt5 is now the default build option, but Qt4 support is still available (for C++ only).
* Fixed support for PyQt5, but disabled PySide2 until we get it working.
* The default plugin's library was changed to ``rviz_default_plugin``.
* Changed to use CMake's ``file(GENERATE ...)`` macro when exporting the default plugin's library name.
* Changed costmap lethal color to be different from illegal values.
* Cleaned-up and generalized the WrenchVisual display:
  * renamed ``WrenchStampedVisual`` to ``WrenchVisual``
  * cleanup: removed deprecated API
* Updated the marker display and tf plugins to update the map of enabled namespaces and frames whenever those frames are enabled/disabled using the check boxes.
  Also updated the plugins so that the map of enabled namespaces and frames does not get erased whenever the plugin is reset. (`#988 <https://github.com/ros-visualization/rviz/issues/988>`_)
  This allows the currently selected namespaces/frames to remain selected after the Reset button is pressed.
* Contributors: Brett, Robert Haschke, William Woodall

1.11.14 (2016-04-03)
--------------------
* Added the ``rviz_QT_VERSION`` cmake variable that exports the Qt version used by rviz.
* Changed the way ``rviz_DEFAULT_PLUGIN_LIBRARIES`` is set so it works with ``catkin_make`` too.
* Contributors: William Woodall

1.11.13 (2016-03-23)
--------------------
* Changed the way the rviz_DEFAULT_PLUGIN_LIBRARIES are generated to support cmake < 2.8.12.
  See pull request: `#981 <https://github.com/ros-visualization/rviz/issues/981>`_
* Contributors: William Woodall

1.11.12 (2016-03-22)
--------------------
* Relaxed the required CMake version to 2.8.11.2 in order to support Ubuntu Saucy.
* Contributors: William Woodall

1.11.11 (2016-03-22)
--------------------
* Added Qt version to rosout and help->about.
* Added optional support for Qt5 with continued support for Qt4.
* Fixed a C++11 warning about literals needing a space after them.
* Added a "duplicate" button for duplicating displays.
* Fixed remove display so that it selects another display after removing one (if one is available).
* Fix for `#959 <https://github.com/ros-visualization/rviz/issues/959>`_: jumping marker in MOVE_3D mode
  See pull request: `#961 <https://github.com/ros-visualization/rviz/issues/961>`_
* Added a raw mode for map vizualization.
  See pull request: `#972 <https://github.com/ros-visualization/rviz/issues/972>`_
* Added an option in many of the topic based Displays to prefer UDP/unreliable transport.
  See pull request: `#976 <https://github.com/ros-visualization/rviz/issues/976>`_
* Fixed the marker display to allow namespaces to be enabled/disabled based on the loaded config.
  Also enabled state is stored for each namespace in a map, which is used to lookup the state whenever a namespace is added to the display.
  See pull request: `#962 <https://github.com/ros-visualization/rviz/issues/962>`_
* Fixed crash in ``Display::deleteStatus()`` when no statuses where created beforehand.
  See pull request: `#960 <https://github.com/ros-visualization/rviz/issues/960>`_
* Read-only properties are now no longer editable.
  See pull request: `#958 <https://github.com/ros-visualization/rviz/issues/958>`_
* The binary STL loading logic has been relaxed to support files that contain more data than expected.
  A warning is printed instead of failing with an error now.
  See pull request: `#951 <https://github.com/ros-visualization/rviz/issues/951>`_
* Fixed an issue where tf configurations were not saved and reloaded from the rviz config file.
  See pull request: `#946 <https://github.com/ros-visualization/rviz/issues/946>`_
* Anti-Aliasing (AA) is now enabled by default, but it can be disabled with ``--disable-anti-aliasing``.
  See pull request: `#931 <https://github.com/ros-visualization/rviz/issues/931>`_
  See pull request: `#950 <https://github.com/ros-visualization/rviz/issues/950>`_
* The default plugin shared library is no longer exported via rviz_LIBRARIES, but in stead is now
  in a cmake variable called rviz_DEFAULT_PLUGIN_LIBRARIES.
  See pull request: `#948 <https://github.com/ros-visualization/rviz/issues/948>`_
  See pull request: `#979 <https://github.com/ros-visualization/rviz/issues/979>`_
* Fixed a bug in billboard line generation where a zero point line caused a crash.
  See pull request: `#942 <https://github.com/ros-visualization/rviz/issues/942>`_
* Downsampled maps will now result in a Warning status, previously it was OK.
  See pull request: `#934 <https://github.com/ros-visualization/rviz/issues/934>`_
* The map display will no longer try to transform a map until one has been received.
  See pull request: `#932 <https://github.com/ros-visualization/rviz/issues/932>`_
* Enable antialiasing
* Contributors: Aaron Hoy, Benjamin Chrétien, Chris Mansley, Dave Coleman, David V. Lu!!, Joao Avelino, Jochen Sprickerhof, Kentaro Wada, Martin Pecka, Mike O'Driscoll, Nikolaus Demmel, Robert Haschke, Simon Schmeisser (isys vision), Stephan, Tobias Berling, William Woodall, bponsler, caguero, frosthand

1.11.10 (2015-10-13)
--------------------
* Fixed Qt assertions triggered in debug build of Qt.
* build: Use PKG_CONFIG_EXECUTABLE
  Instead of using a hard-coded pkg-config to make cross-compiling
  possible where the pkg-config binary is host-prefixed (e.g.
  armv7-unknown-linux-pkg-config when cross-compiling for armv7)
* Fix `#911 <https://github.com/ros-visualization/rviz/issues/911>`_ `#616 <https://github.com/ros-visualization/rviz/issues/616>`_ : TF Segfaults on reset/update
  Do not needlessly delete tree_property\_ elements, update them instead.
  Most likely fixes `#808 <https://github.com/ros-visualization/rviz/issues/808>`_ too.
* python_bindings: sip: Use CATKIN_PACKAGE_LIB_DESTINATION instead of hardcoded lib.
  Fixes build with libdir != lib.
  https://bugs.gentoo.org/show_bug.cgi?id=561480
* Contributors: Alexis Ballier, Arnaud TANGUY, Dave Hershberger, Marvin Schmidt, William Woodall

1.11.9 (2015-09-21)
-------------------
* Updated warning message to indicate triangle count is a 32bit integer, and not 16bit.
* Fixed the error checking of large STL files.
* Smoothed updates for map display plugin.
  Map displays previously only updated when receiving a message. This means that
  if your fixed frame was base_link, the costmaps would not move appropriately
  around the robot unless a message was received in order to update the transform
  that should be applied to the scene. For global costmaps, this is a slow
  update and for static maps, this never happened.
  This fixes that by hooking into rviz' periodic call to continuously update the
  transform to be applied to the scene.
* Displays are not disabled if associated Panel becomes invisible.
  Otherwise the state between Panel & Display becomes inconsistent.
  Fixed symptom:
  When loading a configuration that contains a disabled CameraDisplay,
  during the configuration of the panel(the camera render widget),
  the panel is set visible for a very short period of time.
  Because of the missing logic, the CameraDisplay is enabled
  together with the panel, but the Display remains enabled
  after the Panel is set invisible. One ends up with an enabled
  (and subscribed) CameraDisplay without the corresponding RenderWidget,
  even so the configuration specified that the Display is not enabled.
* Removed shortkeys from ``shortkey_to_tool_map_``
  this should fix `#880 <https://github.com/ros-visualization/rviz/issues/880>`_
* Contributors: Daniel Stonier, Henning Deeken, Jonathan Meyer, Michael Görner, William Woodall

1.11.8 (2015-08-05)
-------------------
* Force and Torque can now be scaled separately in the Wrench display: `#862 <https://github.com/ros-visualization/rviz/issues/862>`_
* Fixed a bug in the Wrench display: `#883 <https://github.com/ros-visualization/rviz/issues/883>`_
* Improved error checking when loading ascii stl files.
* Suppressing some new CMake warnings by setting cmake policies.
* Re-enable most all of the tests.
* Added option to start rviz with the ROS logger level set to Debug
* Fixed setting of status bar from python by checking if the original status bar is being used or not.
* Added a third person follower view controller.
* Fix decaying of tf2 static transforms in the TF display.
* Correctly display color and alpha in pointclouds.
* Restored functionality to force opacity and color for meshes that have null rgba values.
* Use the ``find_package``'ed python version detected by catkin.
  Otherwise it might happen that catkin (and the rest of the workspace)
  uses 2.x and rviz detects & tries to use 3.x. This can produce some nasty
  collisions.
  See rospack, roslz4, qt_gui_cpp and others for similar invokation.
* Fix processing empty of pointclouds.
  Otherwise, given a stream of clouds with some of them empty, the last non-empty message will still be displayed until a the next non-empty cloud comes in.
* Check if position and orientation of links of robots contain NaNs when updating pose of robot links.
* Fixed DELETEALL marker action, by not iterating on the marker list.
* Contributors: Carlos Agüero, Gustavo N Goretkin, Jonathan Bohren, Kei Okada, Michael Ferguson, Ryohei Ueda, Thomas Moinel, William Woodall, loganE, louise, otim, v4hn, 寺田　耕志

1.11.7 (2015-03-02)
-------------------
* Fixed a bug where the timestamp was not set for the /initialpose message published by the 2D Pose Estimate tool.
* Added a method/Qt Signal for refreshing tools called ``refreshTool()``.
  Calling this method updates the name and icon of a tool in the toolbar.
* Fixed a bug with ``setCurrentTool``.
  This fixes a rare gui bug: if an incoming tool directly calls another tool during it's activate() function the tool gets changed accordingly but the toolbar gui becomes inconsistent because Tool* tool pointer is outdated in this case. using Tool* current_tool fixes this.
* Fixed initialization of Tool's ``shortcut_key_`` and fixed a bug in ``toKeys()``.
  * Initialized the ``shortcut_key_`` param with '/0' to be able to check whether a tool has a shortkey assigned or not.
  * Made the tool manager check if a tool has a shortkey before converting the char to a key code.
  * Fixed the ``toKeys()`` method by removing the assertions, making at a boolean returning function and allowing a single key only, as this is what is to be expected from the ``shortcut_key_`` param this should fix `#851 <https://github.com/ros-visualization/rviz/issues/851>`_
* Contributors: Henning Deeken, William Woodall, lsouchet

1.11.6 (2015-02-13)
-------------------
* Fixed a mesh memory leak in ogre_helpers/mesh_shape.h/.cpp
  This fixes a memory leak which is caused due to no meshes ever being
  destroyed without removing the mesh from the mesh manager.
  This gets really bad when drawing meshes with 50K triangles at 10Hz,
  resulting in a leak rate @ ~60MB/sec.
* Add a simple 'About' dialog to the help menu.
* Contributors: Jonathan Bohren, William Woodall, gavanderhoorn

1.11.5 (2015-02-11)
-------------------
* Tools (on the toolbar) can now indicate if they need access to keypresses by setting the ``access_all_keys_`` attribute.
  The handling of keypresses in tools has also been refactored. See: pull request `#838 <https://github.com/ros-visualization/rviz/issues/838>`_
* Path display now has an additional display style called "Billboards" which allows to set the line width of the paths.
  It also now has an offset property to shift the path with regard to the fixed frame origin.
  See: pull request `#842 <https://github.com/ros-visualization/rviz/issues/842>`_
* Meshes now have their ambient values scaled by 0.5 which gives a softer look, which is more in line with Gazebo's look and feel.
  See: pull request `#841 <https://github.com/ros-visualization/rviz/issues/841>`_
* The default ambient color for meshes is now 0,0,0, down from 0.5,0.5,0.5.
  See: pull request `#837 <https://github.com/ros-visualization/rviz/issues/837>`_
* Triangle-list markers are now shaded like other objects.
  See: pull request `#833 <https://github.com/ros-visualization/rviz/issues/833>`_
* Color is now applied to all visuals of the line class, closes `#820 <https://github.com/ros-visualization/rviz/issues/820>`_.
  See: pull request `#827 <https://github.com/ros-visualization/rviz/issues/827>`_
* The find_package logic for assimp/yamlcpp has been moved to before add_library for librviz to fix building on OS X.
  See: pull request `#825 <https://github.com/ros-visualization/rviz/issues/825>`_
* Fixed moc generation errors with boost >= 1.57.
  See: pull request `#826 <https://github.com/ros-visualization/rviz/issues/826>`_
* Contributors: Daniel Stonier, Dave Hershberger, Henning Deeken, Michael Ferguson, Timm Linder, William Woodall, v4hn

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
