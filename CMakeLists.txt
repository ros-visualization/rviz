cmake_minimum_required(VERSION 2.8.3)
project(rviz)
find_package(catkin REQUIRED
  angles
  geometry_msgs
  image_geometry 
  image_transport
  interactive_markers 
  laser_geometry
  message_filters
  nav_msgs
  pluginlib
  python_qt_binding
  resource_retriever 
  rosbag
  rosconsole
  roscpp
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  urdf 
  urdf_interface 
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS thread signals filesystem system program_options)
find_package(Eigen REQUIRED)
find_package(PkgConfig REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS src ${EIGEN_INCLUDE_DIRS}
  LIBRARIES rviz default_plugin
)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
include(${QT_USE_FILE})

add_definitions(-DQT_NO_KEYWORDS)

find_package(OpenGL REQUIRED)
include_directories(SYSTEM ${OPENGL_INCLUDE_DIR} )

find_package(PythonLibs REQUIRED)
include_directories(SYSTEM ${PYTHON_INCLUDE_PATH})

pkg_check_modules(OGRE OGRE)
include_directories(SYSTEM ${OGRE_INCLUDE_DIRS} )
link_directories( ${OGRE_LIBRARY_DIRS} )

## Find OGRE Plugin path (not necessarily platform-independent, I guess)
execute_process(COMMAND 
pkg-config --variable=plugindir OGRE 
OUTPUT_VARIABLE OGRE_PLUGIN_PATH
OUTPUT_STRIP_TRAILING_WHITESPACE
)
message(STATUS OGRE_PLUGIN_PATH=${OGRE_PLUGIN_PATH})


#### If gtk ends up being the best way to get the correct window
#### position under X11, this is how to compile it in.
#
# find_package(GTK2)
# include_directories(${GTK2_INCLUDE_DIRS})
# include_directories(/usr/include/gdk-pixbuf-2.0)

include_directories(src)

add_subdirectory(src)

install(DIRECTORY ogre_media         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY icons              DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY images             DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES default.rviz           DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
