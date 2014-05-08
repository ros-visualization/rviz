cmake_minimum_required(VERSION 2.8.3)
project(rviz)

find_package(Boost REQUIRED
  COMPONENTS
  filesystem
  program_options
  signals
  system
  thread
)


find_package(PkgConfig REQUIRED)

pkg_check_modules(OGRE_OV OGRE OGRE-Overlay)

# Old versions of OGRE (pre 1.9) included OGRE-Overlay in the main package
# (i.e. there was no OGRE-Overlay component).  So if the above
# pkg_check_modules() failed, try looking for just OGRE.
if(NOT OGRE_OV_FOUND)
  pkg_check_modules(OGRE_OV REQUIRED OGRE)
endif(NOT OGRE_OV_FOUND)

## Find OGRE Plugin path (not necessarily platform-independent, I guess)
execute_process(COMMAND
  pkg-config --variable=plugindir OGRE
  OUTPUT_VARIABLE OGRE_PLUGIN_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
  )
message(STATUS OGRE_PLUGIN_PATH=${OGRE_PLUGIN_PATH})

# find absolute path of ogre libraries.
# This is stored in the cache to save time on cmake re-run.
# Using absolute paths is necessary if pkg-config finds Ogre in a different
# location than the default.  This can happen when Ogre is built from source,
# or when 2 versions of Ogre are installed.  Using absolute paths ensures that
# components that link against Ogre (rviz and any packages depending on rviz)
# all find the same Ogre shared library.
if(NOT DEFINED OGRE_OV_LIBRARIES_ABS)
  unset(OGRE_OV_LIBRARIES_ABS_TMP)
  foreach(_lib ${OGRE_OV_LIBRARIES})
    set(OGRE_OV_LIB_TAG "OGRE_OV_RVIZ_LIB_${_lib}")

    find_library(${OGRE_OV_LIB_TAG}
            NAMES ${_lib}
            HINTS ${OGRE_OV_LIBRARY_DIRS}
            PATHS ${OGRE_OV_LIBRARY_DIRS}
    )

    set(OGRE_OV_LIBRARIES_ABS_TMP ${OGRE_OV_LIBRARIES_ABS_TMP} ${${OGRE_OV_LIB_TAG}})
  endforeach(_lib)

  set(OGRE_OV_LIBRARIES_ABS ${OGRE_OV_LIBRARIES_ABS_TMP}
        CACHE FILEPATH "Pathname of library ${_lib}")
endif(NOT DEFINED OGRE_OV_LIBRARIES_ABS)

if(APPLE)
  FIND_LIBRARY(Cocoa_LIBRARIES Cocoa)
  set(rviz_ADDITIONAL_LIBRARIES ${Cocoa_LIBRARIES})
endif()

find_package(OpenGL REQUIRED)

find_package(PythonLibs REQUIRED)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui QtOpenGL)

find_package(urdfdom REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
  angles
  cmake_modules
  geometry_msgs
  image_geometry
  image_transport
  interactive_markers
  laser_geometry
  map_msgs
  message_filters
  nav_msgs
  pluginlib
  python_qt_binding
  resource_retriever
  rosbag
  rosconsole
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  urdf
  visualization_msgs
)

if(${tf_VERSION} VERSION_LESS "1.11.3")
  add_definitions("-DRVIZ_USE_BOOST_SIGNAL_1")
endif()


find_package(Eigen REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
    src
    ${EIGEN_INCLUDE_DIRS}
    ${OGRE_OV_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${urdfdom_INCLUDE_DIRS}
  LIBRARIES
    rviz
    default_plugin
    ${OGRE_OV_LIBRARIES_ABS}
    ${rviz_ADDITIONAL_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${urdfdom_LIBRARIES}
  CATKIN_DEPENDS
    geometry_msgs
    image_geometry
    image_transport
    interactive_markers
    laser_geometry
    map_msgs
    message_filters
    nav_msgs
    pluginlib
    resource_retriever
    roscpp
    roslib
    sensor_msgs
    std_msgs
    tf
    urdf
    visualization_msgs
)

include_directories(SYSTEM
  ${EIGEN_INCLUDE_DIRS}
  ${OGRE_OV_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${PYTHON_INCLUDE_PATH}
  ${urdfdom_INCLUDE_DIRS}
)
include_directories(src ${catkin_INCLUDE_DIRS})

include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

#### If gtk ends up being the best way to get the correct window
#### position under X11, this is how to compile it in.
#
# find_package(GTK2)
# include_directories(${GTK2_INCLUDE_DIRS})
# include_directories(/usr/include/gdk-pixbuf-2.0)

add_subdirectory(src)

install(DIRECTORY ogre_media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY images
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES default.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
