cmake_minimum_required(VERSION 3.0.2)
project(rviz)

if (POLICY CMP0042)
  cmake_policy(SET CMP0042 NEW)
endif()
if (POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Define global caktin_lint suppressions
#catkin_lint: ignore uninstalled_script missing_install_target

find_package(Boost REQUIRED
  COMPONENTS
  filesystem
  program_options
  system
  thread
)

find_package(urdfdom_headers REQUIRED)

set(CMAKE_POLICY_DEFAULT_CMP0012 NEW)  # required due to a bug in assimp 5.0
find_package(ASSIMP REQUIRED)
if(NOT DEFINED ASSIMP_LIBRARIES AND TARGET assimp::assimp)
  # ASSIMP >= 5.0 no longer defines ASSIMP_LIBRARIES
  set(ASSIMP_LIBRARIES assimp::assimp)
endif()

find_package(OGRE QUIET COMPONENTS Overlay)
if(NOT OGRE_FOUND)
  # OGRE doesn't come with correctly installed cmake files on Linux. Use pkg-config instead.
  # We need both, OGRE and OGRE-Overlay. Look for both simulatenously and use prefix X_
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(X REQUIRED OGRE OGRE-Overlay)

  # Set OGRE_* variables as cmake-based find_package would do
  set(OGRE_VERSION ${X_OGRE_VERSION})
  set(OGRE_INCLUDE_DIRS ${X_INCLUDE_DIRS})
  message(STATUS "OGRE_INCLUDE_DIRS=${OGRE_INCLUDE_DIRS}")

  # Find absolute path of OGRE libraries.
  # This is stored in the cache to save time on cmake re-run.
  # Using absolute paths is necessary if pkg-config finds Ogre in a different
  # location than the default.  This can happen when Ogre is built from source,
  # or when 2 versions of Ogre are installed.  Using absolute paths ensures that
  # components that link against Ogre (rviz and any packages depending on rviz)
  # all find the same Ogre shared library.
  if(NOT DEFINED OGRE_LIBRARIES)
    set(_OGRE_LIBRARIES)
    foreach(_lib ${X_LIBRARIES})
      set(_lib_tag "OGRE_RVIZ_${_lib}")

      find_library(${_lib_tag}
              NAMES ${_lib}
              HINTS ${X_LIBRARY_DIRS}
              PATHS ${X_LIBRARY_DIRS}
      )

      list(APPEND _OGRE_LIBRARIES ${${_lib_tag}})
    endforeach(_lib)

    set(OGRE_LIBRARIES ${_OGRE_LIBRARIES} CACHE STRING "Absolute paths to OGRE libs")
  endif(NOT DEFINED OGRE_LIBRARIES)
  message(STATUS "OGRE_LIBRARIES=${OGRE_LIBRARIES}")

  ## Fetch OGRE_PLUGIN_DIR variable from pkg-config
  if(NOT DEFINED OGRE_PLUGIN_DIR)
    execute_process(COMMAND
      ${PKG_CONFIG_EXECUTABLE} --variable=plugindir OGRE
      OUTPUT_VARIABLE OGRE_PLUGIN_DIR
      OUTPUT_STRIP_TRAILING_WHITESPACE
      )
  endif(NOT DEFINED OGRE_PLUGIN_DIR)
  message(STATUS "OGRE_PLUGIN_DIR=${OGRE_PLUGIN_DIR}")
endif()

# TODO: adapt version after upgrade to newer OGRE release
# Disable deprecation warnings for OGRE >= 1.10
if(NOT OGRE_VERSION VERSION_LESS "1.10.0" AND NOT MSVC)
  add_compile_options(-Wno-deprecated-declarations)
endif()

if(APPLE)
  FIND_LIBRARY(Cocoa_LIBRARIES Cocoa)
  set(rviz_ADDITIONAL_LIBRARIES ${Cocoa_LIBRARIES})

  # This definition prevents an Apple header from defining a macro called "check", which if
  # present, collides with another function in the rviz code. See:
  # https://opensource.apple.com/source/CarbonHeaders/CarbonHeaders-18.1/AssertMacros.h
  add_definitions(-D__ASSERT_MACROS_DEFINE_VERSIONS_WITHOUT_UNDERSCORES=0)
endif()

# Prefer newer vender-specific OpenGL library
if (POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()
find_package(OpenGL REQUIRED)

set(CMAKE_AUTOMOC ON)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets OpenGL)
set(QT_LIBRARIES Qt5::Widgets)
set(QTVERSION ${Qt5Widgets_VERSION})
add_definitions(-DQT_NO_KEYWORDS)

find_package(catkin REQUIRED
  COMPONENTS
  cmake_modules
  geometry_msgs
  image_transport
  interactive_markers
  laser_geometry
  map_msgs
  message_filters
  message_generation
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
  tf2_ros
  tf2_geometry_msgs
  urdf
  visualization_msgs
)

find_package(PythonLibs "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}" REQUIRED)

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  #catkin_lint: ignore_once deprecated_cmake_module
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(TinyXML2 REQUIRED)

catkin_python_setup()

# Set the Qt version for use in the extras file.
set(rviz_QT_VERSION ${QTVERSION})

# This variable controls the target name of the default plugin library.
# It is used in the extras file (processed in caktin_package(...)) and the
# cmake code for the default plugin.
# There is a matching instance of this in the plugin_description.xml.
set(rviz_DEFAULT_PLUGIN_LIBRARY_TARGET_NAME rviz_default_plugin)

add_service_files(
  FILES
  SendFilePath.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

set(EXPORT_HEADER_DIR "${CATKIN_DEVEL_PREFIX}/include")
file(MAKE_DIRECTORY "${EXPORT_HEADER_DIR}")

#catkin_lint: ignore_once external_include_path
catkin_package(
  CFG_EXTRAS
    rviz-extras.cmake
  INCLUDE_DIRS
    src
    ${EXPORT_HEADER_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${OGRE_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
  LIBRARIES
    rviz
    ${OGRE_LIBRARIES}
    ${rviz_ADDITIONAL_LIBRARIES}
    ${OPENGL_LIBRARIES}
  CATKIN_DEPENDS
    geometry_msgs
    image_transport
    interactive_markers
    laser_geometry
    map_msgs
    message_filters
    message_runtime
    nav_msgs
    pluginlib
    resource_retriever
    roscpp
    roslib
    sensor_msgs
    std_msgs
    std_srvs
    tf2_ros
    urdf
    visualization_msgs
)

#catkin_lint: ignore_once external_directory
include_directories(src ${EXPORT_HEADER_DIR})
include_directories(SYSTEM
  ${EIGEN3_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${PYTHON_INCLUDE_PATH}
  ${TinyXML2_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
  ${ASSIMP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#### If gtk ends up being the best way to get the correct window
#### position under X11, this is how to compile it in.
#
# find_package(GTK2)
# include_directories(${GTK2_INCLUDE_DIRS})
# include_directories(/usr/include/gdk-pixbuf-2.0)

add_subdirectory(src)

if(OGRE_VERSION VERSION_LESS "1.10")
  set(EXCLUDE_PATTERN "ogre1.10.*")
else()
  set(EXCLUDE_PATTERN "ogre1.9.*")
endif()

install(DIRECTORY ogre_media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "${EXCLUDE_PATTERN}" EXCLUDE
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
install(FILES help/help.html
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/help
)
#catkin_lint: ignore uninstalled_target
