cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
# CMake 2.4.6 doesn't include FindPkgConfig, so we provide our own copy
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Debug)

rospack(rviz)

rospack_add_boost_directories()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories( ${wxWidgets_INCLUDE_DIRS} )

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH})

pkg_check_modules(OGRE OGRE)
include_directories( ${OGRE_INCLUDE_DIRS} )
link_directories( ${OGRE_LIBRARY_DIRS} )

include_directories( src/ogre_visualizer )

# Find the combined swig flags for this project
_rospack_invoke(${PROJECT_NAME} ${PROJECT_NAME} SWIG_FLAGS "export" "--lang=swig" "--attrib=flags")
set(SWIG_FLAGS ${${PROJECT_NAME}_SWIG_FLAGS})

# Find the wxswig executable
find_ros_package(wxswig)
set(WXSWIG_EXECUTABLE ${wxswig_PACKAGE_PATH}/bin/swig)

# Add a custom command for generating the swig output files
set(SWIG_INTERFACE_FILE ${PROJECT_SOURCE_DIR}/src/ogre_visualizer/ogre_visualizer.i)
set(SWIG_OUTPUT_PYTHON_FILE ${PROJECT_SOURCE_DIR}/lib/ogre_visualizer.py)

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" GREATER 2.4)
  set(SWIG_OUTPUT_CPP_FILE ${PROJECT_SOURCE_DIR}/src/ogre_visualizer/ogre_visualizer_swig_generated.cpp)
  set(SWIG_COMMAND ${WXSWIG_EXECUTABLE} ${SWIG_FLAGS} -o ${SWIG_OUTPUT_CPP_FILE} -outdir ../lib -module ${PROJECT_NAME} ${SWIG_INTERFACE_FILE})
else("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" GREATER 2.4)
  set(SWIG_OUTPUT_CPP_FILE src/ogre_visualizer/ogre_visualizer_swig_generated.cpp)
  set(SWIG_COMMAND ${WXSWIG_EXECUTABLE} ${SWIG_FLAGS} -o ../${SWIG_OUTPUT_CPP_FILE} -outdir ../lib -module ${PROJECT_NAME} ${SWIG_INTERFACE_FILE})
endif("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" GREATER 2.4)

set_source_files_properties(${SWIG_OUTPUT_CPP_FILE} PROPERTIES GENERATED true)
add_custom_command(OUTPUT   ${SWIG_OUTPUT_CPP_FILE} 
                            ${SWIG_OUTPUT_PYTHON_FILE}
                   COMMAND  ${SWIG_COMMAND}
                   DEPENDS  ${SWIG_INTERFACE_FILE}
                            src/ogre_visualizer/display.i
                            src/ogre_visualizer/visualization_manager.i
                            src/ogre_visualizer/helpers/color.i
                            src/ogre_visualizer/visualization_manager.h  
                            src/ogre_visualizer/generated/visualization_panel_generated.h
                            src/ogre_visualizer/display.h
                            src/ogre_visualizer/displays/axes_display.h
                            src/ogre_visualizer/displays/grid_display.h
                            src/ogre_visualizer/displays/laser_scan_display.h
                            src/ogre_visualizer/displays/marker_display.h
                            src/ogre_visualizer/displays/planning_display.h
                            src/ogre_visualizer/displays/point_cloud_base.h
                            src/ogre_visualizer/displays/point_cloud_display.h
                            src/ogre_visualizer/displays/robot_model_display.h
                            src/ogre_visualizer/displays/robot_base2d_pose_display.h
                            src/ogre_visualizer/displays/particle_cloud_2d_display.h
                            src/ogre_visualizer/displays/poly_line_2d_display.h
                            src/ogre_visualizer/displays/map_display.h
                            src/ogre_visualizer/displays/tf_display.h
                            src/ogre_visualizer/helpers/color.h)


# We create one lib with the C++...
rospack_add_library(${PROJECT_NAME} src/ogre_visualizer/properties/property.cpp
                                    src/ogre_visualizer/properties/property_manager.cpp
                                    src/ogre_visualizer/tools/tool.cpp
                                    src/ogre_visualizer/tools/move_tool.cpp
                                    src/ogre_visualizer/tools/pose_tool.cpp
                                    src/ogre_visualizer/tools/selection_tool.cpp
                                    src/ogre_visualizer/render_panel.cpp
                                    src/ogre_visualizer/displays_panel.cpp
                                    src/ogre_visualizer/visualization_manager.cpp
                                    src/ogre_visualizer/visualization_frame.cpp
                                    src/ogre_visualizer/display.cpp
                                    src/ogre_visualizer/common.cpp
                                    src/ogre_visualizer/factory.cpp
                                    src/ogre_visualizer/ros_topic_property.cpp
                                    src/ogre_visualizer/new_display_dialog.cpp
                                    src/ogre_visualizer/helpers/robot.cpp
                                    src/ogre_visualizer/displays/marker_display.cpp
                                    src/ogre_visualizer/displays/axes_display.cpp
                                    src/ogre_visualizer/displays/grid_display.cpp
                                    src/ogre_visualizer/displays/point_cloud_base.cpp
                                    src/ogre_visualizer/displays/point_cloud_display.cpp
                                    src/ogre_visualizer/displays/laser_scan_display.cpp
                                    src/ogre_visualizer/displays/robot_model_display.cpp
                                    src/ogre_visualizer/displays/planning_display.cpp
                                    src/ogre_visualizer/displays/robot_base2d_pose_display.cpp
                                    src/ogre_visualizer/displays/particle_cloud_2d_display.cpp
                                    src/ogre_visualizer/displays/poly_line_2d_display.cpp
                                    src/ogre_visualizer/displays/polygonal_map_display.cpp
                                    src/ogre_visualizer/displays/collision_map_display.cpp
                                    src/ogre_visualizer/displays/map_display.cpp
                                    src/ogre_visualizer/displays/tf_display.cpp
                                    src/ogre_visualizer/generated/visualization_panel_generated.cpp)
target_link_libraries(${PROJECT_NAME} ${wxWidgets_LIBRARIES} ${OGRE_LIBRARIES})
rospack_link_boost(${PROJECT_NAME} thread signals filesystem)

# ...and another with the SWIG code.
rospack_add_library(python_${PROJECT_NAME} ${SWIG_OUTPUT_CPP_FILE})
target_link_libraries(python_${PROJECT_NAME} ${PROJECT_NAME})

# swig python needs a shared library named _<modulename>.[so|dll|...]
# this renames the output file to conform to that by prepending an underscore and removing the "lib" prefix
set_target_properties(python_${PROJECT_NAME}
                      PROPERTIES OUTPUT_NAME _${PROJECT_NAME}
                      PREFIX "")

rospack_add_executable(executable src/ogre_visualizer/visualizer_app.cpp)
target_link_libraries(executable ${PROJECT_NAME} ${wxWidgets_LIBRARIES})
set_target_properties(executable
                      PROPERTIES OUTPUT_NAME ${PROJECT_NAME}
                      PREFIX "")

rospack_add_executable(marker_test src/test/marker_test.cpp)
rospack_add_executable(cloud_test src/test/cloud_test.cpp)
 
