include_directories(.)

set(SOURCE_FILES markers/marker_selection_handler.cpp
                                    markers/marker_base.cpp
                                    markers/shape_marker.cpp
                                    markers/arrow_marker.cpp
                                    markers/line_list_marker.cpp
                                    markers/line_strip_marker.cpp
                                    markers/points_marker.cpp
                                    markers/text_view_facing_marker.cpp
                                    markers/mesh_resource_marker.cpp
                                    markers/triangle_list_marker.cpp
                                    interactive_markers/interactive_marker.cpp
                                    interactive_markers/interactive_marker_control.cpp
                                    marker_display.cpp
                                    interactive_marker_display.cpp
                                    axes_display.cpp
                                    grid_display.cpp
                                    point_cloud_transformers.cpp
                                    point_cloud_base.cpp
                                    point_cloud_display.cpp
                                    point_cloud2_display.cpp
                                    laser_scan_display.cpp
                                    robot_model_display.cpp
                                    odometry_display.cpp
                                    pose_array_display.cpp
                                    path_display.cpp
                                    polygon_display.cpp
                                    grid_cells_display.cpp
                                    map_display.cpp
                                    tf_display.cpp
                                    camera_display.cpp
                                    pose_display.cpp
                                    image_display.cpp
                                    init.cpp)

include(CMakeDetermineSystem)
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  rosbuild_add_library_module(default_plugin ${SOURCE_FILES})
else(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  rosbuild_add_library(default_plugin ${SOURCE_FILES})
endif(CMAKE_SYSTEM_NAME MATCHES "Darwin")

                                    
target_link_libraries(default_plugin ${PROJECT_NAME} ${wxWidgets_LIBRARIES} ${OGRE_LIBRARIES})
rosbuild_link_boost(default_plugin thread signals filesystem system program_options)
