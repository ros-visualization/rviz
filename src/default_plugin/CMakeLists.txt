include_directories(.)

set(SOURCE_FILES markers/marker_selection_handler.cpp
                                    markers/marker_base.cpp
                                    markers/shape_marker.cpp
                                    markers/arrow_marker.cpp
                                    markers/line_list_marker.cpp
                                    markers/line_strip_marker.cpp
                                    markers/sphere_list_marker.cpp
                                    markers/cube_list_marker.cpp
                                    markers/points_marker.cpp
                                    marker_display.cpp
                                    axes_display.cpp
                                    grid_display.cpp
                                    point_cloud_base.cpp
                                    point_cloud_display.cpp
                                    laser_scan_display.cpp
                                    robot_model_display.cpp
                                    robot_base2d_pose_display.cpp
                                    particle_cloud_2d_display.cpp
                                    path_display.cpp
                                    polygon_display.cpp
                                    grid_cells_display.cpp
                                    map_display.cpp
                                    tf_display.cpp
                                    camera_display.cpp
                                    init.cpp)

include(CMakeDetermineSystem)
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  rospack_add_library(default_plugin MODULE ${SOURCE_FILES})
else(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  rospack_add_library(default_plugin ${SOURCE_FILES})
endif(CMAKE_SYSTEM_NAME MATCHES "Darwin")

                                    
target_link_libraries(default_plugin ${PROJECT_NAME} ${wxWidgets_LIBRARIES} ${OGRE_LIBRARIES})
rospack_link_boost(default_plugin thread signals filesystem system program_options)
