rosbuild_add_gtest(uniform_string_stream_test uniform_string_stream_test.cpp ../rviz/uniform_string_stream.cpp)

rosbuild_add_executable(marker_test marker_test.cpp)
rosbuild_declare_test(marker_test)

rosbuild_add_executable(mesh_marker_test mesh_marker_test.cpp)
rosbuild_declare_test(mesh_marker_test)

rosbuild_add_executable(interactive_marker_test interactive_marker_test.cpp)
rosbuild_declare_test(interactive_marker_test)

# rosbuild_add_executable(cloud_test EXCLUDE_FROM_ALL cloud_test.cpp)
# rosbuild_declare_test(cloud_test)
# rosbuild_add_executable(image_test EXCLUDE_FROM_ALL image_test.cpp)
# rosbuild_declare_test(image_test)
# 
rosbuild_add_gtest(interactive_marker_client_test
  interactive_marker_client_test.cpp
  ../rviz/default_plugin/interactive_markers/interactive_marker_client.cpp)
 
rosbuild_add_executable(send_lots_of_points send_lots_of_points_node.cpp)
rosbuild_add_executable(send_grid_cells send_grid_cells_node.cpp)
rosbuild_add_executable(send_images send_images.cpp)

rosbuild_add_executable(vis_panel_example vis_panel_example.cpp)
target_link_libraries(vis_panel_example ${PROJECT_NAME} ${QT_LIBRARIES})
rosbuild_declare_test(vis_panel_example)

rosbuild_add_executable(render_panel_test render_panel_test.cpp)
target_link_libraries(render_panel_test ${PROJECT_NAME} ${QT_LIBRARIES})
rosbuild_declare_test(render_panel_test)

rosbuild_add_executable(new_display_dialog_test new_display_dialog_test.cpp)
target_link_libraries(new_display_dialog_test ${PROJECT_NAME} ${QT_LIBRARIES})
rosbuild_declare_test(new_display_dialog_test)

rosbuild_add_executable(color_editor_test color_editor_test.cpp)
target_link_libraries(color_editor_test ${PROJECT_NAME} ${QT_LIBRARIES})
rosbuild_declare_test(color_editor_test)

qt4_wrap_cpp(PROPERTY_TEST_MOC_FILES
  ros_spinner.h
  )
rosbuild_add_executable(property_test property_test.cpp ros_spinner.cpp ${PROPERTY_TEST_MOC_FILES})
target_link_libraries(property_test ${PROJECT_NAME} ${QT_LIBRARIES})
rosbuild_declare_test(property_test)

rosbuild_add_gtest(topic_info_variant_test topic_info_variant_test.cpp)
target_link_libraries(topic_info_variant_test ${QT_LIBRARIES})

rosbuild_add_executable(line_edit_with_button_test line_edit_with_button_test.cpp)
target_link_libraries(line_edit_with_button_test ${PROJECT_NAME} ${QT_LIBRARIES})
rosbuild_declare_test(line_edit_with_button_test)

rosbuild_add_executable(topic_dialog_test topic_dialog_test.cpp)
target_link_libraries(topic_dialog_test ${PROJECT_NAME} ${QT_LIBRARIES})
rosbuild_declare_test(topic_dialog_test)

qt4_wrap_cpp(MOC_FILES
  connect_test.h
  )
rosbuild_add_executable(connect_test connect_test.cpp ${MOC_FILES})
target_link_libraries(connect_test ${QT_LIBRARIES})
rosbuild_declare_test(connect_test)

rosbuild_add_gtest(config_test config_test.cpp ../rviz/uniform_string_stream.cpp ../rviz/config.cpp)

qt4_wrap_cpp(RENDER_POINTS_TEST_MOC_FILES
  render_points_test.h
  )
rosbuild_add_executable(render_points_test render_points_test.cpp ../rviz/ogre_helpers/orbit_camera.cpp ${RENDER_POINTS_TEST_MOC_FILES})
target_link_libraries(render_points_test ${PROJECT_NAME} ${QT_LIBRARIES})
rosbuild_declare_test(render_points_test)

rosbuild_add_executable(two_render_widgets two_render_widgets.cpp)
target_link_libraries(two_render_widgets ${PROJECT_NAME} ${QT_LIBRARIES})
rosbuild_declare_test(two_render_widgets)
