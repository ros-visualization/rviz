add_executable(rviz_image_view
  image_view.cpp
  main.cpp
)

target_link_libraries(rviz_image_view
  ${OGRE_LIBRARIES}
  ${rviz_ADDITIONAL_LIBRARIES}
  ${PROJECT_NAME}
)

set_target_properties(rviz_image_view
                      PROPERTIES OUTPUT_NAME image_view
                      PREFIX "")
