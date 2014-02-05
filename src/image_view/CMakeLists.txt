
qt4_wrap_cpp(MOC_FILES
  image_view.h
)

add_executable(rviz_image_view
  image_view.cpp
  main.cpp
  ${MOC_FILES}
)

target_link_libraries(rviz_image_view
  ${OGRE_OV_LIBRARIES_ABS}
  ${rviz_ADDITIONAL_LIBRARIES}
  ${PROJECT_NAME}
)

set_target_properties(rviz_image_view
                      PROPERTIES OUTPUT_NAME image_view
                      PREFIX "")

