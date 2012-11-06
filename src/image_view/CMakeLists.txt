
qt4_wrap_cpp(MOC_FILES
  image_view.h
)

add_executable(image_view
  image_view.cpp
  main.cpp
  ${MOC_FILES}
)

target_link_libraries(image_view ${OGRE_LIBRARIES} ${PROJECT_NAME})
