# Build file for Python bindings via sip and pyqt.

set(rviz_HDRS_DIR ${PROJECT_SOURCE_DIR}/src/rviz)
set(SIP_BUILD_DIR ${PROJECT_SOURCE_DIR}/build/sip/rviz_sip)
set(SIP_SRC_DIR ${PROJECT_SOURCE_DIR}/src/python_bindings/sip)

set(rviz_sip_DEPENDENT_FILES
  string.sip
  visualization_frame.sip
  visualization_panel.sip
  ${rviz_HDRS_DIR}/visualization_frame.h
  ${rviz_HDRS_DIR}/visualization_panel.h
)

# ugly hack to obtain path to shared sip_configure.py
string(REGEX REPLACE "^([^;]*;)*([^;]+/python_qt_binding)/include(;[^;]*)*$" "\\2" python_qt_binding_SOURCE_DIR "${rviz_INCLUDE_DIRS}")

# maintain context for different named target
set(rviz_sip_INCLUDE_DIRS ${rviz_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/src)
set(rviz_sip_LIBRARIES ${rviz_LIBRARIES} ${PROJECT_NAME})
set(rviz_sip_LIBRARY_DIRS ${rviz_LIBRARY_DIRS} ${CMAKE_SOURCE_DIR}/lib)
set(rviz_sip_LDFLAGS_OTHER ${rviz_LDFLAGS_OTHER} -Wl,-rpath,${CMAKE_SOURCE_DIR}/lib)

rosbuild_include(python_qt_binding sip_helper)

if(sip_helper_FOUND)
  list(APPEND rviz_BINDINGS "sip")
  set(rviz_BINDINGS "${rviz_BINDINGS}" PARENT_SCOPE)

  build_sip_binding(rviz_sip rviz.sip
    SIP_CONFIGURE ${python_qt_binding_SOURCE_DIR}/cmake/sip_configure.py
    SOURCE_DIR ${PROJECT_SOURCE_DIR}/src/python_bindings/sip
    DEPENDS ${rviz_sip_DEPENDENT_FILES}
    DEPENDENCIES rviz
  )
endif()
