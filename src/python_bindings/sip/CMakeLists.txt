# Build file for Python bindings via sip and pyqt.

set(rviz_HDRS_DIR ${PROJECT_SOURCE_DIR}/src/rviz)
set(SIP_BUILD_DIR ${PROJECT_SOURCE_DIR}/build/sip/rviz_sip)
set(SIP_SRC_DIR ${PROJECT_SOURCE_DIR}/src/python_bindings/sip)

set(rviz_SIPS
  string.sip
  visualization_frame.sip
  visualization_panel.sip
)

set(rviz_HDRS
  ${rviz_HDRS_DIR}/visualization_frame.h
  ${rviz_HDRS_DIR}/visualization_panel.h
)

add_custom_command(
  OUTPUT ${SIP_BUILD_DIR}/Makefile
  COMMAND python configure.py ${SIP_BUILD_DIR} rviz.sip ${PROJECT_SOURCE_DIR}/lib
  DEPENDS configure.py ${rviz_SIPS} ${rviz_HDRS}
  WORKING_DIRECTORY ${SIP_SRC_DIR}
  COMMENT "Running SIP generator for librviz Python bindings..."
)

add_custom_target(librviz_sip.so ALL
  COMMAND make
  DEPENDS ${SIP_BUILD_DIR}/Makefile
  WORKING_DIRECTORY ${SIP_BUILD_DIR}
  COMMENT "Compiling generated code for librviz Python bindings with ${SIP_BUILD_DIR}/Makefile..."
)
