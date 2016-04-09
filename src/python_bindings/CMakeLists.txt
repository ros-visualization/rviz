set(rviz_BINDINGS "")

# TODO(wjwwood): re-enabled PySide2 support when it is fixed.
# add_subdirectory(shiboken)
add_subdirectory(sip)

message("Python binding generators: ${rviz_BINDINGS}")
if(NOT rviz_BINDINGS)
  message(FATAL_ERROR "No Python binding generator found.")
endif()
