set(rviz_BINDINGS "")

add_subdirectory(shiboken)
add_subdirectory(sip)

message("Python binding generators: ${rviz_BINDINGS}")
if(NOT rviz_BINDINGS)
  message(FATAL_ERROR "No Python binding generator found.")
endif()
