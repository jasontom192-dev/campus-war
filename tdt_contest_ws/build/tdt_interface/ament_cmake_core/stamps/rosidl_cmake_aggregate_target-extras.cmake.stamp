# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target tdt_interface::tdt_interface
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${tdt_interface_TARGETS}.
if(tdt_interface_TARGETS AND NOT TARGET tdt_interface::tdt_interface)
  add_library(tdt_interface::tdt_interface INTERFACE IMPORTED)
  set_target_properties(tdt_interface::tdt_interface PROPERTIES
    INTERFACE_LINK_LIBRARIES "${tdt_interface_TARGETS}")
endif()
