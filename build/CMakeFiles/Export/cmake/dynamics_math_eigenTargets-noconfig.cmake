#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "dme::dynamics_math_eigen" for configuration ""
set_property(TARGET dme::dynamics_math_eigen APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(dme::dynamics_math_eigen PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libdynamics_math_eigen.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS dme::dynamics_math_eigen )
list(APPEND _IMPORT_CHECK_FILES_FOR_dme::dynamics_math_eigen "${_IMPORT_PREFIX}/lib/libdynamics_math_eigen.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
