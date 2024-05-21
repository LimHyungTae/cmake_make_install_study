get_filename_component(MYPROJECT_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
include(CMakeFindDependencyMacro)

find_dependency(Eigen3 3.3 REQUIRED)
find_dependency(OpenMP REQUIRED)

include("${MYPROJECT_CMAKE_DIR}/myprojectTargets.cmake")
