# MIT License
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
cmake_minimum_required(VERSION 3.16...3.24)
project(MyProject VERSION 0.1.2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

message("CMAKE_CURRENT_LIST_DIR in CMakeLists.txt ${CMAKE_CURRENT_LIST_DIR}")

message("CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
message("CMAKE_BINARY_DIR: ${CMAKE_BINARY_DIR}")
message("CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
message("CMAKE_CURRENT_BINARY_DIR: ${CMAKE_CURRENT_BINARY_DIR}")
message("CMAKE_CURRENT_LIST_DIR: ${CMAKE_CURRENT_LIST_DIR}")
message("CMAKE_CURRENT_LIST_FILE: ${CMAKE_CURRENT_LIST_FILE}")
message("CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
message("PROJECT_BINARY_DIR: ${PROJECT_BINARY_DIR}")
message("CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

# Set build type
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# find_package(catkin QUIET)
# Build always core library

include(3rdparty/find_dependencies.cmake)
include(cmake/CompilerOptions.cmake)

include_directories(${CMAKE_SOURCE_DIR}/include)

# Create an executable from the source files
add_executable(MyExecutable src/main.cpp)

# Specify installation rules
install(TARGETS MyExecutable DESTINATION /usr/local/bin)

# Optionally install headers (if needed)
install(DIRECTORY include/ DESTINATION /usr/local/include)

# Create and install CMake package configuration files
include(CMakePackageConfigHelpers)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/MyProjectConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/MyProjectConfig.cmake"
    INSTALL_DESTINATION /usr/local/lib/cmake/MyProject
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/MyProjectConfig.cmake"
    DESTINATION /usr/local/lib/cmake/MyProject
)
