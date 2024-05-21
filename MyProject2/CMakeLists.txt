cmake_minimum_required(VERSION 3.10)
project(MyProject2)

# Find the MyProject package
find_package(myproject REQUIRED)
find_package(Boost REQUIRED)

# Add the include directories
include_directories(${MyProject_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Add the source files
set(SOURCES
    src/main.cpp
)

# Create an executable from the source files
add_executable(MyProject2Executable ${SOURCES})

# Link the MyProject library and Boost libraries
target_link_libraries(MyProject2Executable myproject::htheader_target ${Boost_LIBRARIES})
