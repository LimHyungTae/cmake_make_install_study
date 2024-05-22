# MIT License
cmake_minimum_required(VERSION 3.16...3.24)
set(INSTALLED_PROJECT_NAME myproject)
project(${INSTALLED_PROJECT_NAME} VERSION 0.1.2 LANGUAGES CXX)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)

message("----For showing various variables in CMakeLists.txt ----")
message("CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
message("CMAKE_BINARY_DIR: ${CMAKE_BINARY_DIR}")
message("CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
message("CMAKE_CURRENT_BINARY_DIR: ${CMAKE_CURRENT_BINARY_DIR}")
message("CMAKE_CURRENT_LIST_DIR: ${CMAKE_CURRENT_LIST_DIR}")
message("CMAKE_CURRENT_LIST_FILE: ${CMAKE_CURRENT_LIST_FILE}")
message("CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
message("CMAKE_INSTALL_LIBDIR: ${CMAKE_INSTALL_LIBDIR}")
message("CMAKE_INSTALL_INCLUDEDIR: ${CMAKE_INSTALL_INCLUDEDIR}")
message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
message("PROJECT_BINARY_DIR: ${PROJECT_BINARY_DIR}")
message("CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
message("-------------------------------------------------------")

# Set build type
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Eigen3 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)

# 이렇게 한 줄로 쓰거나 (Jingnan's Style)
set(TARGET_NAME1 htheader_target)
add_library(${TARGET_NAME1} SHARED include/htproject/htheader.h src/htheader.cpp)
# 아래와 같이 두 줄로 쓸 수 있음 (KISS-ICP의 Nacho 스타일)
# add_library(htheader_target STATIC)
# target_sources(htheader_target PRIVATE include/htproject/htheader.h src/htheader.cpp)
target_link_libraries(${TARGET_NAME1}
        PUBLIC Eigen3::Eigen
        )
target_include_directories(${TARGET_NAME1}
    PRIVATE
        # where the library itself will look for its internal headers
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)


install(TARGETS ${TARGET_NAME1}
        EXPORT ${PROJECT_NAME}-export
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )

# 여러 타겟을 export할 때 사용
# https://github.com/MIT-SPARK/TEASER-plusplus/blob/master/teaser/CMakeLists.txt 내에서 여러 타겟을
# `MYPROJECT_EXPORTED_TARGETS`에 등록함
# 여기서는 사실 타겟도 1개이고, CMakeLists.txt도 1개여서 굳이 필요없음. 하지만 APPEND를 통해 타겟을 관리할 수 있음을 보여주기 위해 추가해 둠
list(APPEND MYPROJECT_EXPORTED_TARGETS ${TARGET_NAME1})
add_library(${PROJECT_NAME}::${TARGET_NAME1} ALIAS ${TARGET_NAME1})
# 아래는 APPEND한 것을 최종적으로 부모 CMakeLists.txt에 전달하기 위함
set(MYPROJECT_EXPORTED_TARGETS "${MYPROJECT_EXPORTED_TARGETS}" PARENT_SCOPE)

# 아래 block을 추가하니 /usr/local/lib/cmake/MyProject에
# MyProjectTargets.cmake와 MyProjectTargets-release.cmake가 생김!
install(EXPORT ${PROJECT_NAME}-export
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    NAMESPACE ${PROJECT_NAME}::
    FILE ${PROJECT_NAME}Targets.cmake
)

# CMAKE_INSTALL_INCLUDEDIR로 파일들을 보낸다는 뜻
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# 아래 두 줄은 PARENT CMakeLists.txt에게 내가 export할 타겟들을 알려주는 것
# 이 레포지토리의 CMakeLists.txt는 단일로 구성돼있어서 위의 `set(MYPROJECT_EXPORTED_TARGETS "${MYPROJECT_EXPORTED_TARGETS}" PARENT_SCOPE)`가 사실 동작할 PARENT가 존재하지 않음.
#   PARENT-Local scope의 예시:
#     https://github.com/MIT-SPARK/TEASER-plusplus/blob/master/CMakeLists.txt <- 이것이 PARENT
#     https://github.com/MIT-SPARK/TEASER-plusplus/blob/master/teaser/CMakeLists.txt <- 이것은 local, 혹은 current scope라고 부른다고 함
# 무튼 아래 export() 명령어를 통해, 현재 내가 make -j ${CORE_NUM}을 한 곳에 `myproject-exports.cmake`가 생기게 함 
export(TARGETS ${MYPROJECT_EXPORTED_TARGETS} FILE ${PROJECT_NAME}-exports.cmake)

# 미리 cmake 폴더에 작성해둔 `myprojectConfig.cmake`을 우리가 `sudo make install`입력하면 
# 이 `myprojectConfig.cmake`가 `/usr/local/lib/cmake/myproject/myprojectConfig.cmake`에 설치됨.
# `/usr/local/lib/cmake` 이 주소에 가면 현재 로컬 컴퓨터에서 설치한 패키지들이 어떤게 있는지 확인할 수 있음

#   Q. `cmake/myprojectConfig.cmake`의 역할
#       `cmake/myprojectConfig.cmake` 내부를 gedit이나 vim으로 열어보면 어떤 package를 필요로하는지 적혀 있음
#       다른 CMakeLists.txt에서 `find_package(MyProject REQUIRED)`를 하면 이 package의 종속 package들도 찾아줌
#       그래서 `MyProject2`의 CMakeLists.txt에서는 `find_package(MyProject REQUIRED)`만 하면 `find_package(Eigen3 REQUIRED)`을 안해도 됨!!!
install(FILES cmake/${PROJECT_NAME}Config.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
        )
