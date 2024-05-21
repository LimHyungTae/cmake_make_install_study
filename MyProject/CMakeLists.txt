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

include(3rdparty/find_dependencies.cmake)

include_directories(${CMAKE_SOURCE_DIR}/include)

# 이렇게 한 줄로 쓰거나 (Jingnan's Style)
set(TARGET_NAME1 htheader_target)
add_library(${TARGET_NAME1} SHARED include/htproject/htheader.h src/htheader.cpp)
# 아래와 같이 두 줄로 쓸 수 있음 (KISS-ICP의 Nacho 스타일)
# add_library(htheader_target STATIC)
# target_sources(htheader_target PRIVATE include/htproject/htheader.h src/htheader.cpp)

# target_link_libraries(htheader PRIVATE tinyply)
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

# 아래 두 줄은 엄마(?) CMakeLists.txt에게 내가 export할 타겟들을 알려주는 것
# https://github.com/MIT-SPARK/TEASER-plusplus/blob/master/CMakeLists.txt
# (NOTE: 위의 참고하라는 CMakeLists.txt는 `teaser` 폴더 내에 있는 sub CMakeLists.txt임)
export(TARGETS ${MYPROJECT_EXPORTED_TARGETS} FILE ${PROJECT_NAME}-exports.cmake)

# 이 package의 종속 package들을 알려줌.
# 다른 CMakeLists.txt에서 `find_package(MyProject REQUIRED)`를 하면 `myprojectConfig.cmake`가 있는지 찾음
install(FILES cmake/${PROJECT_NAME}Config.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
        )
