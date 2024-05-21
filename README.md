# CmakeLists.txt를 통한 나만의 package 만들기 


## Test env.

Ubuntu 20.04

## 파일 설명 

* [Myproject](https://github.com/LimHyungTae/cmake_auto_include_study/tree/master/MyProject: 설치될 header 파일이 있는 프로젝트. `sudo make install`로 로컬 환경에 설치됨
* [MyProject2](https://github.com/LimHyungTae/cmake_auto_include_study/tree/master/MyProject2): Myproject를 참조하는 프로젝트. `find_package(MyProject REQUIRED)`로 참조함

---
## 공부한 것들

<details>
<summary>알아두면 CMAKE 주소 관련 명령어들</summary>

`include(GNUInstallDirs)`을 CMakeLists.txt에 추가해야 함.

* 추가 하기 전 

```angular2html
CMAKE_CURRENT_LIST_DIR in CMakeLists.txt /home/shapelim/git/cmake_auto_include_study
CMAKE_SOURCE_DIR: /home/shapelim/git/cmake_auto_include_study
CMAKE_BINARY_DIR: /home/shapelim/git/cmake_auto_include_study/build
CMAKE_CURRENT_SOURCE_DIR: /home/shapelim/git/cmake_auto_include_study
CMAKE_CURRENT_BINARY_DIR: /home/shapelim/git/cmake_auto_include_study/build
CMAKE_CURRENT_LIST_DIR: /home/shapelim/git/cmake_auto_include_study
CMAKE_CURRENT_LIST_FILE: /home/shapelim/git/cmake_auto_include_study/CMakeLists.txt
CMAKE_INSTALL_PREFIX: /usr/local
CMAKE_INSTALL_LIBDIR: 
CMAKE_INSTALL_INCLUDEDIR: 
PROJECT_SOURCE_DIR: /home/shapelim/git/cmake_auto_include_study
PROJECT_BINARY_DIR: /home/shapelim/git/cmake_auto_include_study/build
```

* 추가 후

```commandline
CMAKE_CURRENT_LIST_DIR in CMakeLists.txt /home/shapelim/git/cmake_auto_include_study
CMAKE_SOURCE_DIR: /home/shapelim/git/cmake_auto_include_study
CMAKE_BINARY_DIR: /home/shapelim/git/cmake_auto_include_study/build
CMAKE_CURRENT_SOURCE_DIR: /home/shapelim/git/cmake_auto_include_study
CMAKE_CURRENT_BINARY_DIR: /home/shapelim/git/cmake_auto_include_study/build
CMAKE_CURRENT_LIST_DIR: /home/shapelim/git/cmake_auto_include_study
CMAKE_CURRENT_LIST_FILE: /home/shapelim/git/cmake_auto_include_study/CMakeLists.txt
CMAKE_INSTALL_PREFIX: /usr/local
CMAKE_INSTALL_LIBDIR: lib
CMAKE_INSTALL_INCLUDEDIR: include
PROJECT_SOURCE_DIR: /home/shapelim/git/cmake_auto_include_study
PROJECT_BINARY_DIR: /home/shapelim/git/cmake_auto_include_study/build
```

</details>

<details>
<summary>.cmake 파일의 역할</summary>

1. 프로젝트 설정:

프로젝트의 이름, 버전, 요구되는 최소 CMake 버전 등을 정의합니다.
예: project(MyProject), cmake_minimum_required(VERSION 3.10)

2. 컴파일 및 링크 설정:

소스 파일과 헤더 파일을 추가하고, 컴파일 옵션과 링크 옵션을 설정합니다.
예: add_executable(MyExecutable ${SOURCES}), target_link_libraries(MyExecutable MyLibrary)

3. 패키지 찾기:

find_package 명령어를 사용하여 필요한 외부 패키지를 찾습니다.
예: find_package(OpenCV REQUIRED)

4. 빌드 디렉토리 설정:

빌드 아티팩트의 출력 디렉토리를 설정합니다.
예: set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

5. 설치 규칙:

install 명령어를 사용하여 빌드된 파일을 시스템의 특정 위치에 설치합니다.
예: install(TARGETS MyExecutable DESTINATION /usr/local/bin)

6. 패키지 구성 파일 생성:

패키지 구성 파일을 생성하여 다른 프로젝트에서 패키지를 쉽게 찾을 수 있도록 합니다.
예: configure_package_config_file(...
</details>


### Install test

Install하니 아래와 같이 뜸

```commandline
-- Install configuration: "Release"
-- Installing: /usr/local/bin/MyExecutable
-- Up-to-date: /usr/local/include
-- Installing: /usr/local/include/htproject
-- Installing: /usr/local/include/htproject/htheader.hpp
-- Installing: /usr/local/lib/cmake/MyProject/MyProjectConfig.cmake
```

```commandline
-- Install configuration: "Release"
-- Installing: /usr/local/lib/libhtheader_target.so
-- Installing: /usr/local/lib/cmake/myproject/myprojectTargets.cmake
-- Installing: /usr/local/lib/cmake/myproject/myprojectTargets-release.cmake
-- Up-to-date: /usr/local/include
-- Up-to-date: /usr/local/include/htproject
-- Up-to-date: /usr/local/include/htproject/htheader.h
-- Up-to-date: /usr/local/lib/cmake/myproject/myprojectConfig.cmake
```

설치해서 `.cmake` 파일을 생성한 후 MyProject2에 다시 사용하는 방법?





---
 
(Deprecated)


### How to build cmake repository by using catkin?

Just add

```
find_package(ament_cmake QUIET) # for ROS2
find_package(catkin QUIET)      # for ROS1
```

Then, it works!

Without that line, we can see


![catkin_build_error](materials/catkin_build_error.png)
