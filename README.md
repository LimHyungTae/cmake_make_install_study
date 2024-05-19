# Cmake study benchmarking Nacho's KISS-ICP

I really want to get familiar with Nacho's OOTB philosophy!!!!

---

## Test env.

Ubuntu 20.04

---
## 공부한 것들

<details>
<summary>알아두면 CMAKE 주소 관련 명령어들</summary>

```angular2html
CMAKE_CURRENT_LIST_DIR in CMakeLists.txt /home/shapelim/git/cmake_auto_include_study
CMAKE_SOURCE_DIR: /home/shapelim/git/cmake_auto_include_study
CMAKE_BINARY_DIR: /home/shapelim/git/cmake_auto_include_study/build
CMAKE_CURRENT_SOURCE_DIR: /home/shapelim/git/cmake_auto_include_study
CMAKE_CURRENT_BINARY_DIR: /home/shapelim/git/cmake_auto_include_study/build
CMAKE_CURRENT_LIST_DIR: /home/shapelim/git/cmake_auto_include_study
CMAKE_CURRENT_LIST_FILE: /home/shapelim/git/cmake_auto_include_study/CMakeLists.txt
CMAKE_INSTALL_PREFIX: /usr/local
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


---

### Checking checksum

$ shasum -a 256 ${ZIP or TAR.GZ name}

![weird_error](materials/hash_miss_matching.png)
---


### How to build cmake repository by using catkin?

Just add

```
find_package(ament_cmake QUIET) # for ROS2
find_package(catkin QUIET)      # for ROS1
```

Then, it works!

Without that line, we can see


![catkin_build_error](materials/catkin_build_error.png)
