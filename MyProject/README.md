# Cmake Install Study for the Stand-Alone Repository


---
## 공부한 것들

설명은 [CMakelists.txt](https://github.com/LimHyungTae/cmake_make_install_study/blob/master/MyProject/CMakeLists.txt)에 self-contained로 주석으로 달아놓았음.

### CMakeLists.txt에서 알아두면 좋은 변수들

```cmake
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
```


### `include(GNUInstallDirs)`

* `sudo apt-get install cmake`로 CMake를 설치하면, `GNUInstallDirs`가 자동으로 설치됨

* sudo make install을 하려면 저 `GNUInstallDirs`를 include하는 것이 필요함, i.e., `include(GNUInstallDirs)`을 CMakeLists.txt에 추가해야 함.
* 안 그러면 `CMAKE_INSTALL_LIBDIR`와 `CMAKE_INSTALL_INCLUDEDIR`가 정의되지 않음.
* 근데 `sudo make install`하려면 두 변수가 필요함!!! (아래의 추가 후 참조)


* **추가 하기 전** 

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

* **추가 후**

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

### 타겟(target)에 대한 이해

* 타겟(Target): 라이브러리를 만들 때 한 묶음이라고 생각하면 됨. 
* 편의성을 위해 여러 개로 쪼개기도 함(의미론적으로 라이브러리를 구분하기 위해. [여기](https://github.com/MIT-SPARK/TEASER-plusplus/blob/master/teaser/CMakeLists.txt#L6-L79) 참조)
* 주로 패턴은 아래오 같음.

#### 1. `add_library`

타겟의 이름을 정하고, 해당하는 파일들을 설정해 줌 (개인적 생각: 헤더 파일은 `include_directories(${CMAKE_SOURCE_DIR}/include)`해뒀으면 굳이 안 해줘도 되는듯? 확실치 않음)
 
* SHARED: 동적 라이브러리가 생성. `.so`. 해당 `.so`가 실행될 때 필요한 라이브러리들을 로드함 (컴파일 시에는 '이러한 라이브러리들 써야지~' 정도의 정보만 기술되어 있음.
* STATIC: 정적 라이브러리가 생성. `.a` (Unix/Linux의 경우). 컴파일 시 라이브러리들의 코드가 프로그램 실행파일에 포함됨

#### 2. `target_link_libraries`

만들고자 하는 라이브러리에 필요한 헤더파일들을 링크해줘야 함. 현재 여기 예제에서는 `Eigen3::Eigen`을 링크시켜 주고 있음! 현재는 SHARED이기 때문에 이 Eigen3는 로컬 컴퓨터에 설치되어 있어야 함

#### 3. `target_include_directories `   

* 뭔가 복잡하게 생겼으나, 필요한 include 파일들의 directory 설정해주는 모양인듯. 저 INTERFACE라는 게 알아서 잘 해주나봄..
* 추가적으로 더 필요한 library가 있으면 [여기](https://github.com/MIT-SPARK/TEASER-plusplus/blob/9ca20d9b52fcb631e7f8c9e3cc55c5ba131cc4e6/teaser/CMakeLists.txt#L33)와 같이 라이브러리의 include directory를 추가해줘야 함

#### 4. `install`

아래의 틀을 그냥 복사 붙여넣기 하면 잘 동작하는 듯 함. `CMAKE_INSTALL_LIBDIR`가 `GNUInstallDirs`를 통해서 할당이 되어서 가능해진 것으로 보임

아래는 [내가 작성한 MyProject의 CMakeLists.txt](https://github.com/LimHyungTae/cmake_make_install_study/blob/master/MyProject/CMakeLists.txt)에 해당하는 코드:

```
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
```

그 이외는 CMakeLists.txt에 주석으로 설명을 달아뒀음

---

### sudo make install

`sudo make install`하면 아래와 같은 메세지가 뜸

```commandline
-- Install configuration: "Release"
-- Installing: /usr/local/lib/libhtheader_target.so
-- Old export file "/usr/local/lib/cmake/myproject/myprojectTargets.cmake" will be replaced.  Removing files [/usr/local/lib/cmake/myproject/myprojectTargets-release.cmake].
-- Installing: /usr/local/lib/cmake/myproject/myprojectTargets.cmake
-- Installing: /usr/local/lib/cmake/myproject/myprojectTargets-release.cmake
-- Up-to-date: /usr/local/include
-- Up-to-date: /usr/local/include/htproject
-- Installing: /usr/local/include/htproject/htheader.h
-- Up-to-date: /usr/local/lib/cmake/myproject/myprojectConfig.cmake
```

* `myprojectTargets.cmake`:
    * 타겟을 정의하고, 타겟 간의 의존성을 설정하며, 타겟이 사용될 때 필요한 모든 정보를 포함
    * CMake에서 find_package(myproject)를 호출할 때 이 파일을 로드하여, myproject 타겟을 현재 프로젝트에서 사용할 수 있도록 해줌

* `myprojectTargets-release.cmake`:
    * Release 빌드에 특화된 타겟 설정을 제공
    * myprojectTargets.cmake 파일과 함께 로드되어, Release 빌드 설정을 구성
    * 최적화된 Release 빌드 타겟이 필요할 때 사용


`/usr/local/lib/cmake/myproject/myprojectTargets-release.cmake`의 내용을 살펴보면 아래와 같음

```cmake
#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "myproject::htheader_target" for configuration "Release"
set_property(TARGET myproject::htheader_target APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(myproject::htheader_target PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libhtheader_target.so"
  IMPORTED_SONAME_RELEASE "libhtheader_target.so"
  )

list(APPEND _cmake_import_check_targets myproject::htheader_target )
list(APPEND _cmake_import_check_files_for_myproject::htheader_target "${_IMPORT_PREFIX}/lib/libhtheader_target.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
```

즉, 어떤 **shared library(`.so` 파일)를 가져와야 하는지 명시 되어있음**!
그리고 `myproject::htheader_target`를 살펴보면, 내가 CMakeLists.txt의 [여기](https://github.com/LimHyungTae/cmake_make_install_study/blob/653f92bb197775a8621cdef97be5f162673dda9f/MyProject/CMakeLists.txt#L63)에서 한 add_library의 이름으로 shared library가 생성된 것을 확인할 수 있다.
* `${PROJECT_NAME}::${TARGET_NAME1}`은 곧 :arrow_right: `myproject::htheader_target `임!
