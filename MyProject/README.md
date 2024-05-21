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


#### `include(GNUInstallDirs)`

* `sudo apt-get install cmake`로 CMake를 설치하면, `GNUInstallDirs`가 포함되어 있음.

* `include(GNUInstallDirs)`을 CMakeLists.txt에 추가해야 함. 안 그러면 `CMAKE_INSTALL_LIBDIR`와 `CMAKE_INSTALL_INCLUDEDIR`가 정의되지 않음.
* 근데 `sudo make install`하려면 두 변수가 필요함!!!


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


`myprojectTargets-release.cmake`의 내용을 살펴보면 아래와 같음

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