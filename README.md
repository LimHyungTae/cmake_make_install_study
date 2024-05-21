# CmakeLists.txt를 통한 나만의 package 만들기 


## Test env.

Ubuntu 20.04

## 파일 설명 

* [**[Myproject]**](https://github.com/LimHyungTae/cmake_make_install_study/tree/master/MyProject): 설치될 header 파일이 있는 프로젝트. `sudo make install`로 로컬 환경에 설치됨
* [**[Myproject2]**](https://github.com/LimHyungTae/cmake_make_install_study/tree/master/MyProject2): Myproject를 참조하는 프로젝트. `find_package(MyProject REQUIRED)`로 참조함

---

## How To Test

1. Clone the repository
 
```angular2html
git clone git@github.com:LimHyungTae/cmake_make_install_study.git
```

2. Build and install `MyProject`

```angular2html
$ cd MyProject && mkdir build && cd build
$ cmake .. && make -j 8 && sudo make install
```

3. Build `MyProject2` and run `./MyProject2Executable`

