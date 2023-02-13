# Cmake study benchmarking Nacho's KISS-ICP

I really want to get familiar with Nacho's OOTB philosophy!!!!

---

## Test env.

Ubuntu 20.04

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
