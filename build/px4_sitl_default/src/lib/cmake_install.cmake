# Install script for directory: /home/yihang/Downloads/Firmware-xw/src/lib

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/src/lib/airspeed/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/src/lib/battery/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/src/lib/bezier/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/src/lib/cdev/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/src/lib/circuit_breaker/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/src/lib/controllib/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/src/lib/conversion/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/src/lib/drivers/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/src/lib/ecl/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/src/lib/FlightTasks/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/src/lib/landing_slope/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/src/lib/led/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/src/lib/mathlib/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/src/lib/mixer/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/src/lib/perf/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/src/lib/pid/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/src/lib/pwm_limit/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/src/lib/rc/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/src/lib/terrain_estimation/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/src/lib/tunes/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/src/lib/version/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/src/lib/WeatherVane/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/src/lib/CollisionPrevention/cmake_install.cmake")

endif()

