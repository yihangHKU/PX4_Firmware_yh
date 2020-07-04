# Install script for directory: /home/yihang/Downloads/Firmware-xw/platforms/posix

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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/px4" TYPE DIRECTORY FILES
    "/home/yihang/Downloads/Firmware-xw/posix-configs"
    "/home/yihang/Downloads/Firmware-xw/ROMFS"
    "/home/yihang/Downloads/Firmware-xw/test"
    "/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/bin"
    USE_SOURCE_PERMISSIONS)
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/px4" TYPE DIRECTORY FILES
    "/home/yihang/Downloads/Firmware-xw/integrationtests"
    "/home/yihang/Downloads/Firmware-xw/launch"
    "/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/bin"
    USE_SOURCE_PERMISSIONS)
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/px4" TYPE FILE FILES
    "/home/yihang/Downloads/Firmware-xw/CMakeLists.txt"
    "/home/yihang/Downloads/Firmware-xw/package.xml"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/px4/Tools" TYPE DIRECTORY FILES "/home/yihang/Downloads/Firmware-xw/Tools/ecl_ekf" USE_SOURCE_PERMISSIONS)
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/px4/Tools" TYPE PROGRAM FILES
    "/home/yihang/Downloads/Firmware-xw/Tools/setup_gazebo.bash"
    "/home/yihang/Downloads/Firmware-xw/Tools/upload_log.py"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/px4/build/px4_sitl_default" TYPE DIRECTORY FILES "/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/build_gazebo" FILES_MATCHING REGEX "/CMakeFiles$" EXCLUDE REGEX "/[^/]*\\.so$")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/px4/Tools/sitl_gazebo" TYPE DIRECTORY FILES
    "/home/yihang/Downloads/Firmware-xw/Tools/sitl_gazebo/models"
    "/home/yihang/Downloads/Firmware-xw/Tools/sitl_gazebo/worlds"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/px4/Tools/sitl_gazebo" TYPE FILE FILES
    "/home/yihang/Downloads/Firmware-xw/Tools/sitl_gazebo/CMakeLists.txt"
    "/home/yihang/Downloads/Firmware-xw/Tools/sitl_gazebo/package.xml"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/yihang/Downloads/Firmware-xw/build/px4_sitl_default/platforms/posix/src/cmake_install.cmake")

endif()

