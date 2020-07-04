# Install script for directory: /home/yihang/Downloads/Firmware-xw

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
    set(CMAKE_INSTALL_CONFIG_NAME "MinSizeRel")
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

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/yihang/Downloads/Firmware-xw/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/DriverFramework/framework/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/drivers/stm32/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/modules/px4iofirmware/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_fmu-v5_default/external/Build/px4io_firmware/boards/px4/io-v2/src/cmake_install.cmake")
  include("/home/yihang/Downloads/Firmware-xw/build/px4_fmu-v5_default/external/Build/px4io_firmware/platforms/nuttx/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/yihang/Downloads/Firmware-xw/build/px4_fmu-v5_default/external/Build/px4io_firmware/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
