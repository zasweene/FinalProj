# Install script for directory: D:/Downloads/nordictest/sdk/zephyr/lib

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files/Zephyr-Kernel")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "C:/ncs/toolchains/b620d30767/opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-objdump.exe")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("D:/Downloads/dwm3000-master/examples/ex_02d_rx_sniff/52840dk/52840/ex_02d_rx_sniff/zephyr/lib/crc/cmake_install.cmake")
  include("D:/Downloads/dwm3000-master/examples/ex_02d_rx_sniff/52840dk/52840/ex_02d_rx_sniff/zephyr/lib/libc/cmake_install.cmake")
  include("D:/Downloads/dwm3000-master/examples/ex_02d_rx_sniff/52840dk/52840/ex_02d_rx_sniff/zephyr/lib/posix/cmake_install.cmake")
  include("D:/Downloads/dwm3000-master/examples/ex_02d_rx_sniff/52840dk/52840/ex_02d_rx_sniff/zephyr/lib/hash/cmake_install.cmake")
  include("D:/Downloads/dwm3000-master/examples/ex_02d_rx_sniff/52840dk/52840/ex_02d_rx_sniff/zephyr/lib/heap/cmake_install.cmake")
  include("D:/Downloads/dwm3000-master/examples/ex_02d_rx_sniff/52840dk/52840/ex_02d_rx_sniff/zephyr/lib/mem_blocks/cmake_install.cmake")
  include("D:/Downloads/dwm3000-master/examples/ex_02d_rx_sniff/52840dk/52840/ex_02d_rx_sniff/zephyr/lib/os/cmake_install.cmake")
  include("D:/Downloads/dwm3000-master/examples/ex_02d_rx_sniff/52840dk/52840/ex_02d_rx_sniff/zephyr/lib/utils/cmake_install.cmake")

endif()

