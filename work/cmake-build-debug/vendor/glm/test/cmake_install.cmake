# Install script for directory: /cygdrive/c/Users/daniel/workspace/cgra350/cgra-350-assignment-3/work/vendor/glm/test

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
    set(CMAKE_INSTALL_CONFIG_NAME "DEBUG")
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
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/cygdrive/c/Users/daniel/workspace/cgra350/cgra-350-assignment-3/work/cmake-build-debug/vendor/glm/test/bug/cmake_install.cmake")
  include("/cygdrive/c/Users/daniel/workspace/cgra350/cgra-350-assignment-3/work/cmake-build-debug/vendor/glm/test/core/cmake_install.cmake")
  include("/cygdrive/c/Users/daniel/workspace/cgra350/cgra-350-assignment-3/work/cmake-build-debug/vendor/glm/test/gtc/cmake_install.cmake")
  include("/cygdrive/c/Users/daniel/workspace/cgra350/cgra-350-assignment-3/work/cmake-build-debug/vendor/glm/test/gtx/cmake_install.cmake")

endif()

