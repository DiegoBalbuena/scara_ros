<<<<<<< HEAD
# Install script for directory: /home/diego/scara_ros/src/scara_ros

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/diego/scara_ros/install")
=======
# Install script for directory: /home/rabios/dev/scara_ros/src/scara_ros

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/rabios/dev/scara_ros/install")
>>>>>>> a939d1951b49873009e624302dcc1bd2adec4208
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/diego/scara_ros/build/scara_ros/catkin_generated/installspace/scara_ros.pc")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/rabios/dev/scara_ros/build/scara_ros/catkin_generated/installspace/scara_ros.pc")
>>>>>>> a939d1951b49873009e624302dcc1bd2adec4208
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/scara_ros/cmake" TYPE FILE FILES
<<<<<<< HEAD
    "/home/diego/scara_ros/build/scara_ros/catkin_generated/installspace/scara_rosConfig.cmake"
    "/home/diego/scara_ros/build/scara_ros/catkin_generated/installspace/scara_rosConfig-version.cmake"
=======
    "/home/rabios/dev/scara_ros/build/scara_ros/catkin_generated/installspace/scara_rosConfig.cmake"
    "/home/rabios/dev/scara_ros/build/scara_ros/catkin_generated/installspace/scara_rosConfig-version.cmake"
>>>>>>> a939d1951b49873009e624302dcc1bd2adec4208
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/scara_ros" TYPE FILE FILES "/home/diego/scara_ros/src/scara_ros/package.xml")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/scara_ros" TYPE FILE FILES "/home/rabios/dev/scara_ros/src/scara_ros/package.xml")
>>>>>>> a939d1951b49873009e624302dcc1bd2adec4208
endif()

