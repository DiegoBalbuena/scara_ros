<<<<<<< HEAD
# Install script for directory: /home/diego/scara_ros/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/diego/scara_ros/install")
=======
# Install script for directory: /home/rabios/dev/scara_ros/src

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
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/diego/scara_ros/install/_setup_util.py")
=======
   "/home/rabios/dev/scara_ros/install/_setup_util.py")
>>>>>>> a939d1951b49873009e624302dcc1bd2adec4208
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/diego/scara_ros/install" TYPE PROGRAM FILES "/home/diego/scara_ros/build/catkin_generated/installspace/_setup_util.py")
=======
file(INSTALL DESTINATION "/home/rabios/dev/scara_ros/install" TYPE PROGRAM FILES "/home/rabios/dev/scara_ros/build/catkin_generated/installspace/_setup_util.py")
>>>>>>> a939d1951b49873009e624302dcc1bd2adec4208
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/diego/scara_ros/install/env.sh")
=======
   "/home/rabios/dev/scara_ros/install/env.sh")
>>>>>>> a939d1951b49873009e624302dcc1bd2adec4208
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/diego/scara_ros/install" TYPE PROGRAM FILES "/home/diego/scara_ros/build/catkin_generated/installspace/env.sh")
=======
file(INSTALL DESTINATION "/home/rabios/dev/scara_ros/install" TYPE PROGRAM FILES "/home/rabios/dev/scara_ros/build/catkin_generated/installspace/env.sh")
>>>>>>> a939d1951b49873009e624302dcc1bd2adec4208
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/diego/scara_ros/install/setup.bash;/home/diego/scara_ros/install/local_setup.bash")
=======
   "/home/rabios/dev/scara_ros/install/setup.bash;/home/rabios/dev/scara_ros/install/local_setup.bash")
>>>>>>> a939d1951b49873009e624302dcc1bd2adec4208
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/diego/scara_ros/install" TYPE FILE FILES
    "/home/diego/scara_ros/build/catkin_generated/installspace/setup.bash"
    "/home/diego/scara_ros/build/catkin_generated/installspace/local_setup.bash"
=======
file(INSTALL DESTINATION "/home/rabios/dev/scara_ros/install" TYPE FILE FILES
    "/home/rabios/dev/scara_ros/build/catkin_generated/installspace/setup.bash"
    "/home/rabios/dev/scara_ros/build/catkin_generated/installspace/local_setup.bash"
>>>>>>> a939d1951b49873009e624302dcc1bd2adec4208
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/diego/scara_ros/install/setup.sh;/home/diego/scara_ros/install/local_setup.sh")
=======
   "/home/rabios/dev/scara_ros/install/setup.sh;/home/rabios/dev/scara_ros/install/local_setup.sh")
>>>>>>> a939d1951b49873009e624302dcc1bd2adec4208
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/diego/scara_ros/install" TYPE FILE FILES
    "/home/diego/scara_ros/build/catkin_generated/installspace/setup.sh"
    "/home/diego/scara_ros/build/catkin_generated/installspace/local_setup.sh"
=======
file(INSTALL DESTINATION "/home/rabios/dev/scara_ros/install" TYPE FILE FILES
    "/home/rabios/dev/scara_ros/build/catkin_generated/installspace/setup.sh"
    "/home/rabios/dev/scara_ros/build/catkin_generated/installspace/local_setup.sh"
>>>>>>> a939d1951b49873009e624302dcc1bd2adec4208
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/diego/scara_ros/install/setup.zsh;/home/diego/scara_ros/install/local_setup.zsh")
=======
   "/home/rabios/dev/scara_ros/install/setup.zsh;/home/rabios/dev/scara_ros/install/local_setup.zsh")
>>>>>>> a939d1951b49873009e624302dcc1bd2adec4208
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/diego/scara_ros/install" TYPE FILE FILES
    "/home/diego/scara_ros/build/catkin_generated/installspace/setup.zsh"
    "/home/diego/scara_ros/build/catkin_generated/installspace/local_setup.zsh"
=======
file(INSTALL DESTINATION "/home/rabios/dev/scara_ros/install" TYPE FILE FILES
    "/home/rabios/dev/scara_ros/build/catkin_generated/installspace/setup.zsh"
    "/home/rabios/dev/scara_ros/build/catkin_generated/installspace/local_setup.zsh"
>>>>>>> a939d1951b49873009e624302dcc1bd2adec4208
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/diego/scara_ros/install/.rosinstall")
=======
   "/home/rabios/dev/scara_ros/install/.rosinstall")
>>>>>>> a939d1951b49873009e624302dcc1bd2adec4208
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/diego/scara_ros/install" TYPE FILE FILES "/home/diego/scara_ros/build/catkin_generated/installspace/.rosinstall")
=======
file(INSTALL DESTINATION "/home/rabios/dev/scara_ros/install" TYPE FILE FILES "/home/rabios/dev/scara_ros/build/catkin_generated/installspace/.rosinstall")
>>>>>>> a939d1951b49873009e624302dcc1bd2adec4208
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
<<<<<<< HEAD
  include("/home/diego/scara_ros/build/gtest/cmake_install.cmake")
  include("/home/diego/scara_ros/build/moveit_robot_urdf_sim/cmake_install.cmake")
  include("/home/diego/scara_ros/build/scara_ros/cmake_install.cmake")
  include("/home/diego/scara_ros/build/robot_urdf/cmake_install.cmake")
=======
  include("/home/rabios/dev/scara_ros/build/gtest/cmake_install.cmake")
  include("/home/rabios/dev/scara_ros/build/scara_ros/cmake_install.cmake")
>>>>>>> a939d1951b49873009e624302dcc1bd2adec4208

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
<<<<<<< HEAD
file(WRITE "/home/diego/scara_ros/build/${CMAKE_INSTALL_MANIFEST}"
=======
file(WRITE "/home/rabios/dev/scara_ros/build/${CMAKE_INSTALL_MANIFEST}"
>>>>>>> a939d1951b49873009e624302dcc1bd2adec4208
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
