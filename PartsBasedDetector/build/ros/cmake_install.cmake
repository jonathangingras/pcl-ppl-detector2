# Install script for directory: /home/jorobot/Desktop/PartsBasedDetector/ros

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}/home/jorobot/Desktop/PartsBasedDetector/bin/object_recognition_by_parts_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/jorobot/Desktop/PartsBasedDetector/bin/object_recognition_by_parts_node")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/jorobot/Desktop/PartsBasedDetector/bin/object_recognition_by_parts_node"
         RPATH "")
  ENDIF()
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/jorobot/Desktop/PartsBasedDetector/bin/object_recognition_by_parts_node")
FILE(INSTALL DESTINATION "/home/jorobot/Desktop/PartsBasedDetector/bin" TYPE EXECUTABLE FILES "/home/jorobot/Desktop/PartsBasedDetector/build/devel/lib/object_recognition_by_parts/object_recognition_by_parts_node")
  IF(EXISTS "$ENV{DESTDIR}/home/jorobot/Desktop/PartsBasedDetector/bin/object_recognition_by_parts_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/jorobot/Desktop/PartsBasedDetector/bin/object_recognition_by_parts_node")
    FILE(RPATH_REMOVE
         FILE "$ENV{DESTDIR}/home/jorobot/Desktop/PartsBasedDetector/bin/object_recognition_by_parts_node")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/jorobot/Desktop/PartsBasedDetector/bin/object_recognition_by_parts_node")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

