# Install script for directory: E:/Proff/Clients/BharatiyaBeaglebone/SOEM

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/out/install/x64-Debug")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/out/build/x64-Debug/soem.lib")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/soem/cmake/soemConfig.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/soem/cmake/soemConfig.cmake"
         "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/out/build/x64-Debug/CMakeFiles/Export/share/soem/cmake/soemConfig.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/soem/cmake/soemConfig-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/soem/cmake/soemConfig.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/soem/cmake" TYPE FILE FILES "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/out/build/x64-Debug/CMakeFiles/Export/share/soem/cmake/soemConfig.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/soem/cmake" TYPE FILE FILES "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/out/build/x64-Debug/CMakeFiles/Export/share/soem/cmake/soemConfig-debug.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/soem" TYPE FILE FILES
    "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/soem/ethercat.h"
    "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/soem/ethercatbase.h"
    "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/soem/ethercatcoe.h"
    "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/soem/ethercatconfig.h"
    "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/soem/ethercatconfiglist.h"
    "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/soem/ethercatdc.h"
    "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/soem/ethercateoe.h"
    "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/soem/ethercatfoe.h"
    "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/soem/ethercatmain.h"
    "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/soem/ethercatprint.h"
    "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/soem/ethercatsoe.h"
    "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/soem/ethercattype.h"
    "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/osal/osal.h"
    "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/osal/win32/inttypes.h"
    "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/osal/win32/osal_defs.h"
    "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/osal/win32/osal_win32.h"
    "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/osal/win32/stdint.h"
    "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/oshw/win32/nicdrv.h"
    "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/oshw/win32/oshw.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("E:/Proff/Clients/BharatiyaBeaglebone/SOEM/out/build/x64-Debug/test/simple_ng/cmake_install.cmake")
  include("E:/Proff/Clients/BharatiyaBeaglebone/SOEM/out/build/x64-Debug/test/linux/slaveinfo/cmake_install.cmake")
  include("E:/Proff/Clients/BharatiyaBeaglebone/SOEM/out/build/x64-Debug/test/linux/eepromtool/cmake_install.cmake")
  include("E:/Proff/Clients/BharatiyaBeaglebone/SOEM/out/build/x64-Debug/test/linux/simple_test/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "E:/Proff/Clients/BharatiyaBeaglebone/SOEM/out/build/x64-Debug/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
