# Copyright 2012 OSMOCOM Project
#
# This file is part of rtl-sdr
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.


########################################################################
# Project setup
########################################################################
cmake_minimum_required(VERSION 2.6)
project(rtl_fm_streamer C)

#select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
   message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

if(NOT LIB_INSTALL_DIR)
   set(LIB_INSTALL_DIR lib)
endif()

# Set the version information here
set(VERSION_INFO_MAJOR_VERSION 0) # increment major on api compatibility changes
set(VERSION_INFO_MINOR_VERSION 5) # increment minor on feature-level changes
set(VERSION_INFO_PATCH_VERSION git) # increment patch for bug fixes and docs
include(Version) # setup version info

########################################################################
# Compiler specific setup
########################################################################
if(CMAKE_COMPILER_IS_GNUCC AND NOT WIN32)
    ADD_DEFINITIONS(-Wall)
    ADD_DEFINITIONS(-Wextra)
    ADD_DEFINITIONS(-Wno-unused-parameter)
    ADD_DEFINITIONS(-Wno-unused)
    ADD_DEFINITIONS(-Wsign-compare)
    ADD_DEFINITIONS(-Wdeclaration-after-statement)
    #http://gcc.gnu.org/wiki/Visibility
    add_definitions(-fvisibility=hidden)
endif()

########################################################################
# Find build dependencies
########################################################################
set(THREADS_USE_PTHREADS_WIN32 true)
find_package(Threads)

if(NOT THREADS_FOUND)
    message(FATAL_ERROR "pthreads(-win32) required to compile rtl-sdr")
endif()
########################################################################
# Setup the include and linker paths
########################################################################
include_directories(
    ${THREADS_PTHREADS_INCLUDE_DIR}
)

#link_directories(
#    ...
#)

# Set component parameters
#set(INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE INTERNAL "" FORCE)

########################################################################
# Create uninstall target
########################################################################
configure_file(
    ${CMAKE_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
@ONLY)

add_custom_target(uninstall
    ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
)

option(DETACH_KERNEL_DRIVER "Detach kernel driver if loaded" OFF)
if (DETACH_KERNEL_DRIVER)
    message (STATUS "Building with kernel driver detaching enabled")
    add_definitions(-DDETACH_KERNEL_DRIVER=1)
else (DETACH_KERNEL_DRIVER)
    message (STATUS "Building with kernel driver detaching disabled, use -DDETACH_KERNEL_DRIVER=ON to enable")
endif (DETACH_KERNEL_DRIVER)

########################################################################
# Add subdirectories
########################################################################
add_subdirectory(src)

########################################################################
# Print Summary
########################################################################
MESSAGE(STATUS "Building for version: ${VERSION} / ${LIBVER}")
MESSAGE(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")
