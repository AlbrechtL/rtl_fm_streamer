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

# JSON-RPC lib
add_subdirectory(jsonrpc-c)

# convenience lib
add_library(convenience_static STATIC
    convenience/convenience.c
)

# libgetopt lib
if(WIN32)
add_library(libgetopt_static STATIC
    getopt/getopt.c
)
endif()

########################################################################
# Build utility
########################################################################
add_executable(rtl_fm_streamer rtl_fm_streamer.c)

set(INSTALL_TARGETS rtl_fm_streamer)

target_link_libraries(rtl_fm_streamer convenience_static
    ${RTLSDR_LIBRARIES}
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

if(RTLSDR_LDFLAGS_OTHER)
set_target_properties(rtl_fm_streamer PROPERTIES LINK_FLAGS "${RTLSDR_LDFLAGS_OTHER}")
endif()

if(UNIX)
target_link_libraries(rtl_fm_streamer m jsonrpc-c)
endif()

if(WIN32)
target_link_libraries(rtl_fm_streamer libgetopt_static jsonrpc-c)
endif()
########################################################################
# Install built library files & utilities
########################################################################
install(TARGETS ${INSTALL_TARGETS}
    RUNTIME DESTINATION bin              # .dll file
)
