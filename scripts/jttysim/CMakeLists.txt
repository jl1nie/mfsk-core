# Minimal build of WSJT-X's JTTY command-line tools (sjtty, rjtty) and the
# Fortran library they need, without Qt, Hamlib or PortAudio.
#
# scripts/build_jttysim.sh exports a tag of the WSJT-X tree, moves upstream's
# own CMakeLists.txt aside and puts this file in its place, so that
# include (CMake/Sources.cmake) and every source path resolve against the
# exported root. Nothing here is upstream code; it exists because upstream's
# top-level build insists on Qt5 (incl. WebSockets), Hamlib and PortAudio for
# a handful of Fortran command-line programs.
cmake_minimum_required (VERSION 3.16)
project (jttysim LANGUAGES C CXX Fortran)

find_package (PkgConfig REQUIRED)
pkg_check_modules (FFTW3F REQUIRED fftw3f)
pkg_check_modules (FFTW3 REQUIRED fftw3)
find_package (Boost REQUIRED)

# Upstream's own flags (CMake/CompilerFlags.cmake), minus warnings.
set (CMAKE_Fortran_FLAGS_RELEASE "-O2 -funroll-loops -fno-f2c -fno-second-underscore -w")
set (CMAKE_C_FLAGS_RELEASE "-O2 -w")
set (CMAKE_CXX_FLAGS_RELEASE "-O2 -w")
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release)
endif ()

# The C sources include the generated wsjtx_config.h; only the version
# fields matter to them.
set (PROJECT_VERSION_MAJOR 3)
set (PROJECT_VERSION_MINOR 2)
set (PROJECT_VERSION_PATCH 0)
set (PROJECT_VERSION_TWEAK 0)
set (BUILD_TYPE_REVISION "rc1")
configure_file (wsjtx_config.h.in wsjtx_config.h)
include_directories (${CMAKE_BINARY_DIR})

include (CMake/Sources.cmake)   # wsjt_FSRCS, wsjt_CSRCS, ...

# The Reed-Solomon (ka9q) and Q65/QRA C sources are irrelevant to JTTY and
# some need extra include paths; leave them out.
list (REMOVE_ITEM wsjt_CSRCS ${ka9q_CSRCS} lib/ftrsd/ftrsdap.c lib/wrapkarn.c)
add_library (wsjt_cxx STATIC ${wsjt_CSRCS} lib/crc10.cpp lib/crc13.cpp lib/crc14.cpp)
target_include_directories (wsjt_cxx PRIVATE ${Boost_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/lib)
set_source_files_properties (lib/crc10.cpp lib/crc13.cpp lib/crc14.cpp
  PROPERTIES COMPILE_FLAGS -fpermissive)

set (MODDIR ${CMAKE_BINARY_DIR}/fortran_modules)
file (MAKE_DIRECTORY ${MODDIR})
add_library (wsjt_fort STATIC ${wsjt_FSRCS})
set_target_properties (wsjt_fort PROPERTIES Fortran_MODULE_DIRECTORY ${MODDIR})
# The root goes on the include path because some sources say
# include '/lib/fftw3.f90': gfortran resolves an absolute INCLUDE against
# each -I directory as well.
target_include_directories (wsjt_fort PUBLIC ${MODDIR} ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/lib ${FFTW3F_INCLUDE_DIRS})
target_link_libraries (wsjt_fort PUBLIC wsjt_cxx ${FFTW3F_LIBRARIES} ${FFTW3_LIBRARIES} m stdc++)

foreach (t sjtty rjtty)
  add_executable (${t} lib/jtty/${t}.f90)
  target_link_libraries (${t} wsjt_fort)
endforeach ()

# Our own oracle driver (not upstream code): runs the TBCC decode ladder on
# generated correlations and prints inputs and outputs. See
# scripts/jttysim/jtty_ladder_oracle.f90.
if (JTTYSIM_DIR)
  add_executable (jtty_ladder_oracle ${JTTYSIM_DIR}/jtty_ladder_oracle.f90)
  target_link_libraries (jtty_ladder_oracle wsjt_fort)
endif ()
