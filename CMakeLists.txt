# project
cmake_minimum_required (VERSION 2.6)
project (libviso2)

# directories
set (LIBVISO2_SRC_DIR src)

# include directory
include_directories("${LIBVISO2_SRC_DIR}")

# use sse3 instruction set
SET(CMAKE_CXX_FLAGS "-msse3")

# sources
FILE(GLOB MONO_SRC_FILES "src/*.cpp" 
                         "src/mono_demo/mono_demo.cpp")

# make release version
set(CMAKE_BUILD_TYPE Release)

# demo program
add_executable(mono_demo ${MONO_SRC_FILES})
target_link_libraries (mono_demo  png)

# sources
FILE(GLOB STEREO_SRC_FILES "src/*.cpp" 
                           "src/stereo_demo/stereo_demo.cpp")

# make release version
set(CMAKE_BUILD_TYPE Release)

# demo program
add_executable(stereo_demo ${STEREO_SRC_FILES})
target_link_libraries (stereo_demo  png)


