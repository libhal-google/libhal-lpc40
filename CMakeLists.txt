cmake_minimum_required(VERSION 3.1...3.21)

project(liblpc40xx VERSION 0.0.1 LANGUAGES CXX)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)
set(CMAKE_FIND_USE_PACKAGE_REGISTRY true)

find_package(libembeddedhal)
find_package(libxbitset)
find_package(libarmcortex)

add_library(liblpc40xx INTERFACE)

target_include_directories(liblpc40xx INTERFACE
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
$<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>)

target_compile_features(liblpc40xx INTERFACE cxx_std_20)
set_target_properties(liblpc40xx PROPERTIES CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_PACKAGE_REGISTRY ON)
export(TARGETS liblpc40xx
NAMESPACE liblpc40xx::
FILE "${CMAKE_CURRENT_BINARY_DIR}/liblpc40xx-config.cmake")
export(PACKAGE liblpc40xx)

add_subdirectory(demos)
