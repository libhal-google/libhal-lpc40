# Almost all CMake files should start with this
# You should always specify a range with the newest
# and oldest tested versions of CMake. This will ensure
# you pick up the best policies.
cmake_minimum_required(VERSION 3.1...3.21)

# This is your project statement. You should always list languages;
# Listing the version is nice here since it sets lots of useful variables
project(liblpc40xx VERSION 0.0.1 LANGUAGES CXX)

# include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
# conan_basic_setup(TARGETS)
set(CMAKE_FIND_USE_PACKAGE_REGISTRY true)
# If you set any CMAKE_ variables, that can go here.
# (But usually don't do this, except maybe for C++ standard)

if (CMAKE_GENERATOR STREQUAL "Ninja" AND
    ((CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.9) OR
     (CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS 3.5)))
    # Force colored warnings in Ninja's output, if the compiler has -fdiagnostics-color support.
    # Rationale in https://github.com/ninja-build/ninja/issues/814
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always")
endif()

# Find packages go here.
find_package(libembeddedhal)
find_package(libxbitset)
find_package(libarmcortex)

# You should usually split this into folders, but this is a simple example

# This is a "default" library, and will match the *** variable setting.
# Other common choices are STATIC, SHARED, and MODULE
# Including header files here helps IDEs but is not required.
# Output libname matches target name, with the usual extensions on your system
add_library(liblpc40xx INTERFACE)

target_include_directories(liblpc40xx INTERFACE
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                           $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>)

# Link each target with other targets or add options, etc.
target_compile_features(liblpc40xx INTERFACE cxx_std_20)
set_target_properties(liblpc40xx PROPERTIES CXX_EXTENSIONS OFF)

# Adding something we can run - Output name matches target name
add_executable(output_pin demos/output_pin.cpp)

# Make sure you link your targets with this command. It can also link libraries
# and even flags, so linking a target that does not exist will not give a
# configure-time error.
target_link_libraries(output_pin
                      PRIVATE
                      liblpc40xx
                      libembeddedhal::libembeddedhal
                      libxbitset::libxbitset
                      libarmcortex::libarmcortex)
