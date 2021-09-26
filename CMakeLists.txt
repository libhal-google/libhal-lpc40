cmake_minimum_required(VERSION 3.1...3.21)

project(liblpc40xx VERSION 0.0.1 LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME}
                           INTERFACE
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                           $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_20)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_EXTENSIONS OFF)

install(TARGETS ${PROJECT_NAME} DESTINATION "."
        PUBLIC_HEADER DESTINATION include
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

find_package(libembeddedhal CONFIG REQUIRED)
find_package(libxbitset CONFIG REQUIRED)
find_package(libarmcortex CONFIG REQUIRED)
find_package(ring-span-lite CONFIG REQUIRED)

add_executable (tests
${PROJECT_NAME}/adc.test.cpp
${PROJECT_NAME}/i2c.test.cpp
${PROJECT_NAME}/input_pin.test.cpp
${PROJECT_NAME}/interrupt_pin.test.cpp
${PROJECT_NAME}/output_pin.test.cpp
${PROJECT_NAME}/uart.test.cpp
${PROJECT_NAME}/main.test.cpp)

target_compile_features(tests INTERFACE cxx_std_20)
set_target_properties(tests PROPERTIES CXX_EXTENSIONS OFF)
target_compile_definitions(tests PRIVATE -DPLATFORM=lpc40xx)
target_link_libraries(tests PRIVATE
liblpc40xx
libembeddedhal::libembeddedhal
libxbitset::libxbitset
libarmcortex::libarmcortex
nonstd::ring-span-lite)
