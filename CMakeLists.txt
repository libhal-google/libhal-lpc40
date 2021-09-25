cmake_minimum_required(VERSION 3.1...3.21)

project(liblpc40xx VERSION 0.0.1 LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})

find_package(libarmcortex CONFIG REQUIRED)
find_package(libxbitset CONFIG REQUIRED)
find_package(libembeddedhal CONFIG REQUIRED)

add_library(${PROJECT_NAME} STATIC
${PROJECT_NAME}/internal/constants.hpp
${PROJECT_NAME}/internal/gpio.hpp
${PROJECT_NAME}/internal/pin.hpp
${PROJECT_NAME}/internal/system_controller.hpp
${PROJECT_NAME}/internal/uart.hpp
${PROJECT_NAME}/adc.cpp
${PROJECT_NAME}/adc.hpp
${PROJECT_NAME}/i2c.cpp
${PROJECT_NAME}/i2c.hpp
${PROJECT_NAME}/input_pin.cpp
${PROJECT_NAME}/input_pin.hpp
${PROJECT_NAME}/interrupt_pin.cpp
${PROJECT_NAME}/interrupt_pin.hpp
${PROJECT_NAME}/output_pin.cpp
${PROJECT_NAME}/output_pin.hpp
${PROJECT_NAME}/uart.hpp)

target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
$<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_EXTENSIONS OFF)

target_link_libraries(${PROJECT_NAME}
                      PUBLIC
                      libembeddedhal::libembeddedhal
                      libxbitset::libxbitset
                      libarmcortex::libarmcortex)

install(TARGETS ${PROJECT_NAME} DESTINATION "."
        PUBLIC_HEADER DESTINATION include
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)
