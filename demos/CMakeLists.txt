cmake_minimum_required(VERSION 3.15)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/arm-gcc-toolchain.cmake)

project(demos VERSION 0.0.1 LANGUAGES CXX)

list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})

find_package(liblpc40xx REQUIRED CONFIG)

set(LINKER_SCRIPT_PATHS ${CMAKE_SOURCE_DIR}/)

set(DEMOS
    adc
    blinker
    can
    empty
    gpio
    i2c
    interrupt_pin
    uart)

set(TARGETS
    lpc4072
    lpc4074
    lpc4076
    lpc4078
    lpc4088)

foreach(target IN LISTS TARGETS)
    foreach(demo IN LISTS DEMOS)
        set(current_project ${target}_${demo})
        message(STATUS "Generating Demo for \"${current_project}\"")
        add_executable(${current_project} main.cpp newlib.cpp
            applications/${demo}.cpp)
        target_include_directories(${current_project} PUBLIC .)
        target_compile_features(${current_project} PRIVATE cxx_std_20)
        set_target_properties(${current_project} PROPERTIES CXX_EXTENSIONS OFF)
        target_compile_options(${current_project} PRIVATE ${CORTEX_M4F_FLAGS})
        target_link_options(${current_project} PRIVATE ${CORTEX_M4F_FLAGS}
            -L${LINKER_SCRIPT_PATHS} -T${LINKER_SCRIPT_PATHS}/${target}.ld
            -Wl,-Map=${current_project}.map,--cref -Wl,--gc-sections
            -Wl,--print-memory-usage)
        target_link_libraries(${current_project} PRIVATE liblpc40xx::liblpc40xx)
        arm_post_build(${current_project})
    endforeach()
endforeach()
