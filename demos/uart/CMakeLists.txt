cmake_minimum_required(VERSION 3.15)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/../cmake/toolchain.cmake)

project(main VERSION 0.1.0 LANGUAGES CXX)

find_package(liblpc40xx REQUIRED CONFIG)

set(CORTEX_M4_FLAGS -mcpu=cortex-m4 -mthumb -mfloat-abi=soft
    -mfpu=fpv4-sp-d16 -mtpcs-frame -mtpcs-leaf-frame)
set(LINKER_SCRIPT_PATHS ${CMAKE_SOURCE_DIR}/../../linker)

add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC .)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(${PROJECT_NAME} PRIVATE ${CORTEX_M4_FLAGS})
target_link_options(${PROJECT_NAME} PRIVATE ${CORTEX_M4_FLAGS}
    -L${LINKER_SCRIPT_PATHS} -T${LINKER_SCRIPT_PATHS}/lpc4078.ld
    -Wl,-Map=${PROJECT_NAME}.map,--cref -Wl,--gc-sections
    -Wl,--print-memory-usage)
target_link_libraries(${PROJECT_NAME} PRIVATE liblpc40xx::liblpc40xx)
arm_post_build(${PROJECT_NAME})
