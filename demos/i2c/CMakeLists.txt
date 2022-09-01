cmake_minimum_required(VERSION 3.15)

set(CMAKE_TOOLCHAIN_FILE
    ${CMAKE_SOURCE_DIR}/../cmake/toolchain-arm-none-eabi.cmake)

project(i2c VERSION 0.0.1 LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})

if(NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
    message(STATUS
        "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
    file(DOWNLOAD
        "https://raw.githubusercontent.com/conan-io/cmake-conan/v0.16.1/conan.cmake"
        "${CMAKE_BINARY_DIR}/conan.cmake"
        EXPECTED_HASH SHA256=396e16d0f5eabdc6a14afddbcfff62a54a7ee75c6da23f32f7a31bc85db23484
        TLS_VERIFY ON)
endif()

include(${CMAKE_BINARY_DIR}/conan.cmake)

conan_cmake_run(REQUIRES liblpc40xx/0.0.1
    BASIC_SETUP CMAKE_TARGETS
    BUILD missing)

set(EXECUTABLE ${PROJECT_NAME}.elf)

add_executable(${EXECUTABLE} ${PROJECT_NAME}.cpp)

target_include_directories(${EXECUTABLE} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>)
target_compile_features(${EXECUTABLE} PRIVATE cxx_std_20)
set_target_properties(${EXECUTABLE} PROPERTIES CXX_EXTENSIONS OFF)
target_compile_definitions(${EXECUTABLE} PRIVATE -DPLATFORM=lpc40xx)
target_compile_options(${EXECUTABLE} PRIVATE
    -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
    -mtpcs-frame -mtpcs-leaf-frame)

target_link_options(${EXECUTABLE} PRIVATE
    -L${CMAKE_SOURCE_DIR}/../../linker/
    -T${CMAKE_SOURCE_DIR}/../../linker/lpc4078.ld
    -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mtpcs-frame
    -mtpcs-leaf-frame -Wl,-Map=${PROJECT_NAME}.map,--cref -Wl,--gc-sections
    -Wl,--print-memory-usage)

target_link_libraries(${EXECUTABLE} PRIVATE CONAN_PKG::liblpc40xx)

# Print executable size
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_SIZE_UTIL} bin/${EXECUTABLE})

# Create hex (intel hex) file
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex bin/${EXECUTABLE}
    bin/${PROJECT_NAME}.hex)

# Create bin (binary) file
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary bin/${EXECUTABLE}
    bin/${PROJECT_NAME}.bin)

# Create disassembly file
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJDUMP} --disassemble --demangle
    bin/${EXECUTABLE} > bin/${PROJECT_NAME}.S)

# Create disassembly file with source information
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJDUMP} --all-headers --source
    --disassemble --demangle bin/${EXECUTABLE}
    > bin/${PROJECT_NAME}.lst)
