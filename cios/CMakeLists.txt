cmake_minimum_required(VERSION 3.13)

find_program(STRIPIOS stripios REQUIRED)

set(REVDS4_MAJOR 1)
set(REVDS4_MINOR 0)
set(REVDS4_PATCH 0)
set(REVDS4_HASH "revds4")

project(
    REVOLUTIONDS4
    VERSION ${REVDS4_MAJOR}.${REVDS4_MINOR}.${REVDS4_PATCH}
    LANGUAGES ASM C
)

set(ARCH -mcpu=arm926ej-s -mthumb -mthumb-interwork -mbig-endian)

set(CMAKE_C_STANDARD 11)

add_compile_options(
    ${ARCH}
    -O2
    -DNDEBUG
    -g3
    -ffreestanding
    -fomit-frame-pointer
    -ffunction-sections
    -Wall
    -fno-builtin
)

add_executable(REVOLUTIONDS4
    source/start.s
    source/main.c
    source/hci_state.c
    source/injmessage.c
    source/input_device.c
    source/button_map.c
    source/fake_wiimote.c
    source/fake_wiimote_mgr.c
    source/libc.c
    source/wiimote_crypto.c
    source/conf.c
)

target_include_directories(REVOLUTIONDS4 PRIVATE
    include
)

target_compile_definitions(REVOLUTIONDS4 PRIVATE
    REVDS4_MAJOR=${REVDS4_MAJOR}
    REVDS4_MINOR=${REVDS4_MINOR}
    REVDS4_PATCH=${REVDS4_PATCH}
    REVDS4_HASH=${REVDS4_HASH}
)

# The Starlet toolchain needs a big-endian libgcc. Point REVDS4_LIBGCC_BE, as a
# CMake cache variable or as an environment variable, at the directory that
# contains it (for example .../libgcc-be/<version>/thumb/be).
set(REVDS4_LIBGCC_BE "$ENV{REVDS4_LIBGCC_BE}" CACHE PATH
    "Directory containing the big-endian libgcc for the Starlet toolchain")
if(NOT REVDS4_LIBGCC_BE)
    message(FATAL_ERROR
        "REVDS4_LIBGCC_BE is not set. Point it at the directory containing the "
        "big-endian libgcc for arm-none-eabi.")
endif()

target_link_directories(REVOLUTIONDS4 PRIVATE
    ${REVDS4_LIBGCC_BE}
)

target_link_libraries(REVOLUTIONDS4 PRIVATE
    support
    gcc
    core
)

target_link_options(REVOLUTIONDS4 PRIVATE
    ${ARCH}
    -nostartfiles
    -nostdlib
    -Wl,-T,${CMAKE_CURRENT_SOURCE_DIR}/link.ld,-Map,${CMAKE_PROJECT_NAME}.map
    -Wl,--gc-sections
    -Wl,-static
)

add_custom_target(app ALL
    DEPENDS ${CMAKE_BINARY_DIR}/REVOLUTIONDS4.app
)

add_custom_command(
    DEPENDS ${CMAKE_PROJECT_NAME}
    OUTPUT ${CMAKE_BINARY_DIR}/REVOLUTIONDS4.app
    COMMAND ${STRIPIOS} ${CMAKE_PROJECT_NAME} ${CMAKE_BINARY_DIR}/REVOLUTIONDS4.app
)

add_subdirectory(controller)
