cmake_minimum_required(VERSION 3.13)

# Platform setup for the Starlet, the console's input/output processor.
if(CMAKE_SYSTEM_NAME MATCHES "Generic-dkP")
    set(CMAKE_SYSTEM_NAME "Starlet")
endif()

set(ARCH -mcpu=arm926ej-s -mthumb -mthumb-interwork -mbig-endian)
set(EXTRA_CFLAGS -fno-builtin)
set(EXTRA_LDFLAGS -nostdlib)
set(EXTRA_LIBS support)

set(CMAKE_C_STANDARD 11)

add_compile_options(
    ${ARCH}
    -O2
    -g3
    -ffreestanding
    -fomit-frame-pointer
    -ffunction-sections
    -Wall
    ${EXTRA_CFLAGS}
)

add_subdirectory(core)
add_subdirectory(support)
