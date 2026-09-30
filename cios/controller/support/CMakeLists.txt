add_library(support STATIC
    direct_syscalls.s
    ios.c
    log.c
    mem.c
    str_utils.c
    syscalls.s
    swi_mload.c
    timer.c
    tools.s
    tiny-printf.c
)

target_include_directories(support PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
