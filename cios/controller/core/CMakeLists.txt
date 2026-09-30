set(PLATFORM_SOURCES
    backends/ogc.c
    backends/ogc_arm.c
)

add_library(core STATIC
    ${PLATFORM_SOURCES}
    egc.c
    usb_drivers/sony_ds4.c
)

target_include_directories(core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_options(core PRIVATE
    ${EXTRA_LDFLAGS}
)

target_link_libraries(core PRIVATE
    ${EXTRA_LIBS}
    gcc
)
