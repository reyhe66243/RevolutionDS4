cmake_minimum_required(VERSION 3.13)

# Target board must be set before pico_sdk_init
set(PICO_BOARD pico_w CACHE STRING "Target board")
set(PICO_PLATFORM rp2040 CACHE STRING "Pico platform")

# Initialize the RP2040 SDK. Its location is taken from the PICO_SDK_PATH
# environment variable, or from an in-tree lib/pico-sdk directory.
if(DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}" CACHE PATH "RP2040 SDK location")
else()
    set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/pico-sdk CACHE PATH "RP2040 SDK location")
endif()
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(REVOLUTIONDS4 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize SDK
pico_sdk_init()

# Read version if available
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../VERSION")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/../VERSION" FIRMWARE_VERSION)
    string(STRIP "${FIRMWARE_VERSION}" FIRMWARE_VERSION)
else()
    set(FIRMWARE_VERSION "1.0.0")
endif()

# Core sources
set(CORE_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/app_registry.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/router/router.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/services/leds/leds.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/services/leds/neopixel/ws2812.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/services/leds/player_leds_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/services/storage/storage.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/services/storage/flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/services/button/button.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/services/codes/codes.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/services/players/manager.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/services/players/feedback.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/services/profiles/profile.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/services/profiles/profile_indicator.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/services/profiles/runtime_profile.c
    ${CMAKE_CURRENT_SOURCE_DIR}/platform/rp2040/platform_rp2040.c
)

# USB Device sources (PS4 output mode + CDC)
set(USB_DEVICE_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/usb/usbd/usbd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/usb/usbd/modes/ps4_mode.c
)

# Bluetooth HID device drivers (DS4 only)
set(BTHID_DEVICE_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/bt/bthid/bthid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bt/bthid/bthid_registry.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bt/bthid/devices/vendors/sony/ds4_bt.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bt/bthid/devices/vendors/sony/ds4_audio.c
)

add_executable(REVOLUTIONDS4)

target_compile_definitions(REVOLUTIONDS4 PRIVATE
    CONFIG_BT2USB=1
    CONFIG_USB=1
    DISABLE_USB_HOST=1
    USBD_DEFAULT_MODE=USB_OUTPUT_MODE_PS4
    DEDICATED_DS4_DONGLE=1
    USE_BOOTSEL_BUTTON=1
    CYW43_LWIP=0
    PICO_CYW43_ARCH_POLL=1
    BTSTACK_USE_CYW43=1
    ENABLE_BTSTACK=1
    CONFIG_BT_HOST=1
    CONFIG_DS4_SPEAKER_AUDIO=1
    FIRMWARE_VERSION="${FIRMWARE_VERSION}"
    BOARD_NAME="${PICO_BOARD}"
)

target_sources(REVOLUTIONDS4 PUBLIC
    ${CORE_SOURCES}
    ${USB_DEVICE_SOURCES}
    ${BTHID_DEVICE_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/bt/btstack/btstack_host.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bt/btstack/bt_device_db.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bt/transport/bt_transport.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bt/transport/bt_transport_cyw43.c
    ${CMAKE_CURRENT_SOURCE_DIR}/apps/revolutionds4/app.c
)

target_include_directories(REVOLUTIONDS4 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/apps/revolutionds4
    ${CMAKE_CURRENT_SOURCE_DIR}/usb/usbd
    ${CMAKE_CURRENT_SOURCE_DIR}/bt
    ${CMAKE_CURRENT_SOURCE_DIR}/bt/btstack
    ${CMAKE_CURRENT_SOURCE_DIR}/bt/bthid
)

target_link_libraries(REVOLUTIONDS4 PRIVATE
    pico_stdlib
    pico_multicore
    pico_rand
    pico_cyw43_arch
    pico_async_context_poll
    pico_btstack_classic
    pico_btstack_cyw43
    pico_btstack_sbc_encoder
    tinyusb_device
    tinyusb_board
    hardware_pio
    hardware_dma
)

target_compile_options(REVOLUTIONDS4 PRIVATE -O3)
pico_add_extra_outputs(REVOLUTIONDS4)
pico_generate_pio_header(REVOLUTIONDS4 ${CMAKE_CURRENT_LIST_DIR}/core/services/leds/neopixel/ws2812.pio)
pico_enable_stdio_usb(REVOLUTIONDS4 0)
pico_enable_stdio_uart(REVOLUTIONDS4 1)
