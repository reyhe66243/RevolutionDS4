// stubs_peripheral.c - Weak stubs for peripheral-only builds
//
// When building in peripheral mode (controller_the adapter app), btstack_host.c and
// bthid are not linked. Shared code (router.c, cdc_commands.c, the input report_mode.c)
// references symbols from those modules. These weak stubs satisfy the linker.

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// bthid_get_device (used by router.c and the input report_mode.c)
typedef struct { int dummy; } bthid_device_t;
__attribute__((weak)) bthid_device_t* bthid_get_device(uint8_t conn_index)
{
    (void)conn_index;
    return NULL;
}

// btstack_host functions (used by cdc_commands.c)
__attribute__((weak)) void btstack_host_delete_all_bonds(void) {}
__attribute__((weak)) bool btstack_host_is_initialized(void) { return false; }
