// Compatibility shim for building the shared USB device drivers against both
// tinyusb 0.20.0 (console platforms — see the 0.20.0 rollback, RP2040 host
// regression) and tinyusb master (WCH/CH32 build via wch/Makefile TINYUSB_ROOT).
//
// Master added an `is_isr` parameter to usbd_edpt_xfer(). All joypad call
// sites run in task context, so the shim pins it to false. The function-like
// macro shares the function's name; C macro blue-painting keeps the expansion
// from recursing.
#pragma once

#include "tusb_option.h"

#if (TUSB_VERSION_MAJOR > 0) || (TUSB_VERSION_MINOR > 20) || \
    ((TUSB_VERSION_MINOR == 20) && (TUSB_VERSION_REVISION >= 1))
#define usbd_edpt_xfer(_rh, _ep, _buf, _len) usbd_edpt_xfer((_rh), (_ep), (_buf), (_len), false)
#endif
