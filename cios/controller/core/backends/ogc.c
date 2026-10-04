#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "ipc.h"
#include "ogc_arm.h"
#include "platform.h"
#include "syscalls.h"
#include "usb.h"
#include "utils.h"

/* Maximum number of connected USB controllers. Increase this if needed. */
#define MAX_ACTIVE_DEVICES 2

/* USBv5 USB_HID IOCTL commands */
#define USBV5_IOCTL_GETVERSION      0
#define USBV5_IOCTL_GETDEVICECHANGE 1
#define USBV5_IOCTL_SHUTDOWN        2
#define USBV5_IOCTL_GETDEVPARAMS    3
#define USBV5_IOCTL_ATTACH          4
#define USBV5_IOCTL_RELEASE         5
#define USBV5_IOCTL_ATTACHFINISH    6
#define USBV5_IOCTL_SETALTERNATE    7
#define USBV5_IOCTL_SUSPEND_RESUME  16
#define USBV5_IOCTL_CANCELENDPOINT  17
#define USBV5_IOCTL_CTRLMSG         18
#define USBV5_IOCTL_INTRMSG         19
#define USBV5_IOCTL_ISOMSG          20 /* Not available in USB_HID */
#define USBV5_IOCTL_BULKMSG         21 /* Not available in USB_HID */

/* Dedicated USB Transfer Slots:
 * Isolating transfer types to dedicated slots guarantees:
 * 1. Zero slot contention: input polling, speaker audio, and rumble never compete.
 * 2. Deterministic execution: no slot leaks or pool exhaustion.
 * 3. Hardware safety: Starlet never receives concurrent duplicate transfers on the same endpoint.
 */
#define XFER_SLOT_INTR_IN    0  /* Dedicated for Input Polling (EP1 IN) */
#define XFER_SLOT_INTR_OUT   1  /* Dedicated for Speaker Audio (EP2 OUT) */
#define XFER_SLOT_CTRL_P1    2  /* Dedicated for Player 1 LEDs/Rumble (EP0) */
#define XFER_SLOT_CTRL_P2    3  /* Dedicated for Player 2 LEDs/Rumble (EP0) */
#define USB_MAX_DEVICES      32
#define MAX_ACTIVE_TRANSFERS 4

/* USBv5 HID message structure */
struct usb_hid_v5_transfer {
    u32 dev_id;
    u32 zero;

    union {
        struct {
            u8 bmRequestType;
            u8 bmRequest;
            u16 wValue;
            u16 wIndex;
            u16 wLength;
            void *rpData;
        } ctrl;

        struct {
            u32 out;
        } intr;

        u32 data[14];
    };
} ATTRIBUTE_PACKED;
static_assert(sizeof(struct usb_hid_v5_transfer) == 64);

typedef struct {
    void *priv;
    /* VID and PID */
    /* TODO: remove, read them from "desc" */
    u16 vid;
    u16 pid;
    /* Used to communicate with Wii's USB module */
    int host_fd;
    u32 dev_id;
    egc_usb_devdesc_t desc;
} egc_usb_device_t;

typedef struct {
    /* This must be the first member, since we use it for casting */
    egc_input_device_t pub;
    union {
        egc_usb_device_t usb;
        // TODO: add bluetooth data here
    };
    /* Timer ID (-1 if unset), its address used as timer cookie */
    int timer_id;
    egc_timer_cb timer_callback;
} ogc_device_t;

typedef struct egc_usb_device_entry_t {
    s32 device_id;
    u16 vid;
    u16 pid;
    u32 token;
} egc_usb_device_entry_t;

typedef struct {
    u8 buffer[128] ATTRIBUTE_ALIGN(32);
    struct usb_hid_v5_transfer transfer ATTRIBUTE_ALIGN(32);
    ioctlv vectors[2] ATTRIBUTE_ALIGN(32);
    areply reply ATTRIBUTE_ALIGN(32);
    struct egc_usb_transfer_t t;
    egc_transfer_cb callback;
    u32 tick; /* submission tick, used to reclaim stale transfers */
} ogc_transfer_t;

static egc_device_description_t s_device_descriptions[MAX_ACTIVE_DEVICES];

/* Maximum number of events in the queue. If the handle_events() function is
 * not called often enough, the queue might fill and events will be lost. */
#define OGC_MAX_EVENTS 32

typedef struct ogc_event_queue_t {
    struct ogc_event_t {
        egc_event_e type;
        u8 device_index;
    } ATTRIBUTE_PACKED events[OGC_MAX_EVENTS];
    u8 current_index;
} ATTRIBUTE_PACKED ogc_event_queue_t;
static ogc_event_queue_t s_ogc_event_queue;

static ogc_device_t s_devices[MAX_ACTIVE_DEVICES] ATTRIBUTE_ALIGN(32);
static egc_usb_device_entry_t device_change_devices[USB_MAX_DEVICES] ATTRIBUTE_ALIGN(32);
static ogc_transfer_t s_transfers[MAX_ACTIVE_TRANSFERS] ATTRIBUTE_ALIGN(32);
static u32 s_transfer_tick = 0;
static int host_fd = -1;
static u8 worker_thread_stack[2048] ATTRIBUTE_ALIGN(32);
static u32 queue_data[64] ATTRIBUTE_ALIGN(32);
static int queue_id = -1;
static egc_event_cb s_event_handler;

/* Async notification messages */
static areply notification_messages[2] = { 0 };
#define MESSAGE_DEVCHANGE    &notification_messages[0]
#define MESSAGE_ATTACHFINISH &notification_messages[1]

static inline u32 enter_critical_section(void)
{
#ifdef __arm__
    /* There are no thread synchronization instructions on ARMv5, so we just
     * disable interrupts, which will prevent (among other things) thread
     * scheduling.
     * Code inspired by libn3ds and fastboot3DS */
    u32 cpsr = get_cpsr();
    set_cpsr_c(cpsr | PSR_I);
    return cpsr;
#else
#error "enter_critical_section not implemented"
#endif
}

static inline void leave_critical_section(u32 level)
{
#ifdef __arm__
    set_cpsr_c(level);
#else
#error "leave_critical_section not implemented"
#endif
}

static inline ogc_device_t *ogc_device_from_input_device(egc_input_device_t *input_device)
{
    return (ogc_device_t *)input_device;
}

static inline ogc_device_t *get_usb_device_for_dev_id(u32 dev_id)
{
    for (int i = 0; i < ARRAY_SIZE(s_devices); i++) {
        if (s_devices[i].pub.connection == EGC_CONNECTION_USB && s_devices[i].usb.dev_id == dev_id)
            return &s_devices[i];
    }

    return NULL;
}

static inline ogc_device_t *get_free_device_slot(void)
{
    for (int i = 0; i < ARRAY_SIZE(s_devices); i++) {
        if (s_devices[i].pub.connection == EGC_CONNECTION_DISCONNECTED)
            return &s_devices[i];
    }

    return NULL;
}

static int ogc_cancel_endpoint(egc_input_device_t *input_device, u8 endpoint)
{
    ogc_device_t *device = ogc_device_from_input_device(input_device);
    if (!device || host_fd < 0 || device->usb.dev_id == 0)
        return -1;

    u32 inbuf[8] ATTRIBUTE_ALIGN(32);
    memset(inbuf, 0, sizeof(inbuf));
    inbuf[0] = device->usb.dev_id;
    inbuf[1] = endpoint;

    return os_ioctl(host_fd, USBV5_IOCTL_CANCELENDPOINT, inbuf, sizeof(inbuf), NULL, 0);
}

static inline bool is_usb_device_connected(u32 dev_id)
{
    return get_usb_device_for_dev_id(dev_id) != NULL;
}

static int usb_hid_v5_get_descriptors(int host_fd, u32 dev_id, egc_usb_devdesc_t *udd)
{
    u32 inbuf[8] ATTRIBUTE_ALIGN(32);
    u8 outbuf[96] ATTRIBUTE_ALIGN(32);

    /* Setup buffer */
    memset(inbuf, 0, sizeof(inbuf));
    inbuf[0] = dev_id;
    inbuf[2] = 0;

    /* Get device parameters */
    int rc =
        os_ioctl(host_fd, USBV5_IOCTL_GETDEVPARAMS, inbuf, sizeof(inbuf), outbuf, sizeof(outbuf));
    if (udd)
        memcpy(udd, outbuf, sizeof(*udd));
    return rc;
}

static inline void build_ctrl_transfer(struct usb_hid_v5_transfer *transfer, ogc_transfer_t *t,
                                       u8 bmRequest, u16 wValue, u16 wIndex)
{
    ogc_device_t *device = ogc_device_from_input_device(t->t.device);
    memset(transfer, 0, sizeof(*transfer));
    transfer->dev_id = device->usb.dev_id;
    transfer->ctrl.bmRequestType = t->t.endpoint;
    transfer->ctrl.bmRequest = bmRequest;
    transfer->ctrl.wValue = wValue;
    transfer->ctrl.wIndex = wIndex;
    transfer->ctrl.wLength = t->t.length;
    transfer->ctrl.rpData = t->t.data;
}

static inline int usb_hid_v5_ctrl_transfer_async(ogc_transfer_t *t, u8 bmRequest, u16 wValue,
                                                 u16 wIndex, int queue_id)
{
    ogc_device_t *device = ogc_device_from_input_device(t->t.device);
    int out = !(t->t.endpoint & EGC_USB_ENDPOINT_IN);

    build_ctrl_transfer(&t->transfer, t, bmRequest, wValue, wIndex);

    t->vectors[0].data = &t->transfer;
    t->vectors[0].len = sizeof(t->transfer);
    t->vectors[1].data = t->t.data;
    t->vectors[1].len = t->t.length;

    return os_ioctlv_async(device->usb.host_fd, USBV5_IOCTL_CTRLMSG, 1 + out, 1 - out, t->vectors,
                           queue_id, &t->reply);
}

static inline int usb_hid_v5_intr_transfer_async(ogc_transfer_t *t, int queue_id)
{
    ogc_device_t *device = ogc_device_from_input_device(t->t.device);
    int out = !(t->t.endpoint & EGC_USB_ENDPOINT_IN);

    memset(&t->transfer, 0, sizeof(t->transfer));
    t->transfer.dev_id = device->usb.dev_id;
    t->transfer.intr.out = out;

    t->vectors[0].data = &t->transfer;
    t->vectors[0].len = sizeof(t->transfer);
    t->vectors[1].data = t->t.data;
    t->vectors[1].len = t->t.length;

    return os_ioctlv_async(device->usb.host_fd, USBV5_IOCTL_INTRMSG, 1 + out, 1 - out, t->vectors,
                           queue_id, &t->reply);
}

static int usb_hid_v5_attach(int host_fd, u32 dev_id)
{
    u32 buf[8] ATTRIBUTE_ALIGN(32);

    memset(buf, 0, sizeof(buf));
    buf[0] = dev_id;

    return os_ioctl(host_fd, USBV5_IOCTL_ATTACH, buf, sizeof(buf), NULL, 0);
}

static int usb_hid_v5_release(int host_fd, u32 dev_id)
{
    u32 buf[8] ATTRIBUTE_ALIGN(32);

    memset(buf, 0, sizeof(buf));
    buf[0] = dev_id;

    return os_ioctl(host_fd, USBV5_IOCTL_RELEASE, buf, sizeof(buf), NULL, 0);
}

static int usb_hid_v5_suspend_resume(int host_fd, int dev_id, int resumed)
{
    u32 buf[8] ATTRIBUTE_ALIGN(32);

    memset(buf, 0, sizeof(buf));
    buf[0] = dev_id;
    buf[2] = resumed;

    return os_ioctl(host_fd, USBV5_IOCTL_SUSPEND_RESUME, buf, sizeof(buf), NULL, 0);
}

static const egc_usb_transfer_t *ogc_ctrl_transfer_async(egc_input_device_t *device, u8 requesttype,
                                                         u8 request, u16 value, u16 index,
                                                         void *data, u16 length,
                                                         egc_transfer_cb callback)
{
    u32 cs = enter_critical_section();
    ogc_device_t *ogc_dev = ogc_device_from_input_device(device);
    int p_idx = (ogc_dev == &s_devices[1]) ? 1 : 0;
    int slot = (p_idx == 1) ? XFER_SLOT_CTRL_P2 : XFER_SLOT_CTRL_P1;
    ogc_transfer_t *t = &s_transfers[slot];
    if (t->t.device != NULL) {
        leave_critical_section(cs);
        return NULL;
    }
    t->t.device = device;
    t->tick = s_transfer_tick;
    leave_critical_section(cs);

    if (length > sizeof(t->buffer)) {
        length = sizeof(t->buffer);
    }
    if (length > 0) {
        memcpy(t->buffer, data, length);
    } else if (requesttype & EGC_USB_ENDPOINT_IN) {
        length = sizeof(t->buffer);
    }
    t->t.transfer_type = EGC_USB_TRANSFER_CONTROL;
    t->t.endpoint = requesttype;
    t->t.length = length;
    t->t.data = t->buffer;
    t->callback = callback;
    int rc = usb_hid_v5_ctrl_transfer_async(t, request, value, index, queue_id);
    if (rc < 0) {
        cs = enter_critical_section();
        t->t.device = NULL; /* Mark as unused */
        leave_critical_section(cs);
        return NULL;
    }
    return &t->t;
}

static const egc_usb_transfer_t *ogc_intr_transfer_async(egc_input_device_t *device, u8 endpoint,
                                                         void *data, u16 length,
                                                         egc_transfer_cb callback)
{
    u32 cs = enter_critical_section();
    int slot = (endpoint & EGC_USB_ENDPOINT_IN) ? XFER_SLOT_INTR_IN : XFER_SLOT_INTR_OUT;
    ogc_transfer_t *t = &s_transfers[slot];
    if (t->t.device != NULL) {
        leave_critical_section(cs);
        return NULL;
    }
    t->t.device = device;
    t->tick = s_transfer_tick;
    leave_critical_section(cs);

    if (length > sizeof(t->buffer)) {
        length = sizeof(t->buffer);
    }
    if (length > 0) {
        memcpy(t->buffer, data, length);
    } else if (endpoint & EGC_USB_ENDPOINT_IN) {
        length = sizeof(t->buffer);
    }
    t->t.transfer_type = EGC_USB_TRANSFER_INTERRUPT;
    t->t.endpoint = endpoint;
    t->t.length = length;
    t->t.data = t->buffer;
    t->callback = callback;
    int rc = usb_hid_v5_intr_transfer_async(t, queue_id);
    if (rc < 0) {
        cs = enter_critical_section();
        t->t.device = NULL; /* Mark as unused */
        leave_critical_section(cs);
        return NULL;
    }
    return &t->t;
}

static int ogc_set_timer(egc_input_device_t *input_device, int time_us, int repeat_time_us,
                         egc_timer_cb callback)
{
    ogc_device_t *device = (ogc_device_t *)input_device;
    int rc;

    if (device->timer_id > 0) {
        os_stop_timer(device->timer_id);
        rc = os_restart_timer(device->timer_id, time_us, repeat_time_us);
    } else {
        device->timer_id =
            os_create_timer(time_us, repeat_time_us, queue_id, (s32)&device->timer_id);
        rc = device->timer_id;
    }
    device->timer_callback = callback;
    return rc;
}

static int ogc_report_input(egc_input_device_t *device, const egc_input_state_t *state)
{
    u32 cs = enter_critical_section();
    memcpy(&device->state, state, sizeof(*state));
    leave_critical_section(cs);
    return 0;
}

static bool queue_event(egc_event_e type, ogc_device_t *device)
{
    u32 cs = enter_critical_section();
    if (s_ogc_event_queue.current_index >= OGC_MAX_EVENTS) {
        leave_critical_section(cs);
        LOG_INFO("OGC event queue is full!");
        return false;
    }

    u8 device_index = device - s_devices;

    struct ogc_event_t *e = &s_ogc_event_queue.events[s_ogc_event_queue.current_index++];
    e->type = type;
    e->device_index = device_index;
    leave_critical_section(cs);
    return true;
}

static void ogc_device_free(ogc_device_t *device)
{
    device->pub.connection = EGC_CONNECTION_DISCONNECTED;
    /* Clear callbacks so removed device won't be notified, but leave t.device
     * in place until usb_hid_worker receives Starlet's completion reply to
     * avoid slot aliasing while IOCTLs are still in-flight in kernel. */
    u32 cs = enter_critical_section();
    for (int i = 0; i < ARRAY_SIZE(s_transfers); i++) {
        if (s_transfers[i].t.device == &device->pub) {
            s_transfers[i].callback = NULL;
        }
    }
    leave_critical_section(cs);
    /* If the desc structure was allocated by us, free it if not shared */
    for (int i = 0; i < ARRAY_SIZE(s_device_descriptions); i++) {
        if (device->pub.desc == &s_device_descriptions[i]) {
            bool shared = false;
            for (int d = 0; d < ARRAY_SIZE(s_devices); d++) {
                if (&s_devices[d] != device && s_devices[d].pub.connection != EGC_CONNECTION_DISCONNECTED &&
                    s_devices[d].pub.desc == &s_device_descriptions[i]) {
                    shared = true;
                    break;
                }
            }
            if (!shared)
                memset(&s_device_descriptions[i], 0, sizeof(egc_device_description_t));
            break;
        }
    }
    device->pub.desc = NULL;
}

static void handle_device_change_reply(int host_fd, areply *reply)
{
    ogc_device_t *device;
    u16 vid, pid;
    u32 dev_id;
    int ret;
    bool found;

    LOG_DEBUG("Device change, #Attached devices: %" PRId32 "\n", reply->result);

    if (reply->result < 0) {
        ret = os_ioctl_async(host_fd, USBV5_IOCTL_GETDEVICECHANGE, NULL, 0, device_change_devices,
                             sizeof(device_change_devices), queue_id, MESSAGE_DEVCHANGE);
        return;
    }

    /* First look for disconnections */
    for (int i = 0; i < ARRAY_SIZE(s_devices); i++) {
        device = &s_devices[i];
        if (device->pub.connection == EGC_CONNECTION_DISCONNECTED)
            continue;

        found = false;
        for (int j = 0; j < reply->result; j++) {
            if (device->pub.connection == EGC_CONNECTION_USB &&
                device->usb.dev_id == device_change_devices[j].device_id) {
                found = true;
                break;
            }
        }

        /* Oops, it got disconnected */
        if (!found) {
            LOG_DEBUG("Device with VID: 0x%04" PRIx16 ", PID: 0x%04" PRIx16 ", dev_id: 0x%" PRIx32
                      " got disconnected\n",
                      device->usb.vid, device->usb.pid, device->usb.dev_id);

            if (!queue_event(EGC_EVENT_DEVICE_REMOVED, device)) {
                /* We normally mark the device as not valid only after the
                 * client has retrieved the event, but since we couldn't add
                 * the event into the queue, let's mark the device as available
                 * here. */
                ogc_device_free(device);
            }
        }
    }

    /* Now look for new connections */
    for (int i = 0; i < reply->result; i++) {
        vid = device_change_devices[i].vid;
        pid = device_change_devices[i].pid;
        dev_id = device_change_devices[i].device_id;
        LOG_DEBUG("[%d] VID: 0x%04" PRIx16 ", PID: 0x%04" PRIx16 ", dev_id: 0x%" PRIx32 "\n", i,
                  vid, pid, dev_id);

        /* Check if we already have that device (same dev_id) connected */
        if (is_usb_device_connected(dev_id))
            continue;

        /* ONLY attach to supported DualShock 4 controllers (Sony VID 0x054c, PID 0x05c4, 0x09cc, 0x0ba0).
         * CRITICAL: NEVER attach, suspend, or query descriptors on USB storage drives / Mass Storage devices!
         * Touching USB drives interferes with USB Loader GX and causes fatal I/O freezes! */
        if (vid != 0x054c || (pid != 0x05c4 && pid != 0x09cc && pid != 0x0ba0))
            continue;

        /* Get an empty device slot */
        device = get_free_device_slot();
        if (!device)
            break;

        /* Now we can attach it to take ownership */
        ret = usb_hid_v5_attach(host_fd, dev_id);
        if (ret != IOS_OK)
            continue;

        /* We must resume the USB device before interacting with it */
        ret = usb_hid_v5_suspend_resume(host_fd, dev_id, 1);
        if (ret != IOS_OK) {
            usb_hid_v5_release(host_fd, dev_id);
            continue;
        }

        /* We must read the USB device descriptor before interacting with the device */
        ret = usb_hid_v5_get_descriptors(host_fd, dev_id, &device->usb.desc);
        if (ret != IOS_OK) {
            usb_hid_v5_release(host_fd, dev_id);
            continue;
        }

        /* We have ownership, populate the device info */
        device->usb.vid = vid;
        device->usb.pid = pid;
        device->usb.host_fd = host_fd;
        device->usb.dev_id = dev_id;
        device->timer_id = -1;
        device->timer_callback = NULL;
        device->pub.connection = EGC_CONNECTION_USB;

        queue_event(EGC_EVENT_DEVICE_ADDED, device);

    }

    ret = os_ioctl_async(host_fd, USBV5_IOCTL_ATTACHFINISH, NULL, 0, NULL, 0, queue_id,
                         MESSAGE_ATTACHFINISH);
    LOG_DEBUG("ioctl(ATTACHFINISH): %d\n", ret);
}

void ogc_ds4_attach_player2(egc_input_device_t *p1_dev)
{
    if (!p1_dev) return;
    ogc_device_t *primary = ogc_device_from_input_device(p1_dev);
    ogc_device_t *sec_device = get_free_device_slot();
    if (!sec_device || !primary) return;

    sec_device->usb.desc = primary->usb.desc;
    sec_device->pub.desc = primary->pub.desc;
    sec_device->usb.vid = primary->usb.vid;
    sec_device->usb.pid = primary->usb.pid;
    sec_device->usb.host_fd = primary->usb.host_fd;
    sec_device->usb.dev_id = primary->usb.dev_id;
    sec_device->timer_id = -1;
    sec_device->timer_callback = NULL;
    sec_device->pub.connection = EGC_CONNECTION_USB;
    queue_event(EGC_EVENT_DEVICE_ADDED, sec_device);
}

void ogc_ds4_detach_player2(egc_input_device_t *p2_dev)
{
    if (!p2_dev) return;
    ogc_device_t *sec_device = ogc_device_from_input_device(p2_dev);
    if (sec_device && sec_device->pub.connection == EGC_CONNECTION_USB) {
        queue_event(EGC_EVENT_DEVICE_REMOVED, sec_device);
    }
}

static int usb_hid_worker(void *arg)
{
    u32 ver[8] ATTRIBUTE_ALIGN(32);
    ogc_device_t *device;
    int ret;

    LOG_DEBUG("usb_hid_worker thread started\n");

    for (int i = 0; i < ARRAY_SIZE(s_devices); i++)
        s_devices[i].pub.connection = EGC_CONNECTION_DISCONNECTED;

    /* USB_HID supports 16 handles, libogc uses handle 0, so we use handle 15...*/
    ret = os_open("/dev/usb/hid", 15);
    if (ret < 0)
        return ret;
    host_fd = ret;

    ret = os_ioctl(host_fd, USBV5_IOCTL_GETVERSION, NULL, 0, ver, sizeof(ver));
    if (ret < 0)
        return ret;

    /* We only support USBv5 for now */
    if (ver[0] != 0x50001)
        return IOS_EINVAL;

    ret = os_ioctl_async(host_fd, USBV5_IOCTL_GETDEVICECHANGE, NULL, 0, device_change_devices,
                         sizeof(device_change_devices), queue_id, MESSAGE_DEVCHANGE);

    while (1) {
        void *message;

        /* Wait for a message from USB devices */
        ret = os_message_queue_receive(queue_id, &message, IOS_MESSAGE_BLOCK);
        if (ret != IOS_OK)
            continue;

        if (message == MESSAGE_DEVCHANGE) {
            handle_device_change_reply(host_fd, message);
        } else if (message == MESSAGE_ATTACHFINISH) {
            ret =
                os_ioctl_async(host_fd, USBV5_IOCTL_GETDEVICECHANGE, NULL, 0, device_change_devices,
                               sizeof(device_change_devices), queue_id, MESSAGE_DEVCHANGE);
        } else if ((areply *)message >= &s_transfers[0].reply &&
                   (areply *)message <= &s_transfers[ARRAY_SIZE(s_transfers) - 1].reply) {
            /* It's a transfer reply */
            for (int i = 0; i < ARRAY_SIZE(s_transfers); i++) {
                ogc_transfer_t *transfer = &s_transfers[i];
                if ((areply *)message == &transfer->reply) {
                    egc_transfer_cb cb = transfer->callback;
                    struct egc_usb_transfer_t t = transfer->t;
                    s32 result = transfer->reply.result;

                    u32 cs = enter_critical_section();
                    transfer->t.status = EGC_USB_TRANSFER_STATUS_UNSET;
                    transfer->t.device = NULL;
                    transfer->callback = NULL;
                    leave_critical_section(cs);

                    if (cb) {
                        if (result >= 0) {
                            t.length = result;
                            t.status = EGC_USB_TRANSFER_STATUS_COMPLETED;
                        } else {
                            t.length = 0;
                            t.status = EGC_USB_TRANSFER_STATUS_ERROR;
                        }
                        cb(&t);
                    }
                    break;
                }
            }
        } else {
            /* Find if this is a timer */
            for (int i = 0; i < ARRAY_SIZE(s_devices); i++) {
                device = &s_devices[i];
                if (device->pub.connection == EGC_CONNECTION_DISCONNECTED)
                    continue;
                if (message == &device->timer_id) {
                    bool keep = device->timer_callback(&device->pub);
                    if (!keep) {
                        os_destroy_timer(device->timer_id);
                        device->timer_id = -1;
                    }
                }
            }
        }
    }

    return 0;
}

static int ogc_init(egc_event_cb event_handler)
{
    int ret;

    s_event_handler = event_handler;

    ret = os_message_queue_create(queue_data, ARRAY_SIZE(queue_data));
    if (ret < 0)
        return ret;
    queue_id = ret;

    /* Worker USB HID thread that receives and dispatches async events */
    ret = os_thread_create(usb_hid_worker, NULL, &worker_thread_stack[sizeof(worker_thread_stack)],
                           sizeof(worker_thread_stack), 0, 0);
    if (ret < 0)
        return ret;
    os_thread_continue(ret);

    return 0;
}

static egc_device_description_t *ogc_alloc_desc(egc_input_device_t *device)
{
    for (int i = 0; i < ARRAY_SIZE(s_device_descriptions); i++) {
        if (s_device_descriptions[i].vendor_id == 0) {
            device->desc = &s_device_descriptions[i];
            return &s_device_descriptions[i];
        }
    }

    return NULL;
}

static const egc_usb_devdesc_t *ogc_get_device_descriptor(egc_input_device_t *device)
{
    ogc_device_t *dev = ogc_device_from_input_device(device);
    return &dev->usb.desc;
}

static int ogc_set_suspended(egc_input_device_t *input_device, bool suspended)
{
    ogc_device_t *device = ogc_device_from_input_device(input_device);
    return usb_hid_v5_suspend_resume(device->usb.host_fd, device->usb.dev_id, !suspended);
}

static int ogc_handle_events()
{
    ogc_event_queue_t queue;
    /* Create a copy of the queue to ensure it doesn't get modified while we
     * process it */
    u32 cs = enter_critical_section();
    memcpy(&queue, &s_ogc_event_queue, sizeof(queue));
    s_ogc_event_queue.current_index = 0; /* empty the queue */
    leave_critical_section(cs);

    for (int i = 0; i < queue.current_index; i++) {
        struct ogc_event_t *event = &queue.events[i];
        ogc_device_t *device = &s_devices[event->device_index];

        if (event->type == EGC_EVENT_DEVICE_ADDED) {
            int ret = s_event_handler(&device->pub, event->type, device->usb.vid, device->usb.pid);
            if (ret != 0) {
                usb_hid_v5_release(host_fd, device->usb.dev_id);
                device->pub.connection = EGC_CONNECTION_DISCONNECTED;
            }
        } else if (event->type == EGC_EVENT_DEVICE_REMOVED) {
            s_event_handler(&device->pub, event->type);
            /* Mark this device as not valid */
            ogc_device_free(device);
        }
    }
    return queue.current_index;
}

const egc_platform_backend_t _egc_platform_backend = {
    .usb = {
        .get_device_descriptor = ogc_get_device_descriptor,
        .set_suspended = ogc_set_suspended,
        .ctrl_transfer_async = ogc_ctrl_transfer_async,
        .intr_transfer_async = ogc_intr_transfer_async,
        .cancel_endpoint = ogc_cancel_endpoint,
    },
    .init = ogc_init,
    .alloc_desc = ogc_alloc_desc,
    .set_timer = ogc_set_timer,
    .report_input = ogc_report_input,
    .handle_events = ogc_handle_events,
};
