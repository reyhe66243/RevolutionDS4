#include "conf.h"
#include "hci.h"
#include "injmessage.h"
#include "l2cap.h"
#include "syscalls.h"
#include "utils.h"

/*
 * Repurpose conf_buffer (16 KB) as a zero-fragmentation fixed-block pool
 * for injected messages. After main() boots SYSCONF at startup, conf_buffer is
 * never accessed again.
 *
 * Using a fixed-block pool completely eliminates Starlet kernel heap
 * fragmentation (which previously exhausted memory after ~7-8 minutes of continuous
 * 200 Hz input and speaker packet allocations).
 */
extern u8 conf_buffer[CONF_SIZE];

#define INJ_SMALL_COUNT 160
#define INJ_SMALL_SIZE  64
#define INJ_LARGE_COUNT 12
#define INJ_LARGE_SIZE  512

#ifdef __arm__
#include "backends/ogc_arm.h"
static inline u32 inj_enter_critical(void) {
    u32 cpsr = get_cpsr();
    set_cpsr_c(cpsr | PSR_I);
    return cpsr;
}
static inline void inj_leave_critical(u32 level) {
    set_cpsr_c(level);
}
#else
static inline u32 inj_enter_critical(void) { return 0; }
static inline void inj_leave_critical(u32 level) { (void)level; }
#endif

static u8 s_small_free[INJ_SMALL_COUNT];
static int s_small_free_top;
static u8 s_large_used[INJ_LARGE_COUNT];

int injmessage_init_heap(void)
{
    u32 cs = inj_enter_critical();
    s_small_free_top = INJ_SMALL_COUNT;
    for (int i = 0; i < INJ_SMALL_COUNT; i++)
        s_small_free[i] = (u8)i;
    for (int i = 0; i < INJ_LARGE_COUNT; i++)
        s_large_used[i] = 0;
    inj_leave_critical(cs);
    return 0;
}

/* Used to allocate messages (bulk in/interrupt) to inject back to the BT SW stack */
static inline injmessage *injmessage_alloc(void **data, u16 size)
{
    u32 total = sizeof(injmessage) + size;
    injmessage *msg = NULL;

    u32 cs = inj_enter_critical();
    if (total <= INJ_SMALL_SIZE && s_small_free_top > 0) {
        u8 idx = s_small_free[--s_small_free_top];
        msg = (injmessage *)&conf_buffer[idx * INJ_SMALL_SIZE];
    } else if (total <= INJ_LARGE_SIZE) {
        for (int i = 0; i < INJ_LARGE_COUNT; i++) {
            if (!s_large_used[i]) {
                s_large_used[i] = 1;
                msg = (injmessage *)&conf_buffer[INJ_SMALL_COUNT * INJ_SMALL_SIZE + i * INJ_LARGE_SIZE];
                break;
            }
        }
    }
    inj_leave_critical(cs);

    if (!msg)
        return NULL;

    msg->size = size;
    *data = msg->data;
    return msg;
}

void injmessage_free(void *msg)
{
    if (!msg)
        return;

    uintptr_t addr = (uintptr_t)msg;
    uintptr_t base = (uintptr_t)conf_buffer;
    uintptr_t small_end = base + (INJ_SMALL_COUNT * INJ_SMALL_SIZE);

    u32 cs = inj_enter_critical();
    if (addr >= base && addr < small_end) {
        u8 idx = (u8)((addr - base) / INJ_SMALL_SIZE);
        if (s_small_free_top < INJ_SMALL_COUNT)
            s_small_free[s_small_free_top++] = idx;
    } else if (addr >= small_end && addr < base + CONF_SIZE) {
        int idx = (int)((addr - small_end) / INJ_LARGE_SIZE);
        if (idx >= 0 && idx < INJ_LARGE_COUNT)
            s_large_used[idx] = 0;
    }
    inj_leave_critical(cs);
}

bool is_message_injected(const void *msg)
{
    return ((uintptr_t)msg >= (uintptr_t)conf_buffer) &&
           ((uintptr_t)msg < ((uintptr_t)conf_buffer + CONF_SIZE));
}

/* HCI and ACL/L2CAP message enqueue (injection) helpers */

/* Set by every HCI command response injection. The command handler uses it to
 * verify that a command swallowed by this module really got its response event:
 * a swallowed command is never handed down to the dongle, so if we fail to
 * answer it the host's HCI command flow control stalls forever. */
bool hci_cmd_response_injected;

static injmessage *alloc_hci_event_msg(void **event_payload, u8 event, u8 event_size)
{
    hci_event_hdr_t *hdr;
    injmessage *msg = injmessage_alloc((void **)&hdr, sizeof(*hdr) + event_size);
    if (!msg)
        return NULL;

    /* Fill the header */
    hdr->event = event;
    hdr->length = event_size;
    *event_payload = (u8 *)hdr + sizeof(*hdr);

    return msg;
}

static injmessage *alloc_hci_acl_msg(void **acl_payload, u16 hci_con_handle, u16 acl_payload_size)
{
    hci_acldata_hdr_t *hdr;
    injmessage *msg = injmessage_alloc((void **)&hdr, sizeof(*hdr) + acl_payload_size);
    if (!msg)
        return NULL;

    /* Fill message data */
    hdr->con_handle = htole16(HCI_MK_CON_HANDLE(hci_con_handle, HCI_PACKET_START, HCI_POINT2POINT));
    hdr->length = htole16(acl_payload_size);
    *acl_payload = (u8 *)hdr + sizeof(*hdr);

    return msg;
}

int inject_hci_event_command_status(u16 opcode)
{
    hci_command_status_ep *ep;
    injmessage *msg = alloc_hci_event_msg((void *)&ep, HCI_EVENT_COMMAND_STATUS, sizeof(*ep));
    if (!msg)
        return IOS_ENOMEM;

    /* Fill event data */
    ep->status = 0;
    ep->num_cmd_pkts = 1;
    ep->opcode = htole16(opcode);

    int ret = inject_msg_to_usb_intr_ready_queue(msg);
    if (ret == IOS_OK)
        hci_cmd_response_injected = true;
    return ret;
}

int inject_hci_event_command_compl(u16 opcode, const void *payload, u32 payload_size)
{
    hci_command_compl_ep *ep;
    injmessage *msg =
        alloc_hci_event_msg((void *)&ep, HCI_EVENT_COMMAND_COMPL, sizeof(*ep) + payload_size);
    if (!msg)
        return IOS_ENOMEM;

    /* Fill event data */
    ep->num_cmd_pkts = 1;
    ep->opcode = htole16(opcode);
    if (payload && payload_size > 0)
        memcpy((u8 *)ep + sizeof(*ep), payload, payload_size);

    int ret = inject_msg_to_usb_intr_ready_queue(msg);
    if (ret == IOS_OK)
        hci_cmd_response_injected = true;
    return ret;
}

int inject_hci_event_con_req(const bdaddr_t *bdaddr, u8 uclass0, u8 uclass1, u8 uclass2,
                             u8 link_type)
{
    hci_con_req_ep *ep;
    injmessage *msg = alloc_hci_event_msg((void *)&ep, HCI_EVENT_CON_REQ, sizeof(*ep));
    if (!msg)
        return IOS_ENOMEM;

    /* Fill event data */
    ep->bdaddr = *bdaddr;
    ep->uclass[0] = uclass0;
    ep->uclass[1] = uclass1;
    ep->uclass[2] = uclass2;
    ep->link_type = link_type;

    return inject_msg_to_usb_intr_ready_queue(msg);
}

int inject_hci_event_discon_compl(u16 con_handle, u8 status, u8 reason)
{
    hci_discon_compl_ep *ep;
    injmessage *msg = alloc_hci_event_msg((void *)&ep, HCI_EVENT_DISCON_COMPL, sizeof(*ep));
    if (!msg)
        return IOS_ENOMEM;

    /* Fill event data */
    ep->status = status;
    ep->con_handle = htole16(con_handle);
    ep->reason = reason;

    return inject_msg_to_usb_intr_ready_queue(msg);
}

int inject_hci_event_con_compl(const bdaddr_t *bdaddr, u16 con_handle, u8 status)
{
    hci_con_compl_ep *ep;
    injmessage *msg = alloc_hci_event_msg((void *)&ep, HCI_EVENT_CON_COMPL, sizeof(*ep));
    if (!msg)
        return IOS_ENOMEM;

    /* Fill event data */
    ep->status = status;
    ep->con_handle = htole16(con_handle);
    ep->bdaddr = *bdaddr;
    ep->link_type = HCI_LINK_ACL;
    ep->encryption_mode = HCI_ENCRYPTION_MODE_NONE;

    return inject_msg_to_usb_intr_ready_queue(msg);
}

int inject_hci_event_role_change(const bdaddr_t *bdaddr, u8 role)
{
    hci_role_change_ep *ep;
    injmessage *msg = alloc_hci_event_msg((void *)&ep, HCI_EVENT_ROLE_CHANGE, sizeof(*ep));
    if (!msg)
        return IOS_ENOMEM;

    /* Fill event data */
    ep->status = 0;
    ep->bdaddr = *bdaddr;
    ep->role = role;

    return inject_msg_to_usb_intr_ready_queue(msg);
}

int inject_hci_event_num_compl_pkts(u8 num_con_handles, const u16 *con_handles,
                                    const u16 *compl_pkts)
{
    hci_num_compl_pkts_ep *ep;
    hci_num_compl_pkts_info *info;
    injmessage *msg =
        alloc_hci_event_msg((void *)&ep, HCI_EVENT_NUM_COMPL_PKTS,
                            sizeof(*ep) + num_con_handles * (sizeof(u16) + sizeof(u16)));
    if (!msg)
        return IOS_ENOMEM;

    info = (void *)((u8 *)ep + sizeof(*ep));

    /* Fill event data */
    ep->num_con_handles = num_con_handles;
    for (int i = 0; i < num_con_handles; i++) {
        info[i].con_handle = htole16(con_handles[i]);
        info[i].compl_pkts = htole16(compl_pkts[i]);
    }

    return inject_msg_to_usb_intr_ready_queue(msg);
}

int inject_hci_event_mode_change(u16 con_handle, u8 unit_mode, u16 interval)
{
    hci_mode_change_ep *ep;
    injmessage *msg = alloc_hci_event_msg((void *)&ep, HCI_EVENT_MODE_CHANGE, sizeof(*ep));
    if (!msg)
        return IOS_ENOMEM;

    /* Fill event data */
    ep->status = 0;
    ep->con_handle = htole16(con_handle);
    ep->unit_mode = unit_mode;
    ep->interval = htole16(interval);

    return inject_msg_to_usb_intr_ready_queue(msg);
}

int inject_hci_event_return_link_keys(u8 num_keys, const bdaddr_t *bdaddr,
                                      const u8 key[][HCI_KEY_SIZE])
{
    hci_return_link_keys_ep *ep;
    injmessage *msg =
        alloc_hci_event_msg((void *)&ep, HCI_EVENT_RETURN_LINK_KEYS,
                            sizeof(*ep) + num_keys * (sizeof(bdaddr_t) + HCI_KEY_SIZE));
    if (!msg)
        return IOS_ENOMEM;

    struct {
        bdaddr_t bdaddr;
        u8 key[HCI_KEY_SIZE];
    } ATTRIBUTE_PACKED *entries = (void *)((u8 *)ep + sizeof(*ep));

    /* Fill event data */
    ep->num_keys = num_keys;
    for (int i = 0; i < num_keys; i++) {
        bacpy(&entries[i].bdaddr, &bdaddr[i]);
        memcpy(entries[i].key, key[i], sizeof(entries[i].key));
    }

    return inject_msg_to_usb_intr_ready_queue(msg);
}

int inject_hci_event_con_pkt_type_changed(u16 con_handle, u16 pkt_type)
{
    hci_con_pkt_type_changed_ep *ep;
    injmessage *msg = alloc_hci_event_msg((void *)&ep, HCI_EVENT_CON_PKT_TYPE_CHANGED, sizeof(*ep));
    if (!msg)
        return IOS_ENOMEM;

    /* Fill event data */
    ep->status = 0;
    ep->con_handle = htole16(con_handle);
    ep->pkt_type = htole16(pkt_type);

    return inject_msg_to_usb_intr_ready_queue(msg);
}

int inject_hci_event_auth_compl(u8 status, u16 con_handle)
{
    hci_auth_compl_ep *ep;
    injmessage *msg = alloc_hci_event_msg((void *)&ep, HCI_EVENT_AUTH_COMPL, sizeof(*ep));
    if (!msg)
        return IOS_ENOMEM;

    /* Fill event data */
    ep->status = status;
    ep->con_handle = htole16(con_handle);

    return inject_msg_to_usb_intr_ready_queue(msg);
}

int inject_hci_event_remote_name_req_compl(u8 status, const bdaddr_t *bdaddr, const char *name)
{
    hci_remote_name_req_compl_ep *ep;
    injmessage *msg =
        alloc_hci_event_msg((void *)&ep, HCI_EVENT_REMOTE_NAME_REQ_COMPL, sizeof(*ep));
    if (!msg)
        return IOS_ENOMEM;

    /* Fill event data */
    ep->status = status;
    bacpy(&ep->bdaddr, bdaddr);
    strcpy(ep->name, name);

    return inject_msg_to_usb_intr_ready_queue(msg);
}

int inject_hci_event_read_remote_features_compl(u16 con_handle,
                                                const u8 features[static HCI_FEATURES_SIZE])
{
    hci_read_remote_features_compl_ep *ep;
    injmessage *msg =
        alloc_hci_event_msg((void *)&ep, HCI_EVENT_READ_REMOTE_FEATURES_COMPL, sizeof(*ep));
    if (!msg)
        return IOS_ENOMEM;

    /* Fill event data */
    ep->status = 0;
    ep->con_handle = htole16(con_handle);
    memcpy(ep->features, features, sizeof(ep->features));

    return inject_msg_to_usb_intr_ready_queue(msg);
}

int inject_hci_event_read_remote_ver_info_compl(u16 con_handle, u8 lmp_version, u16 manufacturer,
                                                u16 lmp_subversion)
{
    hci_read_remote_ver_info_compl_ep *ep;
    injmessage *msg =
        alloc_hci_event_msg((void *)&ep, HCI_EVENT_READ_REMOTE_VER_INFO_COMPL, sizeof(*ep));
    if (!msg)
        return IOS_ENOMEM;

    /* Fill event data */
    ep->status = 0;
    ep->con_handle = htole16(con_handle);
    ep->lmp_version = lmp_version;
    ep->manufacturer = htole16(manufacturer);
    ep->lmp_subversion = htole16(lmp_subversion);

    return inject_msg_to_usb_intr_ready_queue(msg);
}

int inject_hci_event_read_clock_offset_compl(u16 con_handle, u16 clock_offset)
{
    hci_read_clock_offset_compl_ep *ep;
    injmessage *msg =
        alloc_hci_event_msg((void *)&ep, HCI_EVENT_READ_CLOCK_OFFSET_COMPL, sizeof(*ep));
    if (!msg)
        return IOS_ENOMEM;

    /* Fill event data */
    ep->status = 0;
    ep->con_handle = htole16(con_handle);
    ep->clock_offset = htole16(clock_offset);

    return inject_msg_to_usb_intr_ready_queue(msg);
}

static injmessage *alloc_l2cap_msg(void **l2cap_payload, u16 hci_con_handle, u16 dcid, u16 size)
{
    injmessage *msg;
    l2cap_hdr_t *hdr;

    msg = alloc_hci_acl_msg((void **)&hdr, hci_con_handle, sizeof(l2cap_hdr_t) + size);
    if (!msg)
        return NULL;

    /* Fill message data */
    hdr->length = htole16(size);
    hdr->dcid = htole16(dcid);
    *l2cap_payload = (u8 *)hdr + sizeof(l2cap_hdr_t);

    return msg;
}

static injmessage *alloc_l2cap_cmd_msg(void **l2cap_cmd_payload, u16 hci_con_handle, u8 code,
                                       u8 ident, u16 size)
{
    injmessage *msg;
    l2cap_cmd_hdr_t *hdr;

    msg = alloc_l2cap_msg((void **)&hdr, hci_con_handle, L2CAP_SIGNAL_CID,
                          sizeof(l2cap_cmd_hdr_t) + size);
    if (!msg)
        return NULL;

    /* Fill message data */
    hdr->code = code;
    hdr->ident = ident;
    hdr->length = htole16(size);
    *l2cap_cmd_payload = (u8 *)hdr + sizeof(l2cap_cmd_hdr_t);

    return msg;
}

int inject_l2cap_packet(u16 hci_con_handle, u16 dcid, const void *data, u16 size)
{
    void *payload;
    injmessage *msg = alloc_l2cap_msg(&payload, hci_con_handle, dcid, size);
    if (!msg)
        return IOS_ENOMEM;

    /* Fill message data */
    memcpy(payload, data, size);

    return inject_msg_to_usb_bulk_in_ready_queue(msg);
}

int inject_l2cap_connect_req(u16 hci_con_handle, u16 psm, u16 scid)
{
    injmessage *msg;
    l2cap_con_req_cp *req;

    msg = alloc_l2cap_cmd_msg((void **)&req, hci_con_handle, L2CAP_CONNECT_REQ, L2CAP_CONNECT_REQ,
                              sizeof(*req));
    if (!msg)
        return IOS_ENOMEM;

    /* Fill message data */
    req->psm = htole16(psm);
    req->scid = htole16(scid);

    return inject_msg_to_usb_bulk_in_ready_queue(msg);
}

int inject_l2cap_disconnect_req(u16 hci_con_handle, u16 dcid, u16 scid)
{
    injmessage *msg;
    l2cap_discon_req_cp *req;

    msg = alloc_l2cap_cmd_msg((void **)&req, hci_con_handle, L2CAP_DISCONNECT_REQ,
                              L2CAP_DISCONNECT_REQ, sizeof(*req));
    if (!msg)
        return IOS_ENOMEM;

    /* Fill message data */
    req->dcid = htole16(dcid);
    req->scid = htole16(scid);

    return inject_msg_to_usb_bulk_in_ready_queue(msg);
}

int inject_l2cap_disconnect_rsp(u16 hci_con_handle, u8 ident, u16 dcid, u16 scid)
{
    injmessage *msg;
    l2cap_discon_rsp_cp *req;

    msg = alloc_l2cap_cmd_msg((void **)&req, hci_con_handle, L2CAP_DISCONNECT_RSP, ident,
                              sizeof(*req));
    if (!msg)
        return IOS_ENOMEM;

    /* Fill message data */
    req->dcid = htole16(dcid);
    req->scid = htole16(scid);

    return inject_msg_to_usb_bulk_in_ready_queue(msg);
}

int inject_l2cap_config_req(u16 hci_con_handle, u16 remote_cid, u16 mtu, u16 flush_time_out)
{
    injmessage *msg;
    l2cap_cfg_req_cp *req;
    l2cap_cfg_opt_t *opt;
    u32 size = sizeof(l2cap_cfg_req_cp);
    u32 offset = size;

    if (mtu != L2CAP_MTU_DEFAULT)
        size += sizeof(l2cap_cfg_opt_t) + L2CAP_OPT_MTU_SIZE;
    if (flush_time_out != L2CAP_FLUSH_TIMO_DEFAULT)
        size += sizeof(l2cap_cfg_opt_t) + L2CAP_OPT_FLUSH_TIMO_SIZE;

    msg = alloc_l2cap_cmd_msg((void **)&req, hci_con_handle, L2CAP_CONFIG_REQ, L2CAP_CONFIG_REQ,
                              size);
    if (!msg)
        return IOS_ENOMEM;

    /* Fill message data */
    req->dcid = htole16(remote_cid);
    req->flags = htole16(0);

    if (mtu != L2CAP_MTU_DEFAULT) {
        opt = (void *)((u8 *)req + offset);
        offset += sizeof(l2cap_cfg_opt_t);
        opt->type = L2CAP_OPT_MTU;
        opt->length = L2CAP_OPT_MTU_SIZE;
        *(u16 *)((u8 *)req + offset) = htole16(mtu);
        offset += L2CAP_OPT_MTU_SIZE;
    }

    if (flush_time_out != L2CAP_FLUSH_TIMO_DEFAULT) {
        opt = (void *)((u8 *)req + offset);
        offset += sizeof(l2cap_cfg_opt_t);
        opt->type = L2CAP_OPT_FLUSH_TIMO;
        opt->length = L2CAP_OPT_FLUSH_TIMO_SIZE;
        *(u16 *)((u8 *)req + offset) = htole16(flush_time_out);
        offset += L2CAP_OPT_FLUSH_TIMO_SIZE;
    }

    return inject_msg_to_usb_bulk_in_ready_queue(msg);
}

int inject_l2cap_config_rsp(u16 hci_con_handle, u16 remote_cid, u8 ident, const u8 *options,
                            u32 options_len)
{
    injmessage *msg;
    l2cap_cfg_rsp_cp *req;

    msg = alloc_l2cap_cmd_msg((void **)&req, hci_con_handle, L2CAP_CONFIG_RSP, ident,
                              sizeof(*req) + options_len);
    if (!msg)
        return IOS_ENOMEM;

    /* Fill message data */
    req->scid = htole16(remote_cid);
    req->flags = htole16(0);
    req->result = htole16(L2CAP_SUCCESS);
    if (options && options_len > 0)
        memcpy((u8 *)req + sizeof(*req), options, options_len);

    return inject_msg_to_usb_bulk_in_ready_queue(msg);
}
