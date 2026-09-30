# USB link

**The channel between both parts.** The console module manages the USB link with the adapter through continuous, event-driven interrupt IN transfers (`bInterval = 1 ms`): each completed read immediately rearms the next transfer, sustaining an effective input delivery of up to two hundred reports per second from the adapter's incoming stream. A periodic ten millisecond watchdog timer supervises this link: it guarantees that input polling is rearmed if an IN transfer is ever dropped, drains lingering audio batches, and supervises hardware link health. That cadence is the heartbeat of the system: motion reading, audio draining, output dispatch, and disconnect handling all depend on it.

**Deterministic transfer architecture.** Rather than relying on a shared dynamic pool where transfers could suffer resource exhaustion, the USB backend allocates four dedicated, non-overlapping static transfer slots:
1. `XFER_SLOT_INTR_IN`: Dedicated strictly to interrupt IN reads for continuous controller input reports.
2. `XFER_SLOT_INTR_OUT`: Dedicated strictly to interrupt OUT writes for aggregated controller speaker audio.
3. `XFER_SLOT_CTRL_P1`: Dedicated strictly to control endpoint transfers (LED and rumble) for Player 1.
4. `XFER_SLOT_CTRL_P2`: Dedicated strictly to control endpoint transfers (LED and rumble) for Player 2.

Because each data pipeline possesses its own private transfer slot, input polling and speaker audio can never be starved or blocked by rapid bursts of rumble or LED commands.

**Kernel-level endpoint cancellation.** If an endpoint ever stalls or a transfer completion is dropped by the hardware, the module does not simply free software structures while leaving the hardware in an inconsistent state. Instead, it invokes Starlet's native kernel endpoint cancellation (`USBV5_IOCTL_CANCELENDPOINT` via `USB_CancelEndpoint`). This instructs the Starlet OHCI host controller to abort pending transfers at the hardware descriptor level, safely retiring the transaction and releasing the endpoint without corrupting kernel state or requiring a console reboot.

**Output coalescing.** Output writes (rumble and LED updates) are automatically coalesced per player: if a control transfer is already in flight on a player's dedicated slot, any subsequent updates overwrite the pending state and dispatch immediately upon completion of the active transfer. This guarantees that game engines alternating rumble at high frequencies never flood the USB bus or drop commands.

**IOS reload handling and auto-resume.** When launching titles through loaders such as USB Loader GX, the console undergoes an IOS reload, triggering a USB bus suspend condition. The RP2040 firmware detects this state through its USB suspend callback and automatically re-arms endpoints upon bus resume without terminating the Bluetooth connection to paired controllers. The Bluetooth link remains active and bonded across game transitions, allowing the console module in the newly loaded IOS to instantly discover and poll the controllers without physical re-plugging or power cycling.

**The adapter side.** The adapter delivers the motion stream at a measured 247.6 reports per second, with 246.0 fresh gyroscope samples per second, a rate higher than the one of the original remote. The module reads that stream with its poll of two hundred reports per second and keeps the last received value between reads, so the effective rate is at least equal to the one of the real hardware and no information is lost. The combination of a generous supply from the adapter, deterministic slot allocation, and hardware endpoint cancellation makes the link completely resilient against bus saturation and traffic bursts.

