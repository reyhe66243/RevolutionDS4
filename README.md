# REVOLUTIONDS4

REVOLUTIONDS4 lets you use DualShock 4 controllers on the Wii as a Wii Remote with emulated MotionPlus. The project is made of two firmwares that work together: the adapter firmware, in the firmware folder, which links the controllers over Bluetooth and presents them to the console over USB, and an IOS module, in the cios folder, which emulates the remote and the MotionPlus to the game. It requires no hardware modification to the console (see [architecture](docs/architecture.md) and [USB link](docs/usb-link.md) for the end-to-end division of responsibilities and data path).

The objective is to make the DualShock 4 behave as close to a real Wii Remote as physically possible: a full turn of the wrist is a full turn in the game, a tilt is a tilt, the audio plays in the controller speaker, the rumble answers to the game, and two controllers work independently. Where the real remote and the DualShock 4 differ, the difference is measured and closed (documented with physical measurements in [motion](docs/motion.md)).

## What it provides

Motion with a one to one correspondence to the physical gesture, audio to the controller speaker, rumble, player indicators on the light bar, support for two independent controllers, and two usage orientations: vertical, for pointing and general motion games, and horizontal, for racing games.

## What is needed

The complete setup is: an RP2040 based board with Bluetooth, one or two DualShock 4 controllers, a Wii with the module installed, an SD card for the console, and a free USB port for the adapter. Board compatibility: the only board tested is the Pico W and the Pico WH (RP2040 with the CYW43 Bluetooth controller); other RP2040 boards with Bluetooth may work, but nothing else has been verified.

## Controller combinations

The system provides two hold combinations (both require holding for about half a second, so that arm motion cannot trigger them by accident) processed on the fly by the console module, and one physical button on the adapter itself.

| Combination | What it does | When to use it |
|---|---|---|
| L1 + L3 | Switch between standard mode (remote with MotionPlus) and Classic Controller mode | Classic Controller mode is for games that expect a Classic Controller: the console sees the extension port change and the game treats the controller as one. The change is visible to the game while it runs (some games like Mario Kart Wii require pressing the PS button to open and close the Home menu to refresh the controller state mid-game). |
| L1 + R3 | Switch between vertical and horizontal (wheel) orientation | Vertical for pointing and general motion games. Horizontal for racing and sideways games, where the remote is held like a steering wheel. In horizontal mode the motion sensors are reordered: the accelerometer and gyroscope axes are remapped so that holding the DualShock 4 like a wheel produces exactly the motion a Wii Remote produces when held the same way. Games that use the MotionPlus rely on the standard vertical 3D spatial reference and should be played in standard mode. |
| Adapter boot button, short press | Opens a twenty second synchronization window | Pairing a new controller (Player 1, or Player 2 while Player 1 is already connected) |
| Adapter boot button, long press | Disconnects all controllers and clears the stored bonds | Starting over with new pairings |

The light bar dynamically reflects the player slot assigned by the console according to the Nintendo Wii color standard: Player 1 is blue, Player 2 is red, Player 3 is green, and Player 4 is yellow (for example, if real Wiimotes occupy the first slots, the DualShock 4 controllers seamlessly adopt the remaining slots and colors). It also indicates the usage orientation, as horizontal mode uses a dimmer colour than vertical mode. Flashing the adapter erases the stored bonds, so after a firmware update the controllers must be paired again.

**LED indication when the controller connects to the adapter.** The adapter reports the result of the factory calibration with a blink code on the controller light bar: two green blinks mean the calibration was applied, three red blinks mean the controller did not respond, and four yellow blinks mean the calibration was rejected because it did not pass the sanity checks (see the [controllers document](docs/controllers.md) for calibration and drift tracking details).

## Questions

- **Can the Wii's internal Bluetooth radio be used instead of the adapter?**
  No. Running the Bluetooth stack, controller pairing, and 16 kHz SBC audio encoding inside the IOS module would exceed its strict 36 KiB memory budget and risk console stability. The dedicated RP2040 runs these tasks without affecting native remotes. For the complete explanation, see [limits](docs/limits.md).

- **Will other controllers, modes, or attachments be supported?**
  Only hardware that a single DualShock 4 can reproduce faithfully 1:1. The Classic Controller is supported because all its inputs map directly. The Nunchuk is not supported because a single inertial unit cannot describe two independent objects moving in two separate hands without inventing false motion. For details on controller scope, see [limits](docs/limits.md).

- **Why is there no joystick pointer, and why not use the gyroscope for pointing?**
  The touchpad preserves the direct physical intuition of absolute pointing. Gyroscopes lack an absolute spatial reference and accumulate continuous drift without sensor bar optical calibration. For pointer mechanics and vertical cursor suppression, see [limits](docs/limits.md).

- **How can games requiring the Nunchuk be played?**
  Through community game or Riivolution patches that add Classic Controller support (such as Super Mario Galaxy 1 and 2, Donkey Kong Country Returns, and Kirby's Return to Dream Land). The project's Classic Controller mode then plays them directly with the DualShock 4. See [limits](docs/limits.md).

- **How many controllers can be connected simultaneously?**
  Up to two (2) DualShock 4 controllers. The system is designed for two simultaneous streams over USB and Bluetooth without sacrificing real-time 16 kHz audio; see [controllers](docs/controllers.md) for pairing and multi-controller operation, and [limits](docs/limits.md) for the throughput constraints.

## Installation

The IOS module is installed with the **d2x cIOS installer**: **Base 58 (typically slot 247, 248 or 250) is strictly required for full functionality and stability.**

> [!IMPORTANT]
> **Base 58 vs Base 57 Requirement:**
> While d2x technically allows building on Base 57, **Base 57 is not recommended and largely broken for USB HID controllers**:
> 1. In Base 57, Port 0 is seized exclusively by `EHCI.app` for USB storage drives.
> 2. The underlying IOS 57 kernel lacks the updated USB 2.0 / HID v5 stack (`USBv5 0x50001`) present in IOS 58. Consequently, hotplugging, asynchronous transfers, and multi-endpoint polling fail or freeze under Base 57.
> 3. **Base 58 provides the complete USB 2.0 HID stack with full support for both USB ports, hotplugging, and concurrent transfers.**
> Always configure your loader (USB Loader GX, WiiFlow, etc.) to use the **Base 58 cIOS slot**, and ensure **"Block IOS Reload"** is enabled so the game does not drop back to a non-patched retail IOS during minigame transitions.

To install it, copy `REVOLUTIONDS4.app` into the version folder of the d2x cIOS installer on the console SD card and register it in `ciosmaps.xml` in the `<base ios="58">` section (incrementing `contentscount` and `modulescount` by one and adding the content entry for `REVOLUTIONDS4` with `id="0x1e"`). Then run the installer to write the slot, connect the adapter to a USB port, and pair your controllers.

For full building and installation instructions, see [build and installation](docs/build.md). For controller pairing, LED indication codes, and operation details, see [controllers](docs/controllers.md).

A strongly recommended companion setup is Priiloader, with the hack that makes the console boot into the installed cIOS. The same recommendation applies to WAD channels and homebrew applications. It is not dangerous: if the console ever fails to boot, holding the RESET button while powering it on still starts Priiloader, which is the standard recovery path.

## Acknowledgements

REVOLUTIONDS4 stands on the work of three projects. Dolphin, whose published sources were the reference for understanding the remote authentication and the protocol; no code from it is used. Fakemote, from which the emulated remote, the MotionPlus emulation and the input handling were derived and then extensively modified. Joypad OS, whose input routing, player management and feedback services run the adapter firmware. The project would not exist without them; the NOTICE file records this with the exact provenance of each piece.

## License

The project is licensed per component, because the work it derives from is. The console module (cios folder) is licensed under the GNU General Public License version 2, because it derives from Fakemote, which is GPL-2.0. The adapter firmware (firmware folder) is licensed under the Apache License 2.0, because it derives from Joypad OS, which is Apache-2.0. The exact provenance is recorded in the NOTICE file.

For anyone who wants to use or modify this project, the obligations are the same as the upstreams': keep the copyright notices and the NOTICE file, state the changes made to modified files, and distribute each component under its own license. Those two mechanisms, the copyright notices under GPL and the NOTICE file under Apache, are what keep the origin of the project visible in future uses.

## Warranty

The software is provided as is, without any warranty of any kind, express or implied, including but not limited to the warranties of merchantability, fitness for a particular purpose and non-infringement. The authors are not liable for any damage to consoles, controllers, memory cards or data. As with any modification of console software, use it knowing what it does and how to recover from it; the recovery path for the console is described in the installation section.

## Documentation

Each part of the system has its own document:

- [Architecture](docs/architecture.md): the division of responsibilities between the adapter and the console, the end to end data path and the link protocol between both parts.
- [Controllers](docs/controllers.md): how each DualShock 4 is linked and conditioned, the factory calibration, the drift control and the real delivery rates.
- [Motion](docs/motion.md): the emulation of the remote and the MotionPlus, rate encoding, usage orientations, gain calibration, and physical verification measurements.
- [Audio](docs/audio.md): the path of the audio from the game to the controller speaker and the design decisions behind it.
- [USB link](docs/usb-link.md): the USB channel between both parts, its transfer management and its recovery system.
- [Limits](docs/limits.md): what the project cannot do, what it deliberately does not support, and the constraints behind those decisions.
- [Build and installation](docs/build.md): requirements, build commands for both firmwares, repository layout and installation procedure.
