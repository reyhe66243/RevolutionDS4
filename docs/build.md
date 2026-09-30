# Build and installation

How to compile both firmwares and install them, written for someone who has never compiled this project before. The whole sequence is: install the tools, build the adapter, flash the adapter, build the module, install the module on the console, pair the controller.

## What has to be installed first

A Linux machine with a terminal. Five things are needed.

**1. The ARM toolchain.** The compiler for both firmwares. On Debian or Ubuntu it is the `gcc-arm-none-eabi` package; on other systems, any distribution of `arm-none-eabi-gcc` works. Check it with `arm-none-eabi-gcc --version`.

**2. CMake, Ninja, Make and Python 3.** The build systems. On Debian or Ubuntu: `apt install cmake ninja-build build-essential python3`.

**3. The RP2040 SDK.** The development kit for the adapter board. Put it anywhere on the disk and remember the path; the build looks for it in the `PICO_SDK_PATH` environment variable.

**4. A big endian libgcc.** This is the one unusual requirement. The console processor is big endian, so the module needs the big endian version of the compiler's support library; the standard toolchain ships only the little endian one. The path is passed with the `REVDS4_LIBGCC_BE` environment variable, pointing at the directory that contains it. If it is missing, the module link fails with undefined `__aeabi_*` symbols, which is the symptom to look for.

**5. stripios.** The IOS ELF stripper utility required to package the compiled console module into `REVOLUTIONDS4.app`. It is standard in Wii homebrew toolchains (such as devkitPro's tools) or can be built from source. Ensure `stripios` is installed and available in your `PATH`.

## Fast build (both firmwares)

To compile both firmwares in a single step from the repository root:

```sh
./build.sh
# or simply:
make
```

Ready-to-use artifacts are placed in `build/`:
- `build/REVOLUTIONDS4.uf2`
- `build/REVOLUTIONDS4.app`

Individual components can also be compiled separately as described below.

## 1. Build the adapter firmware

From the repository root:

```
PICO_SDK_PATH=/path/to/pico-sdk ./firmware/build.sh
```

The script configures and compiles everything. At the end it prints the path of the result: `build/REVOLUTIONDS4.uf2`. Compilation takes a few minutes the first time and a few seconds afterwards.

## 2. Flash the adapter

Hold the boot button on the board while connecting its USB cable to the computer: the board appears as a storage unit. Copy `REVOLUTIONDS4.uf2` into it and the board flashes itself and restarts. Note that flashing erases the stored controller pairings, so the controllers must be paired again afterwards.

## 3. Build the console module

From the repository root:

```
REVDS4_LIBGCC_BE=/path/to/libgcc-be ./cios/build.sh
```

The result is `build/REVOLUTIONDS4.app`, and the script checks its size against the 36,864 byte limit of the module loader and fails if it does not fit. The current version occupies 36,312 bytes (552 bytes below the limit).

## 4. Install the module on the console

The d2x cIOS installer bundles modules declared in its `ciosmaps.xml` configuration file. To include `REVOLUTIONDS4.app`, both the binary and the XML map must be placed on the SD card:

**1. Copy the module.** Copy `REVOLUTIONDS4.app` into the version directory of the d2x cIOS installer on the console SD card (for example, `apps/d2x-cios-installer/v10/beta52/d2x-v10-beta52/`), alongside the other `.app` modules.

**2. Update `ciosmaps.xml`.** Open `ciosmaps.xml` in the installer folder on the SD card (`apps/d2x-cios-installer/ciosmaps.xml`). The configuration has a separate `<base>` block for each IOS, and you must apply the modification once per base you intend to install.

Each block needs two edits, and the `id` is where mistakes happen:

1. Increment `contentscount` and `modulescount` by 1 (for example, if it says `contentscount="25" modulescount="6"`, change it to `contentscount="26" modulescount="7"`). After the change the two counts must match the real number of `<content .../>` entries in the block, and the number of those carrying a `module=` attribute.
2. Add the new entry at the end of the block, **after the last `<content>` tag**:

   ```xml
   <content id="0x??" module="REVOLUTIONDS4" tmdmoduleid="-1"/>
   ```

**The `id` is the highest `id` already present in that block, plus one, in hexadecimal.** It is different for every base, so do not copy the same value into both blocks: the d2x configuration already inserts its own modules (MLOAD, FAT, EHCI or USBS, DIPP, ES, FFSP) at the end of each block, and your entry continues that sequence. Look at the last `<content>` in the block you are editing and use the next hex value.

Values for the bases shipped with `d2x-v11-beta3`:

| `<base ios=...>` | Last `id` in the block | `id` to use | d2x modules in that block |
|---|---|---|---|
| `37` v5662 | `0x28` | `0x29` | MLOAD, FAT, EHCI, DIPP, ES, FFSP |
| `38` v4123 | `0x1d` | `0x1e` | MLOAD, FAT, EHCI, DIPP, ES, FFSP |
| `53` v5662 | `0x26` | `0x27` | MLOAD, FAT, EHCI, DIPP, ES, FFSP |
| `55` v5662 | `0x1d` | `0x1e` | MLOAD, FAT, EHCI, DIPP, ES, FFSP |
| `56` v5661 | `0x1e` | `0x1f` | MLOAD, FAT, EHCI, DIPP, ES, FFSP |
| **`57` v5918** | `0x22` | **`0x23`** | MLOAD, FAT, EHCI, DIPP, ES, FFSP |
| **`58` v6175** | `0x1d` | **`0x1e`** | MLOAD, FAT, **USBS**, DIPP, ES, FFSP |
| `60` v6174 | `0x14` | `0x15` | MLOAD, FAT, EHCI, DIPP, ES, FFSP |
| `70` v6687 | `0x14` | `0x15` | MLOAD, FAT, EHCI, DIPP, ES, FFSP |
| `80` v6943 | `0x14` | `0x15` | MLOAD, FAT, EHCI, DIPP, ES, FFSP |

Note that base 58 is the one block that differs from the others: d2x inserts `USBS` there instead of `EHCI`, because IOS58 shipped without its own `USB_MSC` module, so its last `id` is `0x1d` and not `0x22`. Using `0x23` in both blocks is exactly the mistake to avoid.

A complete, correct pair of entries reads like this (real `d2x-v11-beta3` blocks, contents abbreviated):

```xml

<base ios="58" version="6175" contentscount="26" modulescount="7">
    <!-- ... the base IOS contents, then the d2x modules ... -->
    <content id="0x18" module="MLOAD" tmdmoduleid="-1"/>
    <content id="0x19" module="FAT" tmdmoduleid="-1"/>
    <content id="0x1a" module="USBS" tmdmoduleid="-1"/>
    <content id="0x1b" module="DIPP" tmdmoduleid="-1"/>
    <content id="0x1c" module="ES" tmdmoduleid="-1"/>
    <content id="0x1d" module="FFSP" tmdmoduleid="-1"/>
    <content id="0x1e" module="REVOLUTIONDS4" tmdmoduleid="-1"/>
</base>
```

A wrong `id` leaves a gap in the block's index sequence. If the installer or the console misbehaves after an edit, restore the original `ciosmaps.xml` and repeat the modification, since a single wrong entry is enough to break the installed cIOS.

**3. Run the installer.** Launch the d2x cIOS installer from the Homebrew Channel on the console. Install **base 58 into slot 247** (or 248 / 250). Base 58 is strictly required for REVOLUTIONDS4 because it contains the updated USB 2.0 / HID v5 stack (`USBv5 0x50001`), supporting both USB ports, hotplugging, and concurrent transfers. While base 57 is technically present in d2x, its port 0 is reserved for USB mass storage (`EHCI.app`) and its legacy USB driver does not reliably support USB HID hotplugging or multi-endpoint polling. Always configure your game launcher (e.g., USB Loader GX) to load games using the base 58 slot, and enable **"Block IOS Reload"** in loader settings.

## 5. Connect and pair

Connect the adapter to a USB port of the console and turn the console on with the adapter connected. 

To link a controller for the first time:
1. Press the adapter boot button once to open a twenty-second pairing window.
2. Put the DualShock 4 into pairing mode by holding **SHARE + PS** until its light bar blinks white.

The controller pairs automatically and confirms calibration with two green blinks. Once paired, pressing the **PS** button reconnects it in future sessions without using the boot button.

For the complete multi-controller procedure (up to two DualShock 4 controllers), dynamic player slot colors, bond clearing, and diagnostic codes, see [controllers](controllers.md).

## 6. Test it

Start a game with motion control and check the three things that matter: a full turn of the wrist is a full turn in the game, the tilt reaches its angle, and the audio plays in the controller speaker. Then check the two hold combinations (L1 + L3 for the Classic Controller mode, L1 + R3 for the horizontal orientation) as described in the README.

## Troubleshooting

| Symptom | Cause |
|---|---|
| The module link fails with undefined `__aeabi_*` | `REVDS4_LIBGCC_BE` is wrong or missing; it must point at the directory containing the big endian libgcc |
| The module build fails with "Could not find STRIPIOS" | `stripios` is missing or not in `PATH`; install the utility or ensure its directory is in your `PATH` |
| The firmware build cannot find the SDK | `PICO_SDK_PATH` is wrong or unset |
| The module does not fit | The module is over 36,864 bytes; something has to be removed or moved to the adapter |
| The controller does not pair | Repeat the sync window with a short press on the boot button, and hold SHARE and PS until the light bar blinks white |
| The controller connects but nothing moves | Check the blink code: three red blinks mean the controller did not answer the calibration request |
| The adapter is not detected at all on base 57 | Base 57 is unsupported/broken for USB HID controllers; use Base 58 |
| Nothing works on a base and the IOS log shows `TST: Error in patcher: Unknown module version.` | The OH1 module of that base is not one of the two builds the module knows how to patch, so it refuses to load; use base 58 |
| The module was installed but the game never sees the controller | Check the `id` in `ciosmaps.xml`: it must be the highest `id` of that block plus one, and the two counts must match the real entries. Reinstall the slot after fixing it |
