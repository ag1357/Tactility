# USB Audio and CardKB2 Mission Notes

## 2026-09-10: Phase 0 reconnaissance

### Repository baseline

- Working branch: `work/aethercore-v15-final-integration`.
- Upstream `origin/main`: `6e55e71e` (`Export atof symbol (#597)`).
- The branch is based directly on that commit and was two commits ahead when
  reconnaissance started.
- All required architecture seams are present:
  `Audio.h`, `AudioService.h`, `BluetoothHidHost.cpp`,
  `lvgl_hardware_keyboard_add_custom()`, and the USB-host binding with
  `peripheral-map`.
- `EpubReaderUI.cpp`, named as a style reference in the mission, is absent from
  this checkout and all local Git history. USB lifecycle work will follow
  `esp32_usbhost_msc.cpp`; UI work will follow adjacent current Tactility apps.
- No rebase is currently required.

### USB host and AetherLink

- The `espressif,esp32-usbhost` parent installs the USB host library once and
  owns the host event task.
- AetherLink's CDC driver is a child client of that parent. It installs only
  the CDC-ACM class driver and does not call `usb_host_install()`.
- The Waveshare board uses `usbhost0` with `peripheral-map = <0>`.
- A UAC driver can be another child client. No AetherLink refactor or USB role
  change is required, so the mission stop condition is not active.
- Device C is currently disconnected and must remain untouched during this
  mission.

### Hub support

- ESP-IDF 5.5's current symbols are enabled:
  `CONFIG_USB_HOST_HUBS_SUPPORTED=y` and
  `CONFIG_USB_HOST_HUB_MULTI_LEVEL=y`.
- The obsolete singular name `CONFIG_USB_HOST_HUB_SUPPORT` is not used.
- Internal-SRAM cost and minimum free heap still need before/after hardware
  measurements.
- Live Device A telemetry exposed a stricter ESP-IDF 5.5 limitation:
  `components/usb/hub.c` rejects any FS/LS downstream device when its parent
  hub is high speed because transaction translators are not implemented
  (`IDF-10023`). The connected full-speed earphones were therefore disabled at
  hub port `[1:4]` before any class driver could bind.
- This blocks simultaneous earphones plus AetherLink on the current high-speed
  hub regardless of UAC driver availability. A direct-earphone connection can
  qualify UAC in isolation, but cannot satisfy final hub coexistence.

### Existing audio path

- The AudioPlayer opens and writes the first ready `AUDIO_STREAM_TYPE` device
  directly with `audio_stream_open_output()`, `audio_stream_write()`, and
  `audio_stream_close()`.
- Volume is controlled through `tt::service::audio` and persisted as
  `outputVolume`.
- The mission's proposed `AUDIO_STREAM_TYPE` boundary was incorrect.
  `audio-stream0` is the one global software aggregate and already owns
  resampling, channel conversion, and shared controls. USB audio therefore
  belongs behind that aggregate as an `AUDIO_CODEC_TYPE` output backend.

### Alternate USB controller

- The official board schematic breaks controller 1 out on J6:
  GPIO24 is USB D- and GPIO25 is USB D+ through 0-ohm links. Ground is adjacent.
- This is a possible future isolation path, but it needs controlled VBUS and is
  not part of the current implementation.

### CardKB2 and global keyboard path

- BLE HID has scan, connect, GATT discovery, report-map parsing, input-report
  subscription, reconnect, and global LVGL keypad registration.
- The path is close enough to test before considering I2C.
- Current CardKB2 documentation and the local library identify BLE HID as
  **Fn+Sym+4**. Fn+Sym+2 selects UART, despite the original mission text.
- A confirmed upstream bug exists: software-keyboard suppression checks kernel
  `KEYBOARD_TYPE` devices but not custom BLE/USB keyboard indevs.
- Pairing telemetry was captured with the existing `BtHidHost` tag and is
  summarized below without ambient device identifiers.

### CardKB2 live result

- CardKB2 entered BLE HID mode with Fn+Sym+4 and paired successfully.
- `BtHidHost` connected, found one input report, established encryption, read
  the 65-byte report map, resolved report ID 1 as keyboard input, subscribed,
  and registered the global LVGL keyboard indev.
- Typed keys worked on Device A. This is a successful BLE HID path, not a scan,
  connection, GATT, report-map, or notification failure.
- The remaining acceptance work is cross-app typing, disconnect cleanup, and
  software-keyboard suppression.

### Current physical state

- Device A serial: `5B91042354`.
- Device C: disconnected.
- The powered hub and USB earphones are connected to Device A's host port.
  They are therefore not expected to appear in the Pi's `lsusb`.

## 2026-09-10: Phase 1 implementation

### UAC output codec

- Pinned `espressif/usb_host_uac` 1.5.0 for ESP32-S3/P4 and resolved it against
  ESP-IDF 5.5.2.
- Added `espressif,esp32-usbhost-uac` as a child of the existing `usbhost0`.
  The UAC driver calls `uac_host_install()` only. It does not call
  `usb_host_install()`, change `peripheral-map`, or touch AetherLink.
- The device implements the standard `AUDIO_CODEC_TYPE` API with OUTPUT
  capability only, native 48 kHz, 16-bit stereo PCM. `audio-stream0` continues
  to provide rate and channel conversion to apps.
- Volume and mute use UAC feature-unit class requests through
  `uac_host_device_set_volume()` and `uac_host_device_set_mute()`.
- RX interfaces are deliberately left unclaimed. The earphones' microphone is
  deferred because advertising a dedicated UAC input would currently displace
  the onboard microphone.
- The PCM ring buffer is sized above the board's
  `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL` threshold so it can reside in PSRAM;
  UAC isochronous URBs remain in DMA-capable memory.
- The class client queues only TX connection events. Disconnect and transfer
  error state is sticky, so queue pressure cannot lose the one-shot unplug
  notification.
- The UAC client claims only the audio streaming interface. The earphones'
  separate HID interface remains available to the existing USB HID client.

### Codec selection and hot-plug finding

- `audio-stream0` already prefers an OUTPUT-only codec such as UAC over the
  BOTH-capable ES8311 when it performs a codec search.
- That search is not repeated on device ready/unready or capability changes.
  `codec_for_direction()` caches the selected `Device*` on first use and
  returns the cached pointer thereafter.
- If output is first resolved before USB audio connects, ES8311 remains bound
  and a later UAC connection cannot take over.
- If UAC is connected before first output resolution, it is preferred. On
  unplug, `audio-stream0` retains the UAC pointer; writes fail against the
  disconnected backend, and a later open does not fall back to ES8311.
- This is the actual Phase 2 upstream gap. Phase 2 should make the existing
  implicit policy re-evaluate safely and become overridable, rather than add a
  second stream or invent USB-specific routing.

### Validation status

- Device A firmware builds successfully with UAC 1.5.0. The image is
  `0x3969b0` bytes with `0x69650` bytes (10%) free in the smallest
  app partition. The simulator build also passes; no CTest tests are defined.
- Device A was flashed through its USB-UART bridge; all written partitions
  passed flash hash verification.
- Direct-earphone unplug/replug passed. The output interface closed and reopened
  without restarting Tactility. Reopening consumed about 2.5 KB internal heap;
  the 32 KB PCM ring appeared in external memory as designed.
- A 44.1 kHz, 16-bit stereo MP3 initially played slowly and with artifacts.
  This exposed an upstream `audio-stream0` issue: its resampler used
  per-sample double arithmetic and reset interpolation state at every write.
  It now uses stateful fixed-point interpolation, with scratch buffers preferring
  external memory.
- The board uses four UAC isochronous URBs with four packets each (16 ms in
  flight instead of the component default 9 ms). With that scheduling window,
  direct USB playback was clear and real-time in the hardware listening test.
- The UAC component's background event task cannot be merged into the Tactility
  client task: interface-open and stream-start control transfers synchronously
  depend on the event loop running in parallel. A one-task experiment timed out
  those requests and was reverted.
- AudioPlayer volume changes reached the UAC feature unit and audibly changed
  output. Setting global Output Mute before playback kept playback silent,
  confirming cached mute replay and the hardware mute class request.
- Unplugging during playback ended the stream with an error rather than
  crashing. Replugging restored the UAC codec and subsequent playback worked,
  but the aggregate still does not switch to ES8311 automatically.
- The earphones' inline HID buttons did not produce actions. UAC leaves the HID
  interface unclaimed, so this is a separate consumer-control mapping gap, not
  an audio-interface ownership conflict.
- Exiting AudioPlayer stops its stream by design; this is not a background
  playback service.
- Stream start still coincided with one existing ST7796 SPI queue-allocation
  failure in telemetry. Audio remained clear, but repeated seek/reopen testing
  drove the internal-heap minimum to roughly 4.4 KB. Further display/decoder
  internal-memory work is advisable before adding more concurrent USB clients.
- Hub coexistence remains blocked by ESP-IDF's missing high-speed-hub
  transaction-translator support, independently of this driver.
