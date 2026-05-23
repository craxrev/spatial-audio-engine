# Galaxy Buds 2 Pro head-tracking — wire protocol

Reverse-engineered from `timschneeb/GalaxyBudsClient` (commit cloned 2026-05-23).
Reference files in that repo:
- `Message/SpatialSensorManager.cs` — flow
- `Message/SppMessage.cs` — frame codec
- `Message/Decoder/SpatialAudioDataDecoder.cs` — quaternion parser
- `Message/SppMessageEnums.cs` — `MsgIds`
- `Model/Constants.cs` — `Uuids`, `SpatialAudioControl`, `SpatialAudioData`
- `Model/Specifications/Buds2ProDeviceSpec.cs` — Buds2Pro device profile
- `Utils/Crc16.cs` — CRC-16-CCITT (table-based)
- `Platform.OSX/Native/src/Bluetooth.mm` — macOS RFCOMM connect/send/recv

## Transport

- Bluetooth Classic, **RFCOMM** (not BLE).
- Service UUID for Buds 2 Pro: **`2e73a4ad-332d-41fc-90e2-16bef06523f2`** (`Uuids.SppNew`).
  - Alt-mode (SMEP) UUID: `f8620674-a1ed-41ab-a8b9-de9ad655729d`. Not needed for default operation.
  - Old Buds (Pro / Plus / 2019 / Live) use standard SPP `00001101-...`.
- macOS path: `IOBluetoothDevice deviceWithAddressString:` → `openConnection` → `performSDPQuery` → `getServiceRecordForUUID` → `getRFCOMMChannelID` → `openRFCOMMChannelSync`. Device must already be **paired** in macOS Bluetooth settings.
- Bidirectional byte stream after channel open. Send full frames; receive a byte stream that must be re-framed (SOM..EOM scan, with re-sync on CRC failure).

## Frame format (Buds 2 Pro — non-legacy)

Buds 2 Pro does **not** support `SppLegacyMessageHeader`. Frame:

```
+------+------+------+------+--------------- ... ----+------+------+------+
| SOM  | hdr_lo | hdr_hi | MsgId | payload[0..N-1]      | CRC1 | CRC2 | EOM  |
| 0xFD |        |        |       |                      |      |      | 0xDD |
+------+------+------+------+--------------- ... ----+------+------+------+
        |<----- 16-bit little-endian header ----->|
```

- `SOM = 0xFD`, `EOM = 0xDD` (from `Buds2ProDeviceSpec.StartOfMessage/EndOfMessage`).
- 16-bit header (little-endian):
  - bits 0..9 → `size = 1 (MsgId) + N (payload) + 2 (CRC) = N + 3`
  - bit 12 (`0x1000`) → `Type` (set = Request, clear = Response).
    NOTE: `SppMessage.Decode` reads it as `(header & 0x1000) != 0 ? Request : Response`,
    but the encoder uses `(byte)MsgTypes` where `Response = 1` → sets bit 4 (`0x10`).
    Inspect actual outbound bytes empirically; the bit positions in the encoder vs decoder don't match cleanly. **TODO: verify on the wire.**
  - bit 13 (`0x2000`) → `IsFragment`.
- `MsgId`: one byte (see ids below).
- `payload`: `N` bytes (`N = size − 3`).
- `CRC1 CRC2`: 16-bit CRC-16-CCITT over `[MsgId, payload...]`, written as `BitConverter.GetBytes(short)` → little-endian on x86/ARM, i.e. `CRC1 = crc & 0xFF`, `CRC2 = (crc >> 8) & 0xFF`. On decode, the *received* order is `CRC1, CRC2`, but they are reversed before feeding into the CRC verifier (verifier expects checksum bytes as big-endian tail) — so when computing the full-frame check, swap them. Empirical: easier to just compute CRC over `[MsgId, payload...]` ourselves and compare to `CRC1 | (CRC2 << 8)`.

CRC-16-CCITT polynomial: standard table-based (256 entries from `Utils/Crc16.cs:11`). Init = 0, no final XOR.

## Message IDs (Buds 2 Pro)

```
SET_SPATIAL_AUDIO       = 124   (0x7C)   request
SPATIAL_AUDIO_DATA      = 194   (0xC2)   inbound — sensor events
SPATIAL_AUDIO_CONTROL   = 195   (0xC3)   request + inbound control reply
```

## `SPATIAL_AUDIO_CONTROL` payload byte (1 byte)

```
Attach                = 0    → enter head-tracking mode
Detach                = 1    → exit
AttachSuccess         = 2    (inbound ack)
DetachSuccess         = 3    (inbound ack)
KeepAlive             = 4    → send every 2000 ms while attached
WearOnOff             = 5
QuerySensorSupported  = 6
SpatialBufOn          = 7
SpatialBufOff         = 8
QueryGyroBiasExistence= 9
ManualGyrocalStart    = 10
ManualGyrocalCancel   = 11
ManualGyrocalQueryReady=12
ResetGyroInUseBias    = 13
DebugResetBiasAll     = 64
DebugResetBiasInUse   = 65
DebugResetPrintTimestamp= 66
```

## `SPATIAL_AUDIO_DATA` payload (inbound)

Layout: `payload[0]` = event ID, `payload[1..]` = event-specific data.

```
Event IDs (SpatialAudioData):
  BudGrv               = 32   ← QUATERNION
  WearOn               = 33
  WearOff              = 34
  BudGyrocal           = 35
  BudSensorStuck       = 36
  SensorSupported      = 37
  GyroBiasExistence    = 38
  ManualGyrocalReady   = 39
  ManualGyrocalNotReady= 40
  BudGyrocalFail       = 41
```

### `BudGrv` (32) — head orientation quaternion

`payload[1..]` (≥ 9 bytes):

```
offset 0..1 : int16 little-endian   q[0]  → q[0] = raw / 10000.0f
offset 2..3 : int16 little-endian   q[1]  → q[1] = raw / 10000.0f
offset 4..5 : int16 little-endian   q[2]  → q[2] = raw / 10000.0f
offset 6..7 : int16 little-endian   q[3]  → q[3] = raw / 10000.0f
offset 8    : uint8                 GrvBoolean (validity flag; 0 = valid in code)
```

**Component order in `GrvFloatArray`: indices 0,1,2,3 — but Samsung does not document which slot is x/y/z/w.** The C# code stuffs them into `Quaternion(x, y, z, w)` constructor in order (`SpatialSensorManager.cs:63`), but `System.Numerics.Quaternion(x,y,z,w)` is `(x, y, z, w)`, so by convention the wire layout is `[x, y, z, w]` little-endian int16, each ÷ 10000. **VERIFY EMPIRICALLY** by parking the buds level and reading values; should be approx `(0, 0, 0, 1)` ± gravity-induced small components.

Sample rate: not specified in the source. Empirically ~50–100 Hz expected (matches similar Samsung IMU streams). Measure when implementing.

Frame convention (the bud's quaternion):
- Unknown a priori. Native needs `+X forward, +Y left, +Z up`. Buds frame is most likely `+X right, +Y forward, +Z up` (typical Android sensor convention) or a remap thereof.
- Calibrate empirically: log raw quat, perform known head gestures (nod, look-left, tilt-right), derive the fixed conjugation/rotation that maps bud-frame → native-frame.

## Required handshake (timeline)

```
1. RFCOMM connect (UUID = SppNew).
2. Send: SET_SPATIAL_AUDIO, payload [0x01]                 → enable sensor
3. Send: SPATIAL_AUDIO_CONTROL, payload [SpatialAudioControl.Attach = 0]
4. Receive: SPATIAL_AUDIO_CONTROL with payload[0] == AttachSuccess(2)
5. Start a 2-second keep-alive timer:
     Send: SPATIAL_AUDIO_CONTROL, payload [SpatialAudioControl.KeepAlive = 4]
6. Receive loop:
     SPATIAL_AUDIO_DATA, payload[0] == BudGrv(32)
       → parse 4× int16 LE / 10000 → quaternion
     Other event ids: ignore for v1 (WearOn/Off optional UI signal later).
7. Tear-down:
     Stop keep-alive.
     Send: SPATIAL_AUDIO_CONTROL, payload [Detach = 1]
     Send: SET_SPATIAL_AUDIO, payload [0x00]
     RFCOMM close.
```

## Implementation notes for the JUCE plugin

- C++ shape:
  ```cpp
  class HeadTracker {
    virtual void start();
    virtual void stop();
    virtual bool isConnected();
    virtual std::optional<juce::Quaternion<float>> latestPose();  // bud-frame
    virtual void recentre();                                       // capture zero
  };
  class BudsSppTracker : HeadTracker { ... };  // .mm, uses IOBluetooth
  ```
- Producer thread:
  - Owns the RFCOMM channel and a re-framing parser (state machine: scan-for-SOM → read-header → read-N+3 bytes → verify EOM+CRC → emit).
  - Writes the latest quaternion to a `std::atomic<Quat>` (lock-free) or a triple-buffered struct.
  - Sends keep-alive on a `juce::Timer`.
- Audio thread:
  - Reads atomic, multiplies by calibration quat, optional one-pole SLERP smoother, pushes to `engine_set_listener_rotation`.
- macOS specifics:
  - `Bluetooth.mm` shows the working recipe; we can pretty much port it verbatim into `Source/Trackers/BudsSppTracker.mm`. License is GPLv3 → we'd be carrying the obligation; check before lifting verbatim. Re-writing from the API surface (it's all public Apple IOBluetooth calls) is the safe path.
  - `IOBluetoothRFCOMMChannelDelegate` callbacks land on the main run loop; bridge to JUCE thread.

## Open questions / verify empirically

- Header bit positions for `Type`/`IsFragment`: encoder uses `0x10`/`0x20`, decoder uses `0x1000`/`0x2000`. Looks like a bug or a deliberate asymmetry. Capture an outbound frame on the wire and confirm.
- Exact quaternion component order (`xyzw` vs `wxyz`).
- Bud-frame axis convention (need calibration step).
- Sample rate (~50 Hz? ~100 Hz?). Affects whether smoothing is needed.
- Whether keep-alive can be slower than 2 s. Lower rate → fewer wakeups.
- Behavior when only one bud is worn (BudGrv is from "BudGrv" — single-source; presumably whichever is primary).
