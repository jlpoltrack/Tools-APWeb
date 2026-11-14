# DroneCAN Updater (web)

A minimal browser version of Mission Planner's DroneCAN firmware updater: pick a serial
SLCAN adapter, pick a node, pick a `.bin`, flash.

## Run

Hosted: **https://jlpoltrack.github.io/Tools-APWeb/dronecan-updater/** — pushes to `main` redeploy it.

Locally:

```sh
./serve.sh          # http://localhost:8000
```

Needs the Web Serial API and a secure context (GitHub Pages is HTTPS; `localhost` also
counts). That means Chromium desktop browsers — Chrome, Edge, Opera, Brave, Arc — or
Firefox 151+ on desktop, where Web Serial is gated behind an add-on that explains the
permission before the port picker appears (and is off by default under Enterprise
Policies). Safari and mobile browsers do not support it.

No WebUSB is involved: Web Serial talks to the port the OS CDC-ACM driver already
provides, so nothing has to claim the USB interface.

## Use

1. **Connect** — choose the flight controller's USB port. The picker is filtered to
   ArduPilot USB IDs (`0x1209:0x5740/0x5741` plus the Cube, Holybro, ThePeach, 3DR and
   VRBrain vendor IDs from hwdef). If your board isn't listed, press Connect again and
   every serial port is shown. Nothing needs to be configured
   on the FC: if the port is streaming MAVLink, the app sets `CAN_SLCAN_CPORT=1`,
   `CAN_P1_DRIVER=1`, `CAN_SLCAN_TIMOUT=0` and then blind-sets `CAN_SLCAN_SERNUM=0`, which
   hands the port over to SLCAN — Mission Planner's sequence, except for the timeout. If
   `CAN_SLCAN_CPORT` was 0 the change only applies after a reboot, so a **Reboot FC**
   button appears. SLCAN also auto-disables while armed. The port stays SLCAN until the FC
   reboots: ArduPilot 4.7 never reads MAVLink again on a port whose SLCAN session timed out,
   so a timeout would only stop reconnects from working. Reboot the FC to get MAVLink back.
2. **Nodes** — nodes appear as their `NodeStatus` broadcasts arrive; names/versions come
   from `GetNodeInfo`. Click a row to select it.
3. **Firmware** — choose the `.bin` and press *Update selected node*.

The node reboots into its bootloader, requests the image from us over
`uavcan.protocol.file.Read`, and reboots when done. The progress bar tracks bytes served.

## How it works

- `js/slcan.js` — Web Serial port + SLCAN framing: `T<id><dlc><data>\r` for classic frames,
  `D<id><dlc code><data>\r` for CAN FD (DLC code 0–F, up to 64 bytes).
- `js/dronecan.js` — DroneCAN/UAVCAN v0 CAN ID packing, tail bytes, multi-frame
  assembly, transfer CRC (CRC-16-CCITT seeded with the data type signature).
- `js/mavlink.js` — enough MAVLink v1/v2 to read a heartbeat and set parameters, so the
  FC can be flipped into SLCAN mode without touching Mission Planner first.
- `js/app.js` — node discovery, `BeginFirmwareUpdate`, and the `file.Read` server that
  feeds the bootloader 256-byte chunks. Each read is answered in the request's frame format:
  on a CAN FD bus a chunk is 5 FD frames instead of 38 classic ones. Our own requests stay
  classic, which every node accepts.

DroneCAN frames were verified byte-for-byte against `pydronecan`, MAVLink frames against `pymavlink`.

## Tests

```sh
test/run.sh                       # file.Read framing vs pydronecan, SLCAN FD lines
node test/hw-update.mjs /dev/cu.usbmodemXXXX firmware.bin <node id>   # real FC + node
```

`test/vectors.json` comes from `test/gen_vectors.py` (needs `pydronecan`). The hardware test runs
`app.js` under a stub page, with `test/serial_bridge.py` (pyserial) standing in for Web Serial;
`HW_TRACE=1` logs every file.Read request and its frame format.

## Limitations

- No dynamic node ID allocation server: the target must already have a node ID.
- One node at a time; no parameter editing, no node restart button.
