# MQRPII ESP32 Controller and LiDAR Safety Stop

## What it does

- Hosts an HTTP server on the local WiFi network that serves a phone-friendly directional control page (forward, back, left, right, stop).
- Translates button presses into 8-byte binary packets and sends them to the Teensy over a UART link.
- Listens for the Teensy's acknowledgement packets on the same link and prints them to the serial monitor (for debugging).
- Polls an RPLidar continuously, runs a cluster-detection algorithm on the points, and overrides any forward command with a STOP if it sees an obstacle in the danger zone.
- Locks the controller to a MAC address whitelist so a random device on the same WiFi can't drive the robot.

## Hardware

| Part | Notes |
|------|-------|
| Arduino Nano ESP32 | The main controller. Uses Serial1 for the LiDAR, Serial2 for the Teensy, USB Serial for the debug monitor. |
| RPLidar A1 | 360-degree 2D LiDAR. Mounted on top of the robot pointing forward. The 0-degree angle direction matters and is set by the physical mount. |
| Teensy 4.0  | Receives the binary motion command and runs the gait. |
| Phone, tablet, or laptop | Anything with a browser on the same WiFi network. |

### Pin map

```
ESP32 pin    Function
---------    --------
RX1 / 2      LiDAR RX
TX1 / 3      LiDAR TX
RX2 / 10     Teensy TX (data into ESP)
TX2 / 9      Teensy RX (data out to robot)
GPIO 8       Status LED
```

Both UARTs run at 115200 baud, 8N1.

## Communication protocol

The ESP and the Teensy exchange the same framed packet in both directions:

```
[0xAA] [0x55] [b0 b1] [b2 b3] [b4 b5] [checksum]
 hdr1   hdr2  int16_1  int16_2  int16_3   sum of 6 payload bytes
```

Three signed 16-bit integers packed big-endian, with a one-byte additive checksum over the payload. Same format both ways, so the parser code is symmetric.

The three values from the ESP are direction flags A, B, C with this encoding:

| Command   | A | B | C |
|-----------|---|---|---|
| FORWARDS  | 1 | 0 | 0 |
| LEFT      | 0 | 1 | 0 |
| RIGHT     | 0 | 0 | 1 |
| BACKWARDS | 1 | 1 | 1 |
| STOP      | 0 | 0 | 0 |

A zero packet is the stop command. The Teensy treats it as both "stand still" and "the LiDAR has just tripped", so it's safe to repeat.

## How it works

### Web UI and command flow

When a client connects, the ESP looks up its MAC by querying its own ARP table for the client's IP. If the MAC isn't on the whitelist, the connection is dropped. If it is, the ESP serves a single-page HTML controller with five buttons. Each button is a normal HTTP GET to a URL like `/FORWARDS`, which the ESP parses, converts to the right A/B/C flag triple, and sends to the Teensy as a binary packet.

The page is plain HTML/CSS with no JavaScript. It re-renders after each command so the displayed state always matches what was last sent. That keeps it usable from any browser and avoids needing to fight with WebSockets on the ESP.

### LiDAR obstacle detection

The LiDAR streams individual range-and-angle measurements as fast as the loop can read them. The obstacle logic does three things:

1. **Filter by distance and angle.** A point counts as a "hit" if its distance is in the danger zone for its angle. The front sector (45 to 135 degrees) uses a tighter cutoff than the sides, because forward motion is the only direction this is really protecting against. Points closer than `THRESHOLD_MM` are ignored — those are reflections from the robot's own body.

2. **Cluster the hits.** A single noisy reading shouldn't stop the robot. The algorithm counts consecutive hits and only triggers if the cluster is wide enough. A short gap of empty readings inside a cluster (up to `MAX_GAP`) is absorbed back in, because partially reflective objects show up as broken cluster.

3. **Scale the cluster size to distance.** A real-world object covers fewer LiDAR samples when it's far away than when it's close. `minPoints` is recomputed at the start of each cluster from the actual hit distance, so a small object close-up triggers a stop just as reliably as a larger one farther away. The formula assumes the LiDAR gives roughly one point per 1.3 degrees and targets a 100 mm-wide object.

4. **Debounce across full rotations.** The `startBit` flag from the LiDAR marks the start of a new 360-degree sweep. The stop stays latched until a full sweep completes *without* re-detecting the obstacle. This stops the robot from twitching between "stop" and "go" if a measurement flickers.

When the stop triggers, the ESP immediately sends a STOP packet to the Teensy. Only the FORWARDS button is gated by the LiDAR — LEFT, RIGHT, BACKWARDS, and STOP all go through even with the latch set, so the operator can strafe or reverse out of the obstacle.

### Security

There's a MAC whitelist and that's it. Anything on the network whose MAC isn't in `ALLOWED_MACS` gets disconnected before it can issue a command. There's no HTTPS, no login, no token. This is fine for a closed lab network with a known set of devices. It is not safe to expose to a network you don't control.

On the very first request from a new device, the ESP's ARP table sometimes hasn't cached the MAC yet, in which case the lookup returns empty. The default behaviour is to let that request through with a warning. There's a commented line in `loop()` that turns this into a hard reject if you'd rather be strict.

## Getting it running

1. Install the Arduino IDE with the Arduino ESP32 board package (Nano ESP32 selected).
2. Install the libraries below.
3. Wire the LiDAR to Serial1 (pins 2 and 3) and the Teensy to Serial2 (pins 9 and 10). Power the LiDAR from its own supply — the ESP can't source enough current.
4. **Open the sketch and edit the top of the file before flashing:**
   - Set `ssid` and `password` to your WiFi network.
   - Replace the two placeholder MAC addresses in `ALLOWED_MACS[]` with the MACs of the devices you want to allow. Uppercase, colon-separated, e.g. `"A0:B1:C2:D3:E4:F5"`. If you don't know a device's MAC, leave a placeholder in for now, flash the sketch, and watch the serial monitor — the ESP prints every connecting client's MAC on the first hit.
5. Flash and open the serial monitor at 115200. It prints the ESP's own IP address on boot. Open that IP in a browser on a whitelisted device, and the controller page should load.

### Dependencies

- `RPLidar` (https://github.com/robopeak/rplidar_arduino, works with the A1)
- `WiFi.h` (bundled with the ESP32 board package)
- `esp_wifi.h`, `lwip/etharp.h`, `lwip/ip_addr.h` (also bundled — used for the ARP-based MAC lookup)

## Tuning notes

The constants near the top of the file are the ones worth changing first if behaviour isn't right on your rig:

| Constant | What it controls |
|----------|------------------|
| `STOP_DISTANCE_FRONT_MM` | How close an object in front has to be before a stop triggers. |
| `STOP_DISTANCE_SIDE_MM`  | Same, for objects to the sides and rear. |
| `THRESHOLD_MM` | Anything closer than this is ignored as a self-reflection. Set it to just outside the LiDAR's view of your own chassis. |
| `MAX_GAP` | How many empty readings inside a cluster get tolerated. Higher means more permissive — useful for shiny or dark obstacles, but raises false-positive risk. |
| The `100.0f` in the `minPoints` formula | Target object width in mm. Make this smaller if you want to detect thinner obstacles, larger if you're getting false stops from clutter. |
| The `1.3f` in the same formula | Approximate degrees-per-sample of the LiDAR. Should be close enough for the A1; verify if you're using a different scanner. |

## Things to know before you trust it

- **The LiDAR angle origin depends on how the LiDAR is mounted.** The angle ranges in `getStopDistance()` assume 0 degrees points forward. If you bolt the LiDAR on rotated 90 degrees, the front quadrant won't be the front any more. The serial monitor prints the angle of any detected object, which is the fastest way to check.
- **The MAC whitelist will be empty as shipped.** The two MACs in the repo are placeholders. Without editing them, no device can connect.
- **The WiFi credentials are in plaintext.** Don't push the sketch back up to a public repo with your real SSID and password baked in.
- **First-connection MAC lookup can fail.** As described above, a brand new device can sneak through the first request. If that matters in your setup, switch to strict mode.
- **The Teensy acknowledgement is not used for control.** The RX state machine is there for debugging the serial link. If you build a closed-loop control flow that needs the robot's state, you'll need to extend `pollSerial2RX()` to do something with `r1`, `r2`, `r3` beyond printing them.

