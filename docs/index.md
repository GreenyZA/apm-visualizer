---
title: "APM 2.5.2 Visualizer — Running Manual"
description: "Manual for building and running the APM 2.5.2 visualizer"
---

# APM 2.5.2 Visualizer — Running Manual

This manual covers how to build, run, and use the **APM 2.5.2 Visualizer** (Rust + macroquad).
It is a desktop GUI that reads MAVLink v1 telemetry directly from an APM 2.5.2 flight
controller over USB serial and renders the craft attitude plus a live telemetry panel.

## 1. Location

Project root:

```text
/home/warpuppy/Projects/apm-visualizer
```

Latest built binary:

```text
/home/warpuppy/Projects/apm-visualizer/target/release/apm-visualizer
```

Source entry point:

```text
/home/warpuppy/Projects/apm-visualizer/src/main.rs
```

## 2. Prerequisites

- Rust toolchain (`cargo`, `rustc`) installed (see `~/.cargo/bin` on this machine).
- APM 2.5.2 flight controller connected over USB (`/dev/ttyACM0` expected).
- Serial port access. The user must be in the `uucp` group (or run with appropriate permissions).
  Verify with:

  ```bash
  id | grep uucp
  ls -l /dev/ttyACM0
  ```

- No MAVProxy is required. The visualizer reads the serial port directly at 115200 baud.

## 3. Build (from source)

```bash
cd /home/warpuppy/Projects/apm-visualizer
cargo build --release
```

This produces the latest binary at:

```text
target/release/apm-visualizer
```

If you only changed source, rebuild with the same command. The release binary is the one to run for normal use.

## 4. Run

### Recommended (run the latest release binary)

```bash
/home/warpuppy/Projects/apm-visualizer/target/release/apm-visualizer
```

### Via Cargo (builds + runs, useful while developing)

```bash
cd /home/warpuppy/Projects/apm-visualizer
cargo run --release
```

### Optional helper script

A launcher script exists at:

```text
/home/warpuppy/run_visualizer.sh
```

If the helper path is wrong or stale, use the direct binary command above.

## 5. What to expect

- A window titled **APM 2.5.2 Visualizer** opens.
- The GUI connects to the selected serial port at 115200 baud.
- Telemetry panel shows:
  - Status (CONNECTED / NO DATA, stale after 3 seconds)
  - Mode, Armed state, last status text
  - Attitude (roll/pitch/yaw)
  - GPS (satellites, fix type, lat/lon/alt)
  - Battery (voltage/current/remaining)
  - Relative altitude
  - IMU (acc/gyro)
- A 3D craft model mirrors roll/pitch/yaw over a grid.

## 6. Controls

| Input | Action |
|------|--------|
| `W` / `S` | Move camera forward / back |
| `A` / `D` | Move camera left / right |
| `Space` | Move camera up |
| `Left Shift` | Move camera down |
| Right mouse button (hold) | Look around |
| Mouse wheel | Adjust camera speed (0.5–20.0) |
| `R` | Reset camera |
| Port dropdown (top-left) | Select serial port |
| **Refresh** button | Re-scan serial ports |

## 7. Port selection

On startup the program lists ports matching `ttyACM`, `ttyUSB`, or `COM`.
The first available port is selected automatically and a reader thread starts.

To switch ports:

1. Click the port dropdown.
2. Select a port from the list.
3. The old reader thread is stopped and a new one starts on the selected port.

If no ports are found, click **Refresh** after plugging in the flight controller.

## 8. Data-flow notes

- The program sends `REQUEST_DATA_STREAM` messages on startup and periodically
  re-requests streams if attitude/GPS have not arrived.
- Expected MAVLink messages parsed:
  `HEARTBEAT`, `ATTITUDE`, `GPS_RAW_INT`, `SYS_STATUS`,
  `GLOBAL_POSITION_INT`, `RAW_IMU`, `STATUSTEXT`.
- Telemetry goes stale after 3 seconds without updates.

## 9. Troubleshooting

### No serial port appears
- Confirm the board is connected: `ls /dev/ttyACM* /dev/ttyUSB*`
- Replug the board, then click **Refresh**.
- Ensure your user is in the `uucp` group, then log out/in.

### "Failed to open serial ..." in console
- Another process may hold the port (check `fuser /dev/ttyACM0`).
- Permissions may be wrong; verify group membership.
- Try a different USB cable/port.

### Telemetry shows NO DATA / stale
- Confirm the flight controller is powered and running firmware.
- Check console for MAVLink receive counts.
- Re-select the port in the dropdown.
- Some firmwares need data streams enabled; the visualizer requests them automatically.

### Window opens but nothing renders / GPU issues
- This is a macroquad (OpenGL) app; run under an X11/Wayland session.
- Check `DISPLAY`/`WAYLAND_DISPLAY` in your environment.

### Rebuild after changes
```bash
cd /home/warpuppy/Projects/apm-visualizer
cargo build --release
```

## 10. Development tips

- Use `cargo run --release` for quick iteration.
- Use `cargo build` (debug) for faster compiles during development; binary is
  `target/debug/apm-visualizer`.
- Source file is `src/main.rs` (single-file project).

## 11. Safety note

This tool only visualizes telemetry. It does **not** arm, configure, or command the
flight controller. Do not rely on it as a primary flight-safety instrument.