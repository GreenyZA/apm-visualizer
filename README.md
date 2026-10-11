# APM 2.5.2 Visualizer

Live 3D telemetry visualizer for an **APM 2.5.2** flight controller. Built in Rust
with `macroquad`. Reads MAVLink v1 directly from a USB serial port (115200 baud),
renders craft attitude over a grid, and shows a telemetry panel (attitude, GPS,
battery, altitude, IMU, status).

## Quick start

```bash
cd /home/warpuppy/Projects/apm-visualizer
cargo build --release
/home/warpuppy/Projects/apm-visualizer/target/release/apm-visualizer
```

## Documentation

Full running manual: [docs/index.md](docs/index.md)

## Controls (summary)

- `WASD` move, `Space`/`Left Shift` up/down, Right-mouse drag to look
- Mouse wheel changes camera speed, `R` resets camera
- Top-left dropdown selects the serial port; **Refresh** re-scans ports

## Notes

- Serial port access requires membership in the `uucp` group.
- No MAVProxy or intermediate forwarder is needed.
- This is a visualization tool only; it does not command or configure the flight controller.
