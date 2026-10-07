# APM Visualiser - 3D FPS Camera + Multi-Board Support Plan

**Project:** apm-visualizer (Rust + macroquad 0.4 + mavlink)
**Date:** 2026-10-06
**Goal:** Extend the existing telemetry visualiser with:
- Dropdown to select between multiple connected APM boards (serial ports).
- Proper 3D floating craft model with clear front/rear/sides orientation.
- Free-fly FPS-style camera (mouse look + keyboard movement) around the craft.
- Keep existing telemetry panel + MAVLink parsing.
- Craft visual = simple rectangular box (cuboid) for clarity.

This plan is written for **any agent**, including less experienced ones. It is explicit, step-by-step, includes code skeletons, pitfalls, and verification steps. Follow it in order. Do not skip sections.

## 1. Current State Summary (Read This First)

- Main file: `src/main.rs` (single file ~440 lines)
- Uses `macroquad::prelude::*` for rendering (current "3D" is 2D projection of a wireframe cube + 2D telemetry panel overlaid).
- MAVLink parsing in a background thread (`serial_reader_thread`) writing to `Arc<Mutex<Telemetry>>`.
- Current craft viz: rotating wireframe cube with red points + axis labels. No real 3D camera.
- Serial: hardcoded to `/dev/ttyACM0:115200`.
- No UI widgets beyond text + lines.
- Cargo.toml has `macroquad = "0.4"`, `mavlink = { version = "0.18", features = ["dialect-ardupilotmega"] }`, and `serialport = "4"` (from previous work).

**Do not delete existing working code.** Extend it. Keep the telemetry panel functional.

## 2. High-Level Architecture

### 2.1 Components
- **Connection Manager** (new): Handles port discovery, dropdown state, connect/disconnect. Supports one active connection at a time (simple for v1).
- **MAVLink Reader Thread** (refactor existing): Restartable. Takes a port string. Signals stop via `Arc<AtomicBool>`.
- **3D Scene** (new major part):
  - `CraftModel`: Simple elongated rectangular box (cuboid) + orientation markers.
  - `FreeCamera`: FPS-style camera (position + yaw/pitch).
  - World: Craft at origin. Camera free in space.
- **Input System**: Keyboard (WASD + QE + Space/Shift) + mouse look (delta when RMB held or always grabbed).
- **UI Layer** (top bar):
  - Dropdown/combobox for available serial ports.
  - Connect/Disconnect button + current port status.
  - Keep existing telemetry panel on left or bottom.

### 2.2 Data Flow (Unchanged Core)
```
Serial Port → Background Thread (parse MAVLink) → Arc<Mutex<Telemetry>>
                                                         ↓
UI Loop (macroquad) ← reads Telemetry for attitude + panel
                  ↓
              Update Camera + Draw 3D Craft (rotated by roll/pitch/yaw)
```

### 2.3 Threading
- Main thread: macroquad event loop (input + draw).
- One worker thread for serial (same as today).
- Use `std::sync::atomic::AtomicBool` + `Arc` for "should_run" to cleanly stop/restart the reader when switching boards.

## 3. Detailed Requirements

### 3.1 Multi-Board Dropdown
- On startup and on "Refresh" button: call `serialport::available_ports()` and filter for likely APM ports (`/dev/ttyACM*`, `/dev/ttyUSB*`, `COM*` on Windows if needed).
- Display friendly names: e.g. `/dev/ttyACM0 (Arduino Mega 2560)` or just the path.
- Dropdown: clickable area at top-left. Click shows list. Click item selects + auto-connects (or separate Connect button).
- Visual: simple rectangle-based dropdown (no egui for minimal deps). Highlight hovered/selected.
- State: `selected_port: Option<String>`, `available_ports: Vec<String>`.
- When changing selection: stop old reader thread, start new one with the chosen port.
- Handle "no ports" or "device busy" gracefully (show in UI).

**Pitfall for weak agents:** Serial ports can disappear. Always refresh list before connecting. Use try/catch style error handling on open.

### 3.2 3D Craft Model (Rectangular Box)
- Replace (or enhance) current wireframe cube with a **simple 3D rectangular box** (length > width > height to look like a vehicle).
  - Length (X or Z): longest axis = "front to rear".
  - Use `draw_cube` with different scale or draw 6 separate faces with colors for clarity.
- Clear orientation cues (must be obvious even from any angle):
  - **Front**: Bright green or red nose/face + small forward arrow/cone or "N" (nose) text.
  - **Rear**: Blue or different color.
  - **Left/Right**: Subtle wing stubs or different side colors (e.g. left = cyan, right = magenta).
  - Optional: small "top" marker or "UP" arrow on top face.
  - Draw body axes (RGB lines) at center for debugging.
- The model **floats** at world origin (0,0,0).
- Rotation: Apply the telemetry `roll, pitch, yaw` (in radians) to the model transform.
  - macroquad order: usually pitch (X), roll (Z), yaw (Y). Test and document the exact order.
- Color: Main body gray or dark. Red accents on the "red object" points if wanted. User said "the red object" — keep some red vertices or make the whole thing have red highlights.

**Implementation tip:**
```rust
// Example craft draw (inside 3D camera block)
let craft_pos = vec3(0.0, 0.0, 0.0);
let rotation = Quat::from_euler(EulerRot::YXZ, yaw, pitch, roll); // tune order

// Draw elongated box
draw_cube(craft_pos, vec3(2.0, 0.5, 1.0), rotation, GRAY); // length=2 (front-rear), height=0.5, width=1

// Front marker (cone or small box)
let front_offset = rotation * vec3(1.2, 0.0, 0.0);
draw_sphere(craft_pos + front_offset, 0.15, None, RED);
```

Use `Quat` and `Vec3` from `macroquad::prelude`.

### 3.3 FPS-Style Camera Controls
- **Camera struct** (new, in main or separate module):
  ```rust
  struct FreeCamera {
      position: Vec3,
      yaw: f32,      // radians, left-right
      pitch: f32,    // radians, up-down (clamp -89..89 deg)
      speed: f32,
  }
  ```
- **Movement (keyboard, like FPS)**:
  - W/S: forward/back along look direction (use yaw+pitch to compute direction).
  - A/D: strafe left/right (perpendicular to look).
  - Space: up (world +Y), LeftShift: down.
  - Optional: Q/E for roll (rarely needed) or speed modifiers.
  - Hold Shift = faster movement.
- **Look (mouse)**:
  - Right mouse button held = look mode (or always when window focused).
  - Capture mouse delta (`mouse_delta` or track previous position).
  - Update yaw += delta_x * sensitivity.
  - Update pitch += delta_y * sensitivity (invert Y if needed).
  - Clamp pitch.
- **Camera in macroquad**:
  ```rust
  set_camera(&Camera3D {
      position: cam.position,
      target: cam.position + cam.forward(),  // or use yaw/pitch to compute target
      up: vec3(0.0, 1.0, 0.0),
      ..Default::default()
  });
  ```
- **Reset**: Key (e.g. R) to reset camera to a good viewing position (e.g. 5m behind and above craft).
- **Zoom/FOV**: Mouse wheel adjusts speed or slight FOV change.
- Make movement frame-rate independent (use `get_frame_time()`).

**Pitfall:** macroquad's 3D camera expects `target`. Compute forward vector:
```rust
fn forward(&self) -> Vec3 {
    let (sin_yaw, cos_yaw) = self.yaw.sin_cos();
    let (sin_pitch, cos_pitch) = self.pitch.sin_cos();
    vec3(cos_pitch * cos_yaw, sin_pitch, cos_pitch * sin_yaw)
}
```

### 3.4 UI Layout
- Top bar (always visible, 2D overlay):
  - Left: "Board:" + dropdown button.
  - Dropdown list appears below when open.
  - "Refresh Ports" button.
  - Status: "Connected to /dev/ttyACM0" or "No connection" (color green/red).
  - Right: FPS counter (already exists), "Reset Camera" button.
- Telemetry panel: Keep on left side (current `draw_telemetry_panel`). Make it semi-transparent.
- 3D view: Full background, behind the 2D overlays.
- When dropdown open, still draw the 3D scene.

**Simple dropdown implementation (no extra crates):**
- Store `show_dropdown: bool`, `dropdown_rect`.
- On click in header area → toggle.
- Draw list items as rectangles below.
- Check mouse clicks against item rects.

### 3.5 Other Requirements
- Keep all existing telemetry fields updating.
- Graceful handling when no board / port disappears (show "NO DATA", stop thread cleanly).
- The craft should always be visible by default (start camera behind it).
- Performance: 3D is cheap. Don't overdraw.
- Config: Hardcode sensitivity = 0.003, initial camera position = vec3(0, 2, -5) or similar.

## 4. Step-by-Step Implementation (Follow Exactly)

**Phase 0: Preparation (5-10 min)**
1. Read entire current `src/main.rs` and `Cargo.toml`.
2. `cargo check` to confirm clean build.
3. Create backup: `cp src/main.rs src/main.rs.bak`

**Phase 1: Add Dependencies (if needed)**
- No new crates required for core features (serialport already present).
- If you want nicer UI later, consider `egui-macroquad`, but **do not** for this plan unless user asks. Keep pure macroquad.

**Phase 2: Refactor Telemetry & Threading for Restartability**
1. Add to top:
   ```rust
   use std::sync::atomic::{AtomicBool, Ordering};
   use std::sync::Arc;
   ```
2. Add field to Telemetry or new struct if needed (no change required for now).
3. Create `fn start_reader_thread(port: String, telemetry: Arc<Mutex<Telemetry>>, running: Arc<AtomicBool>) -> std::thread::JoinHandle<()>`
4. Move current reader logic into this function. At top of thread: `while running.load(Ordering::Relaxed) { ... }`
5. On error or end: clean exit.
6. In `main()`, keep the existing `Arc<Mutex<Telemetry>>`.

**Phase 3: Port Discovery & Dropdown UI**
1. Add function:
   ```rust
   fn list_serial_ports() -> Vec<String> {
       serialport::available_ports()
           .unwrap_or_default()
           .into_iter()
           .filter(|p| p.port_name.contains("ttyACM") || p.port_name.contains("ttyUSB") || p.port_name.contains("COM"))
           .map(|p| p.port_name)
           .collect()
   }
   ```
2. In main loop state (before async fn main or inside):
   - `let mut available_ports: Vec<String> = vec![];`
   - `let mut selected_port: Option<String> = None;`
   - `let mut show_dropdown = false;`
   - `let mut reader_running = Arc::new(AtomicBool::new(false));`
   - `let mut current_reader: Option<std::thread::JoinHandle<()>> = None;`
3. Draw top bar at y=0 (use 2D draw calls, ignore 3D camera).
4. Implement click handling for dropdown items. On selection:
   - Stop current reader (`reader_running.store(false, ...)` + join).
   - Create new `running = Arc::new(AtomicBool::new(true));`
   - Spawn `start_reader_thread(new_port.clone(), telemetry.clone(), running.clone())`
   - Store handle and flag.

**Phase 4: Free Camera System**
1. Define struct outside main:
   ```rust
   #[derive(Clone, Copy)]
   struct FreeCamera {
       position: Vec3,
       yaw: f32,
       pitch: f32,
       speed: f32,
   }
   impl FreeCamera {
       fn new() -> Self { /* default behind craft */ }
       fn update(&mut self, dt: f32);
       fn forward(&self) -> Vec3 { ... }
       fn right(&self) -> Vec3 { ... }
   }
   ```
2. In main loop:
   - `let mut camera = FreeCamera::new();`
   - Handle keys with `is_key_down`.
   - Mouse: `let (mx, my) = mouse_position();` track previous, compute delta when RMB down (`is_mouse_button_down(MouseButton::Right)`).
   - Call `camera.update(get_frame_time());`
3. Before drawing 3D:
   ```rust
   set_camera(&Camera3D {
       position: camera.position,
       target: camera.position + camera.forward(),
       up: vec3(0., 1., 0.),
       fovy: 60.0_f32.to_radians(),
       ..Default::default()
   });
   ```

**Phase 5: 3D Craft Model**
1. Create helper `fn draw_craft(roll: f32, pitch: f32, yaw: f32)`
2. Inside:
   - Compute rotation quat or use `draw_cube` + manual face drawing for colors.
   - Draw main box (elongated on forward axis).
   - Draw front indicator (red cone/sphere or pyramid using `draw_triangle3d` or multiple cubes).
   - Draw rear differently.
   - Draw side markers (thin boxes).
   - Optionally draw small text labels using `draw_text` after projecting or use simple 3D lines.
3. Call `draw_craft(t.roll, t.pitch, t.yaw);` after setting 3D camera.
4. Add a very simple grid or floor for spatial reference: `draw_line_3d` calls for axes or grid.

**Phase 6: Integrate Everything**
1. In main draw loop (after `clear_background`):
   - Draw 3D (set camera, draw_craft, maybe some reference objects).
   - Switch back to 2D implicitly by using screen coords for panel + top bar.
   - Draw top dropdown bar.
   - Draw telemetry panel (existing function).
2. Handle port switching without crashing the reader (use the AtomicBool).
3. On first start, auto-select first available port and connect.

**Phase 7: Polish & Edge Cases**
- Clamp camera pitch.
- Add "Reset View" button that sets camera to (0, 3, -6) looking at origin.
- Show connection errors in UI (add error string to shared state or simple println for now).
- When no attitude data yet, show "Waiting for full telemetry..." in panel.
- Make sure yaw/pitch/roll from MAVLink are in radians (they are).
- Test orientation: arm the craft or use test values. Front should point "forward" in its local space.
- FPS camera should feel natural (test sensitivity ~0.002-0.005).

## 5. File Changes Summary

- `src/main.rs`: Major changes (add structs, functions, refactor main loop and thread starter).
- `Cargo.toml`: No changes expected.
- New (optional but recommended): `src/camera.rs`, `src/model.rs`, `src/ui.rs` for cleanliness. Start in main.rs if agent is weak, then refactor.

**Do not** touch `target/`.

## 6. Verification Steps (After Each Phase)

1. `cargo check` after every significant edit.
2. Run: `./target/debug/apm-visualizer` (or `cargo run`).
3. With one board: dropdown shows it, auto connects, data flows.
4. With two boards: switch dropdown → old stops, new starts, no crash.
5. Mouse + keys move camera around the box. Box rotates with live telemetry.
6. Front of box clearly different from rear.
7. Telemetry panel still updates correctly.
8. Wait 30+ seconds on a fresh boot — attitude and satellites should appear thanks to re-request logic (already present).

## 7. Potential Pitfalls (Warn the Next Agent)

- MAVLink roll/pitch/yaw sign/order — test with known values. Print them.
- Cursor grab / mouse delta in macroquad on Wayland/X11 can be tricky. Use raw delta tracking if `set_cursor_grab` doesn't work.
- Serial thread must be joined or the AtomicBool must be respected on shutdown.
- 3D drawing must happen **after** `set_camera`, before any 2D panel draws.
- macroquad 0.4 Euler angles and rotation order are YXZ or XYZ — verify by watching the model.
- Re-requesting streams too often can spam the old firmware — rate limit to 8s as in current code.
- On Linux, user must be in `dialout` or `uucp` group for serial access.
- Keep the existing `is_stale()` logic for the "NO DATA" status.

## 8. Suggested Order for Implementation (Weak Agent Safe)

1. Add camera struct + basic FPS movement (no 3D model yet).
2. Add simple 3D box at origin (no rotation first).
3. Wire telemetry rotation into the box.
4. Add orientation markers (front cone etc.).
5. Implement port list + dropdown UI (2D only).
6. Wire dropdown to restart the reader thread.
7. Add the re-request logic if not already perfect (it mostly is).
8. Polish camera feel + reset button.
9. Test with real hardware for 10+ minutes.

## 9. Future Extensions (Out of Scope for This Plan)

- Multiple simultaneous boards (tabs or split view).
- Load a real .obj model for the craft.
- egui for professional dropdown.
- Recording/playback of telemetry.
- Ground plane with grid.

---

**End of Plan.**

Next agent: Start by reading this file completely. Then follow Phase 0 → Phase 7 in order. Run `cargo check` frequently. When done, test thoroughly with the actual APM hardware before claiming success.

This plan is designed to be followed literally. Good luck.