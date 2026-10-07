use macroquad::prelude::*;
use mavlink::dialects::ardupilotmega::MavMessage;
use mavlink::Message;
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};
use serialport::SerialPort;

#[derive(Debug, Clone)]
struct Telemetry {
    // Attitude (radians)
    roll: f32,
    pitch: f32,
    yaw: f32,
    rollspeed: f32,
    pitchspeed: f32,
    yawspeed: f32,
    // GPS
    lat: f64,
    lon: f64,
    gps_alt: f64,
    satellites: u8,
    fix_type: u8,
    // Battery
    voltage: f32,
    current: f32,
    battery_remaining: i8,
    // Altitude
    relative_alt: f64,
    // IMU
    xacc: f32,
    yacc: f32,
    zacc: f32,
    xgyro: f32,
    ygyro: f32,
    zgyro: f32,
    // System
    armed: bool,
    mode: String,
    last_status: String,
    // Timing
    last_update: Instant,
}

impl Default for Telemetry {
    fn default() -> Self {
        Self {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            rollspeed: 0.0,
            pitchspeed: 0.0,
            yawspeed: 0.0,
            lat: 0.0,
            lon: 0.0,
            gps_alt: 0.0,
            satellites: 0,
            fix_type: 0,
            voltage: 0.0,
            current: 0.0,
            battery_remaining: 0,
            relative_alt: 0.0,
            xacc: 0.0,
            yacc: 0.0,
            zacc: 0.0,
            xgyro: 0.0,
            ygyro: 0.0,
            zgyro: 0.0,
            armed: false,
            mode: "UNKNOWN".to_string(),
            last_status: "".to_string(),
            last_update: Instant::now(),
        }
    }
}

impl Telemetry {
    fn new() -> Self {
        Self::default()
    }

    fn is_stale(&self) -> bool {
        self.last_update.elapsed() > Duration::from_secs(3)
    }
}

/// Send a REQUEST_DATA_STREAM to the old ArduPilot firmware.
fn request_data_stream(port: &mut dyn SerialPort, req_stream_id: u8, rate: u16) -> std::io::Result<()> {
    let msg = MavMessage::REQUEST_DATA_STREAM(
        mavlink::dialects::ardupilotmega::REQUEST_DATA_STREAM_DATA {
            target_system: 1,
            target_component: 1,
            req_stream_id,
            req_message_rate: rate,
            start_stop: 1,
        },
    );

    let header = mavlink::MavHeader {
        sequence: 0,
        system_id: 255,
        component_id: 0,
    };

    let mut buf = Vec::new();
    if let Err(e) = mavlink::write_v1_msg(&mut buf, header, &msg) {
        eprintln!("[MAVLINK] Failed to serialize request: {:?}", e);
        return Ok(());
    }

    port.write_all(&buf)?;
    let _ = port.flush();
    Ok(())
}

fn serial_reader_thread(port_name: String, baud: u32, telemetry: Arc<Mutex<Telemetry>>, running: Arc<AtomicBool>) {
    let port = match serialport::new(&port_name, baud)
        .timeout(Duration::from_millis(200))
        .open()
    {
        Ok(p) => {
            println!("[MAVLINK] Opened raw serial {} @ {} baud", port_name, baud);
            p
        }
        Err(e) => {
            eprintln!("[MAVLINK] Failed to open serial {}: {}", port_name, e);
            return;
        }
    };

    let mut port: Box<dyn SerialPort> = port;

    println!("[MAVLINK] Requesting data streams from APM (ALL + attitude + GPS)...");
    let _ = request_data_stream(&mut *port, 0, 5);
    let _ = request_data_stream(&mut *port, 1, 5);
    let _ = request_data_stream(&mut *port, 2, 5);
    let _ = request_data_stream(&mut *port, 6, 5);
    let _ = request_data_stream(&mut *port, 10, 5);

    let mut buffer: Vec<u8> = Vec::with_capacity(4096);
    let mut msg_count = 0u64;
    let mut last_request_time = Instant::now();
    let mut has_attitude = false;
    let mut has_gps = false;

    while running.load(Ordering::Relaxed) {
        let mut chunk = [0u8; 512];
        match port.read(&mut chunk) {
            Ok(n) if n > 0 => {
                buffer.extend_from_slice(&chunk[..n]);
            }
            Ok(_) => {
                thread::sleep(Duration::from_millis(5));
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                thread::sleep(Duration::from_millis(5));
            }
            Err(e) => {
                eprintln!("[MAVLINK] read error: {}", e);
                thread::sleep(Duration::from_millis(100));
            }
        }

        while !buffer.is_empty() && buffer[0] != 0xfe && buffer[0] != 0xfd {
            buffer.remove(0);
        }

        if buffer.is_empty() {
            continue;
        }

        let mut consumed = 0;
        while consumed < buffer.len() {
            let start = consumed;
            if start >= buffer.len() { break; }
            let magic = buffer[start];
            if magic != 0xfe { consumed = start + 1; continue; }

            if start + 8 > buffer.len() { break; }
            let payload_len = buffer[start + 1] as usize;
            let total = 6 + payload_len + 2;
            if start + total > buffer.len() { break; }

            let slice = &buffer[start..start + total];
            let cursor = std::io::Cursor::new(slice);
            let mut pr = mavlink::peek_reader::PeekReader::new(cursor);
            match mavlink::read_v1_msg::<MavMessage, _>(&mut pr) {
                Ok((_header, msg)) => {
                    msg_count += 1;
                    if msg_count % 20 == 0 {
                        println!("[MAVLINK] Received {} msgs, latest: {}", msg_count, msg.message_name());
                    }

                    let mut t = telemetry.lock().unwrap();
                    match msg {
                        MavMessage::ATTITUDE(a) => {
                            t.roll = a.roll; t.pitch = a.pitch; t.yaw = a.yaw;
                            t.rollspeed = a.rollspeed; t.pitchspeed = a.pitchspeed; t.yawspeed = a.yawspeed;
                            t.last_update = Instant::now();
                            has_attitude = true;
                        }
                        MavMessage::GPS_RAW_INT(g) => {
                            t.lat = g.lat as f64 / 1e7; t.lon = g.lon as f64 / 1e7;
                            t.gps_alt = g.alt as f64 / 1000.0;
                            t.satellites = g.satellites_visible; t.fix_type = g.fix_type as u8;
                            t.last_update = Instant::now();
                            has_gps = true;
                        }
                        MavMessage::SYS_STATUS(s) => {
                            t.voltage = s.voltage_battery as f32 / 1000.0;
                            t.current = s.current_battery as f32 / 100.0;
                            t.battery_remaining = s.battery_remaining;
                            t.last_update = Instant::now();
                        }
                        MavMessage::GLOBAL_POSITION_INT(g) => {
                            t.relative_alt = g.relative_alt as f64 / 1000.0;
                            t.last_update = Instant::now();
                        }
                        MavMessage::RAW_IMU(i) => {
                            t.xacc = i.xacc as f32 / 1000.0; t.yacc = i.yacc as f32 / 1000.0; t.zacc = i.zacc as f32 / 1000.0;
                            t.xgyro = i.xgyro as f32 / 1000.0; t.ygyro = i.ygyro as f32 / 1000.0; t.zgyro = i.zgyro as f32 / 1000.0;
                            t.last_update = Instant::now();
                        }
                        MavMessage::HEARTBEAT(h) => {
                            t.armed = (h.base_mode & mavlink::dialects::ardupilotmega::MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED).bits() != 0;
                            t.mode = match h.custom_mode {
                                0 => "STABILIZE".to_string(), 2 => "ALT_HOLD".to_string(), 3 => "AUTO".to_string(),
                                4 => "GUIDED".to_string(), 5 => "LOITER".to_string(), 6 => "RTL".to_string(),
                                7 => "CIRCLE".to_string(), 9 => "LAND".to_string(), 13 => "SPORT".to_string(),
                                17 => "POSHOLD".to_string(), _ => format!("MODE_{}", h.custom_mode),
                            };
                            t.last_update = Instant::now();

                            if (!has_attitude || !has_gps) && last_request_time.elapsed() > Duration::from_secs(8) {
                                println!("[MAVLINK] Re-requesting data streams (APM still initialising)...");
                                let _ = request_data_stream(&mut *port, 0, 5);
                                let _ = request_data_stream(&mut *port, 2, 5);
                                let _ = request_data_stream(&mut *port, 10, 5);
                                last_request_time = Instant::now();
                            }
                        }
                        MavMessage::STATUSTEXT(s) => {
                            let txt: String = s.text.iter().filter(|&&c| c != 0).map(|&c| c as char).collect();
                            if !txt.trim().is_empty() {
                                println!("[MAVLINK] STATUS: {}", txt.trim());
                                t.last_status = txt.trim().to_string();
                                t.last_update = Instant::now();
                            }
                        }
                        _ => {}
                    }
                    consumed = start + total;
                    continue;
                }
                Err(_) => {
                    consumed = start + 1;
                    continue;
                }
            }
        }

        if consumed > 0 {
            buffer.drain(0..consumed);
        }
        if buffer.len() > 8192 {
            buffer.drain(0..(buffer.len() - 4096));
        }

        thread::sleep(Duration::from_millis(2));
    }

    println!("[MAVLINK] Reader thread for {} exiting.", port_name);
}

// ─── Free Camera ─────────────────────────────────────────────────────────────

#[derive(Clone, Copy)]
struct FreeCamera {
    position: Vec3,
    yaw: f32,
    pitch: f32,
    speed: f32,
}

impl FreeCamera {
    fn new() -> Self {
        Self {
            position: vec3(0.0, 2.0, -5.0),
            yaw: 0.0,
            pitch: -0.3,
            speed: 3.0,
        }
    }

    fn reset(&mut self) {
        self.position = vec3(0.0, 2.0, -5.0);
        self.yaw = 0.0;
        self.pitch = -0.3;
    }

    fn forward(&self) -> Vec3 {
        let (sin_yaw, cos_yaw) = self.yaw.sin_cos();
        let (sin_pitch, cos_pitch) = self.pitch.sin_cos();
        vec3(cos_pitch * cos_yaw, sin_pitch, cos_pitch * sin_yaw)
    }

    fn right(&self) -> Vec3 {
        let (sin_yaw, cos_yaw) = self.yaw.sin_cos();
        vec3(cos_yaw, 0.0, -sin_yaw)
    }

    fn update(&mut self, dt: f32, prev_mouse: Vec2) {
        let forward = self.forward();
        let right = self.right();
        let up = vec3(0.0, 1.0, 0.0);

        let mut move_dir = Vec3::ZERO;

        if is_key_down(KeyCode::W) { move_dir += forward; }
        if is_key_down(KeyCode::S) { move_dir -= forward; }
        if is_key_down(KeyCode::D) { move_dir += right; }
        if is_key_down(KeyCode::A) { move_dir -= right; }
        if is_key_down(KeyCode::Space) { move_dir += up; }
        if is_key_down(KeyCode::LeftShift) { move_dir -= up; }

        if move_dir != Vec3::ZERO {
            self.position += move_dir.normalize() * self.speed * dt;
        }

        // Mouse look (RMB held)
        if is_mouse_button_down(MouseButton::Right) {
            let (mx, my) = mouse_position();
            let dx = mx - prev_mouse.x;
            let dy = my - prev_mouse.y;
            self.yaw += dx * 0.003;
            self.pitch -= dy * 0.003;
            self.pitch = self.pitch.clamp(-1.55, 1.55);
        }

        // Mouse wheel = speed
        let (_, wheel_y) = mouse_wheel();
        if wheel_y != 0.0 {
            self.speed = (self.speed + wheel_y * 0.5).clamp(0.5, 20.0);
        }

        // Reset camera
        if is_key_pressed(KeyCode::R) {
            self.reset();
        }
    }
}

// ─── 3D Craft Model ──────────────────────────────────────────────────────────

fn draw_craft(roll: f32, pitch: f32, yaw: f32) {
    let craft_pos = vec3(0.0, 0.0, 0.0);
    let rotation = Quat::from_euler(EulerRot::YXZ, yaw, pitch, roll);

    // Helper: draw a rotated box as wireframe edges
    let draw_rotated_box = |center: Vec3, size: Vec3, rot: Quat, color: Color| {
        let half = size * 0.5;
        let vertices = [
            vec3(-half.x, -half.y, -half.z),
            vec3( half.x, -half.y, -half.z),
            vec3( half.x,  half.y, -half.z),
            vec3(-half.x,  half.y, -half.z),
            vec3(-half.x, -half.y,  half.z),
            vec3( half.x, -half.y,  half.z),
            vec3( half.x,  half.y,  half.z),
            vec3(-half.x,  half.y,  half.z),
        ];
        let rotated: Vec<Vec3> = vertices.iter().map(|v| center + rot * *v).collect();
        let edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ];
        for (a, b) in edges.iter() {
            draw_line_3d(rotated[*a], rotated[*b], color);
        }
    };

    // Main body: elongated box (length=2, height=0.5, width=1)
    draw_rotated_box(craft_pos, vec3(2.0, 0.5, 1.0), rotation, Color::new(0.5, 0.5, 0.55, 1.0));

    // Front marker: red sphere at nose
    let front_offset = rotation * vec3(1.2, 0.0, 0.0);
    draw_sphere(craft_pos + front_offset, 0.15, None, RED);

    // Rear marker: blue sphere at tail
    let rear_offset = rotation * vec3(-1.2, 0.0, 0.0);
    draw_sphere(craft_pos + rear_offset, 0.12, None, BLUE);

    // Left wing: cyan thin box
    let left_offset = rotation * vec3(0.0, 0.0, 0.6);
    draw_rotated_box(craft_pos + left_offset, vec3(0.8, 0.05, 0.3), rotation, Color::new(0.0, 0.8, 0.8, 1.0));

    // Right wing: magenta thin box
    let right_offset = rotation * vec3(0.0, 0.0, -0.6);
    draw_rotated_box(craft_pos + right_offset, vec3(0.8, 0.05, 0.3), rotation, Color::new(0.8, 0.0, 0.8, 1.0));

    // Top marker: green small sphere on top
    let top_offset = rotation * vec3(0.0, 0.35, 0.0);
    draw_sphere(craft_pos + top_offset, 0.08, None, GREEN);

    // Body axes (RGB lines at center)
    let axis_len = 1.5;
    let x_end = craft_pos + rotation * vec3(axis_len, 0.0, 0.0);
    let y_end = craft_pos + rotation * vec3(0.0, axis_len, 0.0);
    let z_end = craft_pos + rotation * vec3(0.0, 0.0, axis_len);
    draw_line_3d(craft_pos, x_end, RED);
    draw_line_3d(craft_pos, y_end, GREEN);
    draw_line_3d(craft_pos, z_end, BLUE);
}

fn draw_grid() {
    let grid_size = 10;
    let step = 1.0;
    let color = Color::new(0.15, 0.15, 0.2, 0.5);

    for i in -grid_size..=grid_size {
        let f = i as f32 * step;
        draw_line_3d(vec3(f, -1.0, -grid_size as f32 * step), vec3(f, -1.0, grid_size as f32 * step), color);
        draw_line_3d(vec3(-grid_size as f32 * step, -1.0, f), vec3(grid_size as f32 * step, -1.0, f), color);
    }
}

// ─── Telemetry Panel ─────────────────────────────────────────────────────────

fn draw_telemetry_panel(t: &Telemetry) {
    let panel_x = 10.0;
    let panel_y = 50.0;
    let line_height = 22.0;
    let mut y = panel_y;

    let panel_w = 320.0;
    let panel_h = 460.0;
    draw_rectangle(panel_x, panel_y, panel_w, panel_h, Color::new(0.0, 0.0, 0.0, 0.7));
    draw_rectangle_lines(panel_x, panel_y, panel_w, panel_h, 2.0, Color::new(0.2, 0.8, 1.0, 0.8));

    draw_text("APM 2.5.2 TELEMETRY", panel_x + 10.0, y + 20.0, 24.0, Color::new(0.2, 0.8, 1.0, 1.0));
    y += 35.0;

    let status_color = if t.is_stale() { RED } else { GREEN };
    let status_text = if t.is_stale() { "NO DATA" } else { "CONNECTED" };
    draw_text(&format!("Status: {}", status_text), panel_x + 10.0, y, 18.0, status_color);
    y += line_height;

    draw_text(&format!("Mode: {}", t.mode), panel_x + 10.0, y, 18.0, Color::new(1.0, 0.8, 0.2, 1.0));
    y += line_height;

    let armed_text = if t.armed { "ARMED" } else { "DISARMED" };
    let armed_color = if t.armed { RED } else { GREEN };
    draw_text(&format!("Armed: {}", armed_text), panel_x + 10.0, y, 18.0, armed_color);
    y += line_height;

    if !t.last_status.is_empty() {
        draw_text(&format!("Status: {}", t.last_status), panel_x + 10.0, y, 14.0, Color::new(0.8, 0.8, 0.2, 1.0));
        y += line_height;
    }
    y += 5.0;

    draw_text("--- ATTITUDE ---", panel_x + 10.0, y, 16.0, Color::new(0.5, 0.5, 0.5, 1.0));
    y += line_height;
    draw_text(&format!("Roll:  {:>8.2} deg", t.roll.to_degrees()), panel_x + 10.0, y, 16.0, WHITE);
    y += line_height;
    draw_text(&format!("Pitch: {:>8.2} deg", t.pitch.to_degrees()), panel_x + 10.0, y, 16.0, WHITE);
    y += line_height;
    draw_text(&format!("Yaw:   {:>8.2} deg", t.yaw.to_degrees()), panel_x + 10.0, y, 16.0, WHITE);
    y += line_height + 10.0;

    draw_text("--- GPS ---", panel_x + 10.0, y, 16.0, Color::new(0.5, 0.5, 0.5, 1.0));
    y += line_height;
    draw_text(&format!("Satellites: {}", t.satellites), panel_x + 10.0, y, 16.0, WHITE);
    y += line_height;
    draw_text(&format!("Fix Type:   {}", t.fix_type), panel_x + 10.0, y, 16.0, WHITE);
    y += line_height;
    draw_text(&format!("Lat: {:.6}", t.lat), panel_x + 10.0, y, 16.0, WHITE);
    y += line_height;
    draw_text(&format!("Lon: {:.6}", t.lon), panel_x + 10.0, y, 16.0, WHITE);
    y += line_height;
    draw_text(&format!("GPS Alt: {:.1} m", t.gps_alt), panel_x + 10.0, y, 16.0, WHITE);
    y += line_height + 10.0;

    draw_text("--- BATTERY ---", panel_x + 10.0, y, 16.0, Color::new(0.5, 0.5, 0.5, 1.0));
    y += line_height;
    draw_text(&format!("Voltage: {:.2} V", t.voltage), panel_x + 10.0, y, 16.0, WHITE);
    y += line_height;
    draw_text(&format!("Current: {:.2} A", t.current), panel_x + 10.0, y, 16.0, WHITE);
    y += line_height;
    draw_text(&format!("Remaining: {}%", t.battery_remaining), panel_x + 10.0, y, 16.0, WHITE);
    y += line_height + 10.0;

    draw_text("--- ALTITUDE ---", panel_x + 10.0, y, 16.0, Color::new(0.5, 0.5, 0.5, 1.0));
    y += line_height;
    draw_text(&format!("Relative: {:.1} m", t.relative_alt), panel_x + 10.0, y, 16.0, WHITE);
    y += line_height + 10.0;

    draw_text("--- IMU ---", panel_x + 10.0, y, 16.0, Color::new(0.5, 0.5, 0.5, 1.0));
    y += line_height;
    draw_text(&format!("Acc: {:.2} {:.2} {:.2} g", t.xacc, t.yacc, t.zacc), panel_x + 10.0, y, 14.0, WHITE);
    y += line_height;
    draw_text(&format!("Gyr: {:.2} {:.2} {:.2} rad/s", t.xgyro, t.ygyro, t.zgyro), panel_x + 10.0, y, 14.0, WHITE);
}

// ─── Port Discovery ──────────────────────────────────────────────────────────

fn list_serial_ports() -> Vec<String> {
    serialport::available_ports()
        .unwrap_or_default()
        .into_iter()
        .filter(|p| {
            let name = &p.port_name;
            name.contains("ttyACM") || name.contains("ttyUSB") || name.contains("COM")
        })
        .map(|p| p.port_name)
        .collect()
}

// ─── Dropdown UI ─────────────────────────────────────────────────────────────

struct DropdownState {
    show: bool,
    selected: Option<String>,
    ports: Vec<String>,
    rect: Rect,
}

impl DropdownState {
    fn new() -> Self {
        Self {
            show: false,
            selected: None,
            ports: vec![],
            rect: Rect::new(10.0, 10.0, 200.0, 30.0),
        }
    }

    fn update(&mut self) {
        let (mx, my) = mouse_position();
        let header = self.rect;

        if is_mouse_button_pressed(MouseButton::Left) {
            if mx >= header.x && mx <= header.x + header.w && my >= header.y && my <= header.y + header.h {
                self.show = !self.show;
            } else if self.show {
                let item_h = 25.0;
                for (i, _port) in self.ports.iter().enumerate() {
                    let item_y = header.y + header.h + i as f32 * item_h;
                    if mx >= header.x && mx <= header.x + header.w && my >= item_y && my <= item_y + item_h {
                        self.selected = Some(self.ports[i].clone());
                        self.show = false;
                        break;
                    }
                }
                if self.show {
                    self.show = false;
                }
            }
        }
    }

    fn draw(&self) {
        let header = self.rect;

        let header_color = if self.show {
            Color::new(0.2, 0.4, 0.6, 1.0)
        } else {
            Color::new(0.15, 0.15, 0.2, 1.0)
        };
        draw_rectangle(header.x, header.y, header.w, header.h, header_color);
        draw_rectangle_lines(header.x, header.y, header.w, header.h, 1.0, Color::new(0.3, 0.3, 0.4, 1.0));

        let label = match &self.selected {
            Some(p) => p.clone(),
            None => "Select Port...".to_string(),
        };
        draw_text(&label, header.x + 8.0, header.y + 20.0, 16.0, WHITE);

        let arrow_x = header.x + header.w - 20.0;
        let arrow_y = header.y + 10.0;
        if self.show {
            draw_triangle(
                vec2(arrow_x, arrow_y + 10.0),
                vec2(arrow_x + 10.0, arrow_y + 10.0),
                vec2(arrow_x + 5.0, arrow_y),
                WHITE,
            );
        } else {
            draw_triangle(
                vec2(arrow_x, arrow_y),
                vec2(arrow_x + 10.0, arrow_y),
                vec2(arrow_x + 5.0, arrow_y + 10.0),
                WHITE,
            );
        }

        if self.show {
            let item_h = 25.0;
            for (i, port) in self.ports.iter().enumerate() {
                let item_y = header.y + header.h + i as f32 * item_h;
                let is_hovered = {
                    let (mx, my) = mouse_position();
                    mx >= header.x && mx <= header.x + header.w && my >= item_y && my <= item_y + item_h
                };
                let is_selected = self.selected.as_deref() == Some(port.as_str());

                let bg = if is_selected {
                    Color::new(0.2, 0.5, 0.3, 1.0)
                } else if is_hovered {
                    Color::new(0.25, 0.25, 0.35, 1.0)
                } else {
                    Color::new(0.1, 0.1, 0.15, 1.0)
                };
                draw_rectangle(header.x, item_y, header.w, item_h, bg);
                draw_rectangle_lines(header.x, item_y, header.w, item_h, 1.0, Color::new(0.3, 0.3, 0.4, 0.5));
                draw_text(port, header.x + 8.0, item_y + 17.0, 14.0, WHITE);
            }
        }
    }
}

// ─── Main ────────────────────────────────────────────────────────────────────

#[macroquad::main("APM 2.5.2 Visualizer")]
async fn main() {
    let telemetry = Arc::new(Mutex::new(Telemetry::new()));
    let telemetry_clone = Arc::clone(&telemetry);

    let mut running = Arc::new(AtomicBool::new(true));
    let mut current_reader: Option<std::thread::JoinHandle<()>> = None;

    let mut available_ports = list_serial_ports();
    let mut selected_port: Option<String> = available_ports.first().cloned();

    let mut dropdown = DropdownState::new();
    dropdown.ports = available_ports.clone();
    dropdown.selected = selected_port.clone();

    let mut camera = FreeCamera::new();
    let mut last_frame = Instant::now();
    let mut prev_mouse = vec2(0.0, 0.0);

    // Start initial reader thread
    if let Some(ref port) = selected_port {
        running = Arc::new(AtomicBool::new(true));
        let port_clone = port.clone();
        let telemetry_thread = telemetry_clone.clone();
        let running_thread = running.clone();
        current_reader = Some(std::thread::spawn(move || {
            serial_reader_thread(port_clone, 115200, telemetry_thread, running_thread);
        }));
    }

    loop {
        let dt = last_frame.elapsed().as_secs_f32();
        last_frame = Instant::now();

        // Update mouse position tracking
        let (mx, my) = mouse_position();
        let current_mouse = vec2(mx, my);

        // Update dropdown
        dropdown.ports = available_ports.clone();
        dropdown.update();

        // Handle port change
        if dropdown.selected != selected_port {
            running.store(false, Ordering::Relaxed);
            if let Some(handle) = current_reader.take() {
                let _ = handle.join();
            }

            selected_port = dropdown.selected.clone();

            if let Some(ref port) = selected_port {
                running = Arc::new(AtomicBool::new(true));
                let port_clone = port.clone();
                let telemetry_thread = telemetry_clone.clone();
                let running_thread = running.clone();
                current_reader = Some(std::thread::spawn(move || {
                    serial_reader_thread(port_clone, 115200, telemetry_thread, running_thread);
                }));
            }
        }

        // Update camera
        camera.update(dt, prev_mouse);
        prev_mouse = current_mouse;

        // ─── Draw ───
        clear_background(Color::new(0.05, 0.05, 0.1, 1.0));

        // Set 3D camera
        set_camera(&Camera3D {
            position: camera.position,
            target: camera.position + camera.forward(),
            up: vec3(0.0, 1.0, 0.0),
            fovy: 60.0_f32.to_radians(),
            ..Default::default()
        });

        // Draw 3D scene
        draw_grid();
        {
            let t = telemetry.lock().unwrap();
            draw_craft(t.roll, t.pitch, t.yaw);
        }

        // Switch to 2D for UI
        set_default_camera();

        // Draw top bar background
        draw_rectangle(0.0, 0.0, screen_width(), 45.0, Color::new(0.08, 0.08, 0.12, 0.9));

        // Draw dropdown
        dropdown.draw();

        // Draw Refresh Ports button
        let refresh_rect = Rect::new(230.0, 10.0, 100.0, 25.0);
        let (mx, my) = mouse_position();
        let refresh_hovered = mx >= refresh_rect.x && mx <= refresh_rect.x + refresh_rect.w
            && my >= refresh_rect.y && my <= refresh_rect.y + refresh_rect.h;
        let refresh_color = if refresh_hovered { Color::new(0.3, 0.5, 0.7, 1.0) } else { Color::new(0.2, 0.3, 0.5, 1.0) };
        draw_rectangle(refresh_rect.x, refresh_rect.y, refresh_rect.w, refresh_rect.h, refresh_color);
        draw_rectangle_lines(refresh_rect.x, refresh_rect.y, refresh_rect.w, refresh_rect.h, 1.0, Color::new(0.4, 0.5, 0.6, 1.0));
        draw_text("Refresh", refresh_rect.x + 15.0, refresh_rect.y + 17.0, 14.0, WHITE);
        if is_mouse_button_pressed(MouseButton::Left) && refresh_hovered {
            available_ports = list_serial_ports();
            dropdown.ports = available_ports.clone();
        }

        // Draw status text
        let status_text = match &selected_port {
            Some(p) => format!("Connected: {}", p),
            None => "No connection".to_string(),
        };
        let status_color = if selected_port.is_some() { GREEN } else { RED };
        draw_text(&status_text, 345.0, 28.0, 16.0, status_color);

        // Draw controls hint
        let hint = "WASD:Move  RMB:Look  Wheel:Speed  R:Reset";
        draw_text(hint, screen_width() - 350.0, 28.0, 14.0, Color::new(0.5, 0.5, 0.5, 1.0));

        // Draw telemetry panel
        {
            let t = telemetry.lock().unwrap();
            draw_telemetry_panel(&t);
        }

        // Draw FPS
        let fps = 1.0 / dt;
        draw_text(&format!("FPS: {:.0}", fps), screen_width() - 100.0, 55.0, 16.0, WHITE);

        next_frame().await;
    }
}
