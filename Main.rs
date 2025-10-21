// src/main.rs
extern crate alloc;

use anyhow::{bail, Result};
use log::{error, info, warn};
use std::thread;
use core::ffi::c_char;
use esp_idf_sys as sys;
use alloc::{ffi::CString, string::String};
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    log::EspLogger,
    nvs::EspDefaultNvsPartition,
    wifi::{AuthMethod, BlockingWifi, ClientConfiguration as StaCfg, Configuration as WifiCfg, EspWifi},
};
use esp_idf_svc::hal::{
    delay::FreeRtos,
    peripherals::Peripherals,
    uart::{config::Config as UartConfig, UartDriver},
    units::Hertz,
};
use esp_idf_hal::gpio::{PinDriver, Pin, OutputMode};
use serde_json::json;
use heapless::Vec;

// =================== KONFIGURASI ===================
const WIFI_SSID: &str = "BELLA";
const WIFI_PASS: &str = "bela2012";

const TB_MQTT_URL: &str = "mqtt://demo.thingsboard.io";
const TB_CLIENT_ID: &str = "esp_1";
const TB_ACCESS_TOKEN: &str = "5GMq9cYYUULL3UHURASX";

const INFLUX_URL: &str = "http://10.206.197.182:8086";
const INFLUX_ORG_ID: &str = "462e19f8453b21ba";
const INFLUX_BUCKET: &str = "coba";
const INFLUX_TOKEN: &str = "yJy1EXm-UU4bAOYMHgf_lOdXLP9IOD2V6d8_X08kLIiHG7o__vADWuu-U7GAeF32mARP0GoOkzuFtFcYsak-TQ==";

const MODBUS_ID: u8 = 0x01;
const BAUD: u32 = 9600;

// =================== UTILITIES ===================
#[inline(always)]
fn ms_to_ticks(ms: u32) -> u32 {
    (ms as u64 * (sys::configTICK_RATE_HZ as u64) / 1000) as u32
}

// =================== MQTT Client ===================
struct SimpleMqttClient {
    client: *mut sys::esp_mqtt_client,
    broker_uri: *mut c_char,
    username: *mut c_char,
    password: *mut c_char,
    client_id: *mut c_char,
}

impl SimpleMqttClient {
    fn new(broker_url: &str, username: &str, password: &str, client_id: &str) -> Result<Self> {
        unsafe {
            let broker_uri = CString::new(broker_url)?.into_raw();
            let username_p = CString::new(username)?.into_raw();
            let password_p = CString::new(password)?.into_raw();
            let client_id_p = CString::new(client_id)?.into_raw();

            let mut cfg: sys::esp_mqtt_client_config_t = core::mem::zeroed();
            // binding expects *const u8 for those fields
            cfg.broker.address.uri = broker_uri as *const u8;
            cfg.credentials.username = username_p as *const u8;
            cfg.credentials.authentication.password = password_p as *const u8;
            cfg.credentials.client_id = client_id_p as *const u8;
            cfg.session.keepalive = 30;
            cfg.network.timeout_ms = 20_000;

            let client = sys::esp_mqtt_client_init(&cfg);
            if client.is_null() {
                // free
                let _ = CString::from_raw(broker_uri);
                let _ = CString::from_raw(username_p);
                let _ = CString::from_raw(password_p);
                let _ = CString::from_raw(client_id_p);
                bail!("MQTT init failed");
            }

            let err = sys::esp_mqtt_client_start(client);
            if err != sys::ESP_OK {
                sys::esp_mqtt_client_destroy(client);
                let _ = CString::from_raw(broker_uri);
                let _ = CString::from_raw(username_p);
                let _ = CString::from_raw(password_p);
                let _ = CString::from_raw(client_id_p);
                bail!("MQTT start failed (0x{:X})", err as u32);
            }

            // small delay to let client attempt connect (DNS, etc.)
            sys::vTaskDelay(ms_to_ticks(1500));
            Ok(Self { client, broker_uri, username: username_p, password: password_p, client_id: client_id_p })
        }
    }

    fn publish(&self, topic: &str, data: &str) -> Result<()> {
        unsafe {
            let topic_c = CString::new(topic)?;
            // esp_mqtt_client_publish expects pointer types already mapped earlier
            let msg_id = sys::esp_mqtt_client_publish(
                self.client,
                topic_c.as_ptr() as *const u8,
                data.as_ptr() as *const u8,
                data.len() as i32,
                1,
                0,
            );
            if msg_id < 0 {
                bail!("MQTT publish failed (msg_id < 0)");
            }
            Ok(())
        }
    }
}

impl Drop for SimpleMqttClient {
    fn drop(&mut self) {
        unsafe {
            if !self.client.is_null() {
                sys::esp_mqtt_client_stop(self.client);
                sys::esp_mqtt_client_destroy(self.client);
            }
            if !self.broker_uri.is_null() {
                let _ = CString::from_raw(self.broker_uri);
            }
            if !self.username.is_null() {
                let _ = CString::from_raw(self.username);
            }
            if !self.password.is_null() {
                let _ = CString::from_raw(self.password);
            }
            if !self.client_id.is_null() {
                let _ = CString::from_raw(self.client_id);
            }
        }
    }
}

// =================== CRC16 & Modbus ===================
fn crc16_modbus(mut crc: u16, byte: u8) -> u16 {
    crc ^= byte as u16;
    for _ in 0..8 {
        crc = if (crc & 1) != 0 { (crc >> 1) ^ 0xA001 } else { crc >> 1 };
    }
    crc
}
fn modbus_crc(data: &[u8]) -> u16 {
    let mut crc = 0xFFFF;
    for &b in data {
        crc = crc16_modbus(crc, b);
    }
    crc
}
fn build_read_req(slave: u8, func: u8, start_reg: u16, qty: u16) -> Vec<u8, 256> {
    let mut pdu: Vec<u8, 256> = Vec::new();
    pdu.push(slave).unwrap();
    pdu.push(func).unwrap();
    pdu.push((start_reg >> 8) as u8).unwrap();
    pdu.push((start_reg & 0xFF) as u8).unwrap();
    pdu.push((qty >> 8) as u8).unwrap();
    pdu.push((qty & 0xFF) as u8).unwrap();
    let crc = modbus_crc(&pdu);
    // Modbus CRC is little-endian (low byte first)
    pdu.push((crc & 0xFF) as u8).unwrap();
    pdu.push((crc >> 8) as u8).unwrap();
    pdu
}

// =================== RS485 ===================
// Added extra clears and debug prints to help with timeouts and bus contention.
// Bound MODE by OutputMode so set_high/set_low are available.
fn rs485_write<T: Pin, MODE: OutputMode>(
    uart: &UartDriver<'_>,
    de_pin: &mut PinDriver<'_, T, MODE>,
    data: &[u8],
) -> Result<()> {
    // Ensure RX buffer cleared before transmit
    uart.clear_rx()?;
    FreeRtos::delay_ms(2);

    // Drive DE high (transmit)
    de_pin.set_high()?;
    FreeRtos::delay_ms(3); // give transceiver time to switch to TX

    // debug TX hex
    {
        let mut s = String::new();
        for b in data { let _ = core::fmt::write(&mut s, format_args!("{:02X} ", b)); }
        info!("TX {} bytes: {}", data.len(), s);
    }

    uart.write(data)?;
    // wait for HW to finish
    uart.wait_tx_done(1500)?;
    // small hold to ensure line idles
    FreeRtos::delay_ms(8);

    // return to receive mode
    de_pin.set_low()?;
    FreeRtos::delay_ms(3); // let receiver settle
    Ok(())
}

fn rs485_read(uart: &UartDriver<'_>, dst: &mut [u8], ticks: u32) -> Result<usize> {
    let n = uart.read(dst, ticks)?;
    if n > 0 {
        let mut s = String::new();
        for b in &dst[..n] { let _ = core::fmt::write(&mut s, format_args!("{:02X} ", b)); }
        info!("RX {} bytes: {}", n, s);
    } else {
        info!("RX 0 bytes (timeout)");
    }
    Ok(n)
}

fn parse_read_resp(qty: u16, buf: &[u8]) -> Result<Vec<u16, 64>> {
    if buf.len() < 5 {
        bail!("short response, len={}", buf.len());
    }
    // basic sanity: function code at [1], byte count at [2], data starts [3]
    let bc = buf[2] as usize;
    let expected_len = 1 + 1 + 1 + bc + 2; // addr + func + bc + data + crc
    if buf.len() < expected_len {
        bail!("buffer shorter than expected_len ({} < {})", buf.len(), expected_len);
    }
    // Validate CRC
    let crc_rx = u16::from(buf[expected_len - 1]) << 8 | u16::from(buf[expected_len - 2]);
    let crc_calc = modbus_crc(&buf[..expected_len - 2]);
    if crc_rx != crc_calc {
        bail!("CRC mismatch: rx=0x{:04X}, calc=0x{:04X}", crc_rx, crc_calc);
    }
    let mut out: Vec<u16, 64> = Vec::new();
    for i in 0..(bc / 2) {
        let hi = buf[3 + 2 * i] as u16;
        let lo = buf[3 + 2 * i + 1] as u16;
        out.push((hi << 8) | lo).unwrap();
    }
    Ok(out)
}

fn try_read<T: Pin, MODE: OutputMode>(
    uart: &UartDriver<'_>,
    de_pin: &mut PinDriver<'_, T, MODE>,
    func: u8,
    start: u16,
    qty: u16,
    ticks: u32,
) -> Result<Vec<u16, 64>> {
    // extra attempt-level retry & clearing to handle bus noise
    // build request
    let req = build_read_req(MODBUS_ID, func, start, qty);

    // best-effort: multiple micro-retries for the physical layer
    for phys_try in 1..=2 {
        // ensure buffers cleared and small sleep between tries
        let _ = uart.clear_rx();
        FreeRtos::delay_ms(2 * phys_try);

        // send frame
        if let Err(e) = rs485_write(uart, de_pin, &req) {
            warn!("rs485_write failed (phys_try={}): {:?}", phys_try, e);
            FreeRtos::delay_ms(5 * phys_try);
            continue;
        }

        // read
        let mut buf = [0u8; 128];
        let n = rs485_read(uart, &mut buf, ticks)?;
        if n == 0 {
            // timeout
            warn!("UART read timeout (phys_try={})", phys_try);
            FreeRtos::delay_ms(10 * phys_try);
            continue;
        }
        // parse and return (or error)
        match parse_read_resp(qty, &buf[..n]) {
            Ok(v) => return Ok(v),
            Err(e) => {
                warn!("parse_read_resp failed (phys_try={}): {:?}", phys_try, e);
                FreeRtos::delay_ms(5 * phys_try);
                continue;
            }
        }
    }

    // If reached here => physical attempts failed
    bail!("ESP_ERR_TIMEOUT");
}

fn read_sht20_with_map<T: Pin, MODE: OutputMode>(
    uart: &UartDriver<'_>,
    de_pin: &mut PinDriver<'_, T, MODE>,
    fc: u8,
    start: u16,
    qty: u16,
) -> Result<(f32, f32)> {
    // try up to 3 logical attempts with backoff
    for attempt in 1..=3 {
        match try_read(uart, de_pin, fc, start, qty, 700) {
            Ok(regs) => {
                if regs.len() >= 2 {
                    let raw_t = regs[0];
                    let raw_h = regs[1];
                    return Ok(((raw_t as f32) * 0.1, (raw_h as f32) * 0.1));
                } else if regs.len() == 1 {
                    // try to read the second register in next call
                    if let Ok(regs2) = try_read(uart, de_pin, fc, start.wrapping_add(1), 1, 500) {
                        if regs2.len() >= 1 {
                            return Ok(((regs[0] as f32) * 0.1, (regs2[0] as f32) * 0.1));
                        }
                    }
                    return Ok(((regs[0] as f32) * 0.1, 0.0));
                } else {
                    bail!("No registers returned");
                }
            }
            Err(e) => {
                warn!("try_read attempt {}/3 failed: {:?}", attempt, e);
                // small backoff and clear RX to attempt to resync bus
                let _ = uart.clear_rx();
                FreeRtos::delay_ms(80 * attempt);
                continue;
            }
        }
    }
    bail!("All Modbus attempts failed")
}

fn probe_map<T: Pin, MODE: OutputMode>(
    uart: &UartDriver<'_>,
    de_pin: &mut PinDriver<'_, T, MODE>,
) -> Option<(u8, u16, u16)> {
    // try a few full probe cycles in case bus needs time to stabilize
    for cycle in 1..=3 {
        info!("Probe cycle {}/3", cycle);
        for &fc in &[0x04u8, 0x03u8] {
            for start in 0x0000u16..=0x0010u16 {
                for &qty in &[1u16, 2u16] {
                    match try_read(uart, de_pin, fc, start, qty, 400) {
                        Ok(regs) => {
                            info!(
                                "FOUND (cycle {}): fc=0x{:02X}, start=0x{:04X}, qty={}, regs={:04X?}",
                                cycle, fc, start, qty, regs.as_slice()
                            );
                            return Some((fc, start, qty));
                        }
                        Err(e) => {
                            // swallow and continue probing
                        }
                    }
                }
            }
        }
        // wait a bit then retry full probe
        FreeRtos::delay_ms(200 * cycle);
    }
    None
}

// =================== Wi-Fi ===================
fn connect_wifi(wifi: &mut BlockingWifi<EspWifi<'static>>) -> Result<()> {
    let cfg = WifiCfg::Client(StaCfg {
        ssid: heapless::String::try_from(WIFI_SSID).unwrap(),
        password: heapless::String::try_from(WIFI_PASS).unwrap(),
        auth_method: AuthMethod::WPA2Personal,
        ..Default::default()
    });
    wifi.set_configuration(&cfg)?;
    wifi.start()?;
    info!("⏳ Connecting to Wi-Fi...");
    if let Err(e) = wifi.connect() {
        warn!("Wi-Fi connect() error: {e:?}; lanjut offline");
        return Ok(());
    }
    match wifi.wait_netif_up() {
        Ok(()) => {
            let ip = wifi.wifi().sta_netif().get_ip_info()?;
            info!("✅ Wi-Fi connected. IP = {}", ip.ip);
            // small delay for DNS to be ready
            FreeRtos::delay_ms(3000);
        }
        Err(e) => warn!("⚠ wait_netif_up timeout: {e:?}"),
    }
    Ok(())
}

// =================== Influx Mock ===================
fn influx_line(measurement: &str, device: &str, t_c: f32, h_pct: f32) -> String {
    format!("{},device={} temperature_c={},humidity_pct={}", measurement, device, t_c, h_pct)
}
fn influx_write(line: &str) -> Result<()> {
    info!("(mock) Influx line written: {}", line);
    Ok(())
}

// =================== ENTRY ===================
fn main() -> Result<()> {
    sys::link_patches();
    EspLogger::initialize_default();
    info!("▶ starting user thread with big stack ...");
    thread::Builder::new()
        .name("app".into())
        .stack_size(128 * 1024)
        .spawn(|| {
            if let Err(e) = app_main() {
                error!("app_main error: {e:?}");
            }
        })?;
    loop {
        FreeRtos::delay_ms(1000);
    }
}

fn app_main() -> Result<()> {
    info!("🚀 Modbus RS485 + ThingsBoard MQTT Basic + InfluxDB (improved)");

    let peripherals = Peripherals::take()?;
    let pins = peripherals.pins;
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;
    let mut wifi = BlockingWifi::wrap(EspWifi::new(peripherals.modem, sys_loop.clone(), Some(nvs))?, sys_loop)?;
    connect_wifi(&mut wifi)?;

    // MQTT client (use token as username)
    let mqtt = match SimpleMqttClient::new(TB_MQTT_URL, TB_ACCESS_TOKEN, "", TB_CLIENT_ID) {
        Ok(c) => c,
        Err(_) => {
            warn!("DNS gagal, fallback ke IP 34.117.8.65");
            SimpleMqttClient::new("mqtt://34.117.8.65", TB_ACCESS_TOKEN, "", TB_CLIENT_ID)?
        }
    };
    info!("MQTT ready.");

    // UART TX=17 RX=18 (ubah sesuai board)
    let tx = pins.gpio17;
    let rx = pins.gpio18;
    let cfg = UartConfig::new().baudrate(Hertz(BAUD));
    let uart = UartDriver::new(
        peripherals.uart1,
        tx,
        rx,
        None::<esp_idf_hal::gpio::AnyInputPin>,
        None::<esp_idf_hal::gpio::AnyOutputPin>,
        &cfg,
    )?;

    // DE/RE pin (GPIO21) sebagai output untuk RS485 direction
    let mut de_pin = PinDriver::output(pins.gpio21)?;
    de_pin.set_low()?; // listen mode

    // probe mapping registri (lebih toleran)
    let (mut fc_use, mut start_use, mut qty_use) = (0x04u8, 0x0000u16, 2u16);
    if let Some((fc, start, qty)) = probe_map(&uart, &mut de_pin) {
        (fc_use, start_use, qty_use) = (fc, start, qty);
        info!("Using map: fc=0x{:02X}, start=0x{:04X}, qty={}", fc_use, start_use, qty_use);
    } else {
        warn!("Probe failed after retries. Fallback map will be used (fc=0x04,start=0x0000,qty=2).");
    }

    info!("checkpoint: sebelum loop baca main");

    let topic_tele = "v1/devices/me/telemetry";
    loop {
        match read_sht20_with_map(&uart, &mut de_pin, fc_use, start_use, qty_use) {
            Ok((t, h)) => {
                let ts_ms = unsafe { sys::esp_timer_get_time() } / 1000;
                let payload = json!({
                    "sensor":"sht20",
                    "temperature_c": (t * 10.0).round()/10.0,
                    "humidity_pct": (h * 10.0).round()/10.0,
                    "ts_ms": ts_ms
                }).to_string();

                if let Err(e) = mqtt.publish(topic_tele, &payload) {
                    error!("MQTT publish error: {e:?}");
                } else {
                    info!("Published to ThingsBoard: {}", payload);
                }

                let lp = influx_line("sht20", TB_CLIENT_ID, (t * 10.0).round()/10.0, (h * 10.0).round()/10.0);
                if let Err(e) = influx_write(&lp) {
                    warn!("Influx write failed: {e}");
                }
            }
            Err(e) => {
                error!("Modbus read error: {e:?}. Will retry next cycle.");
            }
        }
        FreeRtos::delay_ms(5000);
    }
}
