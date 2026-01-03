#![no_main]
#![no_std]

mod pins;
mod sd;
mod ssd1677;
mod ui;

use ariel_os::{
    debug::log::info,
    gpio::{Input, Level, Output, Pull},
};

use core::cell::RefCell;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_sdmmc::{LfnBuffer, SdCard, TimeSource, VolumeIdx, VolumeManager};
use embedded_hal::digital::{ErrorType as DigitalErrorType, OutputPin};
use embedded_hal_bus::spi::RefCellDevice;
use heapless::{String, Vec};
use static_cell::StaticCell;

use esp_hal::spi::master::{Spi, Config as SpiConfig};
use esp_hal::spi::Mode as SpiMode;
use esp_hal::delay::Delay as EspDelay;
use esp_hal::time::RateExtU32;

use esp_hal::analog::adc::{Adc, AdcCalCurve, AdcConfig, AdcPin, Attenuation};
use esp_hal::gpio::Input as EspInput;
use esp_hal::peripherals::ADC1;

use core::str::FromStr;

#[cfg(context = "xteink-x4")]
#[allow(unsafe_code)]
fn init_gpio12_output_mux() {
    unsafe {
        // IO_MUX_GPIO12_REG (0x60009034)
        // Fun_IE=1, Fun_DRV=2, MCU_SEL=1
        (0x60009034 as *mut u32).write_volatile((1 << 8) | (2 << 9) | (1 << 12));

        // GPIO_ENABLE_W1TS_REG (0x60004024)
        (0x60004024 as *mut u32).write_volatile(1 << 12);
    }
}

/// Bit-bang ~80 slow clock cycles on GPIO8 (SCK) at ~400kHz with SD CS high.
/// SD spec requires >=74 clocks at <=400kHz before first command.
/// This must be called BEFORE the SPI bus is created.
#[cfg(context = "xteink-x4")]
#[allow(unsafe_code)]
fn sd_slow_dummy_clocks() {
    // First ensure SD CS (GPIO12) is high
    init_gpio12_output_mux();
    unsafe {
        // GPIO_OUT_W1TS_REG - set GPIO12 high
        (0x60004008 as *mut u32).write_volatile(1 << 12);
    }

    // Configure GPIO8 as output for bit-banging SCK
    unsafe {
        // IO_MUX_GPIO8_REG (0x60009024) - set as GPIO function
        // Fun_IE=0, Fun_DRV=2, MCU_SEL=1 (GPIO)
        (0x60009024 as *mut u32).write_volatile((2 << 9) | (1 << 12));

        // GPIO_ENABLE_W1TS_REG - enable output on GPIO8
        (0x60004024 as *mut u32).write_volatile(1 << 8);

        // Start with SCK low
        (0x6000400C as *mut u32).write_volatile(1 << 8); // GPIO_OUT_W1TC
    }

    // Bit-bang 80 clock cycles at ~400kHz (1.25us per half-cycle)
    // We use a simple busy-wait loop. On ESP32-C3 at 160MHz, ~200 iterations ≈ 1.25us
    for _ in 0..80 {
        // SCK high
        unsafe { (0x60004008 as *mut u32).write_volatile(1 << 8); }
        for _ in 0..200 { core::hint::spin_loop(); }
        // SCK low
        unsafe { (0x6000400C as *mut u32).write_volatile(1 << 8); }
        for _ in 0..200 { core::hint::spin_loop(); }
    }

    // Leave SCK low - SPI driver will take over
}

// Raw GPIO12 output (ESP32-C3 special-case) for SD CS, matching rust-launcher.
// We can't rely on the normal HAL pin for GPIO12 on some setups.
#[cfg(context = "xteink-x4")]
struct RawGpio12;

#[cfg(context = "xteink-x4")]
impl RawGpio12 {
    fn new() -> Self {
        init_gpio12_output_mux();
        Self
    }
}

#[cfg(context = "xteink-x4")]
impl DigitalErrorType for RawGpio12 {
    type Error = core::convert::Infallible;
}

#[cfg(context = "xteink-x4")]
impl OutputPin for RawGpio12 {
    #[allow(unsafe_code)]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        unsafe {
            // GPIO_OUT_W1TC_REG
            (0x6000400C as *mut u32).write_volatile(1 << 12);
        }
        Ok(())
    }

    #[allow(unsafe_code)]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        unsafe {
            // GPIO_OUT_W1TS_REG
            (0x60004008 as *mut u32).write_volatile(1 << 12);
        }
        Ok(())
    }
}

// Removed Ariel OS SPI_BUS - using esp-hal SPI directly

#[derive(Clone, Copy, PartialEq, Eq)]
enum Screen {
    Home,
    Apps,
    Library,
    Tools,
    Settings,
    SettingsReader,
    SettingsWifi,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum SettingsItem {
    Reader,
    Wifi,
    Rotation,
}

impl SettingsItem {
    fn all() -> [SettingsItem; 3] {
        [SettingsItem::Reader, SettingsItem::Wifi, SettingsItem::Rotation]
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum ReaderSettingsItem {
    BooksView,
}

impl ReaderSettingsItem {
    fn all() -> [ReaderSettingsItem; 1] {
        [ReaderSettingsItem::BooksView]
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum HomeItem {
    Apps,
    Tools,
    Settings,
}

impl HomeItem {
    fn all() -> [HomeItem; 3] {
        [HomeItem::Apps, HomeItem::Tools, HomeItem::Settings]
    }

    #[allow(dead_code)]
    fn label(self) -> &'static str {
        match self {
            HomeItem::Apps => "Apps",
            HomeItem::Tools => "Tools",
            HomeItem::Settings => "Settings",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum ButtonEvent {
    Back,
    Confirm,
    Left,
    Right,
    Up,
    Down,
    Power,
}

struct InputManager<'d, PIN1, PIN2, BAT> {
    adc1: Adc<'d, ADC1>,
    pin1: AdcPin<PIN1, ADC1, AdcCalCurve<ADC1>>,
    pin2: AdcPin<PIN2, ADC1, AdcCalCurve<ADC1>>,
    bat_pin: AdcPin<BAT, ADC1, AdcCalCurve<ADC1>>,
    power_pin: EspInput<'d>,
    last_state: u8,
}

impl<'d, PIN1, PIN2, BAT> InputManager<'d, PIN1, PIN2, BAT>
where
    PIN1: esp_hal::analog::adc::AdcChannel + esp_hal::gpio::AnalogPin,
    PIN2: esp_hal::analog::adc::AdcChannel + esp_hal::gpio::AnalogPin,
    BAT: esp_hal::analog::adc::AdcChannel + esp_hal::gpio::AnalogPin,
{
    fn new(adc1: ADC1, pin1: PIN1, pin2: PIN2, bat_pin: BAT, power_pin: EspInput<'d>) -> Self {
        let mut config = AdcConfig::new();
        let adc_pin1 = config.enable_pin_with_cal::<_, AdcCalCurve<ADC1>>(pin1, Attenuation::_11dB);
        let adc_pin2 = config.enable_pin_with_cal::<_, AdcCalCurve<ADC1>>(pin2, Attenuation::_11dB);
        let adc_bat = config.enable_pin_with_cal::<_, AdcCalCurve<ADC1>>(bat_pin, Attenuation::_11dB);
        let adc1 = Adc::new(adc1, config);
        Self {
            adc1,
            pin1: adc_pin1,
            pin2: adc_pin2,
            bat_pin: adc_bat,
            power_pin,
            last_state: 0,
        }
    }

    fn read_battery_percentage(&mut self) -> u8 {
        if let Ok(mv) = nb::block!(self.adc1.read_oneshot(&mut self.bat_pin)) {
            // Voltage divider is 2.0x
            let vbat_mv = (mv as u32) * 2;
            let volts = (vbat_mv as f64) / 1000.0;

            // Polynomial derived from LiPo samples (from BatteryMonitor.cpp)
            // y = -144.9390 * volts^3 + 1655.8629 * volts^2 - 6158.8520 * volts + 7501.3202
            let mut y = -144.9390 * volts * volts * volts
                + 1655.8629 * volts * volts
                - 6158.8520 * volts
                + 7501.3202;

            if y < 0.0 {
                y = 0.0;
            }
            if y > 100.0 {
                y = 100.0;
            }
            y as u8
        } else {
            0
        }
    }

    fn poll(&mut self) -> heapless::Vec<ButtonEvent, 7> {
        let mut events = heapless::Vec::new();
        let mut current_state: u8 = 0;

        if self.power_pin.is_low() {
            current_state |= 1 << 6;
        }

        if let Ok(val1) = nb::block!(self.adc1.read_oneshot(&mut self.pin1)) {
            if val1 < 2850 {
                if val1 > 2300 {
                    current_state |= 1 << 0;
                } else if val1 > 1500 {
                    current_state |= 1 << 1;
                } else if val1 > 500 {
                    current_state |= 1 << 2;
                } else {
                    current_state |= 1 << 3;
                }
            }
        }

        if let Ok(val2) = nb::block!(self.adc1.read_oneshot(&mut self.pin2)) {
            if val2 < 2800 {
                if val2 > 1000 {
                    current_state |= 1 << 4;
                } else {
                    current_state |= 1 << 5;
                }
            }
        }

        let pressed = current_state & !self.last_state;
        self.last_state = current_state;

        if pressed & (1 << 0) != 0 {
            let _ = events.push(ButtonEvent::Back);
        }
        if pressed & (1 << 1) != 0 {
            let _ = events.push(ButtonEvent::Confirm);
        }
        if pressed & (1 << 2) != 0 {
            let _ = events.push(ButtonEvent::Left);
        }
        if pressed & (1 << 3) != 0 {
            let _ = events.push(ButtonEvent::Right);
        }
        if pressed & (1 << 4) != 0 {
            let _ = events.push(ButtonEvent::Up);
        }
        if pressed & (1 << 5) != 0 {
            let _ = events.push(ButtonEvent::Down);
        }
        if pressed & (1 << 6) != 0 {
            let _ = events.push(ButtonEvent::Power);
        }

        events
    }
}

#[allow(dead_code)]
struct SdTimeSource;

impl TimeSource for SdTimeSource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp::from_fat(0, 0)
    }
}

#[allow(dead_code)]
type SdDir<'a, D> = embedded_sdmmc::Directory<'a, D, SdTimeSource, 4, 4, 1>;

#[allow(dead_code)]
fn ensure_dir<'a, D>(root: &SdDir<'a, D>, name: &str)
where
    D: embedded_sdmmc::BlockDevice,
{
    if root.open_dir(name).is_ok() {
        return;
    }

    // Best effort: create if missing.
    // (Exact method name depends on embedded-sdmmc version; adjust if the build complains.)
    let _ = root.make_dir_in_dir(name);
}

#[allow(dead_code)]
fn ensure_nested_dir<'a, D>(root: &SdDir<'a, D>, parent: &str, child: &str)
where
    D: embedded_sdmmc::BlockDevice,
{
    ensure_dir(root, parent);
    if let Ok(p) = root.open_dir(parent) {
        ensure_dir(&p, child);
    }
}

#[allow(dead_code)]
fn scan_books_from_sd<D>(
    vm: &mut VolumeManager<D, SdTimeSource, 4, 4, 1>,
    out: &mut Vec<String<64>, 48>,
) -> bool
where
    D: embedded_sdmmc::BlockDevice,
{
    fn ends_with_ignore_ascii_case(s: &str, suffix: &str) -> bool {
        let s = s.as_bytes();
        let suf = suffix.as_bytes();
        if suf.len() > s.len() {
            return false;
        }
        let start = s.len() - suf.len();
        s[start..]
            .iter()
            .zip(suf.iter())
            .all(|(a, b)| a.to_ascii_lowercase() == b.to_ascii_lowercase())
    }

    out.clear();
    info!("Scanning SD /books...");

    let volume = match vm.open_volume(VolumeIdx(0)) {
        Ok(v) => v,
        Err(e) => {
            let _ = e;
            info!("SD open_volume(0) failed");
            return false;
        }
    };

    let root = match volume.open_root_dir() {
        Ok(r) => r,
        Err(_) => {
            info!("SD open_root_dir failed");
            return false;
        }
    };

    // Ensure cache directories.
    ensure_nested_dir(&root, ".ariel", "cache");

    // Ensure /books exists.
    ensure_dir(&root, "books");

    let books_dir = match root.open_dir("books") {
        Ok(d) => d,
        Err(_) => {
            info!("SD: /books open_dir failed (but SD is mounted)");
            return true;
        }
    };

    // Prefer long file names for correct .epub matching.
    let mut lfn_storage = [0u8; 192];
    let mut lfn_buf = LfnBuffer::new(&mut lfn_storage);
    let _ = books_dir.iterate_dir_lfn(&mut lfn_buf, |entry, lfn| {
        // If we have an LFN, use it.
        if let Some(name) = lfn {
            if !ends_with_ignore_ascii_case(name, ".epub") {
                return;
            }
            // Store (possibly truncated) name.
            let mut s: String<64> = String::new();
            for ch in name.chars() {
                if s.push(ch).is_err() {
                    break;
                }
            }
            let _ = out.push(s);
            return;
        }

        // Fallback: short 8.3 names. Extension is at most 3 chars.
        // Many systems truncate ".epub" => "EPU".
        let ext = core::str::from_utf8(entry.name.extension()).unwrap_or("");
        // Accept some likely 8.3 truncations/manglings.
        if !(ext.eq_ignore_ascii_case("EPU")
            || ext.eq_ignore_ascii_case("EPB")
            || ext.eq_ignore_ascii_case("EP"))
        {
            return;
        }

        let base = core::str::from_utf8(entry.name.base_name()).unwrap_or("");
        if base.is_empty() || base.starts_with('_') {
            return;
        }

        let mut s: String<64> = String::new();
        let _ = s.push_str(base);
        let _ = s.push_str(".");
        let _ = s.push_str(ext);
        let _ = out.push(s);
    });

    info!("Found {} book(s)", out.len());

    true
}

#[derive(Clone, Copy)]
struct UiSettings {
    rotation: ui::Rotation,
    books_view_mode: ui::ViewMode,
}

#[derive(Clone)]
struct AppState {
    screen: Screen,
    home_selected: usize,
    apps_selected: usize,
    library_cursor: usize,
    tools_selected: usize,
    settings_selected: usize,
    settings_reader_selected: usize,
    settings_wifi_selected: usize,
    settings: UiSettings,
    sd_present: bool,
    battery_pct: u8,
    wifi_connected: bool,
    ip_address: [u8; 4],
    wifi_rssi: i8,
    wifi_mac: [u8; 6],
    wifi_ssid: String<32>,
    wifi_password: String<64>,
    books: Vec<String<64>, 48>,
}

fn clamp_selection(sel: &mut usize, len: usize) {
    if len == 0 {
        *sel = 0;
        return;
    }
    if *sel >= len {
        *sel = len - 1;
    }
}

fn grid_move(sel: &mut usize, dx: i32, dy: i32, len: usize) {
    if len == 0 {
        *sel = 0;
        return;
    }

    let mut idx = *sel as i32;
    let col = idx % 2;
    let row = idx / 2;

    let mut new_col = col + dx;
    let mut new_row = row + dy;

    if new_col < 0 {
        new_col = 0;
    }
    if new_col > 1 {
        new_col = 1;
    }
    if new_row < 0 {
        new_row = 0;
    }
    if new_row > 2 {
        new_row = 2;
    }

    idx = new_row * 2 + new_col;

    if idx as usize >= len {
        let last_row = ((len - 1) as i32) / 2;
        let last_col = ((len - 1) as i32) % 2;
        idx = last_row * 2 + last_col;
    }

    *sel = idx as usize;
}

fn nav_move(sel: &mut usize, ev: ButtonEvent, view: ui::ViewMode, len: usize) {
    clamp_selection(sel, len);
    match view {
        ui::ViewMode::List => match ev {
            ButtonEvent::Up => {
                if *sel > 0 {
                    *sel -= 1;
                }
            }
            ButtonEvent::Down => {
                if *sel + 1 < len {
                    *sel += 1;
                }
            }
            _ => {}
        },
        ui::ViewMode::Grid => match ev {
            ButtonEvent::Left => grid_move(sel, -1, 0, len),
            ButtonEvent::Right => grid_move(sel, 1, 0, len),
            ButtonEvent::Up => grid_move(sel, 0, -1, len),
            ButtonEvent::Down => grid_move(sel, 0, 1, len),
            _ => {}
        },
    }
}

async fn render_state<D>(
    display: &mut ssd1677::Ssd1677<D>,
    state: &AppState,
    mode: ssd1677::RefreshMode,
) where
    D: embedded_hal_async::spi::SpiDevice<u8>,
{
    display.clear_buffer(BinaryColor::Off);

    let mut canvas = ui::UiCanvas::new(display, state.settings.rotation);
    let l = ui::layout(state.settings.rotation);

    let nav = ui::UiNavHints {
        back: "Back",
        ok: "OK",
        left: "Left",
        right: "Right",
    };

    match state.screen {
        Screen::Home => {
            let status = ui::UiStatus {
                title: "Home",
                battery_pct: Some(state.battery_pct),
                sd_present: state.sd_present,
                wifi_on: state.wifi_connected,
                bt_on: false,
            };
            ui::draw_chrome(&mut canvas, &l, status, nav);
            let items = ["Apps", "Tools", "Settings"];
            ui::draw_list_6(&mut canvas, &l, &items[..], state.home_selected);
        }
        Screen::Apps => {
            let status = ui::UiStatus {
                title: "Apps",
                battery_pct: Some(state.battery_pct),
                sd_present: state.sd_present,
                wifi_on: state.wifi_connected,
                bt_on: false,
            };
            ui::draw_chrome(&mut canvas, &l, status, nav);
            let items = ["Reader"];
            ui::draw_list_6(&mut canvas, &l, &items[..], state.apps_selected);
        }
        Screen::Library => {
            let status = ui::UiStatus {
                title: "Library",
                battery_pct: Some(state.battery_pct),
                sd_present: state.sd_present,
                wifi_on: state.wifi_connected,
                bt_on: false,
            };
            ui::draw_chrome(&mut canvas, &l, status, nav);

            if state.books.is_empty() {
                let items = ["(no books in /books)"];
                ui::draw_list_6(&mut canvas, &l, &items[..], 0);
            } else {
                let page_start = (state.library_cursor / 6) * 6;
                let end = (page_start + 6).min(state.books.len());
                let count = end - page_start;
                let selected_in_page = state.library_cursor - page_start;

                let mut labels: [&str; 6] = ["", "", "", "", "", ""]; 
                for (i, book) in state.books.iter().skip(page_start).take(6).enumerate() {
                    labels[i] = book.as_str();
                }

                match state.settings.books_view_mode {
                    ui::ViewMode::List => {
                        ui::draw_list_6(&mut canvas, &l, &labels[..count], selected_in_page)
                    }
                    ui::ViewMode::Grid => {
                        ui::draw_grid_2x3(&mut canvas, &l, &labels[..count], selected_in_page)
                    }
                }
            }
        }
        Screen::Tools => {
            let status = ui::UiStatus {
                title: "Tools",
                battery_pct: Some(state.battery_pct),
                sd_present: state.sd_present,
                wifi_on: state.wifi_connected,
                bt_on: false,
            };
            ui::draw_chrome(&mut canvas, &l, status, nav);
            let items: [&str; 0] = [];
            ui::draw_list_6(&mut canvas, &l, &items[..], state.tools_selected);
        }
        Screen::Settings => {
            let status = ui::UiStatus {
                title: "Settings",
                battery_pct: Some(state.battery_pct),
                sd_present: state.sd_present,
                wifi_on: state.wifi_connected,
                bt_on: false,
            };
            ui::draw_chrome(&mut canvas, &l, status, nav);

            let mut line1: String<32> = String::new();
            let _ = line1.push_str("Rotation: ");
            let _ = line1.push_str(match state.settings.rotation {
                ui::Rotation::Portrait => "Portrait",
                ui::Rotation::Landscape => "Landscape",
            });

            let items = ["Reader", "Wi-Fi", line1.as_str()];
            ui::draw_list_6(&mut canvas, &l, &items[..], state.settings_selected);
        }
        Screen::SettingsWifi => {
            let status = ui::UiStatus {
                title: "Wi-Fi",
                battery_pct: Some(state.battery_pct),
                sd_present: state.sd_present,
                wifi_on: state.wifi_connected,
                bt_on: false,
            };
            ui::draw_chrome(&mut canvas, &l, status, nav);

            let mut line0: String<64> = String::new();
            let _ = line0.push_str("SSID: ");
            #[cfg(feature = "wifi-esp")]
            let _ = line0.push_str(ariel_os::hal::wifi::WIFI_NETWORK);

            let mut line1: String<64> = String::new();
            let _ = line1.push_str("Status: ");
            let _ = line1.push_str(if state.wifi_connected { "Connected" } else { "Disconnected" });

            let mut line2: String<64> = String::new();
            let _ = line2.push_str("IP: ");
            if state.wifi_connected {
                let _ = core::fmt::write(&mut line2, format_args!("{}.{}.{}.{}", 
                    state.ip_address[0], state.ip_address[1], state.ip_address[2], state.ip_address[3]));
            } else {
                let _ = line2.push_str("0.0.0.0");
            }

            let mut line3: String<64> = String::new();
            let _ = line3.push_str("MAC: ");
            let _ = core::fmt::write(&mut line3, format_args!("{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                state.wifi_mac[0], state.wifi_mac[1], state.wifi_mac[2],
                state.wifi_mac[3], state.wifi_mac[4], state.wifi_mac[5]));

            let mut line4: String<64> = String::new();
            let _ = line4.push_str("RSSI: ");
            let _ = core::fmt::write(&mut line4, format_args!("{} dBm", state.wifi_rssi));

            let items = [line0.as_str(), "Edit Password", line1.as_str(), line2.as_str(), line3.as_str(), line4.as_str(), "Reconnect"];
            ui::draw_list_6(&mut canvas, &l, &items[..], state.settings_wifi_selected);
        }
        Screen::SettingsReader => {
            let status = ui::UiStatus {
                title: "Reader",
                battery_pct: Some(state.battery_pct),
                sd_present: state.sd_present,
                wifi_on: state.wifi_connected,
                bt_on: false,
            };
            ui::draw_chrome(&mut canvas, &l, status, nav);

            let mut line0: String<32> = String::new();
            let _ = line0.push_str("Books view: ");
            let _ = line0.push_str(match state.settings.books_view_mode {
                ui::ViewMode::List => "List",
                ui::ViewMode::Grid => "Grid",
            });
            let items = [line0.as_str()];
            ui::draw_list_6(&mut canvas, &l, &items[..], state.settings_reader_selected);
        }
    }

    canvas.flush_refresh(mode).await;
}

#[ariel_os::task(autostart, peripherals)]
async fn main(peripherals: pins::Peripherals) {
    info!("x4-launcher starting...");

    // SD spec requires >=74 clock cycles at <=400kHz with CS high before first command.
    // We bit-bang these BEFORE creating the SPI bus (which runs at high speed).
    #[cfg(context = "xteink-x4")]
    sd_slow_dummy_clocks();

    // Initialize SD card using direct esp-hal SPI (this works!)
    // Then use Ariel OS SPI for display (which also works)
    // SD card will be re-initialized on-demand when accessing Library
    info!("Testing SD card with esp-hal SPI...");
    #[allow(unsafe_code)]
    let _sd_detected = unsafe {
        let spi2 = esp_hal::peripherals::SPI2::steal();
        let sck = esp_hal::gpio::GpioPin::<8>::steal();
        let miso = esp_hal::gpio::GpioPin::<7>::steal();
        let mosi = esp_hal::gpio::GpioPin::<10>::steal();
        
        let spi_bus = Spi::new(
            spi2,
            SpiConfig::default()
                .with_frequency(20_000_000u32.Hz())
                .with_mode(SpiMode::_0),
        )
        .unwrap()
        .with_sck(sck)
        .with_miso(miso)
        .with_mosi(mosi);
        
        let spi_bus_ref = RefCell::new(spi_bus);
        let delay = EspDelay::new();
        
        let sd_cs = RawGpio12::new();
        let sd_spi = RefCellDevice::new(&spi_bus_ref, sd_cs, delay.clone()).expect("SD SPI failed");
        
        let sd_card = SdCard::new(sd_spi, delay.clone());
        
        match sd_card.num_bytes() {
            Ok(bytes) => {
                info!("SD num_bytes OK: {}", bytes);
                true
            }
            Err(_) => {
                info!("SD num_bytes failed");
                false
            }
        }
        // SPI bus dropped here - will be re-created by Ariel OS
    };

    // Now set up Ariel OS SPI for display
    let spi_bus = pins::SharedSpi::new(
        peripherals.spi_sck,
        peripherals.spi_miso,
        peripherals.spi_mosi,
        ssd1677::default_spi_config(),
    );

    static SPI_BUS: StaticCell<embassy_sync::mutex::Mutex<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, ariel_os::hal::spi::main::Spi>> = StaticCell::new();
    let spi_bus = SPI_BUS.init(embassy_sync::mutex::Mutex::new(spi_bus));

    // Prepare chip-select pins
    let display_cs = Output::new(peripherals.display_cs, Level::High);
    let display_spi = ariel_os::spi::main::SpiDevice::new(spi_bus, display_cs);

    let dc = Output::new(peripherals.display_dc, Level::High);
    let rst = Output::new(peripherals.display_rst, Level::High);
    let busy = Input::new(peripherals.display_busy, Pull::None);

    static FRAMEBUF: StaticCell<[u8; ssd1677::BUFFER_SIZE]> = StaticCell::new();
    let buffer = FRAMEBUF.init_with(|| [0xFF; ssd1677::BUFFER_SIZE]);

    let mut display = ssd1677::Ssd1677::new(display_spi, dc, rst, busy, buffer);
    info!("Initializing SSD1677...");
    display.init().await;
    info!("SSD1677 init done");

    let power_in = EspInput::new(peripherals.power_button, esp_hal::gpio::Pull::Up);
    let mut input = InputManager::new(
        peripherals.adc1,
        peripherals.adc_buttons_1,
        peripherals.adc_buttons_2,
        peripherals.battery_adc,
        power_in,
    );

    // Mark it used for now; SD listing will come next.
    // We'll mount/list only on demand (e.g. entering Library).

    let mut state = AppState {
        screen: Screen::Home,
        home_selected: 0,
        apps_selected: 0,
        library_cursor: 0,
        tools_selected: 0,
        settings_selected: 0,
        settings_reader_selected: 0,
        settings_wifi_selected: 0,
        settings: UiSettings {
            rotation: ui::Rotation::Portrait,
            books_view_mode: ui::ViewMode::List,
        },
        sd_present: false,
        battery_pct: 100,
        wifi_connected: false,
        ip_address: [0, 0, 0, 0],
        wifi_rssi: 0,
        wifi_mac: [0; 6],
        wifi_ssid: String::from_str(ariel_os::hal::wifi::WIFI_NETWORK).unwrap_or_default(),
        wifi_password: String::from_str(ariel_os::hal::wifi::WIFI_PASSWORD).unwrap_or_default(),
        books: Vec::new(),
    };

    state.battery_pct = input.read_battery_percentage();

    info!("Rendering home screen...");
    render_state(&mut display, &state, ssd1677::RefreshMode::Full).await;
    info!("Home screen rendered");

    loop {
        let mut dirty = false;
        
        let new_bat = input.read_battery_percentage();
        if new_bat != state.battery_pct {
            state.battery_pct = new_bat;
            dirty = true;
        }

        // Check Wi-Fi status if enabled
        #[cfg(feature = "wifi-esp")]
        {
            let connected = esp_wifi::wifi::wifi_state() == esp_wifi::wifi::WifiState::StaConnected;
            if connected != state.wifi_connected {
                state.wifi_connected = connected;
                dirty = true;
                
                if connected {
                    if let Some(stack) = ariel_os::net::network_stack().await {
                        if let Some(config) = stack.config_v4() {
                            state.ip_address = config.address.address().octets();
                        }
                    }
                } else {
                    state.ip_address = [0, 0, 0, 0];
                }
            }

            if state.wifi_connected {
                let rssi = esp_wifi::wifi::get_rssi().unwrap_or(0);
                if rssi != state.wifi_rssi {
                    state.wifi_rssi = rssi;
                    dirty = true;
                }
            } else {
                state.wifi_rssi = 0;
            }

            let mac = esp_wifi::wifi::get_mac(esp_wifi::wifi::WifiStaDevice);
            if mac != state.wifi_mac {
                state.wifi_mac = mac;
                dirty = true;
            }
        }

        let events = input.poll();

        for ev in events.iter().copied() {
            match state.screen {
                Screen::Home => {
                    let old = state.home_selected;
                    nav_move(
                        &mut state.home_selected,
                        ev,
                        ui::ViewMode::List,
                        HomeItem::all().len(),
                    );
                    if state.home_selected != old {
                        dirty = true;
                    }

                    if ev == ButtonEvent::Confirm {
                        state.screen = match HomeItem::all()[state.home_selected] {
                            HomeItem::Apps => Screen::Apps,
                            HomeItem::Tools => Screen::Tools,
                            HomeItem::Settings => Screen::Settings,
                        };
                        dirty = true;
                    }
                }
                Screen::Apps => {
                    let old = state.apps_selected;
                    nav_move(
                        &mut state.apps_selected,
                        ev,
                        ui::ViewMode::List,
                        1,
                    );
                    if state.apps_selected != old {
                        dirty = true;
                    }

                    if ev == ButtonEvent::Back {
                        state.screen = Screen::Home;
                        dirty = true;
                    }

                    if ev == ButtonEvent::Confirm {
                        // Enter Library: mount + list books once.
                        // TODO: Re-enable once async SD + FAT adapter is complete
                        // let ok = scan_books_from_sd(&mut volume_manager, &mut state.books);
                        // state.sd_present = ok;
                        state.sd_present = false; // SD not yet working with async
                        state.library_cursor = 0;
                        state.screen = Screen::Library;
                        dirty = true;
                    }
                }
                Screen::Library => {
                    let old = state.library_cursor;
                    if !state.books.is_empty() {
                        match state.settings.books_view_mode {
                            ui::ViewMode::List => {
                                match ev {
                                    ButtonEvent::Up => {
                                        if state.library_cursor > 0 {
                                            state.library_cursor -= 1;
                                        }
                                    }
                                    ButtonEvent::Down => {
                                        if state.library_cursor + 1 < state.books.len() {
                                            state.library_cursor += 1;
                                        }
                                    }
                                    _ => {}
                                }
                            }
                            ui::ViewMode::Grid => {
                                let page_start = (state.library_cursor / 6) * 6;
                                let end = (page_start + 6).min(state.books.len());
                                let page_len = end - page_start;
                                let selected_in_page = state.library_cursor - page_start;

                                let col = selected_in_page % 2;
                                let row = selected_in_page / 2;
                                let mut new_in_page = selected_in_page;

                                match ev {
                                    ButtonEvent::Up => {
                                        if row > 0 {
                                            grid_move(&mut new_in_page, 0, -1, page_len);
                                        } else if page_start >= 6 {
                                            let prev_start = page_start - 6;
                                            let prev_end = (prev_start + 6).min(state.books.len());
                                            let prev_len = prev_end - prev_start;
                                            new_in_page = (4 + col).min(prev_len.saturating_sub(1));
                                            state.library_cursor = prev_start + new_in_page;
                                        }
                                    }
                                    ButtonEvent::Down => {
                                        if row < 2 && selected_in_page + 2 < page_len {
                                            grid_move(&mut new_in_page, 0, 1, page_len);
                                        } else if page_start + 6 < state.books.len() {
                                            let next_start = page_start + 6;
                                            let next_end = (next_start + 6).min(state.books.len());
                                            let next_len = next_end - next_start;
                                            new_in_page = col.min(next_len.saturating_sub(1));
                                            state.library_cursor = next_start + new_in_page;
                                        }
                                    }
                                    ButtonEvent::Left => {
                                        grid_move(&mut new_in_page, -1, 0, page_len);
                                    }
                                    ButtonEvent::Right => {
                                        grid_move(&mut new_in_page, 1, 0, page_len);
                                    }
                                    _ => {}
                                }

                                // If we didn't already jump pages, apply intra-page move.
                                if state.library_cursor == page_start + selected_in_page {
                                    state.library_cursor = page_start + new_in_page;
                                }
                            }
                        }
                    }

                    if state.library_cursor != old {
                        dirty = true;
                    }

                    if ev == ButtonEvent::Back {
                        state.screen = Screen::Apps;
                        dirty = true;
                    }

                    if ev == ButtonEvent::Confirm {
                        if let Some(name) = state.books.get(state.library_cursor) {
                            info!("Selected book: {}", name.as_str());
                        }
                    }
                }
                Screen::Tools => {
                    if ev == ButtonEvent::Back {
                        state.screen = Screen::Home;
                        dirty = true;
                    }
                }
                Screen::Settings => {
                    let old = state.settings_selected;
                    nav_move(
                        &mut state.settings_selected,
                        ev,
                        ui::ViewMode::List,
                        SettingsItem::all().len(),
                    );
                    if state.settings_selected != old {
                        dirty = true;
                    }

                    if ev == ButtonEvent::Back {
                        state.screen = Screen::Home;
                        dirty = true;
                    }

                    if ev == ButtonEvent::Confirm {
                        match SettingsItem::all()[state.settings_selected] {
                            SettingsItem::Reader => {
                                state.screen = Screen::SettingsReader;
                                dirty = true;
                            }
                            SettingsItem::Wifi => {
                                state.screen = Screen::SettingsWifi;
                                dirty = true;
                            }
                            SettingsItem::Rotation => {
                                state.settings.rotation = match state.settings.rotation {
                                    ui::Rotation::Portrait => ui::Rotation::Landscape,
                                    ui::Rotation::Landscape => ui::Rotation::Portrait,
                                };
                                dirty = true;
                            }
                        }
                    }

                    if (ev == ButtonEvent::Left || ev == ButtonEvent::Right)
                        && SettingsItem::all()[state.settings_selected] == SettingsItem::Rotation
                    {
                        state.settings.rotation = match state.settings.rotation {
                            ui::Rotation::Portrait => ui::Rotation::Landscape,
                            ui::Rotation::Landscape => ui::Rotation::Portrait,
                        };
                        dirty = true;
                    }
                }
                Screen::SettingsWifi => {
                    let old = state.settings_wifi_selected;
                    nav_move(
                        &mut state.settings_wifi_selected,
                        ev,
                        ui::ViewMode::List,
                        7,
                    );
                    if state.settings_wifi_selected != old {
                        dirty = true;
                    }

                    if ev == ButtonEvent::Back {
                        state.screen = Screen::Settings;
                        dirty = true;
                    }

                    if ev == ButtonEvent::Confirm {
                        match state.settings_wifi_selected {
                            0 => {
                                // SSID cycle for demo or future keyboard
                                if state.wifi_ssid == "dummy" {
                                    state.wifi_ssid = String::from_str("YourSSID").unwrap_or_default();
                                } else {
                                    state.wifi_ssid = String::from_str("dummy").unwrap_or_default();
                                }
                                ariel_os::hal::wifi::set_credentials(state.wifi_ssid.as_str(), state.wifi_password.as_str());
                                dirty = true;
                            }
                            1 => {
                                // Password cycle for demo
                                if state.wifi_password == "dummy" {
                                    state.wifi_password = String::from_str("YourPassword").unwrap_or_default();
                                } else {
                                    state.wifi_password = String::from_str("dummy").unwrap_or_default();
                                }
                                ariel_os::hal::wifi::set_credentials(state.wifi_ssid.as_str(), state.wifi_password.as_str());
                                dirty = true;
                            }
                            6 => {
                                #[cfg(feature = "wifi-esp")]
                                ariel_os::hal::wifi::esp_wifi::reconnect();
                                info!("Reconnection triggered");
                            }
                            _ => {}
                        }
                    }
                }
                Screen::SettingsReader => {
                    let old = state.settings_reader_selected;
                    nav_move(
                        &mut state.settings_reader_selected,
                        ev,
                        ui::ViewMode::List,
                        ReaderSettingsItem::all().len(),
                    );
                    if state.settings_reader_selected != old {
                        dirty = true;
                    }

                    if ev == ButtonEvent::Back {
                        state.screen = Screen::Settings;
                        dirty = true;
                    }

                    if ev == ButtonEvent::Confirm || ev == ButtonEvent::Left || ev == ButtonEvent::Right {
                        match ReaderSettingsItem::all()[state.settings_reader_selected] {
                            ReaderSettingsItem::BooksView => {
                                state.settings.books_view_mode = match state.settings.books_view_mode {
                                    ui::ViewMode::List => ui::ViewMode::Grid,
                                    ui::ViewMode::Grid => ui::ViewMode::List,
                                };
                                dirty = true;
                            }
                        }
                    }
                }
            }
        }

        if dirty {
            render_state(&mut display, &state, ssd1677::RefreshMode::Fast).await;
        }

        ariel_os::time::Timer::after(ariel_os::time::Duration::from_millis(80)).await;
    }
}
