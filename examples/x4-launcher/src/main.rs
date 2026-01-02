#![no_main]
#![no_std]

mod pins;
mod sd;
mod ssd1677;

use ariel_os::{
    debug::log::info,
    gpio::{Input, Level, Output, Pull},
};

use embassy_sync::mutex::Mutex;
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use embedded_sdmmc::{SdCard, TimeSource, VolumeManager};
use heapless::String;
use once_cell::sync::OnceCell;
use static_cell::StaticCell;

use esp_hal::analog::adc::{Adc, AdcCalCurve, AdcConfig, AdcPin, Attenuation};
use esp_hal::gpio::Input as EspInput;
use esp_hal::peripherals::ADC1;

pub static SPI_BUS: OnceCell<
    Mutex<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, ariel_os::hal::spi::main::Spi>,
> = OnceCell::new();

#[derive(Clone, Copy, PartialEq, Eq)]
enum Screen {
    Home,
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

struct InputManager<'d, PIN1, PIN2> {
    adc1: Adc<'d, ADC1>,
    pin1: AdcPin<PIN1, ADC1, AdcCalCurve<ADC1>>,
    pin2: AdcPin<PIN2, ADC1, AdcCalCurve<ADC1>>,
    power_pin: EspInput<'d>,
    last_state: u8,
}

impl<'d, PIN1, PIN2> InputManager<'d, PIN1, PIN2>
where
    PIN1: esp_hal::analog::adc::AdcChannel + esp_hal::gpio::AnalogPin,
    PIN2: esp_hal::analog::adc::AdcChannel + esp_hal::gpio::AnalogPin,
{
    fn new(adc1: ADC1, pin1: PIN1, pin2: PIN2, power_pin: EspInput<'d>) -> Self {
        let mut config = AdcConfig::new();
        let adc_pin1 = config.enable_pin_with_cal::<_, AdcCalCurve<ADC1>>(pin1, Attenuation::_11dB);
        let adc_pin2 = config.enable_pin_with_cal::<_, AdcCalCurve<ADC1>>(pin2, Attenuation::_11dB);
        let adc1 = Adc::new(adc1, config);
        Self {
            adc1,
            pin1: adc_pin1,
            pin2: adc_pin2,
            power_pin,
            last_state: 0,
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

struct SdTimeSource;

impl TimeSource for SdTimeSource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp::from_fat(0, 0)
    }
}

fn draw_home<D>(display: &mut ssd1677::Ssd1677<D>, selected: usize)
where
    D: embedded_hal_async::spi::SpiDevice<u8>,
{
    display.clear_buffer(BinaryColor::Off);

    let title_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
    Text::new("Home", Point::new(200, 60), title_style)
        .draw(display)
        .ok();

    for (i, item) in HomeItem::all().iter().enumerate() {
        let y = 140 + (i as i32 * 60);
        let prefix = if i == selected { ">" } else { " " };
        let mut line: String<32> = String::new();
        let _ = line.push_str(prefix);
        let _ = line.push_str(" ");
        let _ = line.push_str(item.label());
        Text::new(line.as_str(), Point::new(120, y), title_style)
            .draw(display)
            .ok();
    }
}

async fn render_home<D>(display: &mut ssd1677::Ssd1677<D>, selected: usize, mode: ssd1677::RefreshMode)
where
    D: embedded_hal_async::spi::SpiDevice<u8>,
{
    draw_home(display, selected);
    display.flush().await;
    display.refresh(mode).await;
}

#[ariel_os::task(autostart, peripherals)]
async fn main(peripherals: pins::Peripherals) {
    info!("x4-launcher starting...");

    let spi_bus = pins::SharedSpi::new(
        peripherals.spi_sck,
        peripherals.spi_miso,
        peripherals.spi_mosi,
        ssd1677::default_spi_config(),
    );

    let _ = SPI_BUS.set(Mutex::new(spi_bus));

    let display_cs = Output::new(peripherals.display_cs, Level::High);
    let display_spi = ariel_os::spi::main::SpiDevice::new(SPI_BUS.get().unwrap(), display_cs);

    let sd_cs = Output::new(peripherals.sd_cs, Level::High);
    let sd_spi_async = ariel_os::spi::main::SpiDevice::new(SPI_BUS.get().unwrap(), sd_cs);
    let sd_spi = sd::BlockingSpiDevice::new(sd_spi_async);

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
        power_in,
    );

    let delay = ariel_os::time::Delay;
    let sd_card = SdCard::new(sd_spi, delay);
    let mut volume_manager: VolumeManager<_, SdTimeSource, 4, 4, 1> =
        VolumeManager::new(sd_card, SdTimeSource);

    // Mark it used for now; SD listing will come next.
    let _ = &mut volume_manager;

    let mut selected: usize = 0;
    let screen = Screen::Home;

    info!("Rendering home screen...");
    render_home(&mut display, selected, ssd1677::RefreshMode::Full).await;
    info!("Home screen rendered");

    loop {
        match screen {
            Screen::Home => {
                let old_selected = selected;
                for ev in input.poll().iter().copied() {
                    match ev {
                        ButtonEvent::Up => {
                            if selected > 0 {
                                selected -= 1;
                            }
                        }
                        ButtonEvent::Down => {
                            if selected + 1 < HomeItem::all().len() {
                                selected += 1;
                            }
                        }
                        ButtonEvent::Confirm => {
                            info!("Selected home item: {}", selected);
                        }
                        _ => {}
                    }
                }

                if selected != old_selected {
                    render_home(&mut display, selected, ssd1677::RefreshMode::Fast).await;
                }

                ariel_os::time::Timer::after(ariel_os::time::Duration::from_millis(80)).await;
            }
        }
    }
}
