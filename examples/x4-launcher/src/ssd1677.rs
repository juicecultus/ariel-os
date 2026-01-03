use ariel_os::{
    gpio,
    spi::main::{Kilohertz, highest_freq_in},
    time::{Duration, Timer},
};

use embedded_graphics::{
    draw_target::DrawTarget,
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use embedded_hal_async::spi::{Operation, SpiDevice};

pub const PHYSICAL_WIDTH: u16 = 800;
pub const PHYSICAL_HEIGHT: u16 = 480;

pub const DISPLAY_WIDTH: u16 = 480;
pub const DISPLAY_HEIGHT: u16 = 800;

pub const BUFFER_SIZE: usize = (PHYSICAL_WIDTH as usize * PHYSICAL_HEIGHT as usize) / 8;

const CMD_SOFT_RESET: u8 = 0x12;
const CMD_DRIVER_OUTPUT_CONTROL: u8 = 0x01;
const CMD_BOOSTER_SOFT_START: u8 = 0x0C;
const CMD_TEMP_SENSOR_CONTROL: u8 = 0x18;
const CMD_BORDER_WAVEFORM: u8 = 0x3C;
const CMD_DATA_ENTRY_MODE: u8 = 0x11;
const CMD_SET_RAM_X_RANGE: u8 = 0x44;
const CMD_SET_RAM_Y_RANGE: u8 = 0x45;
const CMD_SET_RAM_X_COUNTER: u8 = 0x4E;
const CMD_SET_RAM_Y_COUNTER: u8 = 0x4F;
const CMD_WRITE_RAM_BW: u8 = 0x24;
const CMD_WRITE_RAM_RED: u8 = 0x26;
const CMD_AUTO_WRITE_BW_RAM: u8 = 0x46;
const CMD_AUTO_WRITE_RED_RAM: u8 = 0x47;
const CMD_DISPLAY_UPDATE_CTRL1: u8 = 0x21;
const CMD_DISPLAY_UPDATE_CTRL2: u8 = 0x22;
const CMD_MASTER_ACTIVATION: u8 = 0x20;

const CTRL1_NORMAL: u8 = 0x00;
const CTRL1_BYPASS_RED: u8 = 0x40;

#[derive(Clone, Copy)]
pub enum RefreshMode {
    Full,
    Fast,
}

pub struct Ssd1677<SPI> {
    spi: SPI,
    dc: gpio::Output,
    rst: gpio::Output,
    busy: gpio::Input,
    buffer: &'static mut [u8; BUFFER_SIZE],
    is_screen_on: bool,
}

impl<SPI> Ssd1677<SPI>
where
    SPI: SpiDevice<u8>,
{
    pub fn new(
        spi: SPI,
        dc: gpio::Output,
        rst: gpio::Output,
        busy: gpio::Input,
        buffer: &'static mut [u8; BUFFER_SIZE],
    ) -> Self {
        buffer.fill(0xFF);
        Self {
            spi,
            dc,
            rst,
            busy,
            buffer,
            is_screen_on: false,
        }
    }

    pub async fn init(&mut self) {
        self.reset().await;

        self.send_command(CMD_SOFT_RESET).await;
        self.wait_busy().await;

        self.send_command(CMD_TEMP_SENSOR_CONTROL).await;
        let temp = [0x80];
        self.send_data(&temp).await;

        self.send_command(CMD_BOOSTER_SOFT_START).await;
        let booster = [0xAE, 0xC7, 0xC3, 0xC0, 0x40];
        self.send_data(&booster).await;

        self.send_command(CMD_DRIVER_OUTPUT_CONTROL).await;
        let driver_out = [
            ((PHYSICAL_HEIGHT - 1) % 256) as u8,
            ((PHYSICAL_HEIGHT - 1) / 256) as u8,
            0x02,
        ];
        self.send_data(&driver_out).await;

        self.send_command(CMD_BORDER_WAVEFORM).await;
        let border = [0x01];
        self.send_data(&border).await;

        self.set_ram_area(0, 0, PHYSICAL_WIDTH, PHYSICAL_HEIGHT).await;

        self.send_command(CMD_AUTO_WRITE_BW_RAM).await;
        let clear = [0xF7];
        self.send_data(&clear).await;
        self.wait_busy().await;

        self.send_command(CMD_AUTO_WRITE_RED_RAM).await;
        self.send_data(&clear).await;
        self.wait_busy().await;
    }

    pub fn clear_buffer(&mut self, color: BinaryColor) {
        let val = match color {
            BinaryColor::On => 0x00,
            BinaryColor::Off => 0xFF,
        };
        self.buffer.fill(val);
    }

    pub fn draw_center_text(&mut self, text: &str, y: i32) {
        let style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
        let x = (DISPLAY_WIDTH as i32 / 2) - ((text.len() as i32 * 10) / 2);
        Text::new(text, Point::new(x.max(0), y), style).draw(self).ok();
    }

    pub async fn flush(&mut self) {
        self.set_ram_area(0, 0, PHYSICAL_WIDTH, PHYSICAL_HEIGHT).await;
        self.send_command(CMD_WRITE_RAM_BW).await;
        self.write_buffer().await;
    }

    pub async fn refresh(&mut self, mode: RefreshMode) {
        self.send_command(CMD_DISPLAY_UPDATE_CTRL1).await;
        let ctrl1 = match mode {
            RefreshMode::Fast => [CTRL1_NORMAL, 0x00],
            RefreshMode::Full => [CTRL1_BYPASS_RED, 0x00],
        };
        self.send_data(&ctrl1).await;

        let mut display_mode: u8 = 0x00;
        if !self.is_screen_on {
            self.is_screen_on = true;
            display_mode |= 0xC0;
        }

        match mode {
            RefreshMode::Full => display_mode |= 0x34,
            RefreshMode::Fast => display_mode |= 0x1C,
        }

        self.send_command(CMD_DISPLAY_UPDATE_CTRL2).await;
        let display_mode_buf = [display_mode];
        self.send_data(&display_mode_buf).await;

        self.send_command(CMD_MASTER_ACTIVATION).await;
        self.wait_busy().await;

        self.set_ram_area(0, 0, PHYSICAL_WIDTH, PHYSICAL_HEIGHT).await;
        self.send_command(CMD_WRITE_RAM_RED).await;
        self.write_buffer().await;
    }

    async fn reset(&mut self) {
        self.rst.set_high();
        Timer::after(Duration::from_millis(20)).await;
        self.rst.set_low();
        Timer::after(Duration::from_millis(2)).await;
        self.rst.set_high();
        Timer::after(Duration::from_millis(20)).await;
    }

    async fn wait_busy(&mut self) {
        while self.busy.is_high() {
            Timer::after(Duration::from_millis(10)).await;
        }
    }

    async fn send_command(&mut self, cmd: u8) {
        self.dc.set_low();
        let cmd_buf = [cmd];
        let mut ops = [Operation::Write(&cmd_buf)];
        let _ = self.spi.transaction(&mut ops).await;
    }

    async fn send_data(&mut self, data: &[u8]) {
        self.dc.set_high();
        let mut ops = [Operation::Write(data)];
        let _ = self.spi.transaction(&mut ops).await;
    }

    async fn write_buffer(&mut self) {
        self.dc.set_high();
        let mut ops = [Operation::Write(&self.buffer[..])];
        let _ = self.spi.transaction(&mut ops).await;
    }

    async fn set_ram_area(&mut self, x: u16, mut y: u16, w: u16, h: u16) {
        y = PHYSICAL_HEIGHT - y - h;

        self.send_command(CMD_DATA_ENTRY_MODE).await;
        let data_entry = [0x01];
        self.send_data(&data_entry).await;

        self.send_command(CMD_SET_RAM_X_RANGE).await;
        let x_range = [
            (x % 256) as u8,
            (x / 256) as u8,
            ((x + w - 1) % 256) as u8,
            ((x + w - 1) / 256) as u8,
        ];
        self.send_data(&x_range).await;

        self.send_command(CMD_SET_RAM_Y_RANGE).await;
        let y_range = [
            ((y + h - 1) % 256) as u8,
            ((y + h - 1) / 256) as u8,
            (y % 256) as u8,
            (y / 256) as u8,
        ];
        self.send_data(&y_range).await;

        self.send_command(CMD_SET_RAM_X_COUNTER).await;
        let x_counter = [(x % 256) as u8, (x / 256) as u8];
        self.send_data(&x_counter).await;

        self.send_command(CMD_SET_RAM_Y_COUNTER).await;
        let y_counter = [
            ((y + h - 1) % 256) as u8,
            ((y + h - 1) / 256) as u8,
        ];
        self.send_data(&y_counter).await;
    }
}

impl<SPI> OriginDimensions for Ssd1677<SPI>
where
    SPI: SpiDevice<u8>,
{
    fn size(&self) -> Size {
        Size::new(DISPLAY_WIDTH as u32, DISPLAY_HEIGHT as u32)
    }
}

impl<SPI> DrawTarget for Ssd1677<SPI>
where
    SPI: SpiDevice<u8>,
{
    type Color = BinaryColor;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, color) in pixels {
            if point.x >= 0
                && point.x < DISPLAY_WIDTH as i32
                && point.y >= 0
                && point.y < DISPLAY_HEIGHT as i32
            {
                let x_hw = point.y as usize;
                let y_hw = (PHYSICAL_HEIGHT as i32 - point.x - 1) as usize;

                let idx = (y_hw * (PHYSICAL_WIDTH as usize / 8)) + (x_hw / 8);
                let bit = 7 - (x_hw % 8);

                if color == BinaryColor::On {
                    self.buffer[idx] &= !(1 << bit);
                } else {
                    self.buffer[idx] |= 1 << bit;
                }
            }
        }

        Ok(())
    }
}

pub fn default_spi_config() -> ariel_os::hal::spi::main::Config {
    let mut spi_config = ariel_os::hal::spi::main::Config::default();
    spi_config.frequency = const { highest_freq_in(Kilohertz::MHz(1)..=Kilohertz::MHz(20)) };
    spi_config.mode = ariel_os::spi::Mode::Mode0;
    spi_config
}

pub fn sd_spi_config() -> ariel_os::hal::spi::main::Config {
    let mut spi_config = ariel_os::hal::spi::main::Config::default();
    spi_config.frequency = const { highest_freq_in(Kilohertz::kHz(125)..=Kilohertz::kHz(500)) };
    spi_config.mode = ariel_os::spi::Mode::Mode0;
    spi_config
}
