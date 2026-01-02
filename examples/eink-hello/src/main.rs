#![no_main]
#![no_std]

mod pins;
mod ssd1677;

use ariel_os::{
    debug::{
        ExitCode, exit,
        log::{info},
    },
    gpio::{Input, Output, Pull, Level},
};

use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;

pub static SPI_BUS: once_cell::sync::OnceCell<
    Mutex<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, ariel_os::hal::spi::main::Spi>,
> = once_cell::sync::OnceCell::new();

#[ariel_os::task(autostart, peripherals)]
async fn main(peripherals: pins::Peripherals) {
    info!("Initializing SPI bus for e-ink...");

    let spi_bus = pins::DisplaySpi::new(
        peripherals.spi_sck,
        peripherals.spi_miso,
        peripherals.spi_mosi,
        ssd1677::default_spi_config(),
    );

    let _ = SPI_BUS.set(Mutex::new(spi_bus));

    let cs = Output::new(peripherals.spi_cs, Level::High);
    let spi_device = ariel_os::spi::main::SpiDevice::new(SPI_BUS.get().unwrap(), cs);

    let dc = Output::new(peripherals.eink_dc, Level::High);
    let rst = Output::new(peripherals.eink_rst, Level::High);
    let busy = Input::new(peripherals.eink_busy, Pull::None);

    static FRAMEBUF: StaticCell<[u8; ssd1677::BUFFER_SIZE]> = StaticCell::new();
    let buffer = FRAMEBUF.init_with(|| [0xFF; ssd1677::BUFFER_SIZE]);

    let mut display = ssd1677::Ssd1677::new(spi_device, dc, rst, busy, buffer);

    info!("Initializing SSD1677...");
    display.init().await;

    info!("Drawing Hello World...");
    display.draw_hello_world();
    display.flush().await;
    display.refresh(ssd1677::RefreshMode::Full).await;

    info!("Done. (If the panel didn’t change, we may need a different init sequence / rotation.)");

    exit(ExitCode::SUCCESS);
}
