use ariel_os::hal::{peripherals, spi};

#[cfg(context = "xteink-x4")]
pub type DisplaySpi = spi::main::SPI2;

#[cfg(context = "xteink-x4")]
ariel_os::hal::define_peripherals!(Peripherals {
    spi_sck: GPIO8,
    spi_miso: GPIO7,
    spi_mosi: GPIO10,
    spi_cs: GPIO21,
    eink_dc: GPIO4,
    eink_rst: GPIO5,
    eink_busy: GPIO6,
});
