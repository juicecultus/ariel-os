use ariel_os::hal::{peripherals, spi};

#[cfg(context = "xteink-x4")]
pub type SharedSpi = spi::main::SPI2;

#[cfg(context = "xteink-x4")]
ariel_os::hal::define_peripherals!(Peripherals {
    // Shared SPI2 bus (display + SD)
    spi_sck: GPIO8,
    spi_miso: GPIO7,
    spi_mosi: GPIO10,

    // Display
    display_cs: GPIO21,
    display_dc: GPIO4,
    display_rst: GPIO5,
    display_busy: GPIO6,

    // SD
    sd_cs: GPIO12,

    // Buttons
    adc1: ADC1,
    adc_buttons_1: GPIO1,
    adc_buttons_2: GPIO2,
    power_button: GPIO3,
});
