use ariel_os_utils::str_from_env_or;
use core::cell::RefCell;
use critical_section::Mutex;
use heapless::String;

#[cfg(feature = "wifi-esp")]
pub mod esp_wifi;

// Internal runtime storage for credentials
pub(crate) static RUNTIME_CONFIG: Mutex<RefCell<WifiConfig>> =
    Mutex::new(RefCell::new(WifiConfig::new()));

pub(crate) struct WifiConfig {
    pub ssid: String<32>,
    pub password: String<64>,
}

impl WifiConfig {
    const fn new() -> Self {
        Self {
            ssid: String::new(),
            password: String::new(),
        }
    }
}

use core::str::FromStr;

pub fn set_credentials(ssid: &str, password: &str) {
    critical_section::with(|cs| {
        let mut config = RUNTIME_CONFIG.borrow(cs).borrow_mut();
        config.ssid = String::from_str(ssid).unwrap_or_default();
        config.password = String::from_str(password).unwrap_or_default();
    });
}

pub(crate) fn get_credentials() -> (String<32>, String<64>) {
    critical_section::with(|cs| {
        let config = RUNTIME_CONFIG.borrow(cs).borrow();
        if config.ssid.is_empty() {
            // Fallback to compile-time defaults
            (
                String::from_str(WIFI_NETWORK).unwrap_or_default(),
                String::from_str(WIFI_PASSWORD).unwrap_or_default(),
            )
        } else {
            (config.ssid.clone(), config.password.clone())
        }
    })
}

// TODO: this should be factored out in ariel-os-embassy again
pub const WIFI_NETWORK: &str =
    str_from_env_or!("CONFIG_WIFI_NETWORK", "", "Wi-Fi SSID (network name)");
pub const WIFI_PASSWORD: &str =
    str_from_env_or!("CONFIG_WIFI_PASSWORD", "", "Wi-Fi password");
