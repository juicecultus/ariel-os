use core::sync::atomic::{AtomicBool, Ordering};
use ariel_os_debug::log::{debug, info};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_wifi::{
    EspWifiController,
    config::PowerSaveMode,
    wifi::{
        ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiStaDevice,
        WifiState,
    },
};
use once_cell::sync::OnceCell;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex as AsyncMutex;
use heapless::{String, Vec};

pub type NetworkDevice = WifiDevice<'static, WifiStaDevice>;

pub static RECONNECT: AtomicBool = AtomicBool::new(false);
pub static SCAN_REQUESTED: AtomicBool = AtomicBool::new(false);

pub struct ScanResult {
    pub ssid: String<32>,
    pub rssi: i8,
}

pub static SCAN_RESULTS: AsyncMutex<CriticalSectionRawMutex, Vec<ScanResult, 16>> =
    AsyncMutex::new(Vec::new());

pub fn reconnect() {
    RECONNECT.store(true, Ordering::SeqCst);
}

pub fn request_scan() {
    SCAN_REQUESTED.store(true, Ordering::SeqCst);
}

// Ideally, all Wi-Fi initialization would happen here.
// Unfortunately that's complicated, so we're using WIFI_INIT to pass the
// `EspWifiInitialization` from `crate::init()`.
// Using a `once_cell::OnceCell` here for critical-section support, just to be
// sure.
pub static WIFI_INIT: OnceCell<EspWifiController<'_>> = OnceCell::new();

pub fn init(peripherals: &mut crate::OptionalPeripherals, spawner: Spawner) -> NetworkDevice {
    let wifi = peripherals.WIFI.take().unwrap();
    let init = WIFI_INIT.get().unwrap();
    let (device, mut controller) =
        esp_wifi::wifi::new_with_mode(init, wifi, WifiStaDevice).unwrap();

    controller.set_power_saving(PowerSaveMode::None).unwrap();

    spawner.spawn(connection(controller)).ok();

    device
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    debug!("start connection task");

    #[cfg(not(feature = "defmt"))]
    debug!("Device capabilities: {:?}", controller.capabilities());

    loop {
        if RECONNECT.load(Ordering::SeqCst) {
            RECONNECT.store(false, Ordering::SeqCst);
            info!("Reconnection requested, restarting Wi-Fi...");
            let _ = controller.stop_async().await;
        }

        if SCAN_REQUESTED.load(Ordering::SeqCst) {
            SCAN_REQUESTED.store(false, Ordering::SeqCst);
            info!("Scanning for Wi-Fi networks...");
            
            // We need to be started to scan
            let was_started = matches!(controller.is_started(), Ok(true));
            if !was_started {
                let _ = controller.start_async().await;
            }

            match controller.scan_n_async::<16>().await {
                Ok((networks, _count)) => {
                    let mut results_guard = SCAN_RESULTS.lock().await;
                    let results: &mut Vec<ScanResult, 16> = &mut *results_guard;
                    results.clear();
                    for net in networks {
                        let mut ssid = String::new();
                        let _ = ssid.push_str(net.ssid.as_str());
                        let _ = results.push(ScanResult {
                            ssid,
                            rssi: net.signal_strength,
                        });
                    }
                    info!("Found {} networks", results.len());
                }
                Err(e) => {
                    info!("Scan failed: {:?}", e);
                }
            }

            if !was_started {
                let _ = controller.stop_async().await;
            }
        }

        match esp_wifi::wifi::wifi_state() {
            WifiState::StaConnected => {
                // wait until we're no longer connected
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_secs(5)).await
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            debug!("Configuring Wi-Fi");
            let (ssid, password) = crate::wifi::get_credentials();
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: ssid.as_str().try_into().unwrap(),
                password: password.as_str().try_into().unwrap(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            debug!("Starting Wi-Fi");
            controller.start_async().await.unwrap();
            debug!("Wi-Fi started!");
        }
        debug!("About to connect...");

        match controller.connect_async().await {
            Ok(_) => info!("Wifi connected!"),
            Err(e) => {
                info!("Failed to connect to Wi-Fi: {:?}", e);
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}
