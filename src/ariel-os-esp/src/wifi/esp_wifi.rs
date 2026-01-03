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
use embassy_sync::signal::Signal;
use heapless::{String, Vec};

pub type NetworkDevice = WifiDevice<'static, WifiStaDevice>;

pub static RECONNECT: AtomicBool = AtomicBool::new(false);
pub static SCAN_REQUESTED: AtomicBool = AtomicBool::new(false);
pub static SCAN_RUNNING: AtomicBool = AtomicBool::new(false);
static WAKEUP_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

pub struct ScanResult {
    pub ssid: String<32>,
    pub rssi: i8,
}

pub static SCAN_RESULTS: AsyncMutex<CriticalSectionRawMutex, Vec<ScanResult, 16>> =
    AsyncMutex::new(Vec::new());

pub fn reconnect() {
    RECONNECT.store(true, Ordering::SeqCst);
    WAKEUP_SIGNAL.signal(());
}

pub fn request_scan() {
    SCAN_REQUESTED.store(true, Ordering::SeqCst);
    WAKEUP_SIGNAL.signal(());
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
        WAKEUP_SIGNAL.reset();

        if RECONNECT.load(Ordering::SeqCst) {
            RECONNECT.store(false, Ordering::SeqCst);
            info!("Reconnection requested, restarting Wi-Fi...");
            let _ = controller.stop_async().await;
        }

        if SCAN_REQUESTED.load(Ordering::SeqCst) {
            SCAN_REQUESTED.store(false, Ordering::SeqCst);
            SCAN_RUNNING.store(true, Ordering::SeqCst);
            info!("Scanning for Wi-Fi networks...");
            
            // We need to be started to scan
            if !matches!(controller.is_started(), Ok(true)) {
                let _ = controller.start_async().await;
            }

            match controller.scan_n_async::<16>().await {
                Ok((networks, _count)) => {
                    let mut results_guard = SCAN_RESULTS.lock().await;
                    let results: &mut Vec<ScanResult, 16> = &mut *results_guard;
                    results.clear();
                    for net in networks {
                        let mut ssid = String::new();
                        let ssid_str = net.ssid.as_str();
                        let _ = ssid.push_str(ssid_str);
                        info!("Found network: SSID=\"{}\", RSSI={}", ssid_str, net.signal_strength);
                        let _ = results.push(ScanResult {
                            ssid,
                            rssi: net.signal_strength,
                        });
                    }
                    info!("Scan complete: found {} networks", results.len());
                }
                Err(e) => {
                    info!("Scan failed: {:?}", e);
                }
            }

            SCAN_RUNNING.store(false, Ordering::SeqCst);
        }

        match esp_wifi::wifi::wifi_state() {
            WifiState::StaConnected => {
                // wait until we're no longer connected or a signal arrives
                embassy_futures::select::select(
                    controller.wait_for_event(WifiEvent::StaDisconnected),
                    WAKEUP_SIGNAL.wait()
                ).await;
                Timer::after(Duration::from_secs(5)).await
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            debug!("Starting Wi-Fi radio...");
            // Always start the radio so background scanning and status checks work
            if let Err(e) = controller.start_async().await {
                info!("Failed to start Wi-Fi radio: {:?}", e);
                Timer::after(Duration::from_secs(5)).await;
                continue;
            }
            debug!("Wi-Fi radio started!");
        }

        let (ssid, password) = crate::wifi::get_credentials();
        if ssid.is_empty() {
            debug!("No SSID configured, waiting for user input or scan request...");
            // Wait for a signal (reconnect/scan) or a timeout
            embassy_futures::select::select(
                WAKEUP_SIGNAL.wait(),
                Timer::after(Duration::from_secs(30))
            ).await;
            continue;
        }

        // Apply configuration if started but not connected
        if esp_wifi::wifi::wifi_state() != WifiState::StaConnected {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: ssid.as_str().try_into().unwrap(),
                password: password.as_str().try_into().unwrap(),
                ..Default::default()
            });
            if let Err(e) = controller.set_configuration(&client_config) {
                info!("Failed to set Wi-Fi config: {:?}", e);
                Timer::after(Duration::from_secs(5)).await;
                continue;
            }
        }

        debug!("About to connect to {}...", ssid);

        match embassy_futures::select::select(
            controller.connect_async(),
            WAKEUP_SIGNAL.wait()
        ).await {
            embassy_futures::select::Either::First(Ok(_)) => info!("Wifi connected!"),
            embassy_futures::select::Either::First(Err(e)) => {
                info!("Failed to connect to Wi-Fi: {:?}", e);
                embassy_futures::select::select(
                    WAKEUP_SIGNAL.wait(),
                    Timer::after(Duration::from_millis(5000))
                ).await;
            }
            embassy_futures::select::Either::Second(_) => {
                debug!("Connection interrupted by signal");
            }
        }
    }
}
