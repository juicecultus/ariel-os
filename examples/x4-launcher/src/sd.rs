use embassy_futures::block_on;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

use embedded_hal::digital::OutputPin;
use embedded_hal::spi::{ErrorType, Operation, SpiBus, SpiDevice as BlockingSpiDeviceTrait};

/// Blocking SPI device that wraps an async Mutex<SpiBus> and manages CS pin.
/// This mimics embedded-hal-bus::RefCellDevice behavior but works with async Mutex.
pub struct BlockingSpiDeviceWithCs<'a, B, CS> {
    bus: &'a Mutex<CriticalSectionRawMutex, B>,
    cs: CS,
}

impl<'a, B, CS> BlockingSpiDeviceWithCs<'a, B, CS>
where
    CS: OutputPin,
{
    pub fn new(bus: &'a Mutex<CriticalSectionRawMutex, B>, mut cs: CS) -> Self {
        let _ = cs.set_high();
        Self { bus, cs }
    }
}

impl<'a, B, CS> ErrorType for BlockingSpiDeviceWithCs<'a, B, CS>
where
    B: embedded_hal_async::spi::ErrorType,
{
    type Error = B::Error;
}

impl<'a, B, CS> BlockingSpiDeviceTrait<u8> for BlockingSpiDeviceWithCs<'a, B, CS>
where
    B: embedded_hal_async::spi::SpiBus<u8>,
    CS: OutputPin,
{
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        // Lock the bus, set CS low, execute operations, flush, set CS high
        let mut bus = block_on(self.bus.lock());
        
        let _ = self.cs.set_low();
        
        let result = operations.iter_mut().try_for_each(|op| match op {
            Operation::Read(buf) => block_on(embedded_hal_async::spi::SpiBus::read(&mut *bus, buf)),
            Operation::Write(buf) => block_on(embedded_hal_async::spi::SpiBus::write(&mut *bus, buf)),
            Operation::Transfer(read, write) => block_on(embedded_hal_async::spi::SpiBus::transfer(&mut *bus, read, write)),
            Operation::TransferInPlace(buf) => block_on(embedded_hal_async::spi::SpiBus::transfer_in_place(&mut *bus, buf)),
            Operation::DelayNs(ns) => {
                let _ = block_on(embedded_hal_async::spi::SpiBus::flush(&mut *bus));
                esp_hal::delay::Delay::new().delay_nanos(*ns);
                Ok(())
            }
        });
        
        // Always flush and deassert CS, even on error
        let _ = block_on(embedded_hal_async::spi::SpiBus::flush(&mut *bus));
        let _ = self.cs.set_high();
        
        result
    }
}

// Keep the old adapters for reference
use embedded_hal::spi::Operation as BlockingOperation;
use embedded_hal_async::spi::Operation as AsyncOperation;
use heapless::Vec;

pub struct BlockingSpiDevice<D> {
    inner: D,
}

impl<D> BlockingSpiDevice<D> {
    pub fn new(inner: D) -> Self {
        Self { inner }
    }
}

impl<D> ErrorType for BlockingSpiDevice<D>
where
    D: embedded_hal_async::spi::ErrorType,
{
    type Error = D::Error;
}

impl<D> BlockingSpiDeviceTrait<u8> for BlockingSpiDevice<D>
where
    D: embedded_hal_async::spi::SpiDevice<u8>,
{
    fn transaction(&mut self, operations: &mut [BlockingOperation<'_, u8>]) -> Result<(), Self::Error> {
        // Preserve transaction semantics: CS must remain asserted for the whole operation list.
        // embedded-sdmmc depends on this.
        {
            let mut async_ops: Vec<AsyncOperation<'_, u8>, 64> = Vec::new();
            let mut overflowed = false;

            for i in 0..operations.len() {
                let mapped = match &mut operations[i] {
                    BlockingOperation::Read(buf) => {
                        let p = (*buf).as_mut_ptr();
                        let len = (*buf).len();
                        let s = unsafe { core::slice::from_raw_parts_mut(p, len) };
                        AsyncOperation::Read(s)
                    }
                    BlockingOperation::Write(buf) => {
                        let p = (*buf).as_ptr();
                        let len = (*buf).len();
                        let s = unsafe { core::slice::from_raw_parts(p, len) };
                        AsyncOperation::Write(s)
                    }
                    BlockingOperation::Transfer(read, write) => {
                        let rp = (*read).as_mut_ptr();
                        let rlen = (*read).len();
                        let r = unsafe { core::slice::from_raw_parts_mut(rp, rlen) };
                        let wp = (*write).as_ptr();
                        let wlen = (*write).len();
                        let w = unsafe { core::slice::from_raw_parts(wp, wlen) };
                        AsyncOperation::Transfer(r, w)
                    }
                    BlockingOperation::TransferInPlace(buf) => {
                        let p = (*buf).as_mut_ptr();
                        let len = (*buf).len();
                        let s = unsafe { core::slice::from_raw_parts_mut(p, len) };
                        AsyncOperation::TransferInPlace(s)
                    }
                    BlockingOperation::DelayNs(ns) => AsyncOperation::DelayNs(*ns),
                };

                if async_ops.push(mapped).is_err() {
                    overflowed = true;
                    break;
                }
            }

            if !overflowed {
                return block_on(self.inner.transaction(async_ops.as_mut_slice()));
            }
        }

        // Fallback: execute sequential transactions. This may deassert CS between operations,
        // but avoids hard failure if the op list is unexpectedly large.
        for i in 0..operations.len() {
            match &mut operations[i] {
                BlockingOperation::Read(buf) => {
                    let mut ops = [AsyncOperation::Read(*buf)];
                    block_on(self.inner.transaction(&mut ops))?;
                }
                BlockingOperation::Write(buf) => {
                    let mut ops = [AsyncOperation::Write(*buf)];
                    block_on(self.inner.transaction(&mut ops))?;
                }
                BlockingOperation::Transfer(read, write) => {
                    let mut ops = [AsyncOperation::Transfer(*read, *write)];
                    block_on(self.inner.transaction(&mut ops))?;
                }
                BlockingOperation::TransferInPlace(buf) => {
                    let mut ops = [AsyncOperation::TransferInPlace(*buf)];
                    block_on(self.inner.transaction(&mut ops))?;
                }
                BlockingOperation::DelayNs(ns) => {
                    let mut ops = [AsyncOperation::DelayNs(*ns)];
                    block_on(self.inner.transaction(&mut ops))?;
                }
            }
        }

        Ok(())
    }
}
