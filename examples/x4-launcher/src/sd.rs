use embassy_futures::block_on;

use embedded_hal::spi::{ErrorType, Operation as BlockingOperation, SpiDevice as BlockingSpiDeviceTrait};
use embedded_hal_async::spi::Operation as AsyncOperation;

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
        // Perform operations sequentially. This avoids lifetime/ownership complexity
        // while still preserving transaction semantics.
        for op in operations.iter_mut() {
            match op {
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
