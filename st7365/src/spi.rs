use embedded_hal_async::spi::{ErrorType, Operation, SpiDevice};

/// Selects and caches a read-safe or write configuration for SPI transactions.
///
/// `configure` receives the device and whether any operation reads data. It must
/// configure an exclusively owned, idle bus synchronously, before CS is asserted.
/// This wrapper cannot configure a shared bus outside that bus's locking scope.
/// A mixed command/read transaction stays entirely at the read-safe clock.
///
/// Successful consecutive transactions with the same direction reuse the last
/// configuration. Errors, cancellation, and mutable device access invalidate
/// the cache, so the next transaction configures the bus again. Cancellation
/// safety of CS and DMA remains the inner device's responsibility: recover an
/// interrupted transport before starting another transaction if required.
pub struct ReadWriteSpi<SPI, Configure> {
    device: SPI,
    configure: Configure,
    configured_for_reads: Option<bool>,
}

impl<SPI, Configure> ReadWriteSpi<SPI, Configure>
where
    SPI: SpiDevice,
    Configure: FnMut(&mut SPI, bool) -> Result<(), SPI::Error>,
{
    pub fn new(device: SPI, configure: Configure) -> Self {
        Self {
            device,
            configure,
            configured_for_reads: None,
        }
    }

    /// Invalidates the configuration cache before exposing the inner device.
    pub fn device_mut(&mut self) -> &mut SPI {
        self.configured_for_reads = None;
        &mut self.device
    }

    pub fn into_inner(self) -> SPI {
        self.device
    }
}

impl<SPI: ErrorType, Configure> ErrorType for ReadWriteSpi<SPI, Configure> {
    type Error = SPI::Error;
}

impl<SPI, Configure> SpiDevice for ReadWriteSpi<SPI, Configure>
where
    SPI: SpiDevice,
    Configure: FnMut(&mut SPI, bool) -> Result<(), SPI::Error>,
{
    async fn transaction(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        let reads = operations.iter().any(|operation| {
            matches!(
                operation,
                Operation::Read(_) | Operation::Transfer(_, _) | Operation::TransferInPlace(_)
            )
        });
        if self.configured_for_reads.take() != Some(reads) {
            (self.configure)(&mut self.device, reads)?;
        }
        let result = self.device.transaction(operations).await;
        if result.is_ok() {
            self.configured_for_reads = Some(reads);
        }
        result
    }
}
