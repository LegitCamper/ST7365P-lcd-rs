use embedded_hal_async::spi::{ErrorType, Operation, SpiDevice};

/// Selects a read-safe or write configuration before each SPI transaction.
///
/// `configure` receives the device and whether any operation reads data. It must
/// configure an exclusively owned, idle bus synchronously, before CS is asserted.
/// This wrapper cannot configure a shared bus outside that bus's locking scope.
/// A mixed command/read transaction stays entirely at the read-safe clock.
///
/// Every transaction reconfigures the bus, including after an error or dropped
/// future. Cancellation safety of CS and DMA remains the inner device's
/// responsibility: recover an interrupted transport before starting another
/// transaction if its implementation requires that. Include configuration time
/// in synchronized drawing budgets.
pub struct ReadWriteSpi<SPI, Configure> {
    device: SPI,
    configure: Configure,
}

impl<SPI, Configure> ReadWriteSpi<SPI, Configure>
where
    SPI: SpiDevice,
    Configure: FnMut(&mut SPI, bool) -> Result<(), SPI::Error>,
{
    pub fn new(device: SPI, configure: Configure) -> Self {
        Self { device, configure }
    }

    pub fn device_mut(&mut self) -> &mut SPI {
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
        (self.configure)(&mut self.device, reads)?;
        self.device.transaction(operations).await
    }
}
