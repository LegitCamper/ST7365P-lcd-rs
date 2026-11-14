use super::*;
use crate::spi::ReadWriteSpi;

#[derive(Default)]
struct ClockSpi {
    modes: Vec<bool>,
    transaction_sizes: Vec<usize>,
    fail_config: bool,
    fail_transfer: bool,
    suspend: bool,
}

impl ErrorType for ClockSpi {
    type Error = ErrorKind;
}

impl SpiDevice for ClockSpi {
    async fn transaction(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        self.transaction_sizes.push(operations.len());
        if self.suspend {
            pending::<()>().await;
        }
        if self.fail_transfer {
            return Err(ErrorKind::Other);
        }
        Ok(())
    }
}

fn configure(device: &mut ClockSpi, reads: bool) -> Result<(), ErrorKind> {
    device.modes.push(reads);
    if device.fail_config {
        return Err(ErrorKind::Other);
    }
    Ok(())
}

#[test]
fn clock_selection_keeps_mixed_reads_in_one_transaction() {
    let mut spi = ReadWriteSpi::new(ClockSpi::default(), configure);
    let mut response = [0; 2];
    run(spi.transaction(&mut [
        Operation::DelayNs(60),
        Operation::Write(&[0x45]),
        Operation::Read(&mut response),
        Operation::DelayNs(65),
    ]))
    .unwrap();
    run(spi.write(&[0x12, 0x34])).unwrap();
    run(spi.transfer(&mut response, &[0x12, 0x34])).unwrap();
    run(spi.transfer_in_place(&mut response)).unwrap();
    run(spi.transaction(&mut [Operation::DelayNs(1)])).unwrap();
    let device = spi.into_inner();
    assert_eq!(device.modes, [true, false, true, true, false]);
    assert_eq!(device.transaction_sizes, [4, 1, 1, 1, 1]);
}

#[test]
fn configuration_error_stops_transaction_and_next_call_reconfigures() {
    let mut spi = ReadWriteSpi::new(
        ClockSpi {
            fail_config: true,
            ..ClockSpi::default()
        },
        configure,
    );
    assert_eq!(run(spi.read(&mut [0])), Err(ErrorKind::Other));
    assert!(spi.device_mut().transaction_sizes.is_empty());
    spi.device_mut().fail_config = false;
    run(spi.write(&[0])).unwrap();
    assert_eq!(spi.device_mut().modes, [true, false]);
}

#[test]
fn failed_and_cancelled_transfers_do_not_cache_clock_selection() {
    let mut spi = ReadWriteSpi::new(
        ClockSpi {
            fail_transfer: true,
            ..ClockSpi::default()
        },
        configure,
    );
    assert_eq!(run(spi.read(&mut [0])), Err(ErrorKind::Other));
    spi.device_mut().fail_transfer = false;
    run(spi.write(&[0])).unwrap();
    spi.device_mut().suspend = true;
    {
        let mut response = [0];
        let mut transfer = std::pin::pin!(spi.read(&mut response));
        let mut context = Context::from_waker(Waker::noop());
        assert!(transfer.as_mut().poll(&mut context).is_pending());
    }
    spi.device_mut().suspend = false;
    run(spi.write(&[0])).unwrap();
    assert_eq!(spi.device_mut().modes, [true, false, true, false]);
    assert_eq!(spi.device_mut().transaction_sizes, [1, 1, 1, 1]);
}
