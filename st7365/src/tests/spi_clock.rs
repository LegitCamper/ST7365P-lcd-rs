use super::*;
use crate::spi::ReadWriteSpi;

#[derive(Default)]
struct ClockSpi {
    modes: Vec<bool>,
    transaction_sizes: Vec<usize>,
    fail_config: bool,
    fail_transfer: bool,
    suspend: bool,
    fail_at: Option<usize>,
    suspend_at: Option<usize>,
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
        if self.suspend || self.suspend_at == Some(self.transaction_sizes.len()) {
            self.suspend = false;
            pending::<()>().await;
        }
        if self.fail_transfer || self.fail_at == Some(self.transaction_sizes.len()) {
            self.fail_transfer = false;
            return Err(ErrorKind::Other);
        }
        Ok(())
    }
}

#[test]
fn cached_clock_is_reused_and_mutable_access_invalidates_it() {
    let mut spi = ReadWriteSpi::new(ClockSpi::default(), configure);
    run(spi.write(&[0])).unwrap();
    run(spi.write(&[1])).unwrap();
    assert_eq!(spi.device_mut().modes, [false]);
    run(spi.write(&[2])).unwrap();
    assert_eq!(spi.into_inner().modes, [false, false]);
}

#[test]
fn failed_configuration_cannot_restore_previous_cached_mode() {
    let mut fail_read = true;
    let mut spi = ReadWriteSpi::new(ClockSpi::default(), |device: &mut ClockSpi, reads| {
        device.modes.push(reads);
        if reads && fail_read {
            fail_read = false;
            return Err(ErrorKind::Other);
        }
        Ok(())
    });
    run(spi.write(&[0])).unwrap();
    assert_eq!(run(spi.read(&mut [0])), Err(ErrorKind::Other));
    run(spi.write(&[0])).unwrap();
    assert_eq!(spi.into_inner().modes, [false, true, false]);
}

#[test]
fn failed_or_dropped_transaction_reconfigures_same_mode_without_raw_access() {
    for cancelled in [false, true] {
        let mut spi = ReadWriteSpi::new(
            ClockSpi {
                fail_at: (!cancelled).then_some(2),
                suspend_at: cancelled.then_some(2),
                ..ClockSpi::default()
            },
            configure,
        );
        run(spi.write(&[0])).unwrap();
        if cancelled {
            let mut future = std::pin::pin!(spi.write(&[0]));
            assert!(future
                .as_mut()
                .poll(&mut Context::from_waker(Waker::noop()))
                .is_pending());
        } else {
            assert_eq!(run(spi.write(&[0])), Err(ErrorKind::Other));
        }
        run(spi.write(&[0])).unwrap();
        assert_eq!(spi.into_inner().modes, [false, false]);
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
    assert_eq!(device.modes, [true, false, true, false]);
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
