use core::{
    future::Future,
    pin::pin,
    task::{Context, Poll, Waker},
};
use std::{cell::Cell, collections::VecDeque, rc::Rc};

use embedded_hal::{
    digital::{ErrorType as PinErrorType, OutputPin},
    spi::{ErrorKind, ErrorType},
};
use embedded_hal_async::{
    delay::DelayNs,
    spi::{Operation, SpiDevice},
};
use st7365p_lcd::{VsyncError, ST7365P};

#[derive(Debug, PartialEq)]
enum Transfer {
    Delay(u32),
    Write(Vec<u8>),
    Read(usize),
}

struct Spi {
    dc: Rc<Cell<bool>>,
    samples: VecDeque<Result<u16, ErrorKind>>,
    transactions: Vec<Vec<Transfer>>,
    fail: bool,
}

impl ErrorType for Spi {
    type Error = ErrorKind;
}

impl SpiDevice for Spi {
    async fn transaction(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        if self.fail {
            return Err(ErrorKind::Other);
        }
        let mut transfers = Vec::new();
        for operation in operations {
            transfers.push(match operation {
                Operation::DelayNs(ns) => Transfer::Delay(*ns),
                Operation::Write(bytes) => Transfer::Write(bytes.to_vec()),
                Operation::Read(bytes) => {
                    assert!(!self.dc.get());
                    assert_eq!(transfers.last(), Some(&Transfer::Write(vec![0x45])));
                    bytes.copy_from_slice(&self.samples.pop_front().unwrap()?.to_be_bytes());
                    Transfer::Read(bytes.len())
                }
                _ => panic!("unexpected SPI operation"),
            });
        }
        self.transactions.push(transfers);
        Ok(())
    }
}

struct Pin {
    high: Rc<Cell<bool>>,
    fail: bool,
}

impl PinErrorType for Pin {
    type Error = embedded_hal::digital::ErrorKind;
}

impl OutputPin for Pin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        if self.fail {
            return Err(embedded_hal::digital::ErrorKind::Other);
        }
        self.high.set(false);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.high.set(true);
        Ok(())
    }
}

struct Delay;

impl DelayNs for Delay {
    async fn delay_ns(&mut self, _ns: u32) {}
}

fn display(samples: &[Result<u16, ErrorKind>], pin_failure: bool) -> ST7365P<Spi, Pin, Pin, Delay> {
    let high = Rc::new(Cell::new(true));
    ST7365P::new(
        Spi {
            dc: high.clone(),
            samples: samples.iter().copied().collect(),
            transactions: Vec::new(),
            fail: false,
        },
        Pin {
            high,
            fail: pin_failure,
        },
        None,
        true,
        false,
        Delay,
    )
}

fn run<F: Future>(future: F) -> F::Output {
    match pin!(future).poll(&mut Context::from_waker(Waker::noop())) {
        Poll::Ready(result) => result,
        Poll::Pending => panic!("mock unexpectedly blocked"),
    }
}

#[test]
fn scanline_read_keeps_command_and_response_in_one_transaction() {
    let mut display = display(&[Ok(0x0123)], false);
    assert_eq!(run(display.read_scanline()), Ok(0x0123));
    assert_eq!(
        display.spi_mut().transactions,
        vec![vec![
            Transfer::Delay(60),
            Transfer::Write(vec![0x45]),
            Transfer::Read(2),
            Transfer::Delay(65),
        ]]
    );
}

#[test]
fn tear_scanline_is_big_endian_and_can_be_disabled() {
    let mut display = display(&[], false);
    run(display.set_tear_scanline(0x0123)).unwrap();
    run(display.set_tear_scanline(0)).unwrap();
    run(display.disable_tearing_effect()).unwrap();
    assert_eq!(
        display.spi_mut().transactions,
        vec![
            vec![Transfer::Write(vec![0x44])],
            vec![Transfer::Write(vec![0x01, 0x23])],
            vec![Transfer::Write(vec![0x44])],
            vec![Transfer::Write(vec![0x00, 0x00])],
            vec![Transfer::Write(vec![0x34])],
        ]
    );
}

#[test]
fn vsync_waits_for_wrap_even_when_zero_is_missed() {
    let mut display = display(&[Ok(0), Ok(150), Ok(320), Ok(490), Ok(2)], false);
    assert_eq!(run(display.wait_for_vsync(5)), Ok(()));
    assert_eq!(display.spi_mut().transactions.len(), 5);
}

#[test]
fn stuck_or_nonwrapping_scanline_times_out() {
    for samples in [[0; 4], [u16::MAX; 4], [123; 4], [0, 1, 2, 3]] {
        let mut display = display(&samples.map(Ok), false);
        assert_eq!(run(display.wait_for_vsync(4)), Err(VsyncError::Timeout));
        assert_eq!(display.spi_mut().transactions.len(), 4);
    }
}

#[test]
fn insufficient_poll_budget_does_not_touch_spi() {
    let mut display = display(&[], false);
    for budget in [0, 1] {
        assert_eq!(
            run(display.wait_for_vsync(budget)),
            Err(VsyncError::Timeout)
        );
    }
    assert!(display.spi_mut().transactions.is_empty());
}

#[test]
fn spi_and_pin_errors_propagate_without_polling_again() {
    for samples in [
        vec![Err(ErrorKind::Other)],
        vec![Ok(100), Err(ErrorKind::Other)],
    ] {
        let mut display = display(&samples, false);
        assert_eq!(run(display.wait_for_vsync(10)), Err(VsyncError::Interface));
        assert!(display.spi_mut().samples.is_empty());
    }
    let mut display = display(&[], true);
    assert_eq!(run(display.wait_for_vsync(10)), Err(VsyncError::Interface));
    assert!(display.spi_mut().transactions.is_empty());
}

#[test]
fn tear_configuration_propagates_spi_errors() {
    let mut display = display(&[], false);
    display.spi_mut().fail = true;
    assert_eq!(run(display.set_tear_scanline(0)), Err(()));
    assert_eq!(run(display.disable_tearing_effect()), Err(()));
}
