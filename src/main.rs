#![no_main]
#![no_std]

use panic_halt as _;
use stm32f0xx_hal as hal;
use crate::hal::{prelude::*};
use stm32f0;
use rtfm;


/// Tries to write a byte to the UART
/// Fails if the transmit buffer is full
fn write(usart: *const hal::stm32::usart1::RegisterBlock, byte: u8) -> nb::Result<(), ()> {
    // NOTE(unsafe) atomic read with no side effects
    let isr = unsafe { (*usart).isr.read() };

    if isr.txe().bit_is_set() {
        // NOTE(unsafe) atomic write to stateless register
        // NOTE(write_volatile) 8-bit write that's not possible through the svd2rust API
        unsafe { core::ptr::write_volatile(&(*usart).tdr as *const _ as *mut _, byte) }
        Ok(())
    } else {
        Err(nb::Error::WouldBlock)
    }
}
/// Tries to read a byte from the UART
fn read(usart: *const hal::stm32::usart1::RegisterBlock) -> nb::Result<u8, hal::serial::Error> {
    // NOTE(unsafe) atomic read with no side effects
    let isr = unsafe { (*usart).isr.read() };

    let err = if isr.pe().bit_is_set() {
        nb::Error::Other(hal::serial::Error::Parity)
    } else if isr.fe().bit_is_set() {
        nb::Error::Other(hal::serial::Error::Framing)
    } else if isr.nf().bit_is_set() {
        nb::Error::Other(hal::serial::Error::Noise)
    } else if isr.ore().bit_is_set() {
        nb::Error::Other(hal::serial::Error::Overrun)
    } else if isr.rxne().bit_is_set() {
        // NOTE(read_volatile) see `write_volatile` below
        return Ok(unsafe { core::ptr::read_volatile(&(*usart).rdr as *const _ as *const _) });
    } else {
        return Err(nb::Error::WouldBlock);
    };

    // NOTE(unsafe) atomic write with no side effects other than clearing the errors we've just handled
    unsafe {
        (*usart).icr.write(|w| {
            w.pecf()
                .set_bit()
                .fecf()
                .set_bit()
                .ncf()
                .set_bit()
                .orecf()
                .set_bit()
        })
    };

    Err(err)
}

fn print(text : &str) {
    for c in text.bytes() {
        nb::block!(write(stm32f0::stm32f0x1::USART1::ptr(), c)).unwrap();
    }
    nb::block!(write(stm32f0::stm32f0x1::USART1::ptr(), '\n' as u8)).unwrap();
}

#[rtfm::app(device=crate::hal::stm32f0::stm32f0x1, peripherals = true)]
const APP: () = {
    #[init]
    fn init(cx: init::Context) {
        let mut p = cx.device;
        let mut rcc = p.RCC.configure().sysclk(8.mhz()).freeze(&mut p.FLASH);

        let gpioa = p.GPIOA.split(&mut rcc);

        let (tx, rx) = cortex_m::interrupt::free(|cs| {
            (gpioa.pa9.into_alternate_af1(cs),
             gpioa.pa10.into_alternate_af1(cs))
        });


        let mut serial = hal::serial::Serial::usart1(p.USART1, (tx, rx), 115200.bps(), &mut rcc);

        serial.listen(hal::serial::Event::Rxne);
    }

    #[task(priority = 1, capacity = 10)]
    fn time_consuming_task(_: time_consuming_task::Context) {
        print("time consuming task");
    }

//    #[task(priority = 2)]
//    fn printx2(_: printx2::Context) {
//    }

    #[task(priority = 8, binds = USART1, spawn = [time_consuming_task])]
    fn usart_handler(_cx: usart_handler::Context) {
        match read(stm32f0::stm32f0x1::USART1::ptr()) {
            Ok(_c) => {
                _cx.spawn.time_consuming_task().unwrap();
            }
            Err(_e) => {
                print("error!");
            }
        }
    }

    extern "C" {
        fn TSC();
        fn DMA1_CH1();
    }
};



