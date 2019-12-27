#![no_main]
#![no_std]

//use panic_semihosting as _;
use panic_halt as _;
use stm32f0xx_hal as hal;
use crate::hal::{prelude::*};
use stm32f0;
use rtfm;
use core::ptr::{write_volatile, read_volatile};


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
//        hprintln!("Parity");
        nb::Error::Other(hal::serial::Error::Parity)
    } else if isr.fe().bit_is_set() {
//        hprintln!("Frame");
        nb::Error::Other(hal::serial::Error::Framing)
    } else if isr.nf().bit_is_set() {
        nb::Error::Other(hal::serial::Error::Noise)
    } else if isr.ore().bit_is_set() {
//        hprintln!("Overrun");
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

fn dbg(text : &str) {
    for c in text.bytes() {
        nb::block!(write(stm32f0::stm32f0x1::USART1::ptr(), c)).unwrap();
    }
    nb::block!(write(stm32f0::stm32f0x1::USART1::ptr(), '\n' as u8)).unwrap();
}
fn dbgc(c : char) {
    nb::block!(write(stm32f0::stm32f0x1::USART1::ptr(), c as u8)).unwrap();
}


fn set_13() {
    unsafe { (*stm32f0::stm32f0x1::GPIOB::ptr()).odr.modify(|_, x| {
        x.odr13().bit(true)
    }); }
}
fn clear_13() {
    unsafe { (*stm32f0::stm32f0x1::GPIOB::ptr()).odr.modify(|_, x| {
        x.odr13().bit(false)
    }); }
}

fn set_14() {
    unsafe { (*stm32f0::stm32f0x1::GPIOB::ptr()).odr.modify(|_, x| {
        x.odr14().bit(true)
    }); }
}
fn clear_14() {
    unsafe { (*stm32f0::stm32f0x1::GPIOB::ptr()).odr.modify(|_, x| {
        x.odr14().bit(false)
    }); }
}

fn set_15() {
    unsafe { (*stm32f0::stm32f0x1::GPIOB::ptr()).odr.modify(|_, x| {
        x.odr15().bit(true)
    }); }
}
fn clear_15() {
    unsafe { (*stm32f0::stm32f0x1::GPIOB::ptr()).odr.modify(|_, x| {
        x.odr15().bit(false)
    }); }

}




#[rtfm::app(device=crate::hal::stm32f0::stm32f0x1, peripherals = true)]
const APP: () = {
    #[init]
    fn init(cx: init::Context) {
        let mut p = cx.device;
        let mut rcc = p.RCC.configure().sysclk(8.mhz()).freeze(&mut p.FLASH);

        let gpioa = p.GPIOA.split(&mut rcc);
        let gpiob = p.GPIOB.split(&mut rcc);



        let (tx, rx, dbg1, dbg2, dbg3) = cortex_m::interrupt::free(|cs| {
            (gpioa.pa9.into_alternate_af1(cs),
             gpioa.pa10.into_alternate_af1(cs),
             gpiob.pb13.into_push_pull_output(cs),
             gpiob.pb14.into_push_pull_output(cs),
             gpiob.pb15.into_push_pull_output(cs),)
        });


        let mut serial = hal::serial::Serial::usart1(p.USART1, (tx, rx), 115200.bps(), &mut rcc);

        serial.listen(hal::serial::Event::Rxne);
    }

    #[task(priority = 1)]
    fn printx(_: printx::Context) {
        set_15();
//        nb::block!(write(stm32f0::stm32f0x1::USART1::ptr(), '?' as u8)).unwrap();
        let mut x = 0_u32;
        loop {
            unsafe {
                let now = read_volatile(&mut x);
                write_volatile(&mut x, now+1);
                if now > 100 {
                    break;
                }
            }
        }
        clear_15();
    }

    #[task(priority = 2)]
    fn printx2(_: printx2::Context) {
    }

    #[task(priority = 8, binds = USART1, spawn = [printx])]
    fn USART1_handler(_cx: USART1_handler::Context) {
        set_13();

        if let Ok(c) = read(stm32f0::stm32f0x1::USART1::ptr()) {
            if c == '\n' as u8 {
                set_14();
                _cx.spawn.printx().unwrap();
                clear_14();
                dbgc('X');
            }
        } else {
            dbg("error!");
        }

        clear_13();
    }

    extern "C" {
        fn TSC();
        fn DMA1_CH1();
    }
};



