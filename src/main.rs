#![no_main]
#![no_std]

use panic_halt as _;
use stm32f0;
use rtfm;

#[rtfm::app(device=stm32f0::stm32f0x1, peripherals = true)]
const APP: () = {
    #[init]
    fn init(_cx: init::Context) {
        unsafe {
            (*stm32f0::stm32f0x1::RCC::ptr()).ahbenr.modify(|_, w| w.iopaen().set_bit());

            _set_alternate_mode(9, 1);
            _set_alternate_mode(10, 1);
            (*stm32f0::stm32f0x1::GPIOA::ptr()).afrl.modify(|_, w| w.afrl0().af9());
            (*stm32f0::stm32f0x1::GPIOA::ptr()).afrl.modify(|_, w| w.afrl0().af10());

            (*stm32f0::stm32f0x1::RCC::ptr()).apb2enr.modify(|_, w| w.usart1en().set_bit());
            (*stm32f0::stm32f0x1::USART1::ptr()).cr1.modify(|_, w| w.ue().clear_bit());
            (*stm32f0::stm32f0x1::USART1::ptr()).brr.modify(|_, w| w.bits(8000000/115200));

            (*stm32f0::stm32f0x1::USART1::ptr()).cr2.reset();
            (*stm32f0::stm32f0x1::USART1::ptr()).cr3.reset();

            (*stm32f0::stm32f0x1::USART1::ptr()).cr1.modify(|_, w| w.te().set_bit().re().set_bit().ue().set_bit());

            (*stm32f0::stm32f0x1::USART1::ptr()).cr1.modify(|_, w| w.rxneie().set_bit());
        }

        print("init done.");
    }

    #[task(priority = 2, capacity = 10)]
    fn time_consuming_task(_: time_consuming_task::Context) {
        print("time consuming task");
    }

//    #[task(priority = 3)]
//    fn printx2(_: printx2::Context) {
//    }

    #[task(priority = 4, binds = USART1, spawn = [time_consuming_task])]
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
        fn DMA1_CH1();
        fn TSC();
    }
};


fn print(text : &str) {
    for c in text.bytes() {
        nb::block!(write(stm32f0::stm32f0x1::USART1::ptr(), c)).unwrap();
    }
    nb::block!(write(stm32f0::stm32f0x1::USART1::ptr(), '\n' as u8)).unwrap();
}




/// Tries to write a byte to the UART
/// Fails if the transmit buffer is full
fn write(usart: *const stm32f0::stm32f0x1::usart1::RegisterBlock, byte: u8) -> nb::Result<(), ()> {
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
fn read(usart: *const stm32f0::stm32f0x1::usart1::RegisterBlock) -> nb::Result<u8, u8> {
    // NOTE(unsafe) atomic read with no side effects
    let isr = unsafe { (*usart).isr.read() };

    let err = if isr.pe().bit_is_set() {
        nb::Error::Other(0)
    } else if isr.fe().bit_is_set() {
        nb::Error::Other(1)
    } else if isr.nf().bit_is_set() {
        nb::Error::Other(2)
    } else if isr.ore().bit_is_set() {
        nb::Error::Other(3)
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

fn _set_alternate_mode (index:usize, mode: u32)
{
    let offset = 2 * index;
    let offset2 = 4 * index;
    unsafe {
        let reg = &(*stm32f0::stm32f0x1::GPIOA::ptr());
        if offset2 < 32 {
            reg.afrl.modify(|r, w| {
                w.bits((r.bits() & !(0b1111 << offset2)) | (mode << offset2))
            });
        } else {
            let offset2 = offset2 - 32;
            reg.afrh.modify(|r, w| {
                w.bits((r.bits() & !(0b1111 << offset2)) | (mode << offset2))
            });
        }
        reg.moder.modify(|r, w| {
            w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset))
        });
    }
}