#![no_main]
#![no_std]

use panic_halt as _;

use stm32f0xx_hal as hal;

use crate::hal::{prelude::*};

use cortex_m::{interrupt::Mutex};
use cortex_m_rt::{entry, exception};
use core::fmt::Write;

use stm32f0::stm32f0x1::interrupt;

use core::{cell::RefCell, ops::DerefMut};
use core::ptr::{write_volatile, read_volatile};

trait Device {
    fn new() -> Self;

    fn led_on(&mut self);
    fn led_off(&mut self);

    fn led_toggle(&mut self);

    fn write_hello(&mut self);
}

struct Hardware {
    led: hal::gpio::gpioc::PC8<hal::gpio::Output<hal::gpio::PushPull>>,
    serial: stm32f0xx_hal::serial::Serial<hal::stm32f0::stm32f0x1::USART1, stm32f0xx_hal::gpio::gpioa::PA9<stm32f0xx_hal::gpio::Alternate<stm32f0xx_hal::gpio::AF1>>, stm32f0xx_hal::gpio::gpioa::PA10<stm32f0xx_hal::gpio::Alternate<stm32f0xx_hal::gpio::AF1>>>
}

impl Device for Hardware {
    fn new() -> Hardware {
        let mut p = hal::stm32::Peripherals::take().unwrap();
        let mut rcc = p.RCC.configure().sysclk(8.mhz()).freeze(&mut p.FLASH);
        let cp = cortex_m::Peripherals::take().unwrap();

        let gpioa = p.GPIOA.split(&mut rcc);
        let gpioc = p.GPIOC.split(&mut rcc);


        let led = cortex_m::interrupt::free(|cs| {
            // (Re-)configure PA1 as output
            gpioc.pc8.into_push_pull_output(cs)
        });


        let (tx, rx) = cortex_m::interrupt::free(|cs| {
            // (Re-)configure PA1 as output
            (gpioa.pa9.into_alternate_af1(cs), gpioa.pa10.into_alternate_af1(cs))
        });

        let mut serial = hal::serial::Serial::usart1(p.USART1, (tx, rx), 115200.bps(), &mut rcc);

        serial.listen(hal::serial::Event::Rxne);
        unsafe {
            hal::stm32f0::stm32f0x1::NVIC::unmask(interrupt::USART1);
        }

        let mut syst = cp.SYST;

        unsafe { syst.cvr.write(1); }

        syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
        syst.set_reload(4_000_000);
        syst.enable_counter();
        syst.enable_interrupt();

        Hardware {
            led,
            serial
        }
    }



    fn led_on(&mut self) {
        self.led.set_high().ok();
    }

    fn led_off(&mut self) {
        self.led.set_low().ok();
    }

    fn led_toggle(&mut self) {
        self.led.toggle().ok();
    }

    fn write_hello(&mut self) {
        self.serial.write_str("Hello World!\n\r").unwrap();
    }
}

static HARDWARE : Mutex<RefCell<Option<Hardware>>> = Mutex::new(RefCell::new(None));

static mut TEST: u8 = 0_u8;

//#[entry]
#[no_mangle]
fn main() -> ! {
    cortex_m::interrupt::free(|cs| {
        let mut hw = Hardware::new();
        hw.led_toggle();
        *HARDWARE.borrow(cs).borrow_mut() = Some(hw);
    });


    loop {
        unsafe {
            if read_volatile(&TEST) > 0 {
                cortex_m::interrupt::free(|cs| {
                    if let &mut Some(ref mut hw) = HARDWARE.borrow(cs).borrow_mut().deref_mut()
                    {
                        TEST = 0;
                        hw.led_toggle();
                        hw.write_hello();
                    }
                });
            }
        }
    }
}

#[exception]
fn SysTick() {
    unsafe {
        write_volatile(&mut TEST, 1);
    };
}

#[interrupt]
fn USART1() {

    unsafe {
        let _cr1 = (*stm32f0::stm32f0x1::USART1::ptr()).cr1.read();
        let _cr2 = (*stm32f0::stm32f0x1::USART1::ptr()).cr2.read();
        let dr = (*stm32f0::stm32f0x1::USART1::ptr()).rdr.read().bits() as u8;

        cortex_m::interrupt::free(|cs| {
            if let &mut Some(ref mut hw) = HARDWARE.borrow(cs).borrow_mut().deref_mut()
            {
                hw.serial.write(dr).unwrap();
                hw.led_toggle();
            }
        });
    };
}