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
use numtoa::NumToA;

use nb::block;
use nb::Error::WouldBlock;

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

const RX_SZ: usize = 100;

pub struct Buffer {
    buffer: [u8; RX_SZ],
    write_pos: usize,
    read_pos: usize,
    count : usize,
}

impl Buffer {
    const fn new() -> Buffer {
        Buffer {
            buffer: [0; RX_SZ],
            write_pos: 0,
            read_pos: 0,
            count : 0,
        }
    }

    pub fn full(&self) -> bool {
        return self.count == self.buffer.len();
    }
    pub fn empty(&self) -> bool {
        return self.count == 0;
    }

    pub fn push(&mut self, data: u8) -> Result<(), ()> {
        if self.full() {
            return Err(());
        }

        cortex_m::interrupt::free(|cs| {
            self.count += 1;
            self.write_pos = (self.write_pos + 1) % self.buffer.len();
            self.buffer[self.write_pos] = data;
        });

        Ok(())
    }

    pub fn read(&mut self) -> Option<u8> {
        if self.empty() {
            return None;
        }

        let mut tmp: Option<u8> = None;

        cortex_m::interrupt::free(|cs| {
            self.count -= 1;
            self.read_pos = (self.read_pos + 1) % self.buffer.len();
            tmp = Some(self.buffer[self.read_pos]);
        });
        return tmp;
    }
}

pub struct CommandsBuffer {
    buffer : Buffer,
    cmds_in_buffer : u8,
}


trait Device {
    fn new() -> Self;

    fn led_on(&mut self);
    fn led_off(&mut self);

    fn led_toggle(&mut self);

    fn write_hello(&mut self);
}

struct Hardware {
    led: hal::gpio::gpioc::PC8<hal::gpio::Output<hal::gpio::PushPull>>,
    serial: stm32f0xx_hal::serial::Serial<hal::stm32f0::stm32f0x1::USART1, stm32f0xx_hal::gpio::gpioa::PA9<stm32f0xx_hal::gpio::Alternate<stm32f0xx_hal::gpio::AF1>>, stm32f0xx_hal::gpio::gpioa::PA10<stm32f0xx_hal::gpio::Alternate<stm32f0xx_hal::gpio::AF1>>>,
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
            serial,
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
static RX_BUFFER : Mutex<RefCell<Option<CommandsBuffer>>> = Mutex::new(RefCell::new(None));

static mut ERROR: u8 = 0_u8;

//#[entry]
#[no_mangle]
fn main() -> ! {
    let mut _rx_buf : *mut CommandsBuffer = core::ptr::null_mut();

    cortex_m::interrupt::free(|cs| {
        let mut hw = Hardware::new();
        hw.led_toggle();
        hw.serial.write_str("Startup!\r\n");
        *HARDWARE.borrow(cs).borrow_mut() = Some(hw);

        let mut rx_buf = CommandsBuffer{
            buffer : Buffer::new(),
            cmds_in_buffer : 0
        };
        _rx_buf = &mut rx_buf;

        *RX_BUFFER.borrow(cs).borrow_mut() = Some(rx_buf);
    });



    let mut buf = [0_u8; 30];
    let mut iter = 0_usize;
    let mut done = false;

    loop {
        let mut tmp = [0_u8; 30];


        cortex_m::interrupt::free(|cs| {
            if let &mut Some(ref mut buffer) = RX_BUFFER.borrow(cs).borrow_mut().deref_mut() {
                if buffer.cmds_in_buffer > 0 {
                    buffer.cmds_in_buffer -= 1;

                    while let Some(now) = buffer.buffer.read() {
                        buf[iter] = now;
                        iter += 1;

                        if now == '\n' as u8 {
                            done = true;
                            break
                        }
                    }
                }
            }
        });

        if done {
            done = false;
            for c in "Got command [length = ".as_bytes() {
                nb::block!(write(stm32f0::stm32f0x1::USART1::ptr(), *c));
            }

            for c in iter.numtoa(10, &mut tmp).iter() {
                nb::block!(write(stm32f0::stm32f0x1::USART1::ptr(), *c as u8));
            }
            nb::block!(write(stm32f0::stm32f0x1::USART1::ptr(), ']' as u8));
            nb::block!(write(stm32f0::stm32f0x1::USART1::ptr(), ':' as u8));
            nb::block!(write(stm32f0::stm32f0x1::USART1::ptr(), ' ' as u8));

            for c in buf.iter().take(iter) {
                nb::block!(write(stm32f0::stm32f0x1::USART1::ptr(), *c));
            }

            iter = 0;
        }


        let mut error_code: u8 = 0;
        unsafe {
            error_code = read_volatile(&ERROR);
        }

        if error_code > 0 {
            unsafe { write_volatile(&mut ERROR, 0); }

            for c in "err\r\n".chars() {
                nb::block!(write(stm32f0::stm32f0x1::USART1::ptr(), c as u8));
            }
        }
    }
}

#[exception]
fn SysTick() {
    cortex_m::interrupt::free(|cs| {
        if let &mut Some(ref mut hw) = HARDWARE.borrow(cs).borrow_mut().deref_mut() {
            hw.led_toggle();
        }
    });
}

#[interrupt]
fn USART1() {
        if let Ok(c) = read(stm32f0::stm32f0x1::USART1::ptr()) {
            cortex_m::interrupt::free(|cs| {
                if let &mut Some(ref mut buffer) = RX_BUFFER.borrow(cs).borrow_mut().deref_mut() {
                    buffer.buffer.push(c).unwrap();
                    if c == '\n' as u8 {
                        buffer.cmds_in_buffer += 1;
                    }
                }
            });
        }
}