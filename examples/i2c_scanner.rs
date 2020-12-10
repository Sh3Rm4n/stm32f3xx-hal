//! Example of using I2C.
//! Scans available I2C devices on bus and print the result.
//! Appropriate pull-up registers should be installed on I2C bus.
//! Target board: STM32F3DISCOVERY

#![no_std]
#![no_main]

use core::ops::Range;

use defmt_rtt as _;
use panic_probe as _;

use cortex_m::asm;
use cortex_m_rt::entry;

use stm32f3xx_hal::{self as hal, pac, prelude::*};

const VALID_ADDR_RANGE: Range<u8> = 0x08..0x78;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);

    // Configure I2C1
    let pins = (
        gpiob.pb6.into_af4(&mut gpiob.moder, &mut gpiob.afrl), // SCL
        gpiob.pb7.into_af4(&mut gpiob.moder, &mut gpiob.afrl), // SDA
    );
    let mut i2c = hal::i2c::I2c::new(dp.I2C1, pins, 100.khz(), clocks, &mut rcc.apb1);

    defmt::info!("Start i2c scanning ...");

    for addr in 0x00_u8..0x80 {
        // Write the empty array and check the slave response.
        if VALID_ADDR_RANGE.contains(&addr) && i2c.write(addr, &[]).is_ok() {
            defmt::info!("{:u8}", addr);
        }
    }

    defmt::info!("Done!");

    loop {
        asm::wfi();
    }
}
