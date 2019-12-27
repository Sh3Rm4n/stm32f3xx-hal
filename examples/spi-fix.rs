#![no_std]
#![no_main]

extern crate panic_semihosting;

use stm32f3xx_hal as hal;

use cortex_m_rt::entry;

use hal::prelude::*;
use hal::spi::{Mode, Phase, Polarity, Spi};
use embedded_hal::spi::MODE_0;
use hal::stm32;

#[entry]
fn main() -> ! {
    let stm = hal::stm32::Peripherals::take().unwrap();
    // let core = cortex_m::Peripherals::take().unwrap();
    //This is done for the peripherals to play nice I think!
    let mut flash = stm.FLASH.constrain();
    let mut rcc = stm.RCC.constrain();

    // TODO Check if the clock configuration is right
    //clock configuration
    let clocks = rcc.cfgr
        .sysclk(64.mhz())
        .pclk1(32.mhz())
        .pclk2(32.mhz())
        .freeze(&mut flash.acr);

    //spi
    let mut gpioa = stm.GPIOA.split(&mut rcc.ahb);
    let mosi = gpioa.pa7.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let miso = gpioa.pa6.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let sck = gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    // let cs = gpioa.pa4.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    let mut spi = hal::spi::Spi::spi1(stm.SPI1, (sck, miso, mosi), MODE_0, 1.mhz(), clocks, &mut rcc.apb2);
    spi.send(0b101010).unwrap();

    loop {}
}
