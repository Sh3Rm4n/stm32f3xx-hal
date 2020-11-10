#![no_std]
#![no_main]

use cortex_m_rt::entry as _;
use defmt_rtt as _;
use panic_probe as _;
use stm32f3xx_hal as _;


// #[entry]
// fn main() -> ! {
//     assert!(false, "TODO: Write actual tests");

//     probe_rs_test::exit();
// }

#[defmt_test::tests]
mod tests {
   #[test]
   fn assert_true() {
       assert!(true)
   }

   #[test]
   fn assert_false() {
       assert!(false)
   }
}
