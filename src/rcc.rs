//! Reset and Clock Control

use crate::stm32::{rcc, RCC};
use cast::u32;

use crate::flash::ACR;
use crate::time::Hertz;

/// Extension trait that constrains the `RCC` peripheral
pub trait RccExt {
    /// Constrains the `RCC` peripheral so it plays nicely with the other abstractions
    fn constrain(self) -> Rcc;
}

impl RccExt for RCC {
    fn constrain(self) -> Rcc {
        Rcc {
            ahb: AHB { _0: () },
            apb1: APB1 { _0: () },
            apb2: APB2 { _0: () },
            cfgr: CFGR {
                hse: None,
                hclk: None,
                pclk1: None,
                pclk2: None,
                sysclk: None,
            },
        }
    }
}

/// Constrained RCC peripheral
pub struct Rcc {
    /// AMBA High-performance Bus (AHB) registers
    pub ahb: AHB,
    /// Advanced Peripheral Bus 1 (APB1) registers
    pub apb1: APB1,
    /// Advanced Peripheral Bus 2 (APB2) registers
    pub apb2: APB2,
    /// Clock configuration
    pub cfgr: CFGR,
}

/// AMBA High-performance Bus (AHB) registers
pub struct AHB {
    _0: (),
}

impl AHB {
    pub(crate) fn enr(&mut self) -> &rcc::AHBENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahbenr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::AHBRSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahbrstr }
    }
}

/// Advanced Peripheral Bus 1 (APB1) registers
pub struct APB1 {
    _0: (),
}

impl APB1 {
    pub(crate) fn enr(&mut self) -> &rcc::APB1ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb1enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::APB1RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb1rstr }
    }
}

/// Advanced Peripheral Bus 2 (APB2) registers
pub struct APB2 {
    _0: (),
}

impl APB2 {
    pub(crate) fn enr(&mut self) -> &rcc::APB2ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb2enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::APB2RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb2rstr }
    }
}

const HSI: u32 = 8_000_000; // Hz

// some microcontrollers do not have USB
#[cfg(any(feature = "stm32f301", feature = "stm32f334",))]
mod usb_clocking {
    pub fn has_usb() -> bool {
        false
    }

    pub fn set_usbpre<W>(w: &mut W, _: bool) -> &mut W {
        w
    }
}

#[cfg(any(
    feature = "stm32f318",
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398",
))]
mod usb_clocking {
    use crate::stm32::rcc;

    pub fn has_usb() -> bool {
        true
    }

    pub fn set_usbpre(w: &mut rcc::cfgr::W, bit: bool) -> &mut rcc::cfgr::W {
        w.usbpre().bit(bit)
    }
}

use self::usb_clocking::{has_usb, set_usbpre};

/// Clock configuration
pub struct CFGR {
    hse: Option<u32>,
    hclk: Option<u32>,
    pclk1: Option<u32>,
    pclk2: Option<u32>,
    sysclk: Option<u32>,
}

struct PllConfig {
    src: rcc::cfgr::PLLSRC_A,
    mul: rcc::cfgr::PLLMUL_A,
    div: Option<rcc::cfgr2::PREDIV_A>,
}

/// Determine the [greatest common divisor](https://en.wikipedia.org/wiki/Greatest_common_divisor)
///
/// This function is based on the [Euclidean algorithm](https://en.wikipedia.org/wiki/Euclidean_algorithm).
fn gcd(mut a: u32, mut b: u32) -> u32 {
    while b != 0 {
        let r = a % b;
        a = b;
        b = r;
    }
    a
}

/// Convert pll multiplicand into equivalent register field type
fn into_pll_mul(mul: u8) -> rcc::cfgr::PLLMUL_A {
    match mul {
        2 => rcc::cfgr::PLLMUL_A::MUL2,
        3 => rcc::cfgr::PLLMUL_A::MUL3,
        4 => rcc::cfgr::PLLMUL_A::MUL4,
        5 => rcc::cfgr::PLLMUL_A::MUL5,
        6 => rcc::cfgr::PLLMUL_A::MUL6,
        7 => rcc::cfgr::PLLMUL_A::MUL7,
        8 => rcc::cfgr::PLLMUL_A::MUL8,
        9 => rcc::cfgr::PLLMUL_A::MUL9,
        10 => rcc::cfgr::PLLMUL_A::MUL10,
        11 => rcc::cfgr::PLLMUL_A::MUL11,
        12 => rcc::cfgr::PLLMUL_A::MUL12,
        13 => rcc::cfgr::PLLMUL_A::MUL13,
        14 => rcc::cfgr::PLLMUL_A::MUL14,
        15 => rcc::cfgr::PLLMUL_A::MUL15,
        16 => rcc::cfgr::PLLMUL_A::MUL16,
        _ => unreachable!(),
    }
}

/// Convert pll dividend into equivalent register field type
fn into_pre_div(div: u8) -> rcc::cfgr2::PREDIV_A {
    match div {
        1 => rcc::cfgr2::PREDIV_A::DIV1,
        2 => rcc::cfgr2::PREDIV_A::DIV2,
        3 => rcc::cfgr2::PREDIV_A::DIV3,
        4 => rcc::cfgr2::PREDIV_A::DIV4,
        5 => rcc::cfgr2::PREDIV_A::DIV5,
        6 => rcc::cfgr2::PREDIV_A::DIV6,
        7 => rcc::cfgr2::PREDIV_A::DIV7,
        8 => rcc::cfgr2::PREDIV_A::DIV8,
        9 => rcc::cfgr2::PREDIV_A::DIV9,
        10 => rcc::cfgr2::PREDIV_A::DIV10,
        11 => rcc::cfgr2::PREDIV_A::DIV11,
        12 => rcc::cfgr2::PREDIV_A::DIV12,
        13 => rcc::cfgr2::PREDIV_A::DIV13,
        14 => rcc::cfgr2::PREDIV_A::DIV14,
        15 => rcc::cfgr2::PREDIV_A::DIV15,
        16 => rcc::cfgr2::PREDIV_A::DIV16,
        _ => unreachable!(),
    }
}

impl CFGR {
    /// Uses HSE (external oscillator) instead of HSI (internal RC oscillator) as the clock source.
    /// Will result in a hang if an external oscillator is not connected or it fails to start.
    pub fn use_hse<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.hse = Some(freq.into().0);
        self
    }

    /// Sets a frequency for the AHB bus
    pub fn hclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.hclk = Some(freq.into().0);
        self
    }

    /// Sets a frequency for the APB1 bus
    pub fn pclk1<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.pclk1 = Some(freq.into().0);
        self
    }

    /// Sets a frequency for the APB2 bus
    pub fn pclk2<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.pclk2 = Some(freq.into().0);
        self
    }

    /// Sets the system (core) frequency
    pub fn sysclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.sysclk = Some(freq.into().0);
        self
    }

    /// Calculate the values for the pll multiplicand (PLLMUL) and the pll dividend (PLLDIV).
    ///
    /// These values are chosen depending on the chosen system clock (SYSCLK) and the frequency of the
    /// hardware clock (HSE / HSI).
    ///
    /// For these devices, PLL_SRC can selected between the internal oscillator (HSI) and
    /// the external oscillator (HSE).
    ///
    /// HSI is divided by 2 before its transferred to PLL_SRC.
    /// HSE can be divided between 2..16, before it is transferred to PLL_SRC.
    /// After this system clock frequency (SYSCLK) can be changed via multiplicand.
    /// The value can be multiplied with 2..16.
    ///
    /// To determine the optimal values, if HSE is chosen as PLL_SRC, the greatest common divisor
    /// is calculated and the limitations of the possible values are taken into consideration.
    ///
    /// HSI is simpler to calculate, but the possible system clocks are less than HSE, because the
    /// division is not configurable.
    #[cfg(not(any(
        feature = "stm32f302xd",
        feature = "stm32f302xe",
        feature = "stm32f303xd",
        feature = "stm32f303xe",
        feature = "stm32f398"
    )))]
    fn calc_pll(&self, sysclk: u32) -> (u32, PllConfig) {
        // Get the optimal value for the pll divisor (PLL_DIV) and multiplicand (PLL_MUL)
        // Only for HSE PLL_DIV can be changed
        let (pll_mul, pll_div): (u32, Some(u32)) = if let Some(hclk) = self.hse {
            // Get the optimal value for the pll divisor (PLL_DIV) and multiplicand (PLL_MUL)
            // with the greatest common divisor calculation.
            let common_divisor = gcd(sysclk, hclk);
            let mut dividend = sysclk / common_divisor;
            let mut divisor = hclk / common_divisor;

            // Check if the dividend can be represented by PLL_MUL
            let pll_mul = if dividend == 1 {
                // PLL_MUL minimal value is 2
                dividend *= 2;
                divisor *= 2;
            } else {
                dividend
            };

            // Check if the divisor can be represented by PRE_DIV
            let pll_div = if divisor == 1 {
                // PLL_MUL minimal value is 2
                dividend *= 2;
                divisor *= 2;
            } else {
                divisor
            };

            // PLL_MUL maximal value is 16
            assert!(pll_mul <= 16);
            // PRE_DIV minimal value is 2
            assert!(pll_div <= 16);
            (pll_mul, Some(pll_div))
        }
        // HSI division is always divided by 2 and has no adjustable division
        else {
            let pll_mul = Some(sysclk / (HSI / 2));
            assert!(pll_mul <= 16);
            (pll_mul, None)
        };

        let pll_mul = pll_mul.unwrap();

        let sysclk = (hclk / pll_div.unwrap_or(1)) * pll_mul;
        assert!(sysclk <= 72_000_000);

        let pllsrc = if self.hse.is_some() {
            rcc::cfgr::PLLSRC_A::HSE_DIV_PREDIV
        } else {
            rcc::cfgr::PLLSRC_A::HSI_DIV2
        };

        // Convert into register bit field types
        let pll_mul_bits = into_pll_mul(pll_mul as u8);
        let pll_div_bits = pll_div.map(|pll_div| into_pre_div(pll_div as u8));

        (
            sysclk,
            PllConfig {
                src: pll_src,
                mul: pll_mul_bits,
                div: pll_div_bits,
            },
        )
    }

    /// Calculate the values for the pll multiplicand (PLLMUL) and the pll dividend (PLLDIV).
    ///
    /// These values are chosen depending on the chosen system clock (SYSCLK) and the frequency of the harware
    /// clk (HSI / HSE).
    ///
    /// For these devices, PLL_SRC can be set to choose between the internal oscillator (HSI) and
    /// the external oscillator (HSE).
    /// After this the system clock frequency (SYSCLK) can be changed via a division and a
    /// multiplication block.
    /// It can be divided from with values 1..16  and multiplicated from 2..16.
    ///
    /// To determine the optimal values, the greatest common divisor is calculated and the
    /// limitations of the possible values are taken into considiration.
    #[cfg(any(
        feature = "stm32f302xd",
        feature = "stm32f302xe",
        feature = "stm32f303xd",
        feature = "stm32f303xe",
        feature = "stm32f398",
    ))]
    fn calc_pll(&self, sysclk: u32) -> (u32, PllConfig) {
        let hclk = self.hse.unwrap_or(HSI);

        // Get the optimal value for the pll divisor (PLL_DIV) and multiplcator (PLL_MUL)
        // with the greatest common divisor calculation.
        let common_divisor = gcd(sysclk, hclk);
        let mut dividend = sysclk / common_divisor;
        let mut divisor = hclk / common_divisor;

        // Check if the dividend can be represented by PLL_MUL
        let pll_mul = if dividend == 1 {
            // PLL_MUL minimal value is 2
            dividend *= 2;
            divisor *= 2;
            dividend
        } else {
            dividend
        };

        // PLL_MUL maximal value is 16
        assert!(dividend <= 16);

        // PRE_DIV maximal value is 16
        assert!(divisor <= 16);
        let pll_div = divisor;

        let sysclk = (hclk / pll_div) * pll_mul;
        assert!(sysclk <= 72_000_000);

        // Select hardware clock source of the PLL
        // TODO Check when HSI_DIV2 could be useful
        let pll_src = if self.hse.is_some() {
            rcc::cfgr::PLLSRC_A::HSE_DIV_PREDIV
        } else {
            rcc::cfgr::PLLSRC_A::HSI_DIV_PREDIV
        };

        // Convert into register bit field types
        let pll_mul_bits = into_pll_mul(pll_mul as u8);
        let pll_div_bits = into_pre_div(pll_div as u8);

        (
            sysclk,
            PllConfig {
                src: pll_src,
                mul: pll_mul_bits,
                div: Some(pll_div_bits),
            },
        )
    }

    /// Get the system clock, the system clock source and the pll_options, if needed.
    ///
    /// The system clock source is determined by the chosen system clock and the provided hardware
    /// clock.
    /// This function does only chose the PLL if needed, otherwise it will use the oscillator clock as system clock.
    // TODO change function name
    fn get_sysclk(&self) -> (u32, rcc::cfgr::SW_A, Option<PllConfig>) {
        // If a sysclk is given, check if the PLL has to be used,
        // else select the system clock source, which is either HSI or HSE.
        if let Some(sysclk) = self.sysclk {
            if let Some(hseclk) = self.hse {
                if sysclk == hseclk {
                    // No need to use the PLL
                    (hseclk, rcc::cfgr::SW_A::HSE, None)
                } else {
                    let clock_with_pll = self.calc_pll(sysclk);
                    (
                        // TODO maybe move sysclk calc to here
                        clock_with_pll.0,
                        rcc::cfgr::SW_A::PLL,
                        Some(clock_with_pll.1),
                    )
                }
            } else if sysclk == HSI {
                // No need to use the PLL
                (HSI, rcc::cfgr::SW_A::HSE, None)
            } else {
                let clock_with_pll = self.calc_pll(sysclk);
                (
                    clock_with_pll.0,
                    rcc::cfgr::SW_A::PLL,
                    Some(clock_with_pll.1),
                )
            }
        } else if let Some(hseclk) = self.hse {
            // Use HSE as system clock
            (hseclk, rcc::cfgr::SW_A::HSE, None)
        } else {
            // Use HSI as system clock
            (HSI, rcc::cfgr::SW_A::HSI, None)
        }
    }

    /// Freezes the clock configuration, making it effective
    pub fn freeze(self, acr: &mut ACR) -> Clocks {
        let (sysclk, sysclk_source, pll_config) = self.get_sysclk();

        let hpre_bits = self
            .hclk
            .map(|hclk| match sysclk / hclk {
                0 => unreachable!(),
                1 => 0b0111,
                2 => 0b1000,
                3..=5 => 0b1001,
                6..=11 => 0b1010,
                12..=39 => 0b1011,
                40..=95 => 0b1100,
                96..=191 => 0b1101,
                192..=383 => 0b1110,
                _ => 0b1111,
            })
            .unwrap_or(0b0111);

        let hclk = sysclk / (1 << (hpre_bits - 0b0111));

        assert!(hclk <= 72_000_000);

        let ppre1_bits = self
            .pclk1
            .map(|pclk1| match hclk / pclk1 {
                0 => unreachable!(),
                1 => 0b011,
                2 => 0b100,
                3..=5 => 0b101,
                6..=11 => 0b110,
                _ => 0b111,
            })
            .unwrap_or(0b011);

        let ppre1 = 1 << (ppre1_bits - 0b011);
        let pclk1 = hclk / u32(ppre1);

        assert!(pclk1 <= 36_000_000);

        let ppre2_bits = self
            .pclk2
            .map(|pclk2| match hclk / pclk2 {
                0 => unreachable!(),
                1 => 0b011,
                2 => 0b100,
                3..=5 => 0b101,
                6..=11 => 0b110,
                _ => 0b111,
            })
            .unwrap_or(0b011);

        let ppre2 = 1 << (ppre2_bits - 0b011);
        let pclk2 = hclk / u32(ppre2);

        assert!(pclk2 <= 72_000_000);

        // adjust flash wait states
        unsafe {
            acr.acr().write(|w| {
                w.latency().bits(if sysclk <= 24_000_000 {
                    0b000
                } else if sysclk <= 48_000_000 {
                    0b001
                } else {
                    0b010
                })
            })
        }

        // the USB clock is only valid if an external crystal is used, the PLL is enabled, and the
        // PLL output frequency is a supported one.
        // usbpre == false: divide clock by 1.5, otherwise no division
        let usb_ok = has_usb() && self.hse.is_some();
        let (usbpre, usbclk_valid) = match (usb_ok, sysclk) {
            (true, 72_000_000) => (false, true),
            (true, 48_000_000) => (true, true),
            _ => (true, false),
        };

        let rcc = unsafe { &*RCC::ptr() };

        if self.hse.is_some() {
            // enable HSE and wait for it to be ready

            rcc.cr.modify(|_, w| w.hseon().set_bit());

            while rcc.cr.read().hserdy().bit_is_clear() {}
        }

        // enable PLL and wait for it to be ready
        if let Some(pll_config) = pll_config {
            rcc.cfgr.modify(|_, w| {
                w.pllmul()
                    .variant(pll_config.mul)
                    .pllsrc()
                    .variant(pll_config.src)
            });

            if let Some(pll_div) = pll_config.div {
                rcc.cfgr2.modify(|_, w| w.prediv().variant(pll_div));
            };

            rcc.cr.modify(|_, w| w.pllon().set_bit());

            while rcc.cr.read().pllrdy().bit_is_clear() {}
        };

        // set prescalers and clock source
        rcc.cfgr.modify(|_, w| unsafe {
            set_usbpre(w, usbpre)
                .ppre2()
                .bits(ppre2_bits)
                .ppre1()
                .bits(ppre1_bits)
                .hpre()
                .bits(hpre_bits)
                .sw()
                .variant(sysclk_source)
        });

        Clocks {
            hclk: Hertz(hclk),
            pclk1: Hertz(pclk1),
            pclk2: Hertz(pclk2),
            ppre1,
            ppre2,
            sysclk: Hertz(sysclk),
            usbclk_valid,
        }
    }
}

/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no longer be changed
#[derive(Clone, Copy)]
pub struct Clocks {
    hclk: Hertz,
    pclk1: Hertz,
    pclk2: Hertz,
    ppre1: u8,
    // TODO remove `allow`
    #[allow(dead_code)]
    ppre2: u8,
    sysclk: Hertz,
    usbclk_valid: bool,
}

impl Clocks {
    /// Returns the frequency of the AHB
    pub fn hclk(&self) -> Hertz {
        self.hclk
    }

    /// Returns the frequency of the APB1
    pub fn pclk1(&self) -> Hertz {
        self.pclk1
    }

    /// Returns the frequency of the APB2
    pub fn pclk2(&self) -> Hertz {
        self.pclk2
    }

    pub(crate) fn ppre1(&self) -> u8 {
        self.ppre1
    }

    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn ppre2(&self) -> u8 {
        self.ppre2
    }

    /// Returns the system (core) frequency
    pub fn sysclk(&self) -> Hertz {
        self.sysclk
    }

    /// Returns whether the USBCLK clock frequency is valid for the USB peripheral
    pub fn usbclk_valid(&self) -> bool {
        self.usbclk_valid
    }
}

// TODO Try this method? https://github.com/stm32-rs/stm32f4xx-hal/blob/c888eaa4d7dcac9670102896e54945efb5c41a3b/src/rcc.rs#L128

// #[test]
// fn test_get_sysclk_source() {
//     // test with HSI without pll
//     let (sysclk, sysclk_source, pll_options) = CFGR::calc_sysclk();
//     assert!(cfgr.hse.is_none());
//     assert_eq!(sysclk == HSI);
//     assert_eq!(sysclk_source == rcc::cfgr::SW_A::HSI);
//     assert!(pll_options.is_none());

//     // test with HSE without pll
//     let cfgr = CFGR::use_hse(16.mhz());
//     let (sysclk, sysclk_source, pll_options) = cfgr.calc_sysclk();
//     assert!(cfgr.hse.is_some());
//     assert_eq!(sysclk == cfgr.hse.unwrap());
//     assert_eq!(sysclk_source == rcc::cfgr::SW_A::HSE);
//     assert!(pll_options.is_none());

//     // test with HSE with pll
//     let cfgr = CFGR::use_hse(8.mhz()).sysclk(48.mhz());
//     let (sysclk, sysclk_source, pll_options) = cfgr.calc_sysclk();
//     assert!(cfgr.hse.is_some());
//     assert_eq!(sysclk == 48.mhz().into());
//     assert_eq!(sysclk_source == rcc::cfgr::SW_A::PLL);
//     assert!(pll_options.is_some())
// }

//#[test]
//fn test_pll_setup() {
//    // Test if HSI_DIV2 was chosen as pll source
//    let cfgr = CFGR::sysclk(32.mhz());
//    let (sysclk, pll_options) = cfgr.calc_pll(cfgr.sysclk.unwrap());
//    assert_eq!(pll_options.src == rcc:::cfgr::PLLSRC_A::HSI_DIV2);

//    // Test if HSE_DIV_PREDIV was chosen as pll source
//    let cfgr = CFGR::use_hse(8.mhz()).sysclk(32.mhz());
//    let (sysclk, pll_options) = cfgr.calc_pll(cfgr.sysclk.unwrap());
//    assert_eq!(pll_options.src == rcc:::cfgr::PLLSRC_A::HSE_DIV_PREDIV);

//    // Test if HSI_DIV_PREDIV was chosen as pll source
//    // TODO HSI_DVI2 is only chosen no division is needed (only multiplication)
//    // For other devices, which have HSI /2 by default, do not allow division by 1, which means
//    // So the difference lies in the case that the other devices allow fine grained pll only for
//    // HSE not for HSI.
//    // TODO See context below
//    // So if something * 1 is needed, we **have** to devide by two
//    // In this case consider PLLTXPRE???
//    // So what todo with HSI/2 than, or when to choose HSI_DIV_PREDIV?
//    // TODO implementation thoughts
//    // Calculate needed divisor and multiplicator
//    // Check how it is realizable with PLL
//    // Special case the fact, that multiplication by one means division by 2 and multiplication by
//    // 2n
//    // Or not???
//    // The division multiplication "finder" can be it's own function
//    // Special caseing the register values has to be split by devices.
//    //
//    // **The real difference between those devices**
//    //
//    // - pll source clock is divided by two by default for HSI/2 devices.
//    // - division for HSI is not programmable for HSI/2 devices
//    #[cfg(any(
//        feature = "stm32f302xd",
//        feature = "stm32f302xe",
//        feature = "stm32f303xd",
//        feature = "stm32f303xe",
//        feature = "stm32f398",
//    ))]
//    {
//        // TODO pll div and multiplication are chosen quite differently
//        // if use want 48 mhz out of 32 mhz you could use (32 / 2) * 3 == 48
//        // or if you want 20 Mhz out of 8 Mhz (8 / 2) * 5
//        // or if you want 10 Mhz out of 8 Mhz (8 / 4) * 5
//        // or if you want 72 Mhz out of 32 Mhz (32 / 4) * 9
//        // or if you want 30 Mhz out of 16 Mhz (32 / 8) * 15
//        // or if you want 13 Mhz out of 16 Mhz (16 / 16) * 15
//        // or if you want 16 Mhz out of 8 Mhz (8 / 2) * 1, which is not possible, as * 1 is not
//        // allowed, so (8 / 2) * 4 is needed.
//        let cfgr = CFGR::use_hse(8.mhz()).sysclk(32.mhz());
//        let (sysclk, pll_options) = cfgr.calc_pll(cfgr.sysclk.unwrap());
//        assert_eq!(pll_options.src == rcc:::cfgr::PLLSRC_A::HSI_DIV_PREDIV);
//    }

//    let cfgr = CFGR::use_hse(32.mhz()).sysclk(16.mhz());
//    let (sysclk, pll_options) = cfgr.calc_pll(cfgr.sysclk.unwrap());
//    assert_eq!(sysclk == 16.mhz().into());
//    assert_eq!(sysclk_source == rcc::cfgr::SW_A::PLL);
//    assert!(pll_options.is_some())
//}
