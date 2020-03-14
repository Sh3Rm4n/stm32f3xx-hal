//! Reset and Clock Control

use core::cmp;

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

enum PllArithmetic {
    MUL(rcc::cfgr::PLLMUL_A),
    DIV(rcc::cfgr2::PREDIV_A),
}

struct PllOptions {
    src: rcc::cfgr::PLLSRC_A,
    arithmetic: Option<PllArithmetic>,
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

    /// Returns a tuple of the (pllsrclk frequency, pllmul, and pllsrc).
    #[cfg(not(any(
        feature = "stm32f302xd",
        feature = "stm32f302xe",
        feature = "stm32f303xd",
        feature = "stm32f303xe",
        feature = "stm32f398"
    )))]
    fn calc_pll(&self) -> (u32, u32, rcc::cfgr::PLLSRC_A) {
        let pllsrcclk = self.hse.unwrap_or(HSI / 2);
        let pllmul = self.sysclk.unwrap_or(pllsrcclk) / pllsrcclk;
        assert!(pllmul > 16);

        let pllsrc = if self.hse.is_some() {
            rcc::cfgr::PLLSRC_A::HSE_DIV_PREDIV
        } else {
            rcc::cfgr::PLLSRC_A::HSI_DIV2
        };
        (pllsrcclk, pllmul, pllsrc)
    }

    /// Calulate the values for the pll multiplicator (PLLMUL) and the pll divisior (PLLDIV).
    ///
    /// These values are chosen depending on the chosen system clock and the frequency of the src
    /// clk
    #[cfg(any(
        feature = "stm32f302xd",
        feature = "stm32f302xe",
        feature = "stm32f303xd",
        feature = "stm32f303xe",
        feature = "stm32f398",
    ))]
    fn calc_pll(&self) -> (u32, PllOptions) {
        let hclk = self.hse.unwrap_or(HSI);

        // Set pll muliplicator or divisor according to sysclk
         let (pll_mul, pll_div): (Option<u32>, Option<u32>) = if let Some(sysclk) = self.sysclk {
            // Use the multiplicator to make sysclk faster than pllsrcclk
            if sysclk > hclk {
                (Some(sysclk / hclk), None)
            // Use the divisor to make sysclk slower than pllsrcclk
            } else if sysclk < hclk {
            // TODO This case differs for different devieces, which caan not devide HSI as it is
            // fixed to /2
                (None, Some(hclk / sysclk))
            } else {
                (None, None)
            }
        } else {
            (None, None)
        };

        // Select hardware clock source of the PLL
        let pll_src = if self.hse.is_some() {
            rcc::cfgr::PLLSRC_A::HSE_DIV_PREDIV
        }
        else {
            // default to sysclk = hclk
            if pll_mul.is_none() && pll_div.is_none() {
                // PLLMUL is multiplicating with 2 by default.
                // Enable the HSI_DIV2 source, which forces PREDIV to be dividing by 2.
                rcc::cfgr::PLLSRC_A::HSI_DIV2
            } else {
                rcc::cfgr::PLLSRC_A::HSI_DIV_PREDIV
            }
        };


        let pll_arithmetic = if let Some(mul) = pll_mul {
            Some(PllArithmetic::MUL(
                match mul as u8 {
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
                    17 => rcc::cfgr::PLLMUL_A::MUL16X,
                    _ => unreachable!(),
                }
            ))
        } else if let Some(div) = pll_div {
            Some(PllArithmetic::DIV(
                match div as u8 {
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
            ))
        } else {
            // No division or multiplication
            None
        };

        let sysclk = (hclk / pll_div.unwrap_or(1)) * pll_mul.unwrap_or(1);
        assert!(sysclk <= 72_000_000);

        (sysclk, PllOptions {
            src: pll_src,
            arithmetic: pll_arithmetic,
        })
    }

    /// Returns a tuple containing the effective sysclk rate and optional pll settings.
    fn calc_sysclk(&self) -> (u32, rcc::cfgr::SW_A, Option<PllOptions>) {
        // If a sysclk is given, check if the PLL has to be used,
        // else select the system clock source, which is either HSI or HSE.
        if let Some(sysclk) = self.sysclk {
            if let Some(hseclk) = self.hse {
                if sysclk == hseclk {
                    // No need to use the PLL
                   (hseclk, rcc::cfgr::SW_A::HSE, None)
                } else {
                    let clock_with_pll = self.calc_pll();
                    (clock_with_pll.0, rcc::cfgr::SW_A::PLL, Some(clock_with_pll.1))
                }
            } else {
                if sysclk == HSI {
                    // No need to use the PLL
                    (HSI,rcc::cfgr::SW_A::HSE, None)
                } else {
                    let clock_with_pll = self.calc_pll();
                    (clock_with_pll.0, rcc::cfgr::SW_A::PLL, Some(clock_with_pll.1))
                }
            }
        } else if let Some(hseclk) = self.hse {
            // Use HSE as system clock
            (hseclk, rcc::cfgr::SW_A::HSE, None)
        } else  {
            // Use HSI as system clock
            (HSI, rcc::cfgr::SW_A::HSI, None)
        }
    }

    /// Freezes the clock configuration, making it effective
    pub fn freeze(self, acr: &mut ACR) -> Clocks {
        let (sysclk, pll_options) = self.calc_sysclk();

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
        // TODO pll_options check needed?
        let usb_ok = has_usb() && self.hse.is_some() && pll_options.is_some();
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

        if let Some((pllmul_bits, pllsrc)) = pll_options {
            // enable PLL and wait for it to be ready
            rcc.cfgr
                .modify(|_, w| w.pllmul().bits(pllmul_bits).pllsrc().variant(pllsrc));

            rcc.cr.modify(|_, w| w.pllon().set_bit());

            while rcc.cr.read().pllrdy().bit_is_clear() {}
        }

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
                // TODO
                .variant(sysclk_source)

        // set predvisor for devices, which have them
        if pllsrc_options.is_some() {
            rcc.cfgr2.modify(|_, w| {
                w.prediv.variant()
            });
        }

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
