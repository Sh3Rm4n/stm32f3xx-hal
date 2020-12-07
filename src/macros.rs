/// Internal macro to seal traits
///
/// It is used to seal trait and avoid repetition,
/// as the super trait implementation does crate repetition
/// and tracking two places, when adding or removing
/// traits or pins carefully.
///
/// # Example
///
/// ```
/// crate::sealed! {
///     #[cfg(any(feature = "gpio-f303", feature = "gpio-f303e", feature = "gpio-f373"))]
///     impl SclPin<I2C2> where pin: gpiof::PF6<AF4>;
///     impl SdaPin<I2C1> where pin: gpiob::PB7<AF4>;
/// }
///
/// // Sub-traits using the sealed trait as super trait have to
/// // com after the macro decleratoin as it defines a interal
/// // module named `private`.
/// pub trait SclPin<I2C>: private::Sealed {}
/// pub trait SdaPin<I2C>: private::Sealed {}
/// ```
#[macro_export]
macro_rules! sealed {
    ($($(#[$gate:meta])* impl $ptype:ty where pin: $pin:ty;)+) => {
        mod private {
            use super::*;
            pub trait Sealed {}

            $(
                $(#[$gate])*
                impl Sealed for $pin {}
            )+
        }

        $(
            $(#[$gate])*
            impl $ptype for $pin {}
            // impl $ptype<$periph> for $pin {}
        )+
    }
}
