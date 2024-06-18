use epd_waveshare::{epd2in9_v2 as epd, prelude::*};
use rp2040_hal::fugit::RateExtU32;
use rp2040_hal::gpio::bank0::{
    Gpio10, Gpio11, Gpio12, Gpio13, Gpio16, Gpio17, Gpio25, Gpio6, Gpio7, Gpio8, Gpio9,
};
use rp2040_hal::gpio::{
    DefaultTypeState, FunctionI2C, FunctionSio, PullDown, PullUp, SioInput, SioOutput,
};
use rp_pico::{
    hal::{
        clocks::init_clocks_and_plls,
        gpio::{FunctionSpi, Interrupt, Pin, Pins},
        pac::{self, I2C1, SPI1},
        spi,
        watchdog::Watchdog,
        Clock, Sio, I2C,
    },
    XOSC_CRYSTAL_FREQ,
};

use crate::touchpanel;

pub type LedPin = Pin<Gpio25, FunctionSio<SioOutput>, <Gpio25 as DefaultTypeState>::PullType>;

pub type EpdTouchPanel = touchpanel::TouchPanel<
    I2C<
        I2C1,
        (
            Pin<Gpio6, FunctionI2C, PullUp>,
            Pin<Gpio7, FunctionI2C, PullUp>,
        ),
    >,
    Pin<Gpio16, FunctionSio<SioOutput>, <Gpio16 as DefaultTypeState>::PullType>,
>;
pub type EpDisplay = epd::Epd2in9<
    EpDisplaySpi,
    Pin<Gpio9, FunctionSio<SioOutput>, <Gpio9 as DefaultTypeState>::PullType>,
    Pin<Gpio13, FunctionSio<SioInput>, PullUp>,
    Pin<Gpio8, FunctionSio<SioOutput>, <Gpio8 as DefaultTypeState>::PullType>,
    Pin<Gpio12, FunctionSio<SioOutput>, <Gpio12 as DefaultTypeState>::PullType>,
    cortex_m::delay::Delay,
>;
pub type EpDisplaySpi = spi::Spi<
    spi::Enabled,
    SPI1,
    (
        Pin<Gpio11, FunctionSpi, PullDown>, // tx
        Pin<Gpio10, FunctionSpi, PullDown>, // sck
    ),
    8,
>;
pub type EpdTouchIrq = Pin<Gpio17, FunctionSio<SioInput>, PullUp>;

pub(crate) struct Peripherals {
    pub delay: cortex_m::delay::Delay,
    pub led: LedPin,
    pub touchpanel: EpdTouchPanel,
    pub touch_irq: EpdTouchIrq,
    pub epd: Option<crate::epaper::EpaperDisplay>,
}

#[inline(always)]
pub(crate) fn setup(pac: pac::Peripherals, core: cortex_m::Peripherals) -> Peripherals {
    let mut resets = pac.RESETS;
    // Start the monotonic
    crate::Mono::start(pac.TIMER, &resets);
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = defmt::unwrap!(init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut resets,
        &mut watchdog,
    )
    .ok());

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut resets);
    let led = pins.gpio25.into_push_pull_output();
    // Delay needed for touchpanel and display setup
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // --- e-paper display
    let spi_sclk = pins
        .gpio10
        .into_pull_down_disabled()
        .into_function::<FunctionSpi>(); // CLK for SPI of epd
    let spi_tx = pins
        .gpio11
        .into_pull_down_disabled()
        .into_function::<FunctionSpi>(); // DIN of epd (MOSI of pico)
    let spi = spi::Spi::<_, _, _, 8>::new(pac.SPI1, (spi_tx, spi_sclk));
    // Exchange the uninitialised SPI driver for an initialised one
    let mut spi = spi.init(
        &mut resets,
        clocks.peripheral_clock.freq(),
        20.MHz(),
        embedded_hal::spi::MODE_0,
    );
    let spi_cs = pins.gpio9.into_push_pull_output(); // chip select of epd
    let dc = pins.gpio8.into_push_pull_output(); // Data/Command control (high: data, low: command)
    let reset = pins.gpio12.into_push_pull_output(); // Reset, low active
    let busy_in = pins.gpio13.into_pull_up_input(); // Busy pin

    let epd = if let Ok(mut epd) =
        epd::Epd2in9::new(&mut spi, spi_cs, busy_in, dc, reset, &mut delay, None)
    {
        // safety: SPI commands are Infallible
        #[allow(clippy::unwrap_used)]
        epd.clear_frame(&mut spi, &mut delay).unwrap();
        Some(crate::epaper::EpaperDisplay::new(spi, epd))
    } else {
        None
    };

    // Touchpanel
    let touch_sda_pin = pins
        .gpio6
        .into_pull_up_disabled()
        .into_function::<FunctionI2C>(); // i2c SDA of touch-panel
    let touch_scl_pin = pins
        .gpio7
        .into_pull_up_disabled()
        .into_function::<FunctionI2C>(); // i2c SCL of touch-panel
    let touch_reset = pins.gpio16.into_push_pull_output(); // Reset of touchpanel, low active
    let touch_irq = pins.gpio17.into_pull_up_input(); // Interrupt of Touch-panel
    touch_irq.set_interrupt_enabled(Interrupt::EdgeLow, true);
    let touch_i2c = I2C::i2c1(
        pac.I2C1,
        touch_sda_pin,
        touch_scl_pin,
        100.kHz(),
        &mut resets,
        clocks.peripheral_clock.freq(),
    );

    let mut touchpanel = touchpanel::TouchPanel::new(touch_i2c, touch_reset);
    touchpanel.init(&mut delay);

    Peripherals {
        delay,
        led,
        touchpanel,
        touch_irq,
        epd,
    }
}
