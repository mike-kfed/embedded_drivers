#![no_std]
#![no_main]

use core::sync::atomic::{AtomicUsize, Ordering};
use defmt_rtt as _;
use panic_probe as _;

mod epaper;
mod setup;
mod touchpanel;

use rtic_monotonics::rp2040::prelude::*;
rp2040_timer_monotonic!(Mono);

defmt::timestamp! {"{=u64}", {
    static COUNT: AtomicUsize = AtomicUsize::new(0);
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n as u64
}
}

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [XIP_IRQ])]
mod app {
    use super::*;
    /// the RTIC application
    use crate::setup::*;
    use defmt::{debug, error, info};
    use embedded_hal::digital::v2::ToggleableOutputPin;

    #[shared]
    struct Shared {
        led_blink: bool,
        delay: cortex_m::delay::Delay,
    }

    #[local]
    struct Local {
        led: LedPin,
        touchpanel: EpdTouchPanel,
        touch_irq: EpdTouchIrq,
        epd: Option<crate::epaper::EpaperDisplay>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        info!("preSetup run");
        let Peripherals {
            delay,
            led,
            touchpanel,
            touch_irq,
            epd,
        } = setup(ctx.device, ctx.core);
        let led_blink = true;

        info!("Setup run");
        epd_redraw::spawn().ok();
        led_blinker::spawn().ok();

        (
            Shared { led_blink, delay },
            Local {
                led,
                touchpanel,
                touch_irq,
                epd,
            },
        )
    }

    #[task(priority = 1, local = [led], shared = [ led_blink])]
    async fn led_blinker(mut ctx: led_blinker::Context) {
        ctx.shared.led_blink.lock(|led_blink| {
            if *led_blink {
                debug!("led-blink");
                ctx.local.led.toggle().ok();
            }
        });
        Mono::delay(500u64.millis()).await;
    }

    /// runs only once and fully redraws the display
    #[task(priority = 1, local = [epd], shared = [delay])]
    async fn epd_redraw(mut ctx: epd_redraw::Context) {
        if let Some(epd) = ctx.local.epd {
            epd.render_firmware_version();
            ctx.shared.delay.lock(|d| {
                info!("drawing");

                if epd.redraw(d).is_ok() {
                    info!("drew");
                }
            });
        }
    }

    /// GPIO interrupt handler
    #[task(binds=IO_IRQ_BANK0, local = [touch_irq, touchpanel], shared = [delay])]
    fn on_gpio(mut ctx: on_gpio::Context) {
        // used for Touchpanel interrupts
        ctx.local
            .touch_irq
            .clear_interrupt(rp_pico::hal::gpio::Interrupt::EdgeLow);
        ctx.shared.delay.lock(|d| {
            let scan = ctx.local.touchpanel.scan(d);
            match scan {
                Ok(touches) => {
                    for t in touches {
                        if t.event_id == 4 {
                            // FIXME: ID 4 seems to be touch ended, somehow store first ID 2 to see
                            // if finger has not moved (also useful to implement drag and drop)
                            info!("x: {} y: {} p: {} id: {}", t.x, t.y, t.p, t.event_id);
                        }
                    }
                }
                Err(code) => error!("touch error: {}", code),
            }
        });
    }

    /// Task with least priority that only runs when nothing else is running.
    /// avoids rtic sending the board to sleep
    #[idle(local = [x: u32 = 0])]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }
}
// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf();
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
