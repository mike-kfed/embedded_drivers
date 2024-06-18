//! e-paper display wrapper
use core::convert::Infallible;
use epd_waveshare::{epd2in9_v2 as epd, prelude::*};

use crate::setup::{EpDisplay, EpDisplaySpi};

use embedded_graphics::prelude::*;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    text::Text,
};

pub struct EpaperDisplay {
    display: epd::Display2in9,
    empty_display: epd::Display2in9,
    spi: EpDisplaySpi,
    epd: EpDisplay,
}

impl EpaperDisplay {
    pub fn new(spi: EpDisplaySpi, epd: EpDisplay) -> Self {
        // Use display graphics from embedded-graphics
        let mut display = epd::Display2in9::default();
        let _ = display.clear(Color::White);
        display.set_rotation(DisplayRotation::Rotate90);
        let mut empty_display = epd::Display2in9::default();
        let _ = empty_display.clear(Color::White);
        empty_display.set_rotation(DisplayRotation::Rotate90);
        Self {
            display,
            empty_display,
            spi,
            epd,
        }
    }

    /// gentle redraw screen
    pub fn draw(&mut self, delay: &mut cortex_m::delay::Delay) -> Result<(), Infallible> {
        // Display updated frame
        self.epd
            .update_old_frame(&mut self.spi, self.empty_display.buffer(), delay)?;
        self.epd
            .update_new_frame(&mut self.spi, self.display.buffer(), delay)?;
        self.epd.display_new_frame(&mut self.spi, delay)?;
        Ok(())
    }

    /// render basic info about machine to display buffer
    pub fn render_firmware_version(&mut self) {
        // Use embedded graphics for drawing Text
        let style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(Color::Black)
            .build();

        let firmware_version = env!("CARGO_PKG_VERSION");
        let device_name = "Waveshare e-Paper Touchpanel";

        let _ = Text::new(device_name, Point::new(10, 10), style).draw(&mut self.display);
        let _ = Text::new(firmware_version, Point::new(10, 20), style).draw(&mut self.display);
    }

    /// completely redraw
    pub fn redraw(&mut self, delay: &mut cortex_m::delay::Delay) -> Result<(), Infallible> {
        // Display updated frame
        self.epd
            .update_frame(&mut self.spi, self.display.buffer(), delay)?;
        self.epd.display_frame(&mut self.spi, delay)?;
        Ok(())
    }
}
