//! Touchpanel of Waveshare Pico-CapTouch-ePaper-2.9
//! communicates over i2c, model number unknown, reverse engineered from waveshare example C code
//! <https://github.com/waveshareteam/Touch_e-Paper_HAT/blob/main/c/lib/Driver/ICNT86X.c>
//! more infos <https://www.waveshare.com/wiki/2.9inch_Touch_e-Paper_HAT>

use core::i32;
use defmt::error;

use embedded_hal::{
    blocking::delay::DelayMs,
    blocking::i2c::{Write, WriteRead},
    digital::v2::OutputPin,
};

const TOUCH_PANEL_I2C_ADDR: u8 = 0x48;

/// abstraction handling the touchpanel deployed by waveshare
pub struct TouchPanel<I, R> {
    i2c: I,
    reset_pin: R,
    ic_version: Option<[u8; 2]>,
    fw_version: Option<[u8; 2]>,
}

impl<I, R, E, PinError> TouchPanel<I, R>
where
    I: WriteRead<Error = E> + Write<Error = E>,
    R: OutputPin<Error = PinError>,
{
    /// Side-effect-free constructor.
    /// Nothing will be read or written before `init()` call.
    pub fn new(i2c: I, reset_pin: R) -> Self {
        Self {
            i2c,
            reset_pin,
            ic_version: None,
            fw_version: None,
        }
    }

    /// initialises the touchpanel by resetting it and reading the firmware information via i2c
    pub fn init(&mut self, delay: &mut dyn DelayMs<u16>) {
        self.reset(delay);
        let mut buf = [0u8; 4];
        match self.write_read(0x000a, &mut buf) {
            Ok(_) => {
                self.ic_version = Some([buf[0], buf[1]]);
                self.fw_version = Some([buf[2], buf[3]]);
            }
            Err(e) => {
                error!("touchpanel init error {:?}", e);
            }
        }
    }
    /// Destroy driver instance, consume self and return I2C bus instance.
    #[allow(dead_code)]
    pub fn destroy(self) -> I {
        self.i2c
    }

    /// returns a tuple of IC and FW version
    pub fn version(&self) -> ([u8; 2], [u8; 2]) {
        let ic = self.ic_version.unwrap_or([0, 0]);
        let fw = self.fw_version.unwrap_or([0, 0]);
        (ic, fw)
    }

    /// resets by pulling the reset pin low
    pub fn reset(&mut self, delay: &mut dyn DelayMs<u16>) {
        // Reset touchpanel
        self.reset_pin.set_high().ok();
        delay.delay_ms(100);
        self.reset_pin.set_low().ok();
        delay.delay_ms(100);
        self.reset_pin.set_high().ok();
        delay.delay_ms(100);
    }

    /// writes given buffer to register
    pub fn write(&mut self, register: u16, data: &[u8]) -> Result<(), i32> {
        let mut tdata = heapless::Vec::<u8, 50>::new();
        let _ = tdata.extend_from_slice(&Self::reg2data(register));
        let _ = tdata.extend_from_slice(data);
        self.i2c.write(TOUCH_PANEL_I2C_ADDR, &tdata).map_err(|_| -1)
    }

    /// converts a register address to bytes
    fn reg2data(r: u16) -> [u8; 2] {
        r.to_be_bytes()
    }

    /// reads from Register into given buffer
    pub fn write_read(&mut self, register: u16, data: &mut [u8]) -> Result<(), i32> {
        self.write(register, &[])?;
        self.i2c
            .write_read(TOUCH_PANEL_I2C_ADDR, &Self::reg2data(register), data)
            .map_err(|_| -1)
    }

    /// asks the touchpanel for the last touch coordinates
    /// maximum 5 touches possible
    pub fn scan(&mut self, delay: &mut dyn DelayMs<u16>) -> Result<heapless::Vec<Touch, 5>, i32> {
        let mut buf = [0u8];
        let mut out = heapless::Vec::<Touch, 5>::new();
        self.write_read(0x1001, &mut buf)?;
        if buf[0] == 0 {
            self.write(0x1001, &[0x00])?;
            delay.delay_ms(1);
            return Err(1001);
        } else {
            let touch_count = buf[0];
            // invalid touch-count
            if !(1..=5).contains(&touch_count) {
                self.write(0x1001, &[0x00])?;
                return Err(1011);
            } else {
                let mut coords = [0u8; 100];
                let buf_slice = &mut coords[..(touch_count as usize * 7)];
                self.write_read(0x1002, buf_slice)?;
                self.write(0x1001, &[0x00])?;
                for t in buf_slice[0..]
                    .chunks(7)
                    .filter(|c| c.len() == 7)
                    .map(|c| Touch {
                        x: u16::from_le_bytes([c[1], c[2]]),
                        y: u16::from_le_bytes([c[3], c[4]]),
                        p: c[5],
                        event_id: c[6],
                    })
                {
                    out.push(t).ok();
                }
            }
        }
        Ok(out)
    }
}

pub struct Touch {
    pub x: u16,
    pub y: u16,
    pub p: u8,
    pub event_id: u8,
}
