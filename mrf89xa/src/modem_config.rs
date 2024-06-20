#![allow(non_camel_case_types)]
use crate::register::ModulationType;

// TODO: setup all configs
// TODO: not necessary for this project, but find solution for static config items
/*
{ RH_MRF89_MODSEL_FSK, 0x0B, 0x63, 0x40 | 0x01, 0x20 }, // FSK_Rb2Fd33
{ RH_MRF89_MODSEL_FSK, 0x0B, 0x27, 0x40 | 0x01, 0x20 }, // FSK_Rb5Fd33
{ RH_MRF89_MODSEL_FSK, 0x0B, 0x13, 0x40 | 0x01, 0x20 }, // FSK_Rb10Fd33
{ RH_MRF89_MODSEL_FSK, 0x09, 0x09, 0x70 | 0x02, 0x20 }, // FSK_Rb20Fd40
{ RH_MRF89_MODSEL_FSK, 0x04, 0x04, 0xB0 | 0x05, 0x40 }, // FSK_Rb40Fd80
{ RH_MRF89_MODSEL_FSK, 0x03, 0x03, 0xD0 | 0x06, 0x40 }, // FSK_Rb50Fd100
{ RH_MRF89_MODSEL_FSK, 0x02, 0x02, 0xE0 | 0x09, 0x60 }, // FSK_Rb66Fd133
{ RH_MRF89_MODSEL_FSK, 0x01, 0x01, 0xF0 | 0x0F, 0x80 }, // FSK_Rb100Fd200
{ RH_MRF89_MODSEL_FSK, 0x01, 0x00, 0xF0 | 0x0F, 0x80 }  // FSK_Rb200Fd200
    */
/*
use heapless::consts::*;
use heapless::LinearMap;

lazy_static::lazy_static! {
    pub(crate) static ref CFG_MAP: LinearMap<ModemConfigChoice, ModemConfig, U16> = [(
        ModemConfigChoice::FSK_Rb2Fd33,
        ModemConfig::new(ModulationType::FSK, 0, 0, 0, 0,)),
        (ModemConfigChoice::FSK_Rb5Fd33,
        ModemConfig::new(ModulationType::FSK, 0, 0, 0, 0,)),
        (ModemConfigChoice::FSK_Rb10Fd33,
        ModemConfig::new(ModulationType::FSK, 0, 0, 0, 0,)),
        (ModemConfigChoice::FSK_Rb20Fd40,
        ModemConfig::new(ModulationType::FSK, 0x09, 0x09, 0x70 | 0x02, 0x20,)),
        (ModemConfigChoice::FSK_Rb40Fd80,
        ModemConfig::new(ModulationType::FSK, 0, 0, 0, 0,)),
        (ModemConfigChoice::FSK_Rb50Fd100,
        ModemConfig::new(ModulationType::FSK, 0, 0, 0, 0,)),
        (ModemConfigChoice::FSK_Rb66Fd133,
        ModemConfig::new(ModulationType::FSK, 0, 0, 0, 0,)),
        (ModemConfigChoice::FSK_Rb100Fd200,
        ModemConfig::new(ModulationType::FSK, 0, 0, 0, 0,)),
                (ModemConfigChoice::FSK_Rb200Fd200,
        ModemConfig::new(ModulationType::FSK, 0, 0, 0, 0,)),
    ]
    .iter()
    .cloned()
    .collect();
}
*/
/// Defines register configuration values for a desired modulation
///
/// Defines values for various configuration fields and registers to
/// achieve a desired modulation speed and frequency deviation.
#[derive(core::clone::Clone)]
pub struct ModemConfig {
    /// Value for MODSEL in DMODREG
    pub modsel: ModulationType,
    /// Value for FDVAL in FDEVREG
    pub fdval: u8,
    /// Value for BRVAL BRSREG
    pub brval: u8,
    /// Value for PASFILV | BUTFILV in FILCREG
    pub filcreg: u8,
    /// cut off frequency in kHz
    pub cut_off_freq: u32,
}

impl ModemConfig {
    pub fn new(
        modsel: ModulationType,
        fdval: u8,
        brval: u8,
        filcreg: u8,
        cut_off_freq: u32,
    ) -> ModemConfig {
        ModemConfig {
            modsel,
            fdval,
            brval,
            filcreg,
            cut_off_freq,
        }
    }
}

/// Choices for setModemConfig() for a selected subset of common
/// data rates and frequency deviations.
/// Rb is the data rate in kbps. Fd is the FSK Frequency deviation in kHz (FDEVREG::fdval).
/// cut_off_freq is set to be higher than Fd
/// These are indexes into MODEM_CONFIG_TABLE. We strongly recommend you use these symbolic
/// definitions and not their integer equivalents: its possible that new values will be
/// introduced in later versions (though we will try to avoid it).
/// OOK is not yet supported.
/// Based on sample configs in MRF89XA.h from Microchip
#[derive(core::clone::Clone, core::cmp::PartialEq, core::cmp::Eq)]
#[repr(u8)]
pub enum ModemConfigChoice {
    /// FSK, No Manchester, Whitened, Rb = 2kbs,    Fd = 33kHz, cut_off_freq=75kHz
    FSK_Rb2Fd33 = 0,
    /// FSK, No Manchester, Whitened, Rb = 5kbs,    Fd = 33kHz, cut_off_freq=75kHz
    FSK_Rb5Fd33,
    /// FSK, No Manchester, Whitened, Rb = 10kbs,   Fd = 33kHz, cut_off_freq=75kHz
    FSK_Rb10Fd33,
    /// FSK, No Manchester, Whitened, Rb = 20kbs,   Fd = 40kHz, cut_off_freq=75kHz
    FSK_Rb20Fd40,
    /// FSK, No Manchester, Whitened, Rb = 40kbs,   Fd = 80kHz, cut_off_freq=125kHz
    FSK_Rb40Fd80,
    /// FSK, No Manchester, Whitened, Rb = 50kbs,   Fd = 100kHz, cut_off_freq=125kHz
    FSK_Rb50Fd100,
    /// FSK, No Manchester, Whitened, Rb = 66kbs,   Fd = 133kHz, cut_off_freq=175kHz
    FSK_Rb66Fd133,
    /// FSK, No Manchester, Whitened, Rb = 100kbs,  Fd = 200kHz, cut_off_freq=225kHz
    FSK_Rb100Fd200,
    /// FSK, No Manchester, Whitened, Rb = 200kbs,  Fd = 200kHz, cut_off_freq=225kHz
    FSK_Rb200Fd200,
}
