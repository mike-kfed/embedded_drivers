//! Configuration of Modem
#![allow(non_camel_case_types)]

#[derive(core::fmt::Debug, ufmt::derive::uDebug, defmt::Format)]
pub enum Mrf89Error {
    DeviceNotFound(u8),
    FrequencyNotAvailable,
    PresetNotImplemented,
    PresetNotFound,
    SyncWordTooLong,
    CutOffFreqTooHigh(u32),
    ChannelActive,
    SpiTransferError,
    MsgTooLong,
    MsgTooShort,
    MsgNotForUs,
    Timeout,
    BufWriteFailed,
    BufLenZero,
    NoPllLock,
    NotImplemented,
}

/// MRF89XA Crystal Frequency in kHz
pub const XTAL_FREQ: u32 = 12800;
pub const BROADCAST_ADDRESS: u8 = 0xff;
pub const MRF89_HEADER_LEN: u8 = 4;
/// is max FIFO size of 64 bytes minus header length
pub const MRF89_MAX_MSG_LEN: u8 = 60;

/// Configuration builder
pub struct Config {
    pub(crate) modem_config: Option<crate::modem_config::ModemConfigChoice>,
}

/// MRF89XA configuration struct
impl Config {
    // Creates a new configuration object with default values
    pub fn new() -> Self {
        Config { modem_config: None }
    }

    pub fn modem_config(&mut self, cfg: crate::modem_config::ModemConfigChoice) -> &mut Self {
        self.modem_config = Some(cfg);
        self
    }
}

pub struct MsgHeader {
    pub to: u8,
    pub from: u8,
    pub id: u8,
    pub flags: u8,
}

impl Default for MsgHeader {
    fn default() -> Self {
        Self {
            to: BROADCAST_ADDRESS,
            from: BROADCAST_ADDRESS,
            id: 0,
            flags: 0,
        }
    }
}
