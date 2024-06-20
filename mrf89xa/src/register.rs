//! Registers of MRF89XA
//!
//! See [datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/MRF89XA-Ultra-Low-Power-Integrated-ISM-Band-Sub-GHz-Transceiver-%20Data-Sheet-DS70000622E.pdf)
#![allow(non_camel_case_types, dead_code, clippy::upper_case_acronyms)]
use modular_bitfield::prelude::*;

use crate::config::Mrf89Error;

#[allow(dead_code)]
#[repr(u8)]
pub enum Register {
    // Register names from Figure 2-18
    GCONREG = 0x00,
    DMODREG = 0x01,
    FDEVREG = 0x02,
    BRSREG = 0x03,
    FLTHREG = 0x04,
    FIFOCREG = 0x05,
    R1CREG = 0x06,
    P1CREG = 0x07,
    S1CREG = 0x08,
    R2CREG = 0x09,
    P2CREG = 0x0a,
    S2CREG = 0x0b,
    PACREG = 0x0c,
    FTXRXIREG = 0x0d,
    FTPRIREG = 0x0e,
    RSTHIREG = 0x0f,
    FILCREG = 0x10,
    PFCREG = 0x11,
    SYNCREG = 0x12,
    // the next 2 is ambiguous in the docs,
    // this seems to agree with whats in the chip:,
    RSVREG = 0x13,
    RSTSREG = 0x14,
    OOKCREG = 0x15,
    SYNCV31REG = 0x16,
    SYNCV23REG = 0x17,
    SYNCV15REG = 0x18,
    SYNCV07REG = 0x19,
    TXCONREG = 0x1a,
    CLKOREG = 0x1b,
    PLOADREG = 0x1c,
    NADDSREG = 0x1d,
    PKTCREG = 0x1e,
    FCRCREG = 0x1f,
}

impl Register {
    /// Get register address
    pub fn addr(self) -> u8 {
        self as u8
    }

    pub fn from_u8(v: u8) -> Self {
        match v {
            0x00 => Self::GCONREG,
            0x01 => Self::DMODREG,
            0x02 => Self::FDEVREG,
            0x03 => Self::BRSREG,
            0x04 => Self::FLTHREG,
            0x05 => Self::FIFOCREG,
            0x06 => Self::R1CREG,
            0x07 => Self::P1CREG,
            0x08 => Self::S1CREG,
            0x09 => Self::R2CREG,
            0x0a => Self::P2CREG,
            0x0b => Self::S2CREG,
            0x0c => Self::PACREG,
            0x0d => Self::FTXRXIREG,
            0x0e => Self::FTPRIREG,
            0x0f => Self::RSTHIREG,
            0x10 => Self::FILCREG,
            0x11 => Self::PFCREG,
            0x12 => Self::SYNCREG,
            0x13 => Self::RSVREG,
            0x14 => Self::RSTSREG,
            0x15 => Self::OOKCREG,
            0x16 => Self::SYNCV31REG,
            0x17 => Self::SYNCV23REG,
            0x18 => Self::SYNCV15REG,
            0x19 => Self::SYNCV07REG,
            0x1a => Self::TXCONREG,
            0x1b => Self::CLKOREG,
            0x1c => Self::PLOADREG,
            0x1d => Self::NADDSREG,
            0x1e => Self::PKTCREG,
            0x1f => Self::FCRCREG,
            _ => panic!(),
        }
    }
}

#[bitfield]
pub struct Gconreg {
    pub rps: bool, // default false = Enable R1/P1/S1 set
    pub vcot: VCOT,
    pub fbs: FBS,
    pub cmod: CMOD,
}

#[derive(BitfieldSpecifier)]
#[bits = 3]
pub enum CMOD {
    Reserved0 = 0b111,
    Reserved1 = 0b110,
    Reserved2 = 0b101,
    TRANSMIT = 0b100,
    RECEIVE = 0b011,
    FS = 0b010, // frequency Synthesizer
    STANDBY = 0b001,
    SLEEP = 0b000,
}

#[derive(BitfieldSpecifier, Debug, core::cmp::PartialEq)]
#[bits = 2]
pub enum FBS {
    FBS_863_870_OR_950_960 = 0b10,
    FBS_915_928 = 0b01, // default
    FBS_902_915 = 0b00,
}

impl FBS {
    pub fn from_frequency(frequency: u32) -> Result<Self, Mrf89Error> {
        match frequency {
            863_000..=870_000 => Ok(Self::FBS_863_870_OR_950_960),
            950_000..=960_000 => Ok(Self::FBS_863_870_OR_950_960),
            915_000..=928_000 => Ok(Self::FBS_915_928),
            902_000..=914_999 => Ok(Self::FBS_902_915),
            _ => Err(Mrf89Error::FrequencyNotAvailable),
        }
    }
}

#[derive(BitfieldSpecifier)]
#[bits = 2]
pub enum VCOT {
    VCOT_180MV = 0b11,
    VCOT_120MV = 0b10,
    VCOT_60MV = 0b01,
    VCOT_TANK = 0b00, //  Vtune determined by tank inductors values (default)
}

#[bitfield]
pub struct Dmodreg {
    pub if_gain: IfGain,
    data_mode1: bool,
    pub ook_type: OokType,
    data_mode0: bool,
    pub modsel: ModulationType,
}

#[derive(Debug, core::cmp::PartialEq)]
pub enum DataOperationMode {
    Continuous, // default
    Buffered,
    Packet,
    PacketLSB,
}

impl Dmodreg {
    pub fn data_operation_mode(&self) -> DataOperationMode {
        match (self.data_mode0(), self.data_mode1()) {
            (false, false) => DataOperationMode::Continuous,
            (true, false) => DataOperationMode::Buffered,
            (false, true) => DataOperationMode::Packet,
            (true, true) => DataOperationMode::PacketLSB,
        }
    }

    pub fn with_data_operation_mode(mut self, dopmode: DataOperationMode) -> Self {
        match dopmode {
            DataOperationMode::Continuous => {
                self.set_data_mode0(false);
                self.set_data_mode1(false);
            }
            DataOperationMode::Buffered => {
                self.set_data_mode0(true);
                self.set_data_mode1(false);
            }
            DataOperationMode::Packet => {
                self.set_data_mode0(false);
                self.set_data_mode1(true);
            }
            DataOperationMode::PacketLSB => {
                self.set_data_mode0(true);
                self.set_data_mode1(true);
            }
        };
        self
    }

    pub fn set_data_operation_mode(&mut self, dopmode: DataOperationMode) {
        match dopmode {
            DataOperationMode::Continuous => {
                self.set_data_mode0(false);
                self.set_data_mode1(false);
            }
            DataOperationMode::Buffered => {
                self.set_data_mode0(true);
                self.set_data_mode1(false);
            }
            DataOperationMode::Packet => {
                self.set_data_mode0(false);
                self.set_data_mode1(true);
            }
            DataOperationMode::PacketLSB => {
                self.set_data_mode0(true);
                self.set_data_mode1(true);
            }
        };
    }
}

#[derive(BitfieldSpecifier)]
#[bits = 2]
pub enum IfGain {
    Minus0dB = 0b00, // default
    Minus4_5dB = 0b01,
    Minus9_0dB = 0b10,
    Minus13_5dB = 0b11,
}

#[derive(BitfieldSpecifier)]
#[bits = 2]
pub enum OokType {
    Reserved = 0b11,
    AverageMode = 0b10,
    PeakMode = 0b01, // default
    FixedThreshold = 0b00,
}

#[derive(BitfieldSpecifier, core::clone::Clone)]
#[bits = 2]
pub enum ModulationType {
    Reserved0 = 0b11,
    Reserved1 = 0b00,
    FSK = 0b10, // default
    OOK = 0b01,
}

/// Frequency deviation control register
#[bitfield]
pub struct Fdevreg {
    fdval: u8,
}

impl Fdevreg {
    ///  implementation according to Section 3.3.3.
    fn compute_fdval(&self, frequency: u32, bit_rate: u32) -> Result<u8, ()> {
        //frequency * 32 * (1 + self.fdval()) = crate::config::XTAL_FREQ;
        if frequency < 33 {
            // fdev at least 33kHz for communication between two MRF89XA
            return Err(());
        }
        let fdval = (crate::config::XTAL_FREQ / (frequency * 32) - 1) as u8;

        let fdev = crate::config::XTAL_FREQ / (32 * (fdval as u32 + 1));
        let beta = 2 * fdev / bit_rate; // modulation index Equation 3.6 section 3.3.3
        if beta < 2 {
            Err(())
        } else {
            Ok(fdval)
        }
    }

    /// frequency in kHz, bit_rate in kbps
    /// tries to set the fdval based on desired frequency deviation and bit_rate
    pub fn try_freq_deviation(
        mut self,
        frequency: u32,
        bit_rate: u32,
        mod_type: ModulationType,
    ) -> Result<Self, ()> {
        match mod_type {
            ModulationType::FSK => {
                self.set_fdval(self.compute_fdval(frequency, bit_rate)?);
                Ok(self)
            }
            ModulationType::OOK => {
                self.set_fdval(0b0000_0011);
                Ok(self)
            } // suggested 100kHz in Section 3.3.4
            _ => Err(()),
        }
    }

    /// compute frequency deviation in kHz
    pub fn freq_deviation(&self) -> u32 {
        crate::config::XTAL_FREQ / (32 * (self.fdval() as u32 + 1))
    }
}

/// Bitrate set Register
#[bitfield]
pub struct Brsreg {
    pub brval: B7,
    reserved: B1,
}

impl Brsreg {
    /// compute bitrate in kbps
    pub fn bitrate(&self) -> u32 {
        crate::config::XTAL_FREQ / (64 * (self.brval() as u32 + 1))
    }
}

/// FIFO transmit PLL and RSSI Interrupt request configuration register
#[bitfield]
pub struct Ftprireg {
    pub lenpll: bool,
    pub lstspll: bool,
    rirqs: bool,
    enrirqs: bool,
    irq0txst: bool,
    txdone: bool,
    fifofsc: bool,
    fifofm: bool,
}

/// Possible values for PACREG: Power Amplifier control Register
#[repr(u8)]
pub enum PACREG {
    PARC_23 = 0b0011_1000, // 23us (default)
    PARC_15 = 0b0011_0000,
    PARC_8_5 = 0b0010_1000,
    PARC_3 = 0b0010_0000,
}

#[bitfield]
pub struct Fifocreg {
    pub ftint: B6,
    pub fsize: FifoSize,
}

#[derive(BitfieldSpecifier)]
#[bits = 2]
pub enum FifoSize {
    fsize_64_bytes = 0b11,
    fsize_48_bytes = 0b10,
    fsize_32_bytes = 0b01,
    fsize_16_bytes = 0b00, // default
}

/// SYNCREG: Sync control register 0x12
#[bitfield]
pub struct Syncreg {
    reserved: bool,
    /// Tolerated Error numbers
    syncten: ToleratedErrors,
    /// Sync word size bit
    pub syncwsz: WordSize,
    /// Sync word recognition enable
    pub syncren: bool,
    /// Bit Synchronizer Enable
    bsyncen: bool,
    /// Polyphase Filter Enable
    polfilen: bool,
}

/// Possible values for SYNCREG::syncwsz
#[derive(BitfieldSpecifier)]
#[bits = 2]
pub enum WordSize {
    /// default
    ws_32_bits = 0b11,
    ws_25_bits = 0b10,
    ws_16_bits = 0b01,
    ws_8_bits = 0b00,
}

/// Possible values for SYNCREG::syncten
#[derive(BitfieldSpecifier)]
#[bits = 2]
pub enum ToleratedErrors {
    ThreeErrors = 0b11,
    TwoErrors = 0b10,
    OneError = 0b01,
    /// default
    ZeroErrors = 0b00,
}

/// TXCONREG: Transmit Paramater Configuration Register
#[bitfield]
pub struct Txconreg {
    reserved: bool,
    /// Transmit Output Power
    pub txopval: TransmitOutputPower,
    /// Transmission Interpolation Filter Cut Off Frequency
    pub txipolfv: B4,
}

/// Possible values for TXCONREG::txopval
#[derive(BitfieldSpecifier)]
#[bits = 3]
pub enum TransmitOutputPower {
    dBm_minus_8 = 0b111,
    dBm_minus_5 = 0b110,
    dBm_minus_2 = 0b101,
    dBm_1 = 0b100,
    dBm_4 = 0b011,
    dBm_7 = 0b010,
    /// default
    dBm_10 = 0b001,
    dBm_13 = 0b000,
}

impl Txconreg {
    /// cut off frequency in kHz
    pub fn compute_txipolfv(frequency: u32) -> Result<u8, Mrf89Error> {
        let txipolfv = (frequency * 1000 / 200 / (crate::config::XTAL_FREQ / 12800) * 8) / 1000 - 1;
        // Make sure it is max 4 bit
        if txipolfv as u8 > 15u8 {
            return Err(Mrf89Error::CutOffFreqTooHigh(frequency));
        }
        Ok(txipolfv as u8)
    }

    pub fn cut_off_freq(&self) -> u32 {
        200 * (crate::config::XTAL_FREQ / 12800) * ((self.txipolfv() as u32 + 1) / 8)
    }
}

/// PKTCREG: Packet configuration register 0x1E
#[bitfield]
pub struct Pktcreg {
    /// Status Check CRC Enable (read-only)
    pub stscrcen: bool,
    /// Address filtering
    pub addfil: AddressFiltering,
    /// Check (or calculation) CRC Enable
    pub chkcrcen: bool,
    /// Whitening/Dewhitening Process Enable
    pub whiteon: bool,
    /// Preamble size
    pub presize: PreambleSize,
    /// Packet Length Format
    pub pktlenf: PacketLenFormat,
}

/// Possible values for PKTCREG::pktlenf
#[derive(BitfieldSpecifier)]
#[bits = 1]
pub enum PacketLenFormat {
    VariableLength = 1,
    /// default
    FixedLength = 0,
}

/// Possible values for PKTCREG::presize
#[derive(BitfieldSpecifier)]
#[bits = 2]
pub enum PreambleSize {
    size_4_bytes = 0b11,
    /// default
    size_3_bytes = 0b10,
    size_2_bytes = 0b01,
    size_1_bytes = 0b00,
}

/// Possible values for PKTCREG::presize
#[derive(BitfieldSpecifier)]
#[bits = 2]
pub enum AddressFiltering {
    /// Node Address & 0x00 & 0xFF Accepted; otherwise, rejected
    filter_00_ff = 0b11,
    ///  Node Address & 0x00 Accepted; otherwise, rejected
    filter_00 = 0b10,
    /// Node Address Accepted; otherwise, rejected
    filter = 0b01,
    /// default
    off = 0b00,
}

#[bitfield]
pub struct Ftxrxireg {
    /// FIFO Overrun Clear - true = FIFO Overrun occurred
    pub fovrrun: bool,
    /// FIFO Empty - true = FIFO *not* empty
    pub fifoempty: bool,
    /// FIFO FULL - true = FIFO full
    pub fifofull: bool,
    /// Transmit IRQ1, selects IRQ1 as source in Transmit mode
    /// true = DCLK (in continuous mode)
    /// true = TXDONE, false = FIFOFULL (in buffered or packet-mode mode)
    pub irq1tx: bool,
    /// IRQ1 Receive Standby Bits
    pub irq1rxs: ReceiveStandby1,
    /// IRQ0 Receive Standby Bits
    pub irq0rxs: ReceiveStandby0,
}

/// mode names are:
/// 1: continuous
/// 2: buffered
/// 3: packet mode
/// `${1}_${2}_${3}`
#[derive(BitfieldSpecifier)]
#[bits = 2]
pub enum ReceiveStandby0 {
    /// in packet mode `sync` or if address filtering enabled `adrsmatch`
    sync_sync_adrsmatch = 0b11,
    sync_fifoempty_fifoempty = 0b10,
    rssi_writebyte_writebyte = 0b01,
    sync_undef_plready = 0b00, // default
}

/// mode names are:
/// 1: continuous
/// 2: buffered
/// 3: packet mode
/// `${1}_${2}_${3}`
#[derive(BitfieldSpecifier)]
#[bits = 2]
pub enum ReceiveStandby1 {
    /// FIFO Threshold in bufferd and packet mode
    dclk_fifothres_fifothres = 0b11,
    dclk_rssi_rssi = 0b10,
    dclk_fifofull_fifofull = 0b01,
    dclk_undef_crcok = 0b00, // default
}

pub(crate) const RADIO_HEAD_REGISTERS: [u8; 32] = [
    0x32, 0x84, 0x9, 0x9, 0x0, 0xC0, 0x77, 0x5E, 0x53, 0x0, 0x0, 0x0, 0x18, 0xC8, 0x3, 0x0, 0x72,
    0x38, 0x38, 0x7, 0x60, 0x0, 0x69, 0x81, 0x7E, 0x96, 0x28, 0x0, 0x40, 0x0, 0xD8, 0x0,
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gconreg() {
        let cr = Gconreg::new()
            .with_cmod(CMOD::TRANSMIT)
            .with_fbs(FBS::FBS_863_870_OR_950_960)
            .with_vcot(VCOT::VCOT_60MV)
            .with_rps(false);
        assert_eq!(cr.into_bytes(), [0b10010010]);
    }

    #[test]
    fn test_gconreg_modify() {
        let cr = Gconreg::from_bytes([0b10010010]).with_cmod(CMOD::STANDBY);
        assert_eq!(cr.into_bytes(), [0b00110010]);
        let cr = Gconreg::from_bytes([0b00110010]).with_cmod(CMOD::STANDBY);
        assert_eq!(cr.into_bytes(), [0b00110010]);
    }

    #[test]
    fn test_dmodreg() {
        let cr = Dmodreg::new()
            .with_modsel(ModulationType::FSK)
            .with_ook_type(OokType::PeakMode)
            .with_data_operation_mode(DataOperationMode::Continuous)
            .with_if_gain(IfGain::Minus4_5dB);
        assert_eq!(cr.data_operation_mode(), DataOperationMode::Continuous);
        assert_eq!(cr.into_bytes(), [0b10001001]);
    }

    #[test]
    fn test_dmodreg_opmode() {
        let mut cr = Dmodreg::new()
            .with_modsel(ModulationType::FSK)
            .with_ook_type(OokType::PeakMode)
            .with_if_gain(IfGain::Minus4_5dB);
        cr.set_data_operation_mode(DataOperationMode::Packet);
        assert_eq!(cr.data_operation_mode(), DataOperationMode::Packet);
        assert_eq!(cr.into_bytes(), [0b10001101]);
    }

    #[test]
    fn test_fbs_from_frequency() {
        let fbs = FBS::from_frequency(915_000).unwrap();
        assert_eq!(fbs, FBS::FBS_915_928);
        let fbs = FBS::from_frequency(902_000).unwrap();
        assert_eq!(fbs, FBS::FBS_902_915);
    }

    #[test]
    fn test_fdval_to_freq_deviation() {
        let brs = Fdevreg::new().with_fdval(0b0000_0011);
        assert_eq!(brs.fdval(), 0b0000_0011);
        assert_eq!(brs.freq_deviation(), 100); // >= 100kHz (default when XTAL_FREQ is 12.8MHz)
        assert_eq!(brs.compute_fdval(100, 25).unwrap(), 3);
        assert!(brs.compute_fdval(20, 25).is_err()); // TODO: check for fdev-too-low-error
        assert!(brs.compute_fdval(100, 250).is_err()); // TODO: check for beta-error
    }

    #[test]
    fn test_brval_to_bitrate() {
        let brs = Brsreg::new().with_brval(0b0000_0111);
        assert_eq!(brs.brval(), 0b0000_0111);
        assert_eq!(brs.bitrate(), 25); // >= 25kbps NRZ (default when XTAL_FREQ is 12.8MHz)
    }

    #[test]
    fn test_syncreg() {
        let sr = Syncreg::new()
            .with_polfilen(true)
            .with_syncwsz(WordSize::ws_32_bits)
            .with_syncten(ToleratedErrors::OneError);
        assert_eq!(sr.into_bytes(), [0b10011010]);
    }

    #[test]
    fn test_txconreg() {
        assert_eq!(Txconreg::compute_txipolfv(200).ok(), Some(0b0111)); // default
        assert_eq!(Txconreg::compute_txipolfv(25).ok(), Some(0b0000));
        assert_eq!(Txconreg::compute_txipolfv(50).ok(), Some(0b0001));
        assert_eq!(Txconreg::compute_txipolfv(75).ok(), Some(0b0010));
        assert_eq!(Txconreg::compute_txipolfv(100).ok(), Some(0b0011));
        assert_eq!(Txconreg::compute_txipolfv(125).ok(), Some(0b0100));
        assert!(Txconreg::compute_txipolfv(2000).is_err());
        let sr = Txconreg::new()
            .with_txopval(TransmitOutputPower::dBm_10)
            .with_txipolfv(Txconreg::compute_txipolfv(200).unwrap());
        assert_eq!(sr.cut_off_freq(), 200);
        assert_eq!(sr.into_bytes(), [0b01110010]);
    }

    #[test]
    fn test_pktcreg() {
        let sr = Pktcreg::new()
            .with_presize(PreambleSize::size_4_bytes)
            .with_chkcrcen(true)
            .with_pktlenf(PacketLenFormat::VariableLength)
            .with_addfil(AddressFiltering::off)
            .with_whiteon(true);
        assert_eq!(sr.into_bytes(), [0b11111000]);
    }
}
