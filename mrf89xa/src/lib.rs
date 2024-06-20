//! MRF89XA driver library

#![no_std]

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi;

use heapless::LinearMap;

mod config;
mod modem_config;
mod register;

use config::{Config, Mrf89Error, MsgHeader, MRF89_HEADER_LEN};
use defmt::{error, info};
use modem_config::{ModemConfig, ModemConfigChoice};
use register::{self as reg, ModulationType, Register};

const SPI_READ: u8 = 0x40;

#[derive(defmt::Format)]
pub enum Event {
    Interrupt,
    IRQ0,
    IRQ1,
    Receive,
    Transmit,
}

#[derive(PartialEq)]
pub enum IntQueueDrain {
    /// exit if nothing in queue, handle one message
    Check,
    /// block until queue has a message
    UntilFirst,
    /// block until queue has a message or timeout reached
    UntilTimeout(u16),
    /// process all Messages in queue and then return
    ProcessAll,
}

#[derive(PartialEq)]
enum OpMode {
    Initialising,
    Sleep,
    /// Channel activity detection in process (if supported)
    Cad,
    Tx,
    Idle,
    Rx,
}

pub struct Mrf89xa<SPI, CSDATA> {
    /// SPI pins to communicate with MRF89XA
    spi: SPI,
    /// ChipSelect Pin for sending Data over SPI to the MRF89XA
    cs_data: CSDATA,
    /// currently active transport operating mode
    mode: OpMode,
    /// modem configuration
    config: Config,
    /// Buffer for incoming message
    buf: heapless::Vec<u8, 64>,
    /// This nodes ID
    this_address: u8,
    /// count of successfully received messages
    rx_good: u16,
    /// count of successfully sent messages
    tx_good: u16,
    /// From Header of last received message
    rx_header: MsgHeader,
    /// Header for sending messages
    tx_header: MsgHeader,
    /// raw value of last RSSI value from RSTSREG
    last_rssi: u8,
    /// Promiscuous mode, accept any message
    promiscuous: bool,
}

impl<SPI, CSDATA, E, PinError> Mrf89xa<SPI, CSDATA>
where
    SPI: spi::SpiDevice<Error = E>,
    CSDATA: OutputPin<Error = PinError>,
{
    /// Creates a new driver from a SPI peripheral with
    /// default configuration.
    pub fn default(spi: SPI, cs_data: CSDATA) -> Result<Self, E> {
        Self::new(spi, cs_data, Config::new())
    }

    /// Takes a config object to initialize the driver
    pub fn new(spi: SPI, cs_data: CSDATA, config: Config) -> Result<Self, E> {
        Ok(Self {
            spi,
            cs_data,
            mode: OpMode::Initialising,
            config,
            buf: heapless::Vec::new(),
            this_address: config::BROADCAST_ADDRESS,
            rx_good: 0,
            tx_good: 0,
            rx_header: Default::default(),
            tx_header: Default::default(),
            last_rssi: 0,
            promiscuous: false,
        })
    }

    /// Initialise module with decent defaults
    pub fn init(&mut self, delay: &mut dyn DelayNs) -> Result<(), config::Mrf89Error> {
        // TODO MRF89 data cant handle SPI greater than 1MHz. verify that?

        // TODO: find register to verify device is a MRF89 series?
        /*
        let id = radio.get_device_id();

        if id != EXPECTED_DEVICE_ID {
            // error
        }
        */
        // Initialise the chip select pins
        self.cs_data.set_high().ok();

        // Make sure we are not in some unexpected mode from a previous run
        self.set_op_mode(reg::CMOD::STANDBY)?;

        // No way to check the device type but lets trivially check there is something there
        // by trying to change a register:
        self.write_reg(reg::Register::FDEVREG, 0xac)?;
        self.write_reg(reg::Register::FDEVREG, 0xac)?;
        self.write_reg(reg::Register::FDEVREG, 0xac)?;
        self.write_reg(reg::Register::FDEVREG, 0xac)?;
        let v = self.read_reg(reg::Register::FDEVREG)?;
        if v != 0xac {
            return Err(config::Mrf89Error::DeviceNotFound(v));
        }
        self.write_reg(reg::Register::FDEVREG, 0x03)?; // back to default
        if self.read_reg(reg::Register::FDEVREG)? != 0x03 {
            return Err(config::Mrf89Error::DeviceNotFound(0));
        }
        // When used with the MRF89XAM9A module, per 75017B.pdf section 1.3, need:
        // crystal freq = 12.8MHz
        // clock output disabled
        // frequency bands 902-915 or 915-928
        // VCOT 60mV
        // OOK max 28kbps
        // Based on 70622C.pdf, section 3.12:
        let cr = reg::Gconreg::new()
            .with_cmod(reg::CMOD::STANDBY)
            .with_fbs(reg::FBS::FBS_902_915)
            .with_vcot(reg::VCOT::V_60MV);
        self.write_reg(reg::Register::GCONREG, cr.into_bytes()[0])?;
        // FSK, Packet mode, LNA 0dB
        let cr = reg::Dmodreg::new()
            .with_modsel(reg::ModulationType::FSK)
            .with_data_operation_mode(reg::DataOperationMode::Packet);
        self.write_reg(reg::Register::DMODREG, cr.into_bytes()[0])?;
        self.write_reg(reg::Register::FDEVREG, 0)?; // done by set_modem_config
        self.write_reg(reg::Register::BRSREG, 0)?; // done by set_modem_config
        self.write_reg(reg::Register::FLTHREG, 0)?; // done by set_modem_config (OOK only)
        let cr = reg::Fifocreg::new().with_fsize(reg::FifoSize::fsize_64_bytes);
        self.write_reg(reg::Register::FIFOCREG, cr.into_bytes()[0])?;
        self.write_reg(reg::Register::R1CREG, 0)?; // done by set_frequency
        self.write_reg(reg::Register::P1CREG, 0)?; // done by set_frequency
        self.write_reg(reg::Register::S1CREG, 0)?; // done by set_frequency
        self.write_reg(reg::Register::R2CREG, 0)?; // Frequency set 2 not used
        self.write_reg(reg::Register::P2CREG, 0)?; // Frequency set 2 not used
        self.write_reg(reg::Register::S2CREG, 0)?; // Frequency set 2 not used
        self.write_reg(reg::Register::PACREG, reg::PACREG::PARC_23 as u8)?;
        // IRQ0 rx mode: SYNC (not used)
        // IRQ1 rx mode: CRCOK
        // IRQ1 tx mode: TXDONE
        let ftxrxi = reg::Ftxrxireg::new()
            .with_irq0rxs(reg::ReceiveStandby0::sync_sync_adrsmatch)
            .with_irq1rxs(reg::ReceiveStandby1::dclk_undef_crcok)
            .with_irq1tx(true);
        self.write_reg(reg::Register::FTXRXIREG, ftxrxi.into_bytes()[0])?;

        let ftpri = reg::Ftprireg::new().with_lenpll(true);
        self.write_reg(reg::Register::FTPRIREG, ftpri.into_bytes()[0])?;
        self.write_reg(reg::Register::RSTHIREG, 0)?; // not used if no RSSI interrupts
        self.write_reg(reg::Register::FILCREG, 0)?; // done by set_modem_config
        self.write_reg(reg::Register::PFCREG, 0x38)?; // 100kHz, recommended, but not used, see RH_MRF89_REG_12_SYNCREG OOK only?
        let syncreg = reg::Syncreg::new()
            .with_syncren(true)
            .with_syncwsz(reg::WordSize::ws_32_bits); // No polyphase, no bsync, sync, 0 errors
        self.write_reg(reg::Register::SYNCREG, syncreg.into_bytes()[0])?;
        self.write_reg(reg::Register::RSVREG, 0x07)?; // default
        self.write_reg(reg::Register::OOKCREG, 0)?; // done by set_modem_config
        self.write_reg(reg::Register::SYNCV31REG, 0)?; // done by set_sync_words
        self.write_reg(reg::Register::SYNCV23REG, 0)?; // done by set_sync_words
        self.write_reg(reg::Register::SYNCV15REG, 0)?; // done by set_sync_words
        self.write_reg(reg::Register::SYNCV07REG, 0)?; // done by set_sync_words
                                                       // TODO: txipolfv set by set_modem_config() and power set by set_tx_power()
        let txconreg = reg::Txconreg::new()
            .with_txopval(reg::TransmitOutputPower::dBm_1)
            .with_txipolfv(reg::Txconreg::compute_txipolfv(200)?);
        self.write_reg(reg::Register::TXCONREG, txconreg.into_bytes()[0])?;
        self.write_reg(reg::Register::CLKOREG, 0)?; // Disable clock output to save power
        self.write_reg(reg::Register::PLOADREG, 0x40)?; // payload=64bytes (no RX-filtering on packet length)
        self.write_reg(reg::Register::NADDSREG, 0)?; // Node Address 0 (default), not used
        let pktcreg = reg::Pktcreg::new()
            .with_presize(reg::PreambleSize::size_3_bytes)
            .with_chkcrcen(true)
            .with_pktlenf(reg::PacketLenFormat::VariableLength)
            .with_addfil(reg::AddressFiltering::off)
            .with_whiteon(true);
        self.write_reg(reg::Register::PKTCREG, pktcreg.into_bytes()[0])?;

        self.write_reg(reg::Register::FCRCREG, 0)?; // default (FIFO access in standby=write, clear FIFO on CRC mismatch)
                                                    // Set some suitable defaults:
        let syncwords = [0x69u8, 0x81, 0x7e, 0x96]; // Same as RH_MRF89XA
        self.set_sync_words(&syncwords)?;

        // try first MRF89XAM9A then MRF89XAM8A
        let mod_type = reg::ModulationType::FSK;
        // failover US to ETSI variant
        if self.set_frequency(915_400, &mod_type, delay).is_err() // US
            && self.set_frequency(865_000, &mod_type, delay).is_err()
        // ETSI
        {
            return Err(config::Mrf89Error::FrequencyNotAvailable);
        }
        let preset = self.config.modem_config.clone();
        let preset = match preset {
            Some(ref preset) => preset,
            None => {
                // Some slow, reliable default speed and modulation
                &modem_config::ModemConfigChoice::FSK_Rb20Fd40
            }
        };
        self.set_modem_config(preset)?;
        self.mode = OpMode::Idle;
        Ok(())
    }

    fn set_modem_config(
        &mut self,
        preset: &modem_config::ModemConfigChoice,
    ) -> Result<(), config::Mrf89Error> {
        let cfg_map: LinearMap<ModemConfigChoice, ModemConfig, 16> = [
            (
                ModemConfigChoice::FSK_Rb2Fd33,
                ModemConfig::new(ModulationType::FSK, 0, 0, 0, 75),
            ),
            (
                ModemConfigChoice::FSK_Rb5Fd33,
                ModemConfig::new(ModulationType::FSK, 0, 0, 0, 75),
            ),
            (
                ModemConfigChoice::FSK_Rb10Fd33,
                ModemConfig::new(ModulationType::FSK, 0, 0, 0, 75),
            ),
            (
                ModemConfigChoice::FSK_Rb20Fd40,
                ModemConfig::new(ModulationType::FSK, 0x09, 0x09, 0x70 | 0x02, 75),
            ),
            (
                ModemConfigChoice::FSK_Rb40Fd80,
                ModemConfig::new(ModulationType::FSK, 0, 0, 0, 125),
            ),
            (
                ModemConfigChoice::FSK_Rb50Fd100,
                ModemConfig::new(ModulationType::FSK, 0, 0, 0, 125),
            ),
            (
                ModemConfigChoice::FSK_Rb66Fd133,
                ModemConfig::new(ModulationType::FSK, 0, 0, 0, 175),
            ),
            (
                ModemConfigChoice::FSK_Rb100Fd200,
                ModemConfig::new(ModulationType::FSK, 0, 0, 0, 225),
            ),
            (
                ModemConfigChoice::FSK_Rb200Fd200,
                ModemConfig::new(ModulationType::FSK, 0, 0, 0, 225),
            ),
        ]
        .iter()
        .cloned()
        .collect();

        let cfg = cfg_map.get(preset);
        match cfg {
            Some(cfg) => {
                // Now update the registers
                let val = self.read_reg(reg::Register::DMODREG)?;
                let cr = reg::Dmodreg::from_bytes([val]).with_modsel(cfg.modsel.clone());
                self.write_reg(reg::Register::DMODREG, cr.into_bytes()[0])?;

                self.write_reg(reg::Register::FDEVREG, cfg.fdval)?;
                self.write_reg(reg::Register::BRSREG, cfg.brval)?;
                self.write_reg(reg::Register::FILCREG, cfg.filcreg)?;

                // The sample configs in MRF89XA.h all use TXIPOLFV = 0xf0 => 375kHz, which is too wide for most modulations
                let val = self.read_reg(reg::Register::TXCONREG)?;
                let txconreg = reg::Txconreg::from_bytes([val])
                    .with_txipolfv(reg::Txconreg::compute_txipolfv(cfg.cut_off_freq)?);
                self.write_reg(reg::Register::TXCONREG, txconreg.into_bytes()[0])?;
                Ok(())
            }
            None => Err(config::Mrf89Error::PresetNotFound),
        }
    }

    fn set_op_mode(&mut self, op_mode: reg::CMOD) -> Result<(), Mrf89Error> {
        // REVISIT: do we need to have time delays when switching between modes?
        let val = self.read_reg(reg::Register::GCONREG)?;
        let cr = reg::Gconreg::from_bytes([val]).with_cmod(op_mode);
        self.write_reg(reg::Register::GCONREG, cr.into_bytes()[0])
    }

    /// frequency in kHz
    fn set_frequency(
        &mut self,
        frequency: u32,
        mod_type: &reg::ModulationType,
        delay: &mut dyn DelayNs,
    ) -> Result<(), Mrf89Error> {
        let r: u8;
        let p: u8;
        let s: u8;
        let fbs = reg::FBS::from_frequency(frequency)?;
        match mod_type {
            reg::ModulationType::FSK => {
                // Based on frequency calcs done in MRF89XA.h
                //    let R: u8 = 100; // Recommended
                r = 119_u8; // Also recommended :-(
                let compare: u32 = (frequency * 8 * (r as u32 + 1)) / (9 * config::XTAL_FREQ);
                p = (((compare - 75) / 76) + 1) as u8;
                s = (compare - (75 * (p as u32 + 1))) as u8;
            }
            reg::ModulationType::OOK => return Err(Mrf89Error::NotImplemented), // TODO: implement
            _ => return Err(Mrf89Error::NotImplemented),
        };
        // Now set the new register values:
        let val = self.read_reg(reg::Register::GCONREG)?;
        let cr = reg::Gconreg::from_bytes([val]).with_fbs(fbs);
        self.write_reg(reg::Register::GCONREG, cr.into_bytes()[0])?;
        self.write_reg(reg::Register::R1CREG, r)?;
        self.write_reg(reg::Register::P1CREG, p)?;
        self.write_reg(reg::Register::S1CREG, s)?;

        self.verify_pll_lock(delay)
    }

    /// Verify PLL-lock per instructions in Note 1 section 3.12 "Initialization"
    /// Need to do this after changing frequency.
    fn verify_pll_lock(&mut self, delay: &mut dyn DelayNs) -> Result<(), Mrf89Error> {
        let val = self.read_reg(reg::Register::FTPRIREG)?;
        let ftpri = reg::Ftprireg::from_bytes([val]).with_lstspll(true); // clear PLL lock bit by writing true
        self.write_reg(reg::Register::FTPRIREG, ftpri.into_bytes()[0])?;
        self.set_op_mode(reg::CMOD::FS)?;
        const HAS_PLL_LOCK: u8 = 0b0000_0010;
        const MAX_TRIES: u8 = 10;
        let mut ftpri_reg = self.read_reg(reg::Register::FTPRIREG)?;
        // other solution: try for one second to see if PLL lock was achieved
        // not doing it here, to not drag in external device specific dependencies
        for _ in 0..MAX_TRIES {
            ftpri_reg = self.read_reg(reg::Register::FTPRIREG)?;
            if (ftpri_reg & HAS_PLL_LOCK) != 0 {
                break;
            }
            delay.delay_ms(100);
        }
        self.set_op_mode(reg::CMOD::STANDBY)?;
        match (ftpri_reg & HAS_PLL_LOCK) != 0 {
            true => Ok(()),
            false => Err(Mrf89Error::NoPllLock),
        }
    }

    /// Get the device ID
    pub fn get_device_id(&mut self) -> u8 {
        // TODO: implement
        /*
        let reg = reg::DEVID.addr();
        let mut output = [1u8];
        self.read_reg(reg, &mut output);
        output[0]
        */
        0
    }

    fn write_reg(&mut self, reg: Register, value: u8) -> Result<(), Mrf89Error> {
        let reg = reg.addr();
        let bytes = [((reg & 0x1f) << 1), value];
        self.spi
            .write(&bytes)
            .map_err(|_| Mrf89Error::SpiTransferError)
    }

    fn read_reg(&mut self, reg: Register) -> Result<u8, Mrf89Error> {
        let reg = reg.addr();
        // send register with read mask, and a 0 after to read
        let mut bytes = [((reg & 0x1f) << 1) | SPI_READ, 0];
        self.spi
            .transfer_in_place(&mut bytes)
            .map_err(|_| Mrf89Error::SpiTransferError)?;

        Ok(bytes[1])
    }

    fn set_sync_words(&mut self, sync_words: &[u8]) -> Result<(), Mrf89Error> {
        let len = sync_words.len();
        if len > 4 {
            return Err(Mrf89Error::SyncWordTooLong);
        }
        // TODO: autoset syncwsz for length of syncwords bits
        /*
        uint8_t syncreg = spiReadRegister(RH_MRF89_REG_12_SYNCREG);
        syncreg = (syncreg & ~RH_MRF89_SYNCWSZ) | (((len - 1) << 3) & RH_MRF89_SYNCWSZ);
        spiWriteRegister(RH_MRF89_REG_12_SYNCREG, syncreg);
        */
        // TODO: make this a loop depending on sync_words len
        // TODO: verify order of bytes is correct for order of regs
        self.write_reg(reg::Register::SYNCV31REG, sync_words[0])?;
        self.write_reg(reg::Register::SYNCV23REG, sync_words[1])?;
        self.write_reg(reg::Register::SYNCV15REG, sync_words[2])?;
        self.write_reg(reg::Register::SYNCV07REG, sync_words[3])?;
        Ok(())
    }

    pub fn send(&mut self, data: &[u8], _delay: &mut dyn DelayNs) -> Result<(), Mrf89Error> {
        let len = data.len() as u8;
        if len > config::MRF89_MAX_MSG_LEN {
            return Err(Mrf89Error::MsgTooLong);
        }

        //self.wait_packet_sent(); // TODO: Make sure we dont interrupt an outgoing message
        if self.mode == OpMode::Tx {
            self.set_mode_idle()?;
            return Err(Mrf89Error::ChannelActive);
        }
        self.set_mode_idle()?;

        if !self.wait_cad() {
            return Err(Mrf89Error::ChannelActive); // Check channel activity
        }

        // First octet is the length of the chip payload
        // 0 length messages are transmitted but never trigger a receive!
        self.write_data(&[
            len + config::MRF89_HEADER_LEN,
            self.tx_header.to,
            self.tx_header.from,
            self.tx_header.id,
            self.tx_header.flags,
        ])?;
        self.write_data(data)?;
        self.set_mode_tx()?; // Start transmitting

        Ok(())
    }

    pub fn recv(&mut self, buf: &mut heapless::Vec<u8, 64>) -> Result<(), Mrf89Error> {
        let mut had_err = false;
        if self.buf.len() < MRF89_HEADER_LEN as usize {
            return Err(Mrf89Error::MsgTooShort);
        } // write internal buffer, interrupt safe, to handed buffer without header-data
        critical_section::with(|_crit_sec| {
            if buf
                .extend_from_slice(&self.buf[(MRF89_HEADER_LEN as usize)..])
                .is_err()
            {
                had_err = true;
            }
        });
        if had_err {
            self.clear_rx_buf();
            return Err(Mrf89Error::BufWriteFailed);
        }
        self.clear_rx_buf();
        Ok(())
    }

    /// checks if there is a message available
    /// not available when in Tx mode of course
    /// not available when the rx-buf is not yet valid
    pub fn available(&mut self) -> Result<bool, Mrf89Error> {
        if self.mode == OpMode::Tx {
            return Ok(false);
        }
        if self.mode == OpMode::Rx {
            self.set_mode_idle()?;
            return Ok(false);
        }
        self.set_mode_rx()?;
        // TODO: wait for CRCOK interrupt
        Ok(true)
    }

    /// spiWriteData
    fn write_data(&mut self, data: &[u8]) -> Result<(), Mrf89Error> {
        // TODO: implement proper FCRCREG
        let acfcrc = 0x80; // Autoclear FIfO crc bit
        self.write_reg(reg::Register::FCRCREG, acfcrc)?;

        let mut result = Err(Mrf89Error::SpiTransferError);
        critical_section::with(|_crit_sec| {
            self.cs_data.set_low().ok();
            result = match self.spi.write(data) {
                Ok(_) => Ok(()),
                Err(_) => Err(Mrf89Error::SpiTransferError),
            };
            self.cs_data.set_high().ok();
        });
        result
    }

    /// spiReadData
    fn read_data(&mut self) -> Result<u8, Mrf89Error> {
        // TODO: implement proper FCRCREG
        let acfcrc = 0x80u8; // Autoclear FIfO crc bit
        let frwaxs = 0x40u8; // ?
        self.write_reg(reg::Register::FCRCREG, acfcrc | frwaxs)?; // read from FIFO

        let mut bytes = [0u8];
        let mut result = Err(Mrf89Error::SpiTransferError);
        critical_section::with(|_crit_sec| {
            self.cs_data.set_low().ok();
            result = match self.spi.transfer_in_place(&mut bytes) {
                Ok(_) => Ok(bytes[0]),
                Err(_) => Err(Mrf89Error::SpiTransferError),
            };
            self.cs_data.set_high().ok();
        });
        result
    }

    /// Wait until no channel activity detected or timeout
    fn wait_cad(&self) -> bool {
        // TODO: implement
        true
    }

    /// Wait until available or timeout
    pub fn wait_available_timeout(&mut self, timeout: u16) -> Result<bool, Mrf89Error> {
        self.set_mode_rx()?;
        if timeout == 0 {
            return Ok(false);
        }
        // TODO: implement waiting
        Ok(true)
    }

    fn set_mode_idle(&mut self) -> Result<(), Mrf89Error> {
        info!("enable idle mode");
        if self.mode != OpMode::Idle {
            self.set_op_mode(reg::CMOD::STANDBY)?;
            self.mode = OpMode::Idle;
        }
        Ok(())
    }

    fn set_mode_tx(&mut self) -> Result<(), Mrf89Error> {
        info!("enable tx mode");
        if self.mode != OpMode::Tx {
            self.set_op_mode(reg::CMOD::TRANSMIT)?;
            self.mode = OpMode::Tx;
        }
        Ok(())
    }

    fn set_mode_rx(&mut self) -> Result<(), Mrf89Error> {
        info!("enable rx mode");
        if self.mode != OpMode::Rx {
            self.set_op_mode(reg::CMOD::RECEIVE)?;
            self.mode = OpMode::Rx;
        }
        Ok(())
    }

    #[allow(dead_code)]
    fn set_mode_sleep(&mut self) -> Result<(), Mrf89Error> {
        if self.mode != OpMode::Sleep {
            self.set_op_mode(reg::CMOD::SLEEP)?;
            self.mode = OpMode::Sleep;
        }
        Ok(())
    }

    #[allow(dead_code)]
    fn set_mode_cad(&mut self) {
        if self.mode != OpMode::Cad {
            // TODO Cad is some driver state not a hardware feature
            // self.set_op_mode(reg::CMOD::FS);
            self.mode = OpMode::Cad;
        }
    }
    /// "interrupt" handler for this instance
    /// MRF89XA is unusual in that it has 2 interrupt lines, and not a single, combined one.
    /// Only one of the several interrupt lines (IRQ1) from the RFM95 needs to be
    /// connnected to the processor.
    /// We use this to get CRCOK and TXDONE  interrupts
    pub fn handle_interrupt(&mut self, event: Event) -> Result<(), Mrf89Error> {
        match event {
            Event::Interrupt => {
                info!("some irq cam in");
            }
            Event::IRQ0 => {
                info!("IRQ0 came in");
                match self.mode {
                    OpMode::Tx => {
                        // means FIFO_THRESHOLD
                        info!("means FIFO_THRESHOLD");
                    }
                    OpMode::Rx => {
                        // means SYNC/ADDRESS MATCH
                        info!("means SYNC/ADDRESS MATCH");
                    }
                    _ => {
                        error!("opmode unexpected");
                    }
                }
            }
            Event::IRQ1 => {
                match self.mode {
                    OpMode::Tx => {
                        info!("opmode TX TXDONE");
                        // IRQ1 when transmitting means TXDONE
                        self.tx_good += 1;
                        self.set_mode_idle()?;
                        // TODO: weird to go to RX mode right after sending??
                        //       definitely not power-efficient
                        // self.set_mode_rx();
                    }
                    OpMode::Rx => {
                        info!("opmode RX CRCOK");
                        // IRQ1 when receiving means CRCOK
                        // We have received a packet.
                        // First byte in FIFO is packet length

                        // Capture last rssi from RSTSREG
                        // based roughly on Figure 3-9
                        // TODO: make RSSI accessible in API
                        // 4. Go to Standby mode
                        self.set_mode_idle()?;
                        self.last_rssi = self.read_reg(Register::RSTSREG)?;

                        self.rx_good += 1;
                        let buf_len = self.read_data()?;

                        if buf_len == 0 {
                            return Err(Mrf89Error::BufLenZero);
                        }

                        info!("buflen {} ", buf_len);
                        // 5. Now drain all the data from the FIFO into self.buf
                        // TODO: interrupt free
                        loop {
                            let r = self.read_reg(Register::FTXRXIREG)?;
                            let rxi = reg::Ftxrxireg::from_bytes([r]);
                            if rxi.fifoempty() {
                                // weird but if bit is set it means not-empty
                                match self.read_data() {
                                    Ok(b) => {
                                        if self.buf.push(b).is_err() {
                                            return Err(Mrf89Error::BufWriteFailed);
                                        }
                                    }
                                    Err(_) => continue,
                                }
                            } else {
                                break;
                            }
                        }
                        info!("buf {:?} ", self.buf.as_slice());
                        info!("validate {}", self.validate_rx_buf());
                        // Drained the FIFO and stop processing, buf_len too short
                        if buf_len < config::MRF89_HEADER_LEN {
                            self.clear_rx_buf();
                            return Err(Mrf89Error::MsgTooShort);
                        }
                        // Drained the FIFO but the first byte indicating message length is wrong
                        if buf_len > config::MRF89_HEADER_LEN + config::MRF89_MAX_MSG_LEN {
                            self.clear_rx_buf();
                            return Err(Mrf89Error::MsgTooLong);
                        }
                        // All good. See if its for us
                        if self.validate_rx_buf() {
                            self.set_mode_idle()?;
                        } else {
                            self.clear_rx_buf();
                            return Err(Mrf89Error::MsgNotForUs);
                        }
                    }
                    OpMode::Cad => {
                        info!("Opmode CAD");
                        self.rx_good += 1;
                        self.set_mode_idle()?;
                    }
                    OpMode::Initialising => {
                        info!("Opmode initialising");
                    }
                    OpMode::Sleep => {
                        info!("Opmode sleep");
                    }
                    OpMode::Idle => {
                        info!("Opmode idle");
                    }
                }
            }
            Event::Receive | Event::Transmit => { /* .. */ }
        }
        Ok(())
    }

    pub fn validate_rx_buf(&mut self) -> bool {
        if self.buf.len() < config::MRF89_HEADER_LEN.into() {
            return false; // Too short to be a real message
        }
        // Extract the 4 headers
        self.rx_header.to = self.buf[0];
        self.rx_header.from = self.buf[1];
        self.rx_header.id = self.buf[2];
        self.rx_header.flags = self.buf[3];
        if self.promiscuous
            || self.rx_header.to == self.this_address
            || self.rx_header.to == config::BROADCAST_ADDRESS
        {
            self.rx_good += 1;
            return true;
        }
        false
    }

    fn clear_rx_buf(&mut self) {
        self.buf.clear();
    }

    pub fn tx_good(&self) -> u16 {
        self.tx_good
    }

    pub fn rx_good(&self) -> u16 {
        self.rx_good
    }

    /// computes dBm from the last raw RSSI value.
    /// This is just an approximation, because the real formula according to the specs is
    /// dBm = 0.55 * raw_rssi - 118.5
    /// and floating point math is not strong on most embedded devices
    pub fn last_rssi_dbm(&self) -> i8 {
        (self.last_rssi >> 1) as i8 - 120
    }

    /// Last raw value from RSTSREG
    pub fn last_rssi_raw(&self) -> u8 {
        self.last_rssi
    }

    /// prints and compares registers with default from RadioHead on Arduino
    pub fn print_registers(&mut self) {
        for i in 0u8..32 {
            let rh_val = reg::RADIO_HEAD_REGISTERS[i as usize];
            let val = self.read_reg(reg::Register::from_u8(i));
            info!("{}: 0x{:x} 0x{:x}", i, val, rh_val);
        }
    }
}
