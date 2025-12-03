# ADS131M02 Support
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

from klippy import ConfigWrapper, Printer
from klippy.extras import bus
from klippy.extras.bulk_sensor import BulkDataQueue, FixedFreqReader, \
    BatchBulkHelper
from klippy.pins import PrinterPins

# Constants
BYTES_PER_SAMPLE = 4
UPDATE_INTERVAL = 0.10
RESET_CMD = 0x0011
WAKEUP_CMD = 0x0033
STATUS_REG = 0x01
MODE_REG = 0x02
CLOCK_REG = 0x03
GAIN_REG = 0x04
PWR_HR = 0b10  # High resolution mode
STATUS_RESET_BIT = 1 << 10  # RESET bit in STATUS register
# Error codes from MCU (match sensor_ads131m02.c)
SAMPLE_ERROR_CRC = -0x80000000  # 1 << 31 as signed
SAMPLE_ERROR_RESET = 0x40000000  # 1 << 30

# Chip Documentation: https://www.ti.com/lit/ds/symlink/ads131m02.pdf

class ADS131M02:
    def __init__(self, config: ConfigWrapper):
        self.printer: Printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.last_error_count = 0
        self.consecutive_fails = 0
        # Config
        # Channel selection: 0 or 1 (default CH1 differential bridge)
        self.channel = config.getint('channel', 1, minval=0, maxval=1)
        # Select the sample rate
        # The sample rate is set by the Oversampling Ratio (OSR) and the
        # power mode. Since we want the best accuracy for this application (not
        # battery powered or intermittent) we will always be in high
        # power mode. So the sample rate really sets the OSR value.
        self.sample_rate_options = {
            '250': 250, '500': 500, '1000': 1000, '2000': 2000, '4000': 4000,
            '8000': 8000, '16000': 16000, '32000': 32000
        }
        self.sps = config.getchoice(
            "sample_rate", self.sample_rate_options, default='500'
        )
        # PGA Gain
        self.gain_options = {'1': 0, '2': 1, '4': 2, '8': 3, '16': 4, '32': 5, '64': 6, '128': 7}
        self.gain = config.getchoice('gain', self.gain_options, default='128')
        # SPI Setup
        # Default speed: 8.192 MHz
        self.spi = bus.MCU_SPI_from_config(config, 1, default_speed=8192000)
        self.mcu = mcu = self.spi.get_mcu()
        self.oid = mcu.create_oid()
        # Data Ready (DRDY) Pin
        drdy_pin: str = config.get('data_ready_pin')
        ppins: PrinterPins = self.printer.lookup_object('pins')
        drdy_ppin = ppins.lookup_pin(drdy_pin)
        self.data_ready_pin = drdy_ppin['pin']
        drdy_pin_mcu = drdy_ppin['chip']
        if drdy_pin_mcu != self.mcu:
            raise config.error("ADS131M02 config error: SPI communication and"
                               " data_ready_pin must be on the same MCU")

        # Bulk Sensor Setup
        chip_smooth = self.sps * UPDATE_INTERVAL * 2
        self.ffreader: FixedFreqReader = FixedFreqReader(mcu, chip_smooth, "<i")
        self.batch_bulk: BatchBulkHelper = BatchBulkHelper(
            self.printer, self._process_batch, self._start_measurements,
            self._finish_measurements, UPDATE_INTERVAL)

        # Command Configuration
        self.attach_probe_cmd = None
        mcu.add_config_cmd(
            "config_ads131m02 oid=%d spi_oid=%d data_ready_pin=%s channel=%d"
            % (self.oid, self.spi.get_oid(), self.data_ready_pin, self.channel))
        mcu.add_config_cmd("query_ads131m02 oid=%d rest_ticks=0"
                           % (self.oid,), on_restart=True)
        mcu.register_config_callback(self._build_config)
        self.query_ads131m02_cmd = None

    def _build_config(self):
        cq = self.spi.get_command_queue()
        self.query_ads131m02_cmd = self.mcu.lookup_command(
            "query_ads131m02 oid=%c rest_ticks=%u", cq=cq)
        self.attach_probe_cmd = self.mcu.lookup_command(
            "ads131m02_attach_load_cell_probe oid=%c load_cell_probe_oid=%c")
        self.ffreader.setup_query_command("query_ads131m02_status oid=%c",
                                          oid=self.oid, cq=cq)

    def get_mcu(self):
        return self.mcu

    def get_samples_per_second(self):
        return self.sps

    def get_range(self):
        # 24-bit signed range
        return -0x800000, 0x7FFFFF

    def add_client(self, callback):
        self.batch_bulk.add_client(callback)

    def attach_load_cell_probe(self, load_cell_probe_oid):
        self.attach_probe_cmd.send([self.oid, load_cell_probe_oid])

    # Measurement decoding
    def _convert_samples(self, samples):
        adc_factor = 1. / (1 << 23)
        count = 0
        for ptime, val in samples:
            if val == SAMPLE_ERROR_CRC or val == SAMPLE_ERROR_RESET:
                self.last_error_count += 1
                break  # additional errors are duplicates
            samples[count] = (round(ptime, 6), val, round(val * adc_factor, 9))
            count += 1
        del samples[count:]

    # Start, stop, and process message batches
    def _start_measurements(self):
        self.last_error_count = 0
        self.consecutive_fails = 0
        self.reset_chip()
        self.setup_chip()
        rest_ticks = self.mcu.seconds_to_clock(1. / (10. * self.sps))
        self.query_ads131m02_cmd.send([self.oid, rest_ticks])
        logging.info("ADS131M02 starting '%s' measurements", self.name)
        self.ffreader.note_start()

    def _finish_measurements(self):
        if self.printer.is_shutdown():
            return
        self.query_ads131m02_cmd.send_wait_ack([self.oid, 0])
        self.ffreader.note_end()
        logging.info("ADS131M02 finished '%s' measurements", self.name)

    def _process_batch(self, eventtime):
        samples = self.ffreader.pull_samples()
        self._convert_samples(samples)
        return {'data': samples, 'errors': self.last_error_count,
                'overflows': self.ffreader.get_last_overflows()}

    # --- SPI Communication Helpers ---
    # The ADS131M02 uses 24-bit SPI words. Commands and register data are
    # 16-bit values, MSB-aligned with zero padding: [MSB, LSB, 0x00]
    def _to_bytes(self, *values_16bit):
        """Convert 16-bit values to 24-bit words as a byte array."""
        payload = []
        for val in values_16bit:
            payload.extend([(val >> 8) & 0xFF, val & 0xFF, 0x00])
        return payload

    def _send_frame(self, *values_16bit):
        """Send a frame of 16-bit values as 24-bit words."""
        self.spi.spi_send(self._to_bytes(*values_16bit))

    def _transfer_frame(self, *values_16bit):
        """Send frame and return response as list of 16-bit values."""
        params = self.spi.spi_transfer(self._to_bytes(*values_16bit))
        resp = params.get('response', [])
        result = []
        for i in range(0, len(resp), 3):
            if i + 1 < len(resp):
                result.append((resp[i] << 8) | resp[i + 1])
        return result

    # --- Command Helpers ---
    # WREG: 011a_aaaa_nnnn_nnnn (a=address, n=count-1)
    # RREG: 101a_aaaa_nnnn_nnnn
    def _wreg_cmd(self, addr, count=1):
        return (0b011 << 13) | ((addr & 0x3F) << 7) | ((count - 1) & 0x7F)

    def _rreg_cmd(self, addr, count=1):
        return (0b101 << 13) | ((addr & 0x3F) << 7) | ((count - 1) & 0x7F)

    def _read_reg(self, addr):
        """Read a single register, returns 16-bit value."""
        resp = self._transfer_frame(self._rreg_cmd(addr), 0x0000, 0x0000, 0x0000)
        if len(resp) < 4:
            raise self.printer.command_error(
                "ADS131M02 %s: no response reading reg 0x%02x" % (self.name, addr))
        return resp[3]

    def _write_reg(self, addr, value):
        """Write a single register."""
        self._send_frame(self._wreg_cmd(addr), value, 0x0000, 0x0000)

    def _write_and_verify_reg(self, addr, value):
        """Write a register and verify the value was set."""
        self._write_reg(addr, value)
        actual = self._read_reg(addr)
        if actual != value:
            raise self.printer.command_error(
                "ADS131M02 %s: reg 0x%02x write failed: "
                "wrote 0x%04x, read 0x%04x" % (self.name, addr, value, actual))

    def reset_chip(self):
        self._send_frame(RESET_CMD, 0x0000, 0x0000, 0x0000)
        status = self._read_reg(STATUS_REG)
        if not (status & STATUS_RESET_BIT):
            raise self.printer.command_error(
                "ADS131M02 %s: reset failed, STATUS=0x%04x "
                "(expected RESET bit set). Check wiring and connections."
                % (self.name, status))

    def setup_chip(self):
        # MODE register (0x02): clear RESET bit (bit 10), set 24-bit word length (bits 9:8 = 01)
        WLENGTH_24 = 0b01 << 8
        CLEAR_RESET = 1 << 10  # Writing 1 clears the RESET status
        self._write_reg(MODE_REG, WLENGTH_24 | CLEAR_RESET)
        # Verify only WLENGTH bits (RESET bit auto-clears after write)
        mode_val = self._read_reg(MODE_REG)
        if (mode_val & 0x0300) != WLENGTH_24:
            raise self.printer.command_error(
                "ADS131M02 %s: MODE reg write failed: got 0x%04x"
                % (self.name, mode_val))
        # CLOCK register (0x03): set OSR for sample rate, high resolution mode
        # OSR[2:0] at bits 4:2, PWR[1:0] at bits 1:0
        # PWR=10 for high-resolution mode, CH0_EN and CH1_EN at bits 8:9
        channel_enable = 1 << 8 if self.channel == 0 else 1 << 9
        # SPS -> OSR code: OSR=fMOD/fDATA, fMOD=fCLKIN/2=4.096MHz
        osr_codes = {
            32000: 0,  # OSR=128
            16000: 1,  # OSR=256
            8000:  2,  # OSR=512
            4000:  3,  # OSR=1024
            2000:  4,  # OSR=2048
            1000:  5,  # OSR=4096
            500:   6,  # OSR=8192
            250:   7,  # OSR=16384
        }
        osr_code = osr_codes.get(self.sps)
        clock_val = channel_enable | (osr_code << 2) | PWR_HR
        self._write_and_verify_reg(CLOCK_REG, clock_val)
        # GAIN register (0x04): PGAGAIN0[2:0] at bits 2:0, PGAGAIN1[2:0] at bits 6:4
        gain_shift = 4 if self.channel == 1 else 0
        gain_val = self.gain << gain_shift
        self._write_and_verify_reg(GAIN_REG, gain_val)
        # WAKEUP command (0x0033) to start conversions
        self._send_frame(WAKEUP_CMD, 0x0000, 0x0000, 0x0000)


ADS131M02_SENSOR_TYPE = {"ads131m02": ADS131M02}
