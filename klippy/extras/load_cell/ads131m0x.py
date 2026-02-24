# ADS131M0X Support (M02, M03, M04 variants)
#
# This file may be distributed under the terms of the GNU GPLv3 license.

#  Implementation assumptions and limitations:
#  - At least one channel must be enabled.
#  - The hardware offset counts are applied by ADC so the raw sample values are already compenstated.
#  - The output count is the sum of the absolute value of the raw counts across all channels.
#  - Disabled channels read as zero.
#  - The same gain setting is applied to all enabled channels.
#  - Only the high resolution power mode is supported with nominal external clock frequency of 8Mhz 
#    (provided by configured clock pin or other external hardware clock generator).
#  - The SPI communication word size is fixed at 24 bits
#  - CRC error checking of the SPI data from MCU to ADC is not implemented.  Once ADC is setup only
#    NULL data frames are sent to the ADC.  TODO is this true?


import logging

from klippy import ConfigWrapper, Printer
from klippy.extras import bus
from klippy.extras.bulk_sensor import BatchBulkHelper, FixedFreqReader
from klippy.extras.load_cell.interfaces import LoadCellSensor
from klippy.pins import PrinterPins
from klippy.reactor import Reactor

# Constants
WORD_SIZE = 3  # there are 3 bytes in a word in the protocol
# TODO: delete me BYTES_PER_SAMPLE = 4
UPDATE_INTERVAL = 0.10
NULL_CMD = 0x0000
RESET_CMD = 0x0011
RESET_REPLY_OKAY_PREFIX = 0xFF20
# TODO: delete me WAKEUP_CMD = 0x0033
RREG_CMD = 0b101 << 13
WREG_CMD = 0b011 << 13
WREG_REPLY = 0b010 << 13
STATUS_REG = 0x01
MODE_REG = 0x02
MODE_INIT = 0x0110  # default after reset: WLENGTH[8:9]=01, TIMEOUT[4]=1, others 0
CLOCK_REG = 0x03
GAIN_REG = 0x04
PWR_HR = 0b10  # High resolution mode
STATUS_RESET_BIT = 1 << 10  # RESET bit in STATUS register
# Error codes from MCU (match src/sensor_ads131m0x.c)
SAMPLE_ERROR_CRC = -0x80000000  # 1 << 31 as signed
SAMPLE_ERROR_RESET = 0x40000000  # 1 << 30


def hexify(byte_array):
    return "[%s]" % (", ".join([hex(b) for b in byte_array]))


# Chip Documentation: https://www.ti.com/lit/ds/symlink/ads131m02.pdf
# Chip Documentation: https://www.ti.com/lit/ds/symlink/ads131m03.pdf
# Chip Documentation: https://www.ti.com/lit/ds/symlink/ads131m04.pdf

# Variant specifications: chip model name -> total number of channels
VARIANTS = {
    "ads131m02": {"name": "ADS131M02", "channels": 2, "description": "2-channel ADC"},
    "ads131m03": {"name": "ADS131M03", "channels": 3, "description": "3-channel ADC"},
    "ads131m04": {"name": "ADS131M04", "channels": 4, "description": "4-channel ADC"},
}


class ADS131M0X(LoadCellSensor):
    def __init__(self, config: ConfigWrapper):
        self.printer: Printer = config.get_printer()
        self.reactor: Reactor = self.printer.get_reactor()
        self.config_name = config.get_name()
        self.name = config.get_name().split()[-1]
        self.last_error_count = 0
        self.consecutive_fails = 0
        
        # Determine variant from sensor_type config parameter
        self.variant_key = config.getchoice("sensor_type", list(VARIANTS.keys()))
        self.variant = VARIANTS[self.variant_key]
        self.reset_okay_reply = RESET_REPLY_OKAY_PREFIX + self.variant["channels"]
        self.words_in_frame = self.variant["channels"] + 2  #status/ch0data/../chNdata/CRC
        
        self.total_ch = self.variant["channels"]
        
        self.enabled_channels = []
        for ch in range(self.total_ch):
            key = "enable_ch%d" % (ch,)
            if config.getboolean(key, ch == 0):
                self.enabled_channels.append(ch)
                
        if not self.enabled_channels:
            raise config.error(
                "%s %s: at least one channel must be enabled (enable_ch0 through "
                "enable_ch%d)" % (self.variant["name"], self.name, self.total_ch - 1)
            )
        
        self.offset_counts = {}
        for ch in range(self.total_ch):
            key = "offset_counts_ch%d" % (ch,)
            self.offset_counts[ch] = config.getint(
                key, 0, minval=-0x800000, maxval=0x7FFFFF  #limit to 24-bit signed range
            )
            
        # Select the sample rate
        # The sample rate is set by the Oversampling Ratio (OSR) and the
        # power mode. Since we want the best accuracy for this application (not
        # battery powered or intermittent) we will always be in high
        # power mode. So the sample rate really sets the OSR value.
        self.sample_rate_options = {
            "250": 250,
            "500": 500,
            "1000": 1000,
            "2000": 2000,
            "4000": 4000,
            "8000": 8000,
            "16000": 16000,
            "32000": 32000,
        }
        self.sps = config.getchoice(
            "sample_rate", self.sample_rate_options, default="500"
        )
        # PGA Gain
        self.gain_options = {
            "1": 0,
            "2": 1,
            "4": 2,
            "8": 3,
            "16": 4,
            "32": 5,
            "64": 6,
            "128": 7,
        }
        self.gain = config.getchoice("gain", self.gain_options, default="128")
        
        # SPI Setup
        # Default speed: 8.192 MHz
        self.spi = bus.MCU_SPI_from_config(config, 1, default_speed=8192000)
        self.mcu = mcu = self.spi.get_mcu()
        self.oid = mcu.create_oid()
        # Data Ready (DRDY) Pin
        drdy_pin: str = config.get("data_ready_pin")
        ppins: PrinterPins = self.printer.lookup_object("pins")
        drdy_ppin = ppins.lookup_pin(drdy_pin)
        self.data_ready_pin = drdy_ppin["pin"]
        drdy_pin_mcu = drdy_ppin["chip"]
        if drdy_pin_mcu != self.mcu:
            raise config.error(
                "%s config error: SPI communication and"
                " data_ready_pin must be on the same MCU"
                % (self.variant["name"],)
            )

        clock_pin_name: str = config.get("clock_pin", default=None)
        self.clock_pin = None
        self.clock_pin_mcu = None
        self.clock_pin_invert = False
        self.clock_frequency = 8000000.0
        if clock_pin_name is not None:
            clock_pin_params = ppins.lookup_pin(clock_pin_name, can_invert=True)
            self.clock_pin = clock_pin_params["pin"]
            self.clock_pin_mcu = clock_pin_params["chip"]
            self.clock_pin_invert = bool(clock_pin_params["invert"])
            self.clock_pin_mcu.register_config_callback(self._build_clock_output)

        # Bulk Sensor Setup
        chip_smooth = self.sps * UPDATE_INTERVAL * 2
        self.ffreader: FixedFreqReader = FixedFreqReader(mcu, chip_smooth, "<i")
        self.batch_bulk: BatchBulkHelper = BatchBulkHelper(
            self.printer,
            self._process_batch,
            self._start_measurements,
            self._finish_measurements,
            UPDATE_INTERVAL,
        )

        # Command Configuration
        # Firmware reports one value per sample that is the sum of absolute
        # counts across all configured channels on this ADC instance.
        mcu.add_config_cmd(
            "config_ads131m0x oid=%d spi_oid=%d data_ready_pin=%s total_ch=%d"
            % (self.oid, self.spi.get_oid(), self.data_ready_pin, self.total_ch)
        )
        mcu.add_config_cmd(
            "query_ads131m0x oid=%d rest_ticks=0" % (self.oid,), on_restart=True
        )
        mcu.register_config_callback(self._build_config)
        self.attach_probe_cmd = None
        self.query_ads131m0x_cmd = None
        self._streaming_active = False
        self._register_commands(config)

    def _register_commands(self, config: ConfigWrapper):
        gcode = self.printer.lookup_object("gcode")
        name_parts = config.get_name().split()
        mux_name = name_parts[-1]
        gcode.register_mux_command(
            "LOAD_CELL_MEASURE_OFFSETS",
            "LOAD_CELL",
            mux_name,
            self.cmd_LOAD_CELL_MEASURE_OFFSETS,
            desc=self.cmd_LOAD_CELL_MEASURE_OFFSETS_help,
        )
        if len(name_parts) == 1:
            gcode.register_mux_command(
                "LOAD_CELL_MEASURE_OFFSETS",
                "LOAD_CELL",
                None,
                self.cmd_LOAD_CELL_MEASURE_OFFSETS,
                desc=self.cmd_LOAD_CELL_MEASURE_OFFSETS_help,
            )

    def _build_config(self):
        cq = self.spi.get_command_queue()
        self.query_ads131m0x_cmd = self.mcu.lookup_command(
            "query_ads131m0x oid=%c rest_ticks=%u", cq=cq
        )
        self.attach_probe_cmd = self.mcu.lookup_command(
            "ads131m0x_attach_load_cell_probe oid=%c load_cell_probe_oid=%c"
        )
        self.ffreader.setup_query_command(
            "query_ads131m0x_status oid=%c", oid=self.oid, cq=cq
        )

    def _build_clock_output(self):
        if self.clock_pin is None:
            return
        self.clock_pin_mcu.lookup_command(
            "stm32_timer_output pin=%u cycle_ticks=%u on_ticks=%hu"
        )
        mcu_freq = self.clock_pin_mcu.seconds_to_clock(1.0)
        cycle_ticks = int(mcu_freq // self.clock_frequency)
        mcu_freq_rev = int(cycle_ticks * self.clock_frequency)
        if mcu_freq_rev != mcu_freq:
            raise self.printer.config_error(
                "%s %s: clock_pin frequency must divide MCU clock exactly "
                "(%i != %i)" % (self.variant["name"], self.name, mcu_freq, mcu_freq_rev)
            )
        on_ticks = int(0.5 * cycle_ticks)
        if self.clock_pin_invert:
            on_ticks = cycle_ticks - on_ticks
        self.clock_pin_mcu.add_config_cmd(
            "stm32_timer_output pin=%s cycle_ticks=%d on_ticks=%d"
            % (self.clock_pin, cycle_ticks, on_ticks)
        )

    cmd_LOAD_CELL_MEASURE_OFFSETS_help = (
        "Measure per-channel ADS131M0X offset_counts_ch* values"
    )

    def cmd_LOAD_CELL_MEASURE_OFFSETS(self, gcmd):
        load_cell = self.printer.lookup_object(self.config_name, None)
        if load_cell is None:
            raise self.printer.command_error(
                "%s %s: load_cell object '%s' not found"
                % (self.variant["name"], self.name, self.config_name)
            )
        num_samples = gcmd.get_int(
            "SAMPLES", self.get_samples_per_second() * 10, minval=10, maxval=200000
        )
        confirm_samples = self.get_samples_per_second()
        original_enabled = list(self.enabled_channels)
        original_offsets = dict(self.offset_counts)
        measured_offsets = dict(self.offset_counts)
        gcmd.respond_info(
            "%s %s: measuring offsets on enabled channels %s with %d samples/channel"
            " and %d-sample sign confirmation"
            % (
                self.variant["name"],
                self.name,
                original_enabled,
                num_samples,
                confirm_samples,
            )
        )
        was_streaming = self._suspend_streaming()
        try:
            for ch in original_enabled:
                self.enabled_channels = [ch]

                # Base magnitude measurement with zero offset.
                self.offset_counts[ch] = 0
                self._suspend_streaming()
                self.setup_chip()
                self._resume_streaming()
                counts = int(round(load_cell.avg_counts(num_samples=num_samples)))

                # Confirm sign using short 1-second measurements.
                self.offset_counts[ch] = counts
                self._suspend_streaming()
                self.setup_chip()
                self._resume_streaming()
                plus_residual = int(
                    round(load_cell.avg_counts(num_samples=confirm_samples))
                )

                self.offset_counts[ch] = -counts
                self._suspend_streaming()
                self.setup_chip()
                self._resume_streaming()
                minus_residual = int(
                    round(load_cell.avg_counts(num_samples=confirm_samples))
                )

                if plus_residual <= minus_residual:
                    chosen_offset = counts
                    chosen_residual = plus_residual
                else:
                    chosen_offset = -counts
                    chosen_residual = minus_residual

                measured_offsets[ch] = chosen_offset
                self.offset_counts[ch] = chosen_offset
                gcmd.respond_info(
                    "%s %s: ch%d measured=%d residual(+)= %d residual(-)= %d"
                    " selected offset=%d residual=%d"
                    % (
                        self.variant["name"],
                        self.name,
                        ch,
                        counts,
                        plus_residual,
                        minus_residual,
                        chosen_offset,
                        chosen_residual,
                    )
                )
        finally:
            self.enabled_channels = original_enabled
            for ch in range(self.total_ch):
                self.offset_counts[ch] = measured_offsets.get(
                    ch, original_offsets.get(ch, 0)
                )
            self._suspend_streaming()
            self.reset_chip()
            self.setup_chip()
            if was_streaming:
                self._resume_streaming()
        configfile = self.printer.lookup_object("configfile")
        for ch in original_enabled:
            configfile.set(
                self.config_name,
                "offset_counts_ch%d" % (ch,),
                "%d" % (self.offset_counts[ch],),
            )
        gcmd.respond_info(
            "%s %s: updated offsets staged in config: %s\n"
            "Run SAVE_CONFIG to persist and restart."
            % (self.variant["name"], self.name, self.offset_counts)
        )

    def get_mcu(self):
        return self.mcu

    def get_samples_per_second(self):
        return self.sps

    #TODO, summing counts exceed 24-bit
    def get_range(self):
        # 24-bit signed range
        return -0x800000, 0x7FFFFF

    def add_client(self, callback):
        self.batch_bulk.add_client(callback)

    def attach_load_cell_probe(self, load_cell_probe_oid):
        self.attach_probe_cmd.send([self.oid, load_cell_probe_oid])

    # Measurement decoding
    def _convert_samples(self, samples):
        adc_factor = 1.0 / (1 << 23)
        count = 0
        for ptime, val in samples:
            if val == SAMPLE_ERROR_CRC or val == SAMPLE_ERROR_RESET:
                self.last_error_count += 1
                logging.info("%s Sample Error: %s" % (self.variant["name"], val))
                break  # additional errors are duplicates
            samples[count] = (round(ptime, 6), val, round(val * adc_factor, 9))
            count += 1
        del samples[count:]

    # Start, stop, and process message batches
    def _suspend_streaming(self):
        if not self._streaming_active or self.query_ads131m0x_cmd is None:
            return False
        self.query_ads131m0x_cmd.send_wait_ack([self.oid, 0])
        self.ffreader.note_end()
        self.ffreader.pull_samples()
        self._streaming_active = False
        return True

    def _resume_streaming(self):
        if self._streaming_active or self.query_ads131m0x_cmd is None:
            return
        rest_ticks = self.mcu.seconds_to_clock(1.0 / (10.0 * self.sps))
        self.query_ads131m0x_cmd.send([self.oid, rest_ticks])
        self.ffreader.note_start()
        self._streaming_active = True

    def _start_measurements(self):
        self.last_error_count = 0
        self.consecutive_fails = 0
        self.reset_chip()
        self.setup_chip()
        self._resume_streaming()
        logging.info("%s starting '%s' measurements", self.variant["name"], self.name)

    def _finish_measurements(self):
        if self.printer.is_shutdown():
            return
        self._suspend_streaming()
        logging.info("%s finished '%s' measurements", self.variant["name"], self.name)

    def _process_batch(self, eventtime):
        samples = self.ffreader.pull_samples()
        logging.info("%s received %d samples", self.variant["name"], len(samples))
        self._convert_samples(samples)
        return {
            "data": samples,
            "errors": self.last_error_count,
            "overflows": self.ffreader.get_last_overflows(),
        }

    # --- SPI Communication Helpers ---
    # The ADS131M02 uses 24-bit SPI words. Commands and register data are
    # 16-bit values, MSB-aligned with zero padding: [MSB, LSB, 0x00]
    @staticmethod
    def _to_words(*values_16bit):
        """Convert 16-bit values to 24-bit words as a byte array."""
        payload = []
        for val in values_16bit:
            payload.extend([(val >> 8) & 0xFF, val & 0xFF, 0x00])
        return payload

    def _send_command(self, cmd_16bit):
        """Send a command and return the response"""
        self._transfer_frame(cmd_16bit)
        resp = self._transfer_frame(NULL_CMD)
        logging.info("%s Command Response: %s", self.variant["name"], hexify(resp))
        return resp[0] if resp else None
    
    def _transfer_frame(self, *values_16bit):
        """Send frame and return response as list of 16-bit values."""
        # Send frame with values_16bit and pad to full frame size with NULL_CMD. 
        frame_size = self.words_in_frame * WORD_SIZE
        transfer_words = self._to_words(*values_16bit)
        # pad to target size
        while len(transfer_words) < frame_size:
            transfer_words.extend(self._to_words(NULL_CMD))
        logging.info("%s frame out: %s", self.variant["name"], hexify(transfer_words))
        params = self.spi.spi_transfer(transfer_words)
        resp = params.get("response", [])
        logging.info("%s frame in: %s", self.variant["name"], hexify(resp))
        result = []
        for i in range(0, len(resp), 3):
            if i + 1 < len(resp):
                result.append((resp[i] << 8) | resp[i + 1])
        logging.info(
            "%s calculated transfer response: %s", self.variant["name"], hexify(result)
        )
        return result

    # --- Command Helpers ---
    # WREG: 011a_aaaa_annn_nnnn (a=6-bit address, n=7-bit count-1)
    # RREG: 101a_aaaa_annn_nnnn

    def _build_register_command(self, cmd, addr, count=1):
        """build a 16bit command with address and count"""
        return cmd | ((addr & 0x3F) << 7) | ((count - 1) & 0x7F)

    def _wreg_cmd(self, addr, count=1):
        return self._build_register_command(WREG_CMD, addr, count)

    def _rreg_cmd(self, addr, count=1):
        return self._build_register_command(RREG_CMD, addr, count)

    def _read_reg(self, addr):
        """Read a single register, returns 16-bit value."""
        # Send read request with target register address, then send NULL command to get response
        self._transfer_frame(self._rreg_cmd(addr))
        resp = self._transfer_frame(NULL_CMD)
        logging.info("Read Reg Response: " + hexify(resp))
        return resp[0] if resp else None

    def _write_reg(self, addr, value):
        """Write a single register, check response for success."""
        # Send write request with target register address and value, then send NULL command to get response
        self._transfer_frame(self._wreg_cmd(addr), value)
        resp = self._transfer_frame(NULL_CMD)  
        logging.info("Write Reg Response: " + hexify(resp))
        #this should be 010a_aaaa_ann_nnnn, gettting 0x4, 0x5, 0xf,
        # if not resp or resp[0] != self._build_register_command(WREG_REPLY, addr):
        #     raise self.printer.command_error(
        #         "%s %s: write reg 0x%02x failed: expected 0x%04x, got %s"
        #         % (self.variant["name"], self.name, addr, self._rreg_cmd(addr), resp[0] if resp else None)
        #     )

    def _write_and_verify_reg(self, addr, value):
        """Write a register and verify the value was set."""
        self._write_reg(addr, value)
        actual = self._read_reg(addr)
        if actual != value:
            raise self.printer.command_error(
                "%s %s: reg 0x%02x write failed: "
                "wrote 0x%04x, read 0x%04x" % (self.variant["name"], self.name, addr, value, actual)
            )

    def reset_chip(self):
        self._transfer_frame(RESET_CMD)
        # Wait for reset to complete (~2ms per datasheet, use 20ms to be safe)
        self.reactor.pause(self.reactor.monotonic() + 0.020)
        # Send NULL command to get reset response
        resp = self._transfer_frame(NULL_CMD)
        if resp is None or not resp or resp[0] != self.reset_okay_reply:
            raise self.printer.command_error(
            "%s %s: reset failed: expected 0x%04x, got %s"
            % (self.variant["name"], self.name, self.reset_okay_reply, resp[0] if resp else None)
            )
        
    def setup_chip(self):
        # Write default values to MODE register to clear reset bit
        self._write_reg(MODE_REG, MODE_INIT)
        mode_val = self._read_reg(MODE_REG)
        if mode_val != MODE_INIT:
            raise self.printer.command_error(
                "%s %s: MODE reg write failed: got 0x%04x"
                % (self.variant["name"], self.name, mode_val)
            )

        logging.info("%s ...clock", self.variant["name"])

        # CLOCK register (0x03): set OSR for sample rate, high resolution mode
        # Channel_en[11:8]
        # OSR[4:2] 
        # PWR[1:0] = 10 for high resolution mode

        channel_bitmask = 0
        for ch in self.enabled_channels:
            channel_bitmask |= 1 << (8 + ch)
        
        # SPS -> OSR code: OSR=fMOD/fDATA, fMOD=fCLKIN/2=4.096MHz
        osr_codes = {
            32000: 0,  # OSR=128
            16000: 1,  # OSR=256
            8000: 2,  # OSR=512
            4000: 3,  # OSR=1024
            2000: 4,  # OSR=2048
            1000: 5,  # OSR=4096
            500: 6,  # OSR=8192
            250: 7,  # OSR=16384
        }
        osr_code = osr_codes.get(self.sps)
        clock_val = channel_bitmask | (osr_code << 2) | PWR_HR
        self._write_and_verify_reg(CLOCK_REG, clock_val)
       
        logging.info("%s ...clock value=0x%04x", self.variant["name"], clock_val)
        
        # GAIN register (0x04): PGAGAIN0[2:0] at bits 2:0,  PGAGAIN1[2:0] at bits 6:4,
        #                       PGAGAIN2[2:0] at bits 10:8, PGAGAIN3[2:0] at bits 14:12
        # Apply same gain to all enabled channels
        gain_val = 0
        for ch in self.enabled_channels:
            gain_shift = ch * 4  # Each channel's gain field is 4 bits apart
            gain_val |= self.gain << gain_shift
        self._write_and_verify_reg(GAIN_REG, gain_val)
        logging.info("%s ...gain value=0x%04x", self.variant["name"], gain_val)
        
        # Write offsets to registers, Ch0 0x0A MSB, 0x0B LSB; Ch1 0x0F MSB, 0x10 LSB, 
        #                             Ch2 0x14 MSB, 0x15 LSB; Ch3 0x19 MSB, 0x1A LSB
        for ch in range(self.total_ch):
            if ch in self.enabled_channels:
                offset = self.offset_counts.get(ch, 0)
            else:
                offset = 0
            if offset < 0:
                offset = (1 << 24) + offset  # convert to unsigned
            msb = (offset >> 8) & 0xFFFF
            lsb = (offset << 8) & 0xFF00
            logging.info("%s ...offset ch%d=0x%04x (msb=0x%02x, lsb=0x%02x)", self.variant["name"], ch, offset, msb, lsb)
            if ch == 0:
                self._write_and_verify_reg(0x0A, msb)
                self._write_and_verify_reg(0x0B, lsb)
            elif ch == 1:
                self._write_and_verify_reg(0x0F, msb)
                self._write_and_verify_reg(0x10, lsb)
            elif ch == 2:
                self._write_and_verify_reg(0x14, msb)
                self._write_and_verify_reg(0x15, lsb)
            elif ch == 3:
                self._write_and_verify_reg(0x19, msb)
                self._write_and_verify_reg(0x1A, lsb)
            logging.info("%s ...offset ch%d=0x%04x", self.variant["name"], ch, offset)

ADS131M0X_SENSOR_TYPES = {
    "ads131m02": ADS131M0X,
    "ads131m03": ADS131M0X,
    "ads131m04": ADS131M0X,
}
