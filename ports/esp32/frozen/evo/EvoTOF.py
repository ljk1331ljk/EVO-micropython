# EvoTOF.py
# VL53L0X Time-of-Flight distance sensor driver for Evo MicroPython.
# Ported to follow the working Pololu/ST-style C library sequence more closely.
#
# Main differences from the earlier version:
#   - checks model ID 0xEE during init
#   - implements measurement timing budget calculation and re-application
#   - performs VHV calibration with SYSTEM_SEQUENCE_CONFIG = 0x01
#   - performs phase calibration with SYSTEM_SEQUENCE_CONFIG = 0x02
#   - reads range directly from RESULT_RANGE_STATUS + 10, matching the C library
#   - timeoutOccurred() clears the timeout flag, matching the C library

from micropython import const
from I2CDevice import *
import time

# -----------------------------------------------------------------------------
# Registers
# -----------------------------------------------------------------------------
_SYSRANGE_START = const(0x00)
_SYSTEM_SEQUENCE_CONFIG = const(0x01)
_SYSTEM_INTERMEASUREMENT_PERIOD = const(0x04)
_SYSTEM_INTERRUPT_CONFIG_GPIO = const(0x0A)
_SYSTEM_INTERRUPT_CLEAR = const(0x0B)
_RESULT_INTERRUPT_STATUS = const(0x13)
_RESULT_RANGE_STATUS = const(0x14)

_GPIO_HV_MUX_ACTIVE_HIGH = const(0x84)
_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = const(0x89)
_MSRC_CONFIG_CONTROL = const(0x60)
_MSRC_CONFIG_TIMEOUT_MACROP = const(0x46)

_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = const(0x44)
_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = const(0x47)
_FINAL_RANGE_CONFIG_VALID_PHASE_LOW = const(0x48)
_FINAL_RANGE_CONFIG_VCSEL_PERIOD = const(0x70)
_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = const(0x71)

_PRE_RANGE_CONFIG_VCSEL_PERIOD = const(0x50)
_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = const(0x51)
_PRE_RANGE_CONFIG_VALID_PHASE_HIGH = const(0x56)
_PRE_RANGE_CONFIG_VALID_PHASE_LOW = const(0x57)

_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = const(0xB0)
_GLOBAL_CONFIG_REF_EN_START_SELECT = const(0xB6)
_GLOBAL_CONFIG_VCSEL_WIDTH = const(0x32)
_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = const(0x4E)
_DYNAMIC_SPAD_REF_EN_START_OFFSET = const(0x4F)

_ALGO_PHASECAL_LIM = const(0x30)
_ALGO_PHASECAL_CONFIG_TIMEOUT = const(0x30)
_OSC_CALIBRATE_VAL = const(0xF8)

_I2C_SLAVE_DEVICE_ADDRESS = const(0x8A)
_IDENTIFICATION_MODEL_ID = const(0xC0)
_IDENTIFICATION_REVISION_ID = const(0xC2)

_VCSEL_PRE_RANGE = const(0)
_VCSEL_FINAL_RANGE = const(1)

_ADDRESS_DEFAULT = const(0x29)


def _decode_vcsel_period(reg_val):
    return ((reg_val + 1) << 1)


def _encode_vcsel_period(period_pclks):
    return (period_pclks >> 1) - 1


def _calc_macro_period_ns(vcsel_period_pclks):
    # Same formula as C library: ((((uint32_t)2304 * period * 1655) + 500) / 1000)
    return ((2304 * vcsel_period_pclks * 1655) + 500) // 1000


class EvoTOF:
    def __init__(self, channel, i2c=I2CA, address=_ADDRESS_DEFAULT, io_timeout_ms=300, auto_init=True):
        self.i2c = i2c
        self.channel = channel
        self.address = address & 0x7F
        self.io_timeout_ms = io_timeout_ms

        self._stop_variable = 0
        self._did_timeout = False
        self._continuous = False
        self._initialized = False
        self._measurement_timing_budget_us = 0

        if auto_init:
            self.init()

    # -------------------------------------------------------------------------
    # Low-level I2C helpers
    # -------------------------------------------------------------------------
    def _select(self):
        # Keep this explicit because Evo sensors are normally behind I2C mux channels.
        self.i2c.switch_channel(self.channel)

    def _read8(self, reg):
        self._select()
        return self.i2c.readfrom_mem(self.channel, self.address, reg, 1)[0]

    def _write8(self, reg, value):
        self._select()
        self.i2c.writeto_mem(self.channel, self.address, reg, bytes((value & 0xFF,)))

    def _read16(self, reg):
        self._select()
        data = self.i2c.readfrom_mem(self.channel, self.address, reg, 2)
        return (data[0] << 8) | data[1]

    def _write16(self, reg, value):
        self._select()
        self.i2c.writeto_mem(
            self.channel,
            self.address,
            reg,
            bytes(((value >> 8) & 0xFF, value & 0xFF)),
        )

    def _read32(self, reg):
        self._select()
        data = self.i2c.readfrom_mem(self.channel, self.address, reg, 4)
        return (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]

    def _write32(self, reg, value):
        self._select()
        self.i2c.writeto_mem(
            self.channel,
            self.address,
            reg,
            bytes(((value >> 24) & 0xFF, (value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF)),
        )

    def _read_multi(self, reg, count):
        self._select()
        return self.i2c.readfrom_mem(self.channel, self.address, reg, count)

    def _write_multi(self, reg, data):
        self._select()
        self.i2c.writeto_mem(self.channel, self.address, reg, bytes(data))

    def _check_timeout(self, start):
        return self.io_timeout_ms and time.ticks_diff(time.ticks_ms(), start) > self.io_timeout_ms

    # -------------------------------------------------------------------------
    # Public API
    # -------------------------------------------------------------------------
    def setTimeout(self, timeout_ms):
        self.io_timeout_ms = timeout_ms

    def timeoutOccurred(self):
        tmp = self._did_timeout
        self._did_timeout = False
        return tmp

    def isInitialized(self):
        return self._initialized

    def getModelId(self):
        return self._read8(_IDENTIFICATION_MODEL_ID)

    def getRevisionId(self):
        return self._read8(_IDENTIFICATION_REVISION_ID)

    def setAddress(self, new_address):
        new_address &= 0x7F
        self._write8(_I2C_SLAVE_DEVICE_ADDRESS, new_address)
        self.address = new_address

    def setSignalRateLimit(self, limit_mcps):
        if limit_mcps < 0 or limit_mcps > 511.99:
            return False
        # Q9.7 fixed point format.
        self._write16(_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, int(limit_mcps * (1 << 7)))
        return True

    def getSignalRateLimit(self):
        return self._read16(_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / float(1 << 7)

    def init(self, io_2v8=True):
        self._did_timeout = False
        self._continuous = False
        self._initialized = False

        time.sleep_ms(10)

        # The working C library refuses to continue if the model ID is not 0xEE.
        if self._read8(_IDENTIFICATION_MODEL_ID) != 0xEE:
            return False

        if io_2v8:
            self._write8(_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
                         self._read8(_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01)

        self._write8(0x88, 0x00)  # Set I2C standard mode.

        self._write8(0x80, 0x01)
        self._write8(0xFF, 0x01)
        self._write8(0x00, 0x00)
        self._stop_variable = self._read8(0x91)
        self._write8(0x00, 0x01)
        self._write8(0xFF, 0x00)
        self._write8(0x80, 0x00)

        # Disable SIGNAL_RATE_MSRC and SIGNAL_RATE_PRE_RANGE checks.
        self._write8(_MSRC_CONFIG_CONTROL, self._read8(_MSRC_CONFIG_CONTROL) | 0x12)

        self.setSignalRateLimit(0.25)
        self._write8(_SYSTEM_SEQUENCE_CONFIG, 0xFF)

        spad_count, spad_is_aperture = self._get_spad_info()
        if spad_count is None:
            return False

        ref_spad_map = bytearray(self._read_multi(_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6))

        self._write8(0xFF, 0x01)
        self._write8(_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00)
        self._write8(_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C)
        self._write8(0xFF, 0x00)
        self._write8(_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4)

        first_spad_to_enable = 12 if spad_is_aperture else 0
        spads_enabled = 0

        for i in range(48):
            byte_index = i // 8
            bit_mask = 1 << (i % 8)
            if i < first_spad_to_enable or spads_enabled == spad_count:
                ref_spad_map[byte_index] &= ~bit_mask
            elif ref_spad_map[byte_index] & bit_mask:
                spads_enabled += 1

        self._write_multi(_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map)

        self._load_tuning_settings()

        self._write8(_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)
        self._write8(_GPIO_HV_MUX_ACTIVE_HIGH, self._read8(_GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10)
        self._write8(_SYSTEM_INTERRUPT_CLEAR, 0x01)

        self._measurement_timing_budget_us = self.getMeasurementTimingBudget()

        # Disable MSRC and TCC by default, matching the C library.
        self._write8(_SYSTEM_SEQUENCE_CONFIG, 0xE8)

        # Recalculate and re-apply the timing budget. This was missing before.
        if not self.setMeasurementTimingBudget(self._measurement_timing_budget_us):
            return False

        # VHV calibration must run with sequence config 0x01.
        self._write8(_SYSTEM_SEQUENCE_CONFIG, 0x01)
        if not self._perform_single_ref_calibration(0x40):
            return False

        # Phase calibration must run with sequence config 0x02.
        self._write8(_SYSTEM_SEQUENCE_CONFIG, 0x02)
        if not self._perform_single_ref_calibration(0x00):
            return False

        self._write8(_SYSTEM_SEQUENCE_CONFIG, 0xE8)

        self._initialized = True
        return True

    def readRangeSingleMillimeters(self):
        if not self._initialized:
            return 65535

        self._write8(0x80, 0x01)
        self._write8(0xFF, 0x01)
        self._write8(0x00, 0x00)
        self._write8(0x91, self._stop_variable)
        self._write8(0x00, 0x01)
        self._write8(0xFF, 0x00)
        self._write8(0x80, 0x00)

        self._write8(_SYSRANGE_START, 0x01)

        start = time.ticks_ms()
        while self._read8(_SYSRANGE_START) & 0x01:
            if self._check_timeout(start):
                self._did_timeout = True
                return 65535

        return self.readRangeContinuousMillimeters()

    def readRangeContinuousMillimeters(self):
        if not self._initialized:
            return 65535

        start = time.ticks_ms()
        while (self._read8(_RESULT_INTERRUPT_STATUS) & 0x07) == 0:
            if self._check_timeout(start):
                self._did_timeout = True
                return 65535

        # Same as C library: readReg16Bit(RESULT_RANGE_STATUS + 10)
        distance = self._read16(_RESULT_RANGE_STATUS + 10)
        self._write8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        self._did_timeout = False
        return distance

    def read_mm(self):
        value = self.readRangeSingleMillimeters()
        if value == 65535 or value == 8191 or value == 0:
            return None
        return value

    def getDistance(self):
        return self.read_mm()

    def distance(self):
        return self.read_mm()

    def startContinuous(self, period_ms=0):
        if not self._initialized:
            return False

        self._write8(0x80, 0x01)
        self._write8(0xFF, 0x01)
        self._write8(0x00, 0x00)
        self._write8(0x91, self._stop_variable)
        self._write8(0x00, 0x01)
        self._write8(0xFF, 0x00)
        self._write8(0x80, 0x00)

        if period_ms != 0:
            osc_calibrate_val = self._read16(_OSC_CALIBRATE_VAL)
            if osc_calibrate_val != 0:
                period_ms *= osc_calibrate_val
            self._write32(_SYSTEM_INTERMEASUREMENT_PERIOD, period_ms)
            self._write8(_SYSRANGE_START, 0x04)
        else:
            self._write8(_SYSRANGE_START, 0x02)

        self._continuous = True
        return True

    def stopContinuous(self):
        if not self._initialized:
            return

        self._write8(_SYSRANGE_START, 0x01)
        self._write8(0xFF, 0x01)
        self._write8(0x00, 0x00)
        self._write8(0x91, 0x00)
        self._write8(0x00, 0x01)
        self._write8(0xFF, 0x00)
        self._continuous = False

    def readContinuousMM(self):
        if not self._continuous:
            return None
        value = self.readRangeContinuousMillimeters()
        if value == 65535 or value == 8191 or value == 0:
            return None
        return value

    def debug(self):
        data = self._read_multi(_RESULT_RANGE_STATUS, 12)
        return {
            "initialized": self._initialized,
            "model_id": self.getModelId(),
            "revision_id": self.getRevisionId(),
            "interrupt_status": self._read8(_RESULT_INTERRUPT_STATUS),
            "range_status_raw": data[0],
            "distance_raw": (data[10] << 8) | data[11],
            "timing_budget_us": self._measurement_timing_budget_us,
            "timeout": self._did_timeout,
        }

    # -------------------------------------------------------------------------
    # Timing budget / VCSEL helpers ported from working C library
    # -------------------------------------------------------------------------
    def getMeasurementTimingBudget(self):
        enables = self._get_sequence_step_enables()
        timeouts = self._get_sequence_step_timeouts(enables)

        start_overhead = 1910
        end_overhead = 960
        msrc_overhead = 660
        tcc_overhead = 590
        dss_overhead = 690
        pre_range_overhead = 660
        final_range_overhead = 550

        budget_us = start_overhead + end_overhead

        if enables["tcc"]:
            budget_us += timeouts["msrc_dss_tcc_us"] + tcc_overhead
        if enables["dss"]:
            budget_us += 2 * (timeouts["msrc_dss_tcc_us"] + dss_overhead)
        elif enables["msrc"]:
            budget_us += timeouts["msrc_dss_tcc_us"] + msrc_overhead
        if enables["pre_range"]:
            budget_us += timeouts["pre_range_us"] + pre_range_overhead
        if enables["final_range"]:
            budget_us += timeouts["final_range_us"] + final_range_overhead

        self._measurement_timing_budget_us = budget_us
        return budget_us

    def setMeasurementTimingBudget(self, budget_us):
        enables = self._get_sequence_step_enables()
        timeouts = self._get_sequence_step_timeouts(enables)

        start_overhead = 1910
        end_overhead = 960
        msrc_overhead = 660
        tcc_overhead = 590
        dss_overhead = 690
        pre_range_overhead = 660
        final_range_overhead = 550

        used_budget_us = start_overhead + end_overhead

        if enables["tcc"]:
            used_budget_us += timeouts["msrc_dss_tcc_us"] + tcc_overhead
        if enables["dss"]:
            used_budget_us += 2 * (timeouts["msrc_dss_tcc_us"] + dss_overhead)
        elif enables["msrc"]:
            used_budget_us += timeouts["msrc_dss_tcc_us"] + msrc_overhead
        if enables["pre_range"]:
            used_budget_us += timeouts["pre_range_us"] + pre_range_overhead

        if enables["final_range"]:
            used_budget_us += final_range_overhead
            if used_budget_us > budget_us:
                return False

            final_range_timeout_us = budget_us - used_budget_us
            final_range_timeout_mclks = self._timeout_us_to_mclks(
                final_range_timeout_us,
                timeouts["final_range_vcsel_period_pclks"],
            )

            if enables["pre_range"]:
                final_range_timeout_mclks += timeouts["pre_range_mclks"]

            self._write16(_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                          self._encode_timeout(final_range_timeout_mclks))
            self._measurement_timing_budget_us = budget_us

        return True

    def getVcselPulsePeriod(self, period_type):
        if period_type == _VCSEL_PRE_RANGE:
            return _decode_vcsel_period(self._read8(_PRE_RANGE_CONFIG_VCSEL_PERIOD))
        if period_type == _VCSEL_FINAL_RANGE:
            return _decode_vcsel_period(self._read8(_FINAL_RANGE_CONFIG_VCSEL_PERIOD))
        return 255

    def setVcselPulsePeriod(self, period_type, period_pclks):
        vcsel_period_reg = _encode_vcsel_period(period_pclks)
        enables = self._get_sequence_step_enables()
        timeouts = self._get_sequence_step_timeouts(enables)

        if period_type == _VCSEL_PRE_RANGE:
            if period_pclks == 12:
                self._write8(_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18)
            elif period_pclks == 14:
                self._write8(_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30)
            elif period_pclks == 16:
                self._write8(_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40)
            elif period_pclks == 18:
                self._write8(_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50)
            else:
                return False

            self._write8(_PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08)
            self._write8(_PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg)

            new_pre_range_timeout_mclks = self._timeout_us_to_mclks(timeouts["pre_range_us"], period_pclks)
            self._write16(_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, self._encode_timeout(new_pre_range_timeout_mclks))

            new_msrc_timeout_mclks = self._timeout_us_to_mclks(timeouts["msrc_dss_tcc_us"], period_pclks)
            self._write8(_MSRC_CONFIG_TIMEOUT_MACROP,
                         255 if new_msrc_timeout_mclks > 256 else new_msrc_timeout_mclks - 1)

        elif period_type == _VCSEL_FINAL_RANGE:
            if period_pclks == 8:
                self._write8(_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10)
                self._write8(_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08)
                self._write8(_GLOBAL_CONFIG_VCSEL_WIDTH, 0x02)
                self._write8(_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C)
                self._write8(0xFF, 0x01)
                self._write8(_ALGO_PHASECAL_LIM, 0x30)
                self._write8(0xFF, 0x00)
            elif period_pclks == 10:
                self._write8(_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28)
                self._write8(_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08)
                self._write8(_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03)
                self._write8(_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09)
                self._write8(0xFF, 0x01)
                self._write8(_ALGO_PHASECAL_LIM, 0x20)
                self._write8(0xFF, 0x00)
            elif period_pclks == 12:
                self._write8(_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38)
                self._write8(_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08)
                self._write8(_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03)
                self._write8(_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08)
                self._write8(0xFF, 0x01)
                self._write8(_ALGO_PHASECAL_LIM, 0x20)
                self._write8(0xFF, 0x00)
            elif period_pclks == 14:
                self._write8(_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48)
                self._write8(_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08)
                self._write8(_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03)
                self._write8(_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07)
                self._write8(0xFF, 0x01)
                self._write8(_ALGO_PHASECAL_LIM, 0x20)
                self._write8(0xFF, 0x00)
            else:
                return False

            self._write8(_FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg)

            new_final_range_timeout_mclks = self._timeout_us_to_mclks(timeouts["final_range_us"], period_pclks)
            if enables["pre_range"]:
                new_final_range_timeout_mclks += timeouts["pre_range_mclks"]
            self._write16(_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                          self._encode_timeout(new_final_range_timeout_mclks))
        else:
            return False

        self.setMeasurementTimingBudget(self._measurement_timing_budget_us)

        sequence_config = self._read8(_SYSTEM_SEQUENCE_CONFIG)
        self._write8(_SYSTEM_SEQUENCE_CONFIG, 0x02)
        self._perform_single_ref_calibration(0x00)
        self._write8(_SYSTEM_SEQUENCE_CONFIG, sequence_config)
        return True

    # -------------------------------------------------------------------------
    # Internal helpers
    # -------------------------------------------------------------------------
    def _get_spad_info(self):
        self._write8(0x80, 0x01)
        self._write8(0xFF, 0x01)
        self._write8(0x00, 0x00)

        self._write8(0xFF, 0x06)
        self._write8(0x83, self._read8(0x83) | 0x04)
        self._write8(0xFF, 0x07)
        self._write8(0x81, 0x01)

        self._write8(0x80, 0x01)
        self._write8(0x94, 0x6B)
        self._write8(0x83, 0x00)

        start = time.ticks_ms()
        while self._read8(0x83) == 0x00:
            if self._check_timeout(start):
                self._did_timeout = True
                return None, None

        self._write8(0x83, 0x01)
        tmp = self._read8(0x92)
        count = tmp & 0x7F
        is_aperture = ((tmp >> 7) & 0x01) != 0

        self._write8(0x81, 0x00)
        self._write8(0xFF, 0x06)
        self._write8(0x83, self._read8(0x83) & ~0x04)
        self._write8(0xFF, 0x01)
        self._write8(0x00, 0x01)
        self._write8(0xFF, 0x00)
        self._write8(0x80, 0x00)

        return count, is_aperture

    def _get_sequence_step_enables(self):
        sequence_config = self._read8(_SYSTEM_SEQUENCE_CONFIG)
        return {
            "tcc": (sequence_config >> 4) & 0x01,
            "dss": (sequence_config >> 3) & 0x01,
            "msrc": (sequence_config >> 2) & 0x01,
            "pre_range": (sequence_config >> 6) & 0x01,
            "final_range": (sequence_config >> 7) & 0x01,
        }

    def _get_sequence_step_timeouts(self, enables):
        pre_range_vcsel_period_pclks = self.getVcselPulsePeriod(_VCSEL_PRE_RANGE)

        msrc_dss_tcc_mclks = self._read8(_MSRC_CONFIG_TIMEOUT_MACROP) + 1
        msrc_dss_tcc_us = self._timeout_mclks_to_us(msrc_dss_tcc_mclks, pre_range_vcsel_period_pclks)

        pre_range_mclks = self._decode_timeout(self._read16(_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI))
        pre_range_us = self._timeout_mclks_to_us(pre_range_mclks, pre_range_vcsel_period_pclks)

        final_range_vcsel_period_pclks = self.getVcselPulsePeriod(_VCSEL_FINAL_RANGE)
        final_range_mclks = self._decode_timeout(self._read16(_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI))

        if enables["pre_range"]:
            final_range_mclks -= pre_range_mclks

        final_range_us = self._timeout_mclks_to_us(final_range_mclks, final_range_vcsel_period_pclks)

        return {
            "pre_range_vcsel_period_pclks": pre_range_vcsel_period_pclks,
            "msrc_dss_tcc_mclks": msrc_dss_tcc_mclks,
            "msrc_dss_tcc_us": msrc_dss_tcc_us,
            "pre_range_mclks": pre_range_mclks,
            "pre_range_us": pre_range_us,
            "final_range_vcsel_period_pclks": final_range_vcsel_period_pclks,
            "final_range_mclks": final_range_mclks,
            "final_range_us": final_range_us,
        }

    def _decode_timeout(self, reg_val):
        return ((reg_val & 0x00FF) << ((reg_val & 0xFF00) >> 8)) + 1

    def _encode_timeout(self, timeout_mclks):
        if timeout_mclks <= 0:
            return 0

        ls_byte = timeout_mclks - 1
        ms_byte = 0
        while ls_byte & 0xFFFFFF00:
            ls_byte >>= 1
            ms_byte += 1
        return ((ms_byte << 8) | (ls_byte & 0xFF)) & 0xFFFF

    def _timeout_mclks_to_us(self, timeout_period_mclks, vcsel_period_pclks):
        macro_period_ns = _calc_macro_period_ns(vcsel_period_pclks)
        return ((timeout_period_mclks * macro_period_ns) + 500) // 1000

    def _timeout_us_to_mclks(self, timeout_period_us, vcsel_period_pclks):
        macro_period_ns = _calc_macro_period_ns(vcsel_period_pclks)
        return ((timeout_period_us * 1000) + (macro_period_ns // 2)) // macro_period_ns

    def _perform_single_ref_calibration(self, vhv_init_byte):
        self._write8(_SYSRANGE_START, 0x01 | vhv_init_byte)

        start = time.ticks_ms()
        while (self._read8(_RESULT_INTERRUPT_STATUS) & 0x07) == 0:
            if self._check_timeout(start):
                self._did_timeout = True
                return False

        self._write8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        self._write8(_SYSRANGE_START, 0x00)
        return True

    def _load_tuning_settings(self):
        seq = (
            (0xFF, 0x01), (0x00, 0x00),
            (0xFF, 0x00), (0x09, 0x00), (0x10, 0x00), (0x11, 0x00),
            (0x24, 0x01), (0x25, 0xFF), (0x75, 0x00),
            (0xFF, 0x01), (0x4E, 0x2C), (0x48, 0x00), (0x30, 0x20),
            (0xFF, 0x00), (0x30, 0x09), (0x54, 0x00), (0x31, 0x04),
            (0x32, 0x03), (0x40, 0x83), (0x46, 0x25), (0x60, 0x00),
            (0x27, 0x00), (0x50, 0x06), (0x51, 0x00), (0x52, 0x96),
            (0x56, 0x08), (0x57, 0x30), (0x61, 0x00), (0x62, 0x00),
            (0x64, 0x00), (0x65, 0x00), (0x66, 0xA0),
            (0xFF, 0x01), (0x22, 0x32), (0x47, 0x14), (0x49, 0xFF),
            (0x4A, 0x00),
            (0xFF, 0x00), (0x7A, 0x0A), (0x7B, 0x00), (0x78, 0x21),
            (0xFF, 0x01), (0x23, 0x34), (0x42, 0x00), (0x44, 0xFF),
            (0x45, 0x26), (0x46, 0x05), (0x40, 0x40), (0x0E, 0x06),
            (0x20, 0x1A), (0x43, 0x40),
            (0xFF, 0x00), (0x34, 0x03), (0x35, 0x44),
            (0xFF, 0x01), (0x31, 0x04), (0x4B, 0x09), (0x4C, 0x05),
            (0x4D, 0x04),
            (0xFF, 0x00), (0x44, 0x00), (0x45, 0x20), (0x47, 0x08),
            (0x48, 0x28), (0x67, 0x00), (0x70, 0x04), (0x71, 0x01),
            (0x72, 0xFE), (0x76, 0x00), (0x77, 0x00),
            (0xFF, 0x01), (0x0D, 0x01),
            (0xFF, 0x00), (0x80, 0x01), (0x01, 0xF8),
            (0xFF, 0x01), (0x8E, 0x01), (0x00, 0x01),
            (0xFF, 0x00), (0x80, 0x00),
        )

        for reg, val in seq:
            self._write8(reg, val)


if __name__ == "__main__":
    tof = EvoTOF(I2C1)
    print("Init:", tof.isInitialized())
    print("Model ID:", tof.getModelId())
    print("Revision ID:", tof.getRevisionId())

    while True:
        print("Distance:", tof.getDistance(), "Debug:", tof.debug())
        time.sleep_ms(100)
