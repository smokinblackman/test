import machine
import time
import ustruct

class VL53L0X:

    def __init__(self, i2c, address=0x29):
        self.i2c = i2c
        self.address = address
        self.stop_variable = 0x00  # Initialize stop_variable
        self.init_sensor()
        self.enable_sequence_step = [False] * 6  # Add this line to initialize the attribute as a list of six False values

    def print_registers(self):
        registers = [
            0x00, 0x01, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x14,
            0x1E, 0x20, 0x21, 0x40, 0x42, 0x45, 0x46, 0x50,
            0x51, 0x52, 0x60, 0x61, 0x70, 0x71, 0x72, 0x80,
            0x81, 0x91, 0xFF,
        ]
        print("Register values:")
        for reg in registers:
            value = self.read8(reg)
            print("  0x{:02X}: 0x{:02X}".format(reg, value))

    def write8(self, reg, value):
        self.i2c.writeto_mem(self.address, reg, bytearray([value]))

    def read8(self, reg):
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]

    def write16(self, reg, value):
        self.i2c.writeto_mem(self.address, reg, ustruct.pack(">H", value))

    def read16(self, reg):
        return ustruct.unpack(">H", self.i2c.readfrom_mem(self.address, reg, 2))[0]

    def init_sensor(self, io_2v8=True):
        # VL53L0X_DataInit()
        self.write8(0x88, 0x00)
        self.write8(0x80, 0x01)
        self.write8(0xFF, 0x01)
        self.write8(0x00, 0x00)
        self.stop_variable = self.read8(0x91)
        self.write8(0x00, 0x01)
        self.write8(0xFF, 0x00)
        self.write8(0x80, 0x00)

        # VL53L0X_StaticInit()
        self.pre_range_vcsel_period_pclks = 0
        self.perform_ref_spad_management()

        # VL53L0X_PerformRefCalibration()
        self.perform_ref_calibration()

        # Configure sensor for 2V8 mode if io_2v8 is true
        VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89
        if io_2v8:
            self.write8(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
                self.read8(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01) # set bit 0
        else:
            self.write8(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
                self.read8(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) & ~0x01) # clear bit 0

        # VL53L0X_SetDeviceMode()
        self.write8(0x80, 0x01)
        self.write8(0xFF, 0x01)
        self.write8(0x00, 0x00)
        self.write8(0x91, self.stop_variable)
        self.write8(0x00, 0x01)
        self.write8(0xFF, 0x00)
        self.write8(0x80, 0x00)

    def perform_ref_spad_management(self):
        # VL53L0X_PerformRefSpadManagement()

        self.write8(0xFF, 0x01)
        self.write8(0x4E, 0x2C)
        self.write8(0xFF, 0x00)
        self.write8(0xB6, 0xB4)
        self.write8(0xFF, 0x01)
        self.write8(0x00, 0x00)
        self.write8(0xFF, 0x00)
        self.write8(0x09, 0x00)
        self.write8(0x10, 0x00)
        self.write8(0x11, 0x00)

        self.write8(0x24, 0x01)
        self.write8(0x25, 0xFF)
        self.write8(0x75, 0x00)

        self.write8(0xFF, 0x01)
        self.write8(0x4E, 0x2C)
        self.write8(0xFF, 0x00)
        self.write8(0xB6, 0xC0)
        self.write8(0xDA, 0x0A)
        self.write8(0x81, 0x04)

        self.write8(0xFF, 0x01)
        self.write8(0x00, 0x01)
        self.write8(0xFF, 0x00)
        self.write8(0x80, 0x01)

        self.write8(0x94, 0x6B)
        self.write8(0x83, 0x00)

        start = time.ticks_ms()
        while self.read8(0x83) == 0x00:
            if time.ticks_diff(time.ticks_ms(), start) > 100:
                break

        self.write8(0x83, 0x01)
        tmp = self.read8(0x92)

        num_good_spads = tmp & 0x7F
        num_requested_spads = 12

        if num_good_spads < num_requested_spads:
            raise RuntimeError("Failed to find minimum good SPADs")

        self.write8(0x81, 0x00)
        self.write8(0xFF, 0x01)
        self.write8(0x00, 0x00)
        self.write8(0xFF, 0x00)
        self.write8(0x80, 0x00)

    def perform_ref_calibration(self):
        # VL53L0X_PerformRefCalibration()
        self.write8(0xFF, 0x01)
        self.write8(0x4F, 0x2C)
        self.write8(0xFF, 0x00)
        self.write8(0xB6, 0xB4)
		self.write8(0xFF, 0x01)
        self.write8(0x00, 0x00)
        self.write8(0xFF, 0x00)
        self.write8(0x09, 0x00)
        self.write8(0x10, 0x00)
        self.write8(0x11, 0x00)

        self.write8(0x24, 0x01)
        self.write8(0x25, 0xFF)
        self.write8(0x75, 0x00)

        self.write8(0xFF, 0x01)
        self.write8(0x4F, 0x2C)
        self.write8(0xFF, 0x00)
        self.write8(0xB6, 0xC0)
        self.write8(0xDA, 0x0B)
        self.write8(0x81, 0x04)

        self.write8(0xFF, 0x01)
        self.write8(0x00, 0x01)
        self.write8(0xFF, 0x00)
        self.write8(0x80, 0x01)

        self.write8(0x94, 0x6B)
        self.write8(0x83, 0x00)

        start = time.ticks_ms()
        while self.read8(0x83) == 0x00:
            if time.ticks_diff(time.ticks_ms(), start) > 100:
                break

        self.write8(0x83, 0x01)

        self.write8(0x81, 0x00)
        self.write8(0xFF, 0x01)
        self.write8(0x00, 0x00)
        self.write8(0xFF, 0x00)
        self.write8(0x80, 0x00)

    def set_vcsel_pulse_period(self, vcsel_period_type, period_pclks):
        # VL53L0X_SetVcselPulsePeriod()

        if vcsel_period_type == VcselPeriodPreRange:
            if period_pclks < 12 or period_pclks > 18:
                raise ValueError("PreRange period out of bounds")

            # Apply specific settings for the requested clock period
            new_pre_range_vcsel_period_pclks = period_pclks
            old_pre_range_vcsel_period_pclks = self.get_vcsel_pulse_period(VcselPeriodPreRange)

            if new_pre_range_vcsel_period_pclks != old_pre_range_vcsel_period_pclks:
                # apply new VCSEL period
                self.write8(0xFF, 0x01)
                self.write8(0x00, 0x00)
                self.write8(0xFF, 0x00)
                self.write8(0x80, 0x01)
                self.write8(0xFF, 0x01)
                self.write8(0x00, 0x04)
                self.write8(0xFF, 0x00)
                self.write8(0x80, 0x00)

                self.write8(0x70, (new_pre_range_vcsel_period_pclks - 12) | 0x18)
                self.write8(0xFF, 0x01)
                self.write8(0x00, 0x00)
				                self.write8(0xFF, 0x00)
                self.write8(0x80, 0x01)
                self.write8(0xFF, 0x01)
                self.write8(0x00, 0x04)
                self.write8(0xFF, 0x00)
                self.write8(0x80, 0x00)

                # apply new VCSEL period
                self.write8(0x70, (new_pre_range_vcsel_period_pclks - 12) | 0x18)

                # update timeouts
                new_pre_range_timeout_mclks = self.timeout_microseconds_to_mclks(
                    self.get_measurement_timing_budget_micro_seconds(), old_pre_range_vcsel_period_pclks
                )
                new_pre_range_timeout_mclks *= new_pre_range_vcsel_period_pclks // old_pre_range_vcsel_period_pclks

                self.set_timeout(PreRange, self.timeout_mclks_to_microseconds(new_pre_range_timeout_mclks, new_pre_range_vcsel_period_pclks))

        elif vcsel_period_type == VcselPeriodFinalRange:
            if period_pclks < 8 or period_pclks > 14:
                raise ValueError("FinalRange period out of bounds")

            # Apply specific settings for the requested clock period
            new_final_range_vcsel_period_pclks = period_pclks
            old_final_range_vcsel_period_pclks = self.get_vcsel_pulse_period(VcselPeriodFinalRange)

            if new_final_range_vcsel_period_pclks != old_final_range_vcsel_period_pclks:
                self.write8(0xFF, 0x01)
                self.write8(0x00, 0x00)
                self.write8(0xFF, 0x00)
                self.write8(0x80, 0x01)
                self.write8(0xFF, 0x01)
                self.write8(0x00, 0x04)
                self.write8(0xFF, 0x00)
                self.write8(0x80, 0x00)

                self.write8(0x71, (new_final_range_vcsel_period_pclks - 8) | 0x10)
                self.write8(0xFF, 0x01)
                self.write8(0x00, 0x00)
                self.write8(0xFF, 0x00)
                self.write8(0x80, 0x01)
                self.write8(0xFF, 0x01)
                self.write8(0x00, 0x04)
                self.write8(0xFF, 0x00)
                self.write8(0x80, 0x00)

                # update timeouts
                new_final_range_timeout_mclks = self.timeout_microseconds_to_mclks(
                    self.get_measurement_timing_budget_micro_seconds(), old_final_range_vcsel_period_pclks
                )
                new_final_range_timeout_mclks *= new_final_range_vcsel_period_pclks // old_final_range_vcsel_period_pclks

                self.set_timeout(FinalRange, self.timeout_mclks_to_microseconds(new_final_range_timeout_mclks, new_final_range_vcsel_period_pclks))

        else:
            raise ValueError("Invalid period type")

    def get_vcsel_pulse_period(self, vcsel_period_type):
        # VL53L0X_GetVcsEL_PulsePeriod
        if vcsel_period_type == VcselPeriodPreRange:
            return self.read8(0x70) + 12
        elif vcsel_period_type == VcselPeriodFinalRange:
            return self.read8(0x71) + 12
        else:
            raise ValueError("Invalid period type")

    def get_measurement_timing_budget_micro_seconds(self):
        # Get sequence step enables
        sequence_config = self.read8(0x01)
        tcc = (sequence_config >> 4) & 0x1
        dss = (sequence_config >> 3) & 0x1
        msrc = (sequence_config >> 2) & 0x1
        pre_range = (sequence_config >> 6) & 0x1
        final_range = (sequence_config >> 7) & 0x1

        # Get sequence step timeouts
        msrc_dss_tcc_mclks = (self.read8(0x46) + 1) << 1
        pre_range_mclks = self.timeout_encoded_to_mclks(self.read8(0x51), self.get_vcsel_pulse_period(VcselPeriodPreRange))
        final_range_mclks = self.timeout_encoded_to_mclks(self.read8(0x71), self.get_vcsel_pulse_period(VcselPeriodFinalRange))

        # Get sequence step timeout in microseconds
        msrc_dss_tcc_us = self.timeout_mclks_to_microseconds(msrc_dss_tcc_mclks, self.get_vcsel_pulse_period(VcselPeriodPreRange))
        pre_range_us = self.timeout_mclks_to_microseconds(pre_range_mclks, self.get_vcsel_pulse_period(VcselPeriodPreRange))
        final_range_us = self.timeout_mclks_to_microseconds(final_range_mclks, self.get_vcsel_pulse_period(VcselPeriodFinalRange))

        # Calculate the total timing budget
        budget_us = msrc_dss_tcc_us * (tcc + dss)
        budget_us += pre_range_us * pre_range
        budget_us += final_range_us * final_range

        return budget_us

    def set_measurement_timing_budget_micro_seconds(self, budget_us):
        if budget_us <= 1910:
            raise ValueError("Invalid timing budget")

        used_budget_us = 1320
        final_range_timeout_us = budget_us - used_budget_us
        final_range_timeout_mclks = self.timeout_microseconds_to_mclks(
            final_range_timeout_us, self.get_vcsel_pulse_period(VcselPeriodFinalRange)
        )

        self.set_timeout(FinalRange, self.timeout_mclks_to_microseconds(final_range_timeout_mclks, self.get_vcsel_pulse_period(VcselPeriodFinalRange)))

    def set_timeout(self, timeout_type, timeout_us):
        if timeout_type == PreRange:
            encoded_timeout = self.timeout_microseconds_to_encoded(timeout_us, self.get_vcsel_pulse_period(VcselPeriodPreRange))
            self.write8(0x51, encoded_timeout)
        elif timeout_type == FinalRange:
            encoded_timeout = self.timeout_microseconds_to_encoded(timeout_us, self.get_vcsel_pulse_period(VcselPeriodFinalRange))
            self.write8(0x71, encoded_timeout)
        else:
            raise ValueError("Invalid timeout type")

    def timeout_microseconds_to_mclks(self, timeout_us, vcsel_period_pclks):
        macro_period_ns = ((2304 * (vcscsel_period_pclks) + 1655) // 1655
        return ((timeout_us * 1000) + (macro_period_ns // 2)) // macro_period_ns

    def timeout_mclks_to_microseconds(self, timeout_mclks, vcsel_period_pclks):
        macro_period_ns = ((2304 * (vcsel_period_pclks) + 1655) // 1655
        return ((timeout_mclks * macro_period_ns) + 500) // 1000

    def timeout_encoded_to_mclks(self, timeout_encoded, vcsel_period_pclks):
        return (timeout_encoded + 1) * (1 << (vcsel_period_pclks - 1))

    def timeout_microseconds_to_encoded(self, timeout_us, vcsel_period_pclks):
        timeout_mclks = self.timeout_microseconds_to_mclks(timeout_us, vcsel_period_pclks)
        return self.timeout_mclks_to_encoded(timeout_mclks, vcsel_period_pclks)

    def timeout_mclks_to_encoded(self, timeout_mclks, vcsel_period_pclks):
        return (timeout_mclks - 1) // (1 << (vcsel_period_pclks - 1))

    def set_signal_rate_limit(self, limit_mcps):
        self.write16(0x44, int(limit_mcps * (1 << 7)))

    def get_signal_rate_limit(self):
        return self.read16(0x44) / (1 << 7)

    def set_pre_range_config_timeout_micro_seconds(self, timeout_us):
        self.set_timeout(PreRange, timeout_us)

    def get_pre_range_config_timeout_micro_seconds(self):
        return self.get_timeout(PreRange)

    def set_final_range_config_timeout_micro_seconds(self, timeout_us):
        self.set_timeout(FinalRange, timeout_us)

    def get_final_range_config_timeout_micro_seconds(self):
        return self.get_timeout(FinalRange)

    def get_timeout(self, timeout_type):
        if timeout_type == PreRange:
            return self.timeout_mclks_to_microseconds(
                self.timeout_encoded_to_mclks(self.read8(0x51), self.get_vcsel_pulse_period(VcselPeriodPreRange)),
                self.get_vcsel_pulse_period(VcselPeriodPreRange),
            )
        elif timeout_type == FinalRange:
            return self.timeout_mclks_to_microseconds(
                self.timeout_encoded_to_mclks(self.read8(0x71), self.get_vcsel_pulse_period(VcselPeriodFinalRange)),
                self.get_vcsel_pulse_period(VcselPeriodFinalRange),
            )
        else:
            raise ValueError("Invalid timeout type")

    def perform_single_ref_calibration(self, vhv_init_byte):
        self.write8(0xBB, vhv_init_byte | 0x01)
        self.perform_single_measurement()
        self.write8(0xBB, vhv_init_byte & ~0x01)

    def get_spad_info(self):
        self.write8(0x80, 0x01)
        self.write8(0xFF, 0x01)
        self.write8(0x00, 0x00)
        self.write8(0xFF, 0x06)
        self.write8(0x83, self.read8(0x83) | 0x04)
        self.write8(0xFF, 0x07)
        self.write8(0x81, 0x01)

        self.write        8(0x80, 0x01)
        spad_count, is_aperture = self.read8(0x96), self.read8(0x92) & 0x07
        self.write8(0x81, 0x00)
        self.write8(0xFF, 0x06)
        self.write8(0x83, self.read8(0x83) & ~0x04)
        self.write8(0xFF, 0x01)
        self.write8(0x00, 0x01)
        self.write8(0xFF, 0x00)
        self.write8(0x80, 0x00)

        return spad_count, is_aperture

    def perform_vhv_calibration(self):
        self.write8(0x88, self.read8(0x88) | 0x80)
        self.write8(0xFF, 0x01)
        self.write8(0x00, 0x00)
        self.write8(0xFF, 0x00)
        self.write8(0x80, 0x00)
        self.write8(0x91, 0x3C)
        self.write8(0x01, 0x00)

    def perform_phase_calibration(self):
        self.write8(0xFF, 0x01)
        self.write8(0x00, 0x00)
        self.write8(0xFF, 0x00)
        self.write8(0x80, 0x00)
        self.write8(0x92, 0x01)
        self.write8(0x00, 0x00)
        self.write8(0xFF, 0x01)
        self.write8(0x00, 0x00)
        self.write8(0xFF, 0x00)
        self.write8(0x80, 0x00)

    def perform_ref_calibration(self):
        self.perform_single_ref_calibration(0x40)
        self.perform_single_ref_calibration(0x00)

    def perform_single_measurement(self):
        self.write8(0x80, 0x01)
        self.write8(0xFF, 0x01)
        self.write8(0x00, 0x00)
        self.write8(0xFF, 0x00)
        self.write8(0x80, 0x00)

    def get_sequence_step_enables(self):
        sequence_config = self.read8(0x01)
        tcc = (sequence_config >> 4) & 0x1
        dss = (sequence_config >> 3) & 0x1
        msrc = (sequence_config >> 2) & 0x1
        pre_range = (sequence_config >> 6) & 0x1
        final_range = (sequence_config >> 7) & 0x1

        return (tcc, dss, msrc, pre_range, final_range)

    def get_sequence_step_timeouts(self, pre_range):
        pre_range_vcsel_period_pclks = self.get_vcsel_pulse_period(VcselPeriodPreRange)
        msrc_dss_tcc_mclks = (self.read8(0x46) + 1) << 1
        msrc_dss_tcc_us = self.timeout_mclks_to_microseconds(msrc_dss_tcc_mclks, pre_range_vcsel_period_pclks)

    def get_vcsel_pulse_period(self, vcsel_period_type):
		if vcsel_period_type == VcselPeriodPreRange:
			vcsel_period_reg = self.read8(0x50)
			return decode_vcsel_period(vcsel_period_reg)
		elif vcsel_period_type == VcselPeriodFinalRange:
			vcsel_period_reg = self.read8(0x70)
			return decode_vcsel_period(vcsel_period_reg)
		else:
			raise ValueError("Invalid vcsel period type")

	def set_vcsel_pulse_period(self, vcsel_period_type, period_pclks):
		if vcsel_period_type == VcselPeriodPreRange:
			vcsel_period_reg = encode_vcsel_period(period_pclks)
			self.write8(0x50, vcsel_period_reg)
		elif vcsel_period_type == VcselPeriodFinalRange:
			vcsel_period_reg = encode_vcsel_period(period_pclks)
			self.write8(0x70, vcsel_period_reg)
		else:
			raise ValueError("Invalid vcsel period type")

	def start_continuous(self, period_ms=0):
		self.write8(0x80, 0x01)
		self.write8(0xFF, 0x01)
		self.write8(0x00, 0x00)
		self.write8(0x91, self.stop_variable)
		self.write8(0x00, 0x01)
		self.write8(0xFF, 0x00)
		self.write8(0x80, 0x00)

		if period_ms != 0:
			osc_calibrate_val = self.read16(0xF8)
			if osc_calibrate_val != 0:
				period_mclks = (period_ms * osc_calibrate_val)
				self.write16(0x51, period_mclks)
			else:
				raise ValueError("Invalid oscillator calibration value")

			self.write8(0x80, 0x01)
			self.write8(0xFF, 0x01)
			self.write8(0x00, 0x00)
			self.write8(0x91, self.stop_variable)
			self.write8(0x00, 0x01)
			self.write8(0xFF, 0x00)
			self.write8(0x80, 0x00)
			self.write8(0x81, 0x03)

	def stop_continuous(self):
		self.write8(0x81, 0x00)
		self.write8(0xFF, 0x01)
		self.write8(0x00, 0x00)
		self.write8(0x91, self.stop_variable)
		self.write8(0x00, 0x01)
		self.write8(0xFF, 0x00)

	def get_distance(self):
		self.write8(0x80, 0x01)
		self.write8(0xFF, 0x01)
		self.write8(0x00, 0x00)
		self.write8(0x91, self.stop_variable)
		self.write8(0x00, 0x01)
		self.write8(0xFF, 0x00)
		self.write8(0x80, 0x00)

		self.write8(0x00, 0x01)
		self.write8(0xFF, 0x01)
		self.write8(0x02, self.global_config_timing_budget_us // 32)
		self.write8(0x01, 0xFF)
		sequence_config = self.get_sequence_step_enables()
		if sequence_config[3]: # pre_range_enabled
			self.write8(0x01, (sequence_config[4] << 3) | (sequence_config[3] << 2) | (sequence_config[2] << 1) | sequence_config[1])
			self.perform_single_measurement()
			sequence_config = self.get_sequence_step_enables()
		if sequence_config[4]: # final_range_enabled
			self.write8(0x01, (sequence_config[4] << 3) | (sequence_config[3] << 2) | (sequence_config[2] << 1) | sequence_config[1])
			self.perform_single_measurement()
			range_status = self.read8(0x14)
		if range_status == 14:
			raise RuntimeError("Error: Detected invalid range")

	def get_signal_rate_limit(self):
		return self.read16(0x44) / (1 << 7)

	def set_signal_rate_limit(self, limit_mcps):
		if limit_mcps < 0 or limit_mcps > 511.99:
			raise ValueError("Invalid signal rate limit value")
			self.write16(0x44, int(limit_mcps * (1 << 7)))
			
	def get_spad_count(self):
		self.perform_single_measurement()
		self.write8(0x80, 0x01)
		self.write8(0xFF, 0x01)
		self.write8(0x00, 0x00)
		self.write8(0xFF, 0x06)
		self.write8(0x83, self.read8(0x83) | 0x04)
		self.write8(0xFF, 0x07)
		self.write8(0x81, 0x01)
		self.write8(0x80, 0x01)
		self.write8(0x94, 0x6b)
		self.write8(0x83, 0x00)
		self.start_timeout()
		while (self.read8(0x83) == 0x00):
			if self.check_timeout_expired():
				raise RuntimeError("Timeout waiting for SPAD count")

		spad_count = self.read8(0x83)
		self.write8(0x81, 0x00)
		self.write8(0xFF, 0x06)
		self.write8(0x83, self.read8(0x83) & ~0x04)
		self.write8(0xFF, 0x01)
		self.write8(0x00, 0x01)
		self.write8(0xFF, 0x00)
		self.write8(0x80, 0x00)
		return spad_count
	
	def set_roi(self, x, y, width, height):
		if (x < 0 or y < 0 or width < 0 or height < 0 or x + width > 16 or y + height > 16):
			raise ValueError("Invalid ROI parameters")
			self.write8(0xFF, 0x01)

			self.write8(0x04, (x & 0xFF))
			self.write8(0x05, ((x >> 8) & 0xFF))
			self.write8(0x06, (height - 1) & 0xFF)
			self.write8(0x07, ((height - 1) >> 8) & 0xFF)
			self.write8(0x08, (width - 1) & 0xFF)
			self.write8(0x09, ((width - 1) >> 8) & 0xFF)
			self.write8(0xFF, 0x00)
			self.write8(0x80, 0x01)
			self.write8(0xFF, 0x01)
			self.write8(0x00, 0x00)
			self.write8(0x80, 0x00)

	def set_user_roi(self, roi):
		if (roi['x'] < 0 or roi['y'] < 0 or roi['width'] < 0 or roi['height'] < 0 or roi['x'] + roi['width'] > 16 or roi['y'] + roi['height'] > 16):
		raise ValueError("Invalid ROI parameters")

		x = roi['x']
		y = roi['y']
		height = roi['height']
		width = roi['width']

		self.write8(0xFF, 0x01)
		self.write8(0x02, (y & 0xFF))
		self.write8(0x03, ((y >> 8) & 0xFF))
		self.write8(0x04, (x & 0xFF))
		self.write8(0x05, ((x >> 8) & 0xFF))
		self.write8(0x06, (height - 1) & 0xFF)
		self.write8(0x07, ((height - 1) >> 8) & 0xFF)
		self.write8(0x08, (width - 1) & 0xFF)
		self.write8(0x09, ((width - 1) >> 8) & 0xFF)
		self.write8(0xFF, 0x00)
		self.write8(0x80, 0x01)
		self.write8(0xFF, 0x01)
		self.write8(0x00, 0x00)
		self.write8(0x80, 0x00)

	def get_sequence_step_enables(self):
		sequence_config = self.read8(0x01)
		tcc = (sequence_config >> 4) & 0x1
		dss = (sequence_config >> 3) & 0x1
		msrc = (sequence_config >> 2) & 0x1
		pre_range = (sequence_config >> 6) & 0x1
		final_range = (sequence_config >> 7) & 0x1
		return (tcc, dss, msrc, pre_range, final_range)

	def get_sequence_step_timeouts(self, pre_range):
		timeouts = {}

		timeouts['pre_range_vcsel_period_pclks'], timeouts['msrc_dss_tcc_mclks'], timeouts['msrc_dss_tcc_us'], timeouts['pre_range_mclks'], timeouts['pre_range_us'], timeouts['final_range_vcsel_period_pclks'], timeouts['final_range_mclks'], timeouts['final_range_us'] = [0] * 8

		pre_range_vcsel_period_pclks = self.get_vcsel_pulse_period(VcselPeriodPreRange)

		msrc_dss_tcc_mclks = self.read8(0x46)
		msrc_dss_tcc_us = self.calc_macro_period(
		self.read16_bit(
		MSRC_CONFIG_TIMEOUT_MACROP_HI) + self.read8(
		MSRC_CONFIG_TIMEOUT_MACROP_LO), pre_range_vcsel_period_pclks)

		pre_range_mclks = decode_timeout(self.read16_bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI))

		pre_range_us = timeout_mclks_to_microseconds(pre_range_mclks, pre_range_vcsel_period_pclks)

		final_range_vcsel_period_pclks = self.get_vcsel_pulse_period(VcselPeriodFinalRange)

		final_range_mclks = decode_timeout(self.read16_bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI))

		if pre_range:
			final_range_mclks -= pre_range_mclks

			final_range_us = timeout_mclks_to_microseconds(final_range_mclks, final_range_vcsel_period_pclks)

			timeouts['pre_range_vcsel_period_pclks'] = pre_range_vcsel_period_pclks
			timeouts['msrc_dss_tcc_mclks'] = msrc_dss_tcc_mclks
			timeouts['msrc_dss_tcc_us'] = msrc_dss_tcc_us
			timeouts['pre_range_mclks'] = pre_range_mclks
			timeouts['pre_range_us'] = pre_range_us
			timeouts['final_range_vcsel_period_pclks'] = final_range_vcsel_period_pclks
			timeouts['final_range_mclks'] = final_range_mclks
			timeouts['final_range_us'] = final_range_us

			return timeouts

	def get_sequence_step_timeout_mclks(self, step):
		pre_range = self.get_sequence_step_enables()[3]
		if step == SequenceStep.TCC:
		return self.calc_timeout_mclks(self.get_vcsel_pulse_period(VcselPeriodPreRange),
		self.read8(TCC_MSRC_CONFIG__TIMEOUT_MACROP) + 1)
		elif step == SequenceStep.DSS:
		return self.calc_timeout_mclks(
		2 * self.get_vcsel_pulse_period(VcselPeriodPreRange),
		self.read16_bit(MSRC_CONFIG_TIMEOUT_MACROP_HI) +
		self.read8(MSRC_CONFIG_TIMEOUT_MACROP_LO) + 1)
		elif step == SequenceStep.MSRC:
		return self.calc_timeout_mclks(
		self.get_vcsel_pulse_period(VcselPeriodMsrc),
		self.read8(MSRC_CONFIG_TIMEOUT_MACROP) + 1)
		elif step == SequenceStep.PRE_RANGE:
		return decode_timeout(self.read16_bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI))
		elif step == SequenceStep.FINAL_RANGE:
		return self.calc_timeout_mclks(self.get_vcsel_pulse_period(VcselPeriodFinalRange),
		self.read16_bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI) -
		(0 if pre_range else decode_timeout(
		self.read16_bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI))))
		else:
		raise ValueError("Invalid sequence step")

		def get_vcsel_pulse_width(self, vcsel_period_type):
		if vcsel_period_type == VcselPeriodPreRange:
		vcsel_period_reg = self.read8(PRE_RANGE_CONFIG_VCSEL_PERIOD)
		return decode_vcsel_period(vcsel_period_reg)
		elif vcsel_period_type == VcselPeriodFinalRange:
		vcsel_period_reg = self.read8(FINAL_RANGE_CONFIG_VCSEL_PERIOD)
		return decode_vcsel_period(vcsel_period_reg)
		else:
		raise ValueError("Invalid vcsel period type")
		
	def set_vcsel_pulse_width(self, vcsel_period_type, period_pclks):
		if vcsel_period_type == VcselPeriodPreRange:
			vcsel_period_reg = encode_vcsel_period(period_pclks)
			self.write8(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg)
		elif vcsel_period_type == VcselPeriodFinalRange:
			vcsel_period_reg = encode_vcsel_period(period_pclks)
			self.write8(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg)
		else:
			raise ValueError("Invalid vcsel period type")

	def perform_ref_calibration(self, vhv_init_byte):
		self.write8(SYSRANGE_START, 0x01 | vhv_init_byte) # VL53L0X_REG_SYSRANGE_MODE_START_STOP

		self.start_timeout()
	while (self.read8(REG_RESULT_INTERRUPT_STATUS) & 0x07) == 0:
		if self.check_timeout_expired():
			raise RuntimeError("Timeout waiting for MM1 calibration")

			self.write8(SYSTEM_INTERRUPT_CLEAR, 0x01)

			self.write8(SYSRANGE_START, 0x00)

	def encode_timeout(self, timeout_macro_period_mclks):
		timeout_mclks = timeout_macro_period_mclks * (self.get_clock_speed() / 1000000.0)
		return timeout_mclks_to_mclks(timeout_mclks)

	def decode_timeout(self, reg_val):
		timeout_mclks = timeout_regs[reg_val & 0x0F] << (reg_val >> 4)
		return timeout_mclks

	def calc_timeout_mclks(self, period_pclks, timeout_period_us):
		timeout_mclks = timeout_period_us / \
		(period_pclks * (self.get_clock_speed() / 1000000.0))
		if timeout_mclks > 0xFF:
			raise ValueError("Timeout value is out of range")
		return int(timeout_mclks)

	def calc_macro_period(self, timeout_period_us, vcsel_period_pclks):
		macro_period_ns = (timeout_period_us * 1000) + (vcsel_period_pclks * 2048)
		return int(macro_period_ns / 1000)

	def set_measurement_timing_budget(self, budget_us):
		if budget_us < 20000:
			raise ValueError("Minimum measurement timing budget is 20 ms")

			used_budget_us = self.get_measurement_timing_budget()

			seq = self.get_sequence_step_enables()
			tcc = (seq[0] == 1)
			dss = (seq[1] == 1)
			msrc = (seq[2] == 1)
			pre_range = (seq[3] == 1)
			final_range = (seq[4] == 1)

		if tcc:
			used_budget_us += (self.calc_timeout_mclks(self.get_vcsel_pulse_period(VcselPeriodPreRange),
				self.read8(TCC_MSRC_CONFIG__TIMEOUT_MACROP) + 1) * 2 * 1000)
		if dss:
			used_budget_us += ((self.calc_timeout_mclks(2 * self.get_vcsel_pulse_period(VcselPeriodPreRange),
				self.read16_bit(MSRC_CONFIG_TIMEOUT_MACROP_HI) + self.read8(MSRC_CONFIG_TIMEOUT_MACROP_LO) + 1)) * 2 * 1000)
		elif msrc:
			used_budget_us += ((self.calc_timeout_mclks(self.get_vcsel_pulse_period(VcselPeriodMsrc),
				self.read8(MSRC_CONFIG_TIMEOUT_MACROP) + 1)) * 2 * 1000)

		if pre_range:
			used_budget_us += ((self.decode_timeout(self.read16_bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI))) * 2 + 1) * 1000
		if final_range:
			used_budget_us += ((self.decode_timeout(self.read16_bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI)) -
			(0 if pre_range else self.decode_timeout(self.read16_bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI)))) * 2 + 1) * 1000
			final_range_vcsel_period_pclks = self.get_vcsel_pulse_period(VcselPeriodFinalRange)
			final_range_timeout_us = timeout_mclks_to_microseconds(self.calc_timeout_mclks(final_range_vcsel_period_pclks, 1000), final_range_vcsel_period_pclks)
		if used_budget_us + final_range_timeout_us > budget_us:
			raise ValueError("Requested measurement timing budget is too high.")
			sequence_config = self.get_sequence_step_enables()
			pre_range_vcsel_period_pclks = self.get_vcsel_pulse_period(VcselPeriodPreRange)
			msrc_dss_tcc_us = (self.calc_macro_period(self.read16_bit(MSRC_CONFIG_TIMEOUT_MACROP_HI) +
			self.read8(MSRC_CONFIG_TIMEOUT_MACROP_LO),
			pre_range_vcsel_period_pclks) +
			self.get_sequence_step_timeout_mclks(SequenceStep.DSS) * (pre_range_vcsel_period_pclks + 1))
			self.write8(0xFF, 0x01)
			self.write8(0x04, (msrc_dss_tcc_us >> 24) & 0xFF)
			self.write8(0x03, (msrc_dss_tcc_us >> 16) & 0xFF)
			self.write8(0x02, (msrc_dss_tcc_us >> 8) & 0xFF)
			self.write8(0x01, msrc_dss_tcc_us & 0xFF)
			self.write8(0xFF, 0x00)
			final_range_timeout_mclks = self.calc_timeout_mclks(final_range_vcsel_period_pclks, final_range_timeout_us)
		if pre_range:
			final_range_timeout_mclks -= (self.get_sequence_step_timeout_mclks(SequenceStep.PRE_RANGE) + 1)
			self.write16_bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, self.encode_timeout(final_range_timeout_mclks))
			self.global_config_timing_budget_us = budget_us
			self.set_measurement_timing_budget_microseconds(budget_us)



	def start_continuous(self, period_ms=0):
		sequence_config = self.get_sequence_step_enables()
		if sequence_config.count(True) != 1:
			raise ValueError("Must have exactly one sequence step enabled")
		if sequence_config[SequenceStep.REPEAT_LAST] or sequence_config[SequenceStep.LAST]:
			raise ValueError("REPEAT_LAST and LAST cannot both be enabled")
			self.write8(0x80, 0x01)
			self.write8(0xFF, 0x01)
			self.write8(0x00, 0x00)
			self.write8(0x91, self.stop_variable)
			self.write8(0x00, 0x01)
			self.write8(0xFF, 0x00)
			self.write8(0x80, 0x00)

			if period_ms != 0:
				osc_calibrate_val = self.read8(OSC_CALIBRATE_VAL)
				if osc_calibrate_val == 0:
					osc_calibrate_val = 1
				period_mclks = round(period_ms / (0.000001 * self.get_clock_speed() / osc_calibrate_val))
				if period_mclks < 1:
					period_mclks = 1
					self.write16_bit(ALGO_RECALIBRATE_MASK_CTRL, period_mclks)
			self.write8(SYSRANGE_START, 0x04)

	def stop_continuous(self):
		self.write8(SYSRANGE_START, 0x01)
		self.write8(0xFF, 0x01)
		self.write8(0x00, 0x00)
		self.write8(0x91, 0x00)
		self.write8(0x00, 0x01)
		self.write8(0xFF, 0x00)
		self.write8(0x80, 0x00)

	def read_range_continuous_millimeters(self, period_ms=0):
		if period_ms != 0:
			self.start_continuous(period_ms)
			start_time = time.monotonic()
		while (self.read8(REG_RESULT_INTERRUPT_STATUS) & 0x07) == 0:
			if self.check_timeout_expired():
				raise RuntimeError("Timeout waiting for measurement")
				range_mm = self.read_range_millimeters()
				self.write8(SYSTEM_INTERRUPT_CLEAR, 0x01)
			if period_ms != 0:
				self.stop_continuous()
			return (range_mm, time.monotonic() - start_time)

	def start_single(self, auto_calibration=True):
		sequence_config = self.get_sequence_step_enables()
		if sequence_config.count(True) != 1:
			raise ValueError("Must have exactly one sequence step enabled")
		if sequence_config[SequenceStep.REPEAT_LAST] or sequence_config[SequenceStep.LAST]:
			raise ValueError("REPEAT_LAST and LAST cannot both be enabled")
		if auto_calibration:
			self.perform_single_ref_calibration(SequenceStep.FINAL_RANGE)
			self.write8(0x80, 0x01)
			self.write8(0xFF, 0x01)
			self.write8(0x00, 0x00)
			self.write8(0x91, self.stop_variable)
			self.write8(0x00, 0x01)
			self.write8(0xFF, 0x00)
			self.write8(0x80, 0x00)

			self.write8(SYSRANGE_START, 0x01)
	def perform_single_ref_calibration(self, vhv_init_byte):
		self.write8(SYSRANGE_START, 0x01 | vhv_init_byte) # VL53L0X_REG_SYSRANGE_MODE_START_STOP

		self.start_timeout()
		while (self.read8(REG_RESULT_INTERRUPT_STATUS) & 0x07) == 0:
			if self.check_timeout_expired():
				raise RuntimeError("Timeout waiting for MM1 calibration")

		self.write8(SYSTEM_INTERRUPT_CLEAR, 0x01)

		self.write8(SYSRANGE_START, 0x00)

	def read_range_single_millimeters(self, blocking=True):
		if blocking:
			self.start_timeout()
			while not self.timeout_occurred():
				system_status = self.get_system_status()
				if system_status == 0x01:
					break
				if self.check_timeout_expired():
					raise RuntimeError("Timeout waiting for measurement")
		self.write8(SYSTEM_INTERRUPT_CLEAR, 0x01)
		range_mm = self.read_range_millimeters()
		return range_mm

	def read_range_millimeters(self):
		range_mm = self.read16_bit(REG_RESULT_RANGE_STATUS + 10)
		return range_mm

	def timeout_occurred(self):
		return self.check_timeout_expired()

	def set_device_mode(self, mode):
		if mode not in [DeviceMode.SINGLE_RANGE, DeviceMode.CONTINUOUS_RANGE]:
			raise ValueError("Invalid mode")
		self.device_mode = mode
		if mode == DeviceMode.CONTINUOUS_RANGE:
			self.start_continuous()
		else:
			self.stop_continuous()

	def get_device_mode(self):
		return self.device_mode

	def get_system_status(self, clear=True):
		system_status = self.read8(REG_SYSTEM_STATUS)
		if clear:
			self.write8(SYSTEM_INTERRUPT_CLEAR, 0x01)
		return system_status

	def set_address(self, new_address):
		if not (1 <= new_address <= 127):
			raise ValueError("Invalid I2C address")
		self.write8(I2C_SLAVE_DEVICE_ADDRESS, new_address & 0x7F)
		self.address = new_address

	def get_address(self):
		return self.address

	def get_info(self):
		return {
			"model_id": self.read8(0xC2),
			"module_type": self.read8(0xC0),
			"revision_id": self.read8(0xC5),
			"product_id": self.read16_bit(0xC2),
			"range_status": self.read8(REG_RESULT_RANGE_STATUS),
			"interrupt_status": self.read8(REG_RESULT_INTERRUPT_STATUS),
			"device_status": self.get_system_status(clear=False),
			"address": self.get_address()
		}
