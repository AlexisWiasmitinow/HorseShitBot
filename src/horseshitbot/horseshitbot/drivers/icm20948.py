#!/usr/bin/env python3
"""Minimal ICM-20948 I2C driver used by the ROS 2 IMU node."""

from __future__ import annotations

import time

# ICM-20948
ICM_WHO_AM_I = 0xEA
REG_BANK_SEL = 0x7F

# Bank 0
B0_WHO_AM_I = 0x00
B0_USER_CTRL = 0x03
B0_PWR_MGMT_1 = 0x06
B0_PWR_MGMT_2 = 0x07
B0_INT_PIN_CFG = 0x0F
B0_ACCEL_XOUT_H = 0x2D

# Bank 2
B2_GYRO_CONFIG_1 = 0x01
B2_ACCEL_CONFIG = 0x14

# AK09916 magnetometer
AK_ADDR = 0x0C
AK_WHO_AM_I = 0x09
AK_WIA2 = 0x01
AK_ST1 = 0x10
AK_HXL = 0x11
AK_ST2 = 0x18
AK_CNTL2 = 0x31
AK_CNTL3 = 0x32

ACCEL_FS_MAP = {
    2: (0x00, 16384.0),
    4: (0x01, 8192.0),
    8: (0x02, 4096.0),
    16: (0x03, 2048.0),
}

GYRO_FS_MAP = {
    250: (0x00, 131.0),
    500: (0x01, 65.5),
    1000: (0x02, 32.8),
    2000: (0x03, 16.4),
}

MAG_UT_PER_LSB = 0.15


class ICM20948:
    """Low-level reader for the ICM-20948 and its AK09916 magnetometer."""

    def __init__(self, bus: int, address: int = 0x68) -> None:
        import smbus2

        self.bus_number = bus
        self.address = address
        self.bus = smbus2.SMBus(bus)

        self._bank = -1
        self.accel_lsb_per_g: float | None = None
        self.gyro_lsb_per_dps: float | None = None

    def close(self) -> None:
        self.bus.close()

    def _set_bank(self, bank: int, address: int | None = None) -> None:
        if address is None:
            address = self.address

        if address == self.address and self._bank == bank:
            return

        self.bus.write_byte_data(
            address,
            REG_BANK_SEL,
            (bank & 0x03) << 4,
        )

        if address == self.address:
            self._bank = bank

        time.sleep(0.001)

    def read_reg(
        self,
        bank: int,
        register: int,
        address: int | None = None,
    ) -> int:
        if address is None:
            address = self.address

        self._set_bank(bank, address)
        return self.bus.read_byte_data(address, register)

    def read_block(
        self,
        bank: int,
        register: int,
        length: int,
        address: int | None = None,
    ) -> bytes:
        if address is None:
            address = self.address

        self._set_bank(bank, address)

        return bytes(
            self.bus.read_i2c_block_data(
                address,
                register,
                length,
            )
        )

    def write_reg(
        self,
        bank: int,
        register: int,
        value: int,
        address: int | None = None,
    ) -> None:
        if address is None:
            address = self.address

        self._set_bank(bank, address)
        self.bus.write_byte_data(address, register, value & 0xFF)

    def who_am_i(self) -> int:
        return self.read_reg(0, B0_WHO_AM_I)

    def initialize(
        self,
        accel_full_scale_g: int = 2,
        gyro_full_scale_dps: int = 250,
    ) -> None:
        if accel_full_scale_g not in ACCEL_FS_MAP:
            raise ValueError(
                f"Unsupported accelerometer range: {accel_full_scale_g}"
            )

        if gyro_full_scale_dps not in GYRO_FS_MAP:
            raise ValueError(
                f"Unsupported gyroscope range: {gyro_full_scale_dps}"
            )

        detected_id = self.who_am_i()
        if detected_id != ICM_WHO_AM_I:
            raise RuntimeError(
                f"Unexpected ICM-20948 WHO_AM_I value: "
                f"0x{detected_id:02X}; expected 0x{ICM_WHO_AM_I:02X}"
            )

        # Reset.
        self.write_reg(0, B0_PWR_MGMT_1, 0x80)
        time.sleep(0.1)

        # Reset invalidates the cached bank.
        self._bank = -1

        # Wake and use the automatic clock source.
        self.write_reg(0, B0_PWR_MGMT_1, 0x01)
        time.sleep(0.05)

        # Enable all accelerometer and gyroscope axes.
        self.write_reg(0, B0_PWR_MGMT_2, 0x00)

        accel_bits, self.accel_lsb_per_g = ACCEL_FS_MAP[
            accel_full_scale_g
        ]
        gyro_bits, self.gyro_lsb_per_dps = GYRO_FS_MAP[
            gyro_full_scale_dps
        ]

        # FCHOICE=1 enables the digital low-pass filter.
        self.write_reg(
            2,
            B2_ACCEL_CONFIG,
            (accel_bits << 1) | 0x01,
        )
        self.write_reg(
            2,
            B2_GYRO_CONFIG_1,
            (gyro_bits << 1) | 0x01,
        )

        time.sleep(0.01)

    @staticmethod
    def _signed_int16_be(high: int, low: int) -> int:
        value = (high << 8) | low
        return value - 65536 if value & 0x8000 else value

    @staticmethod
    def _signed_int16_le(low: int, high: int) -> int:
        value = (high << 8) | low
        return value - 65536 if value & 0x8000 else value

    def read_accel_gyro_temp(self) -> dict:
        if (
            self.accel_lsb_per_g is None
            or self.gyro_lsb_per_dps is None
        ):
            raise RuntimeError("ICM-20948 has not been initialized")

        data = self.read_block(0, B0_ACCEL_XOUT_H, 14)

        raw_values = [
            self._signed_int16_be(data[index], data[index + 1])
            for index in range(0, 12, 2)
        ]

        temperature_raw = self._signed_int16_be(data[12], data[13])

        ax, ay, az, gx, gy, gz = raw_values

        return {
            "accel_g": (
                ax / self.accel_lsb_per_g,
                ay / self.accel_lsb_per_g,
                az / self.accel_lsb_per_g,
            ),
            "gyro_dps": (
                gx / self.gyro_lsb_per_dps,
                gy / self.gyro_lsb_per_dps,
                gz / self.gyro_lsb_per_dps,
            ),
            "temp_c": temperature_raw / 333.87 + 21.0,
        }

    def enable_magnetometer_bypass(self) -> None:
        # Disable the ICM's internal auxiliary I2C master.
        self.write_reg(0, B0_USER_CTRL, 0x00)

        # Expose the AK09916 directly on the main I2C bus.
        self.write_reg(0, B0_INT_PIN_CFG, 0x02)

        time.sleep(0.01)

    def magnetometer_who_am_i(self) -> int:
        return self.bus.read_byte_data(AK_ADDR, AK_WIA2)

    def initialize_magnetometer(self) -> None:
        detected_id = self.magnetometer_who_am_i()

        if detected_id != AK_WHO_AM_I:
            raise RuntimeError(
                f"Unexpected AK09916 WHO_AM_I value: "
                f"0x{detected_id:02X}; expected 0x{AK_WHO_AM_I:02X}"
            )

        # Soft reset.
        self.bus.write_byte_data(AK_ADDR, AK_CNTL3, 0x01)
        time.sleep(0.01)

        # Continuous measurement mode, 100 Hz.
        self.bus.write_byte_data(AK_ADDR, AK_CNTL2, 0x08)
        time.sleep(0.01)

    def read_magnetometer(self, timeout: float = 0.03) -> dict:
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            status_1 = self.bus.read_byte_data(AK_ADDR, AK_ST1)

            if status_1 & 0x01:
                break

            time.sleep(0.002)
        else:
            raise TimeoutError("AK09916 data-ready timeout")

        data = self.bus.read_i2c_block_data(AK_ADDR, AK_HXL, 6)

        # ST2 must be read to release/latch the next measurement.
        status_2 = self.bus.read_byte_data(AK_ADDR, AK_ST2)
        overflow = bool(status_2 & 0x08)

        mx_raw = self._signed_int16_le(data[0], data[1])
        my_raw = self._signed_int16_le(data[2], data[3])
        mz_raw = self._signed_int16_le(data[4], data[5])

        return {
            "mag_ut": (
                mx_raw * MAG_UT_PER_LSB,
                my_raw * MAG_UT_PER_LSB,
                mz_raw * MAG_UT_PER_LSB,
            ),
            "overflow": overflow,
        }