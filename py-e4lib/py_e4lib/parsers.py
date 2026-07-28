# py_e4lib/parsers.py

import struct
from collections import deque
from typing import List, Tuple, Optional
from .constants import FIR_COEFFICIENTS, BVP_SCALE_FACTOR, TEMP_CALIBRATION


class BVPParser:
    """
    Stateful BVP parser. Needs to maintain filter buffers between packets.
    Usage: parser = BVPParser(); values = parser.parse(data)
    """

    def __init__(self):
        self._green_offset = 0
        self._red_offset = 0
        self._fir1_buffer = deque([0.0] * len(FIR_COEFFICIENTS), maxlen=len(FIR_COEFFICIENTS))
        self._fir2_buffer = deque([0.0] * len(FIR_COEFFICIENTS), maxlen=len(FIR_COEFFICIENTS))
        self._fir3_buffer = deque([0.0] * len(FIR_COEFFICIENTS), maxlen=len(FIR_COEFFICIENTS))
        self._kalman_p = 1.0
        self._kalman_x = 0.0
        self._kalman_q = 0.01
        self._kalman_r = 0.1

    def parse(self, data: bytes) -> Optional[List[float]]:
        """Parse BVP packet into list of BVP values."""
        if len(data) < 20:
            return None

        # Stage 1: Decode
        decoded = self._decode_packet(data)

        # Stage 2-7: Process through filter chain
        bvp_readings = []
        for i in range(min(11, len(decoded) // 2)):
            green = decoded[i * 2]
            red = decoded[i * 2 + 1]

            self._green_offset += red
            self._red_offset += green

            # FIR Filter 1
            self._fir1_buffer.append(float(red))
            filtered_red = sum(c * s for c, s in zip(FIR_COEFFICIENTS, self._fir1_buffer))

            # FIR Filter 2
            weighted = (red + green * 10.0) / 11.0
            self._fir2_buffer.append(weighted)
            filtered_weighted = sum(c * s for c, s in zip(FIR_COEFFICIENTS, self._fir2_buffer))

            # Kalman Filter
            kalman_out = self._kalman_filter(filtered_red, filtered_weighted)

            # FIR Filter 3
            self._fir3_buffer.append(kalman_out)
            filtered_kalman = sum(c * s for c, s in zip(FIR_COEFFICIENTS, self._fir3_buffer))

            bvp = round(-filtered_kalman * BVP_SCALE_FACTOR, 2)
            bvp_readings.append(bvp)

        return bvp_readings if bvp_readings else None

    def _decode_packet(self, data: bytes) -> List[int]:
        """Decode 7-bit packed delta encoding."""
        decoded = []
        uVar28 = 0

        for uVar37 in range(0x14):
            bVar21 = data[uVar37]
            iVar31 = uVar37 % 7
            uVar36 = iVar31 + 1

            uVar28 = (bVar21 >> uVar36) | uVar28

            output_val = uVar28 & 0x7F
            if output_val & 0x40:
                output_val |= 0x80
            if output_val > 127:
                output_val -= 256
            decoded.append(output_val)

            mask = (1 << uVar36) - 1
            uVar28 = ((bVar21 & mask) << (6 - iVar31)) & 0xFF

            if uVar36 == 7:
                output_val = uVar28 & 0x7F
                if output_val & 0x40:
                    output_val |= 0x80
                if output_val > 127:
                    output_val -= 256
                decoded.append(output_val)
                uVar28 = 0

        final_val = data[0x13] & 0x3F
        if final_val & 0x20:
            final_val |= 0xC0
        if final_val > 127:
            final_val -= 256
        decoded.append(final_val)

        return decoded

    def _kalman_filter(self, measurement1: float, measurement2: float) -> float:
        """Kalman filter combining two measurements."""
        measurement = (measurement1 + measurement2) / 2.0

        # Prediction
        p_pred = self._kalman_p + self._kalman_q

        # Update
        kalman_gain = p_pred / (p_pred + self._kalman_r)
        self._kalman_x = self._kalman_x + kalman_gain * (measurement - self._kalman_x)
        self._kalman_p = (1 - kalman_gain) * p_pred

        # Adaptive covariance
        diff1 = abs(measurement1 - measurement2)
        if diff1 > 20:
            self._kalman_p = min(self._kalman_p * 1.2, 10.0)
        else:
            self._kalman_p = max(self._kalman_p * 0.95, 0.01)

        return self._kalman_x


def parse_gsr(data: bytes) -> Optional[List[float]]:
    """
    Parse GSR/EDA packet into list of values in microsiemens.
    Uses 24-bit big endian encoding, 6 samples per packet.
    """
    if len(data) < 20:
        return None

    readings = []
    i = 0

    while i + 3 <= len(data) - 2:
        byte1 = data[i]
        byte2 = data[i + 1]
        byte3 = data[i + 2]

        raw_value = (byte1 << 16) | (byte2 << 8) | byte3
        eda_microsiemens = 1000000.0 / raw_value if raw_value > 0 else 0

        readings.append(eda_microsiemens)
        i += 3

    return readings if readings else None


def parse_temp(data: bytes) -> Optional[List[float]]:
    """
    Parse temperature packet into list of values in Celsius.
    Uses unsigned 16-bit little endian encoding, 4 samples per packet.
    """
    if len(data) < 12:
        return None

    temp_readings = []
    i = 0

    while i < 8:
        raw = struct.unpack_from('<H', data, i)[0]
        temp = ((raw * 0.02) - 276.0) + TEMP_CALIBRATION
        temp_readings.append(temp)
        i += 2

    return temp_readings if temp_readings else None


def parse_acc(data: bytes) -> Optional[List[Tuple[int, int, int]]]:
    """
    Parse accelerometer packet into list of (x, y, z) tuples.
    Values are raw, divide by 64.0 to get g-force.
    """
    acc_readings = []
    i = 0

    while i + 3 <= len(data):
        try:
            x, y, z = struct.unpack_from('<bbb', data, i)
            acc_readings.append((x, y, z))
            i += 3
        except struct.error:
            break

    return acc_readings if acc_readings else None