"""Incremental HFI-A9 serial decoder; no configuration commands are sent.

Frame lengths / float layout follow the manufacturer's reference:
https://github.com/HANDS-FREE/handsfree_ros_imu/blob/master/scripts/hfi_a9_ros.py
CRC-16/MODBUS over length+payload, stored little endian, was checked against
captured frames from this HFI-A9. Tests include original sensor bytes.
Values retain the sensor's axes and raw units; this is not a navigation driver.
"""

import struct


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


class A9Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0
        self.discarded_bytes = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 3:
            start = self.buffer.find(b'\xaa\x55')
            if start < 0:
                keep = 1 if self.buffer[-1] == 0xAA else 0
                self.discarded_bytes += len(self.buffer) - keep
                self.buffer[:] = self.buffer[-1:] if keep else b''
                break
            if start:
                self.discarded_bytes += start
                del self.buffer[:start]
            if len(self.buffer) < 3:
                break
            payload_length = self.buffer[2]
            if payload_length not in (0x2C, 0x14):
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            length = payload_length + 5
            if len(self.buffer) < length:
                break
            frame = bytes(self.buffer[:length])
            if crc16(frame[2:-2]) != int.from_bytes(frame[-2:], 'little'):
                self.crc_errors += 1
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            del self.buffer[:length]
            count = 9 if payload_length == 0x2C else 3
            frames.append({
                'kind': 'motion' if count == 9 else 'attitude',
                'stamp': int.from_bytes(frame[7:11], 'little'),
                'values': struct.unpack_from(f'<{count}f', frame, 11),
            })
        return frames
