########################################################################################
# Copyright (C) 2024 MAE Robotics Inc. - All Rights Reserved
#
# This source code is protected under international copyright laws. All rights
# reserved by the copyright holders. This file is proprietary and may only be
# accessed, used, or distributed by authorized individuals with explicit permission
# from the copyright holders. Unauthorized access, use, or distribution is strictly
# prohibited. If you have received this file in error, please notify the copyright
# holders immediately and delete all copies of this file.
#
# Visit: www.maerobotics.com
########################################################################################

__all__ = [
    "Ascii",
    "FTS_DEFAULT_ARGUMENT_SIZE",
    "FTS_TX_BYTE_ORDER",
    "FTS_TX_BUFFER_SIZE_SERIAL",
    "FTS_TX_REQUEST_MAX_BYTES_UDP",
    "FTS_RX_BUFFER_MAX_SIZE",
]


FTS_DEFAULT_ARGUMENT_SIZE = 4
FTS_TX_BYTE_ORDER = "big"


FTS_TX_BUFFER_SIZE_SERIAL = 16
"""
Maximum amount of bytes transmitted through serial communication.
These bytes must follow a protocol based on the following layout.
    [Ascii: SOH, STX]    [2 bytes]
    [Command]            [2 bytes]
    [Command Arguments]  [4 bytes]
    [Ascii: ETX, EOT]    [2 bytes]
    [Padding]            [(TX_BUFFER_SIZE_SERIAL-10) bytes]
"""

FTS_TX_REQUEST_MAX_BYTES_UDP = 128
"""
Maximum amount of bytes transmitted through udp communication.
    [Command]            [2 bytes]
    [Command Arguments]  [4+ bytes]
"""

FTS_RX_BUFFER_MAX_SIZE = 2048
"""
Maximum amount of bytes received from the sensor.
"""


import enum


class Ascii(enum.Enum):
    SOH = 0x01
    "Start of Header"

    STX = 0x02
    "Start of Text"

    ETX = 0x03
    "End of Text"

    EOT = 0x04
    "End of Transmission"

    @property
    def byte(self):
        # NOTE: The firmware uses little-endian to store values in memory.
        return self.value.to_bytes(length=1, byteorder=FTS_TX_BYTE_ORDER, signed=False)
