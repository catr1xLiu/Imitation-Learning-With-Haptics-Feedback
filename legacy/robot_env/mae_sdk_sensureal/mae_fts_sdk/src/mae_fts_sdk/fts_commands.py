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


import enum

from .fts_constants import FTS_TX_BYTE_ORDER
from .fts_responses import *

__all__ = [
    "FtsCommand",
    "FTS_COMMAND_RESPONSE",
    "FTS_STREAMING_COMMANDS",
]


class FtsCommand(enum.Enum):
    # Basic commands
    STREAM_STOP = 0x00
    STREAM_FT_START = 0x01

    STREAM_FT_PEAKS_START = 0x10
    RESET_PEAKS = 0x12

    BIAS_SET = 0x30
    BIAS_READ = 0x31
    BIAS_RESET = 0x32

    SAMPLING_PERIOD_READ = 0x40
    SAMPLING_PERIOD_SET = 0x41
    SAMPLING_PERIOD_RESET = 0x42
    SAMPLING_PERIOD_READ_STRING = 0x43

    # TODO: Currently not available.
    # BUFFER_SIZE_READ = 0x50
    # BUFFER_SIZE_SET = 0x51
    # BUFFER_SIZE_RESET = 0x52

    IP_ADDRESS_READ = 0x70
    IP_ADDRESS_SET = 0x71
    IP_ADDRESS_RESET = 0x72
    IP_ADDRESS_READ_STRING = 0x73

    UDP_PORT_READ = 0x80

    TRANSDUCER_SET = 0x81
    TRANSDUCER_RESET = 0x84

    TOOL_TRANSFORMATION_READ_STRING = 0x90
    TOOL_TX_SET = 0x91
    TOOL_TY_SET = 0x92
    TOOL_TZ_SET = 0x93
    TOOL_RX_SET = 0x94
    TOOL_RY_SET = 0x95
    TOOL_RZ_SET = 0x96
    TOOL_TRANSFORMATION_RESET = 0x97

    FIRMWARE_VERSION_READ = 0x0100
    FIRMWARE_STATUS_CLEAR = 0x0101

    SENSOR_INFO_READ = 0x0200

    SETTINGS_RESET = 0xC000
    SETTINGS_SAVE = 0xC001

    def to_bytes(self, length=2, byteorder=FTS_TX_BYTE_ORDER, signed=False):
        # NOTE: The firmware uses little-endian to store values in memory.
        return self.value.to_bytes(length=length, byteorder=byteorder, signed=signed)


FTS_COMMAND_RESPONSE = {
    FtsCommand.BIAS_READ: FTResponse,
    FtsCommand.BIAS_RESET: None,
    FtsCommand.BIAS_SET: None,
    FtsCommand.SAMPLING_PERIOD_READ: UInt32Response,
    FtsCommand.SAMPLING_PERIOD_SET: None,
    FtsCommand.SAMPLING_PERIOD_RESET: None,
    FtsCommand.SAMPLING_PERIOD_READ_STRING: StringResponse,
    FtsCommand.FIRMWARE_VERSION_READ: UInt32Response,
    FtsCommand.FIRMWARE_STATUS_CLEAR: None,
    FtsCommand.IP_ADDRESS_READ: IpV4Response,
    FtsCommand.IP_ADDRESS_SET: None,
    FtsCommand.IP_ADDRESS_RESET: None,
    FtsCommand.IP_ADDRESS_READ_STRING: StringResponse,
    FtsCommand.STREAM_FT_PEAKS_START: FTPeaksResponse,
    FtsCommand.RESET_PEAKS: None,
    FtsCommand.UDP_PORT_READ: UInt32Response,
    FtsCommand.TRANSDUCER_SET: None,
    FtsCommand.TRANSDUCER_RESET: None,
    FtsCommand.TOOL_TRANSFORMATION_READ_STRING: StringResponse,
    FtsCommand.TOOL_TX_SET: None,
    FtsCommand.TOOL_TY_SET: None,
    FtsCommand.TOOL_TZ_SET: None,
    FtsCommand.TOOL_RX_SET: None,
    FtsCommand.TOOL_RY_SET: None,
    FtsCommand.TOOL_RZ_SET: None,
    FtsCommand.TOOL_TRANSFORMATION_RESET: None,
    FtsCommand.STREAM_FT_START: FTResponse,
    FtsCommand.SENSOR_INFO_READ: StringResponse,
    FtsCommand.SETTINGS_RESET: None,
    FtsCommand.SETTINGS_SAVE: None,
}

FTS_STREAMING_COMMANDS = [
    str(command)
    for command in [
        FtsCommand.STREAM_FT_START,
        FtsCommand.STREAM_FT_PEAKS_START,
    ]
]
