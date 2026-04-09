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


import numpy as np
from robot_env.mae_sdk_sensureal.mae_sdk.src.mae_sdk.str_utils import convert_int_to_ip_string


class FTResponse: ...


class FTResponse:
    """When transmitting feedback, the data sent can have any arbitrary size.
    The feedback has the following layout:

    [Ascii: SOH, STX] [2 bytes]\n
    [Payload Size]    [2 bytes]\n
    [Payload]         [N bytes]\n
    [Ascii: ETX, EOT] [2 bytes]\n

    NOTE: The transmission of multiple bytes follows Most Significant Byte
    (MSB) first.

    NOTE: The FTResponse is to be feed with the payload bytes.

    The layout of the payload bytes are as follows:

    [Command Id]        [2 bytes]\n
    [Timestamp Count]   [8 bytes]\n
    [Status]            [4 bytes]\n
    [Sample Count]      [4 bytes]\n
    [Sample Id]         [4 bytes]\n


    [Force X]           [4 bytes]\n
    [Force Y]           [4 bytes]\n
    [Force Z]           [4 bytes]\n
    [Torque X]          [4 bytes]\n
    [Torque Y]          [4 bytes]\n
    [Torque Z]          [4 bytes].
    """

    EXPECTED_SIZE = 2 + 8 + 3 * 4 + 6 * 4

    def factory(read_bytes: bytes) -> list[FTResponse]:
        """Used when receiving a buffer of samples"""

        if len(read_bytes) % FTResponse.EXPECTED_SIZE != 0:
            message = (
                f"{FTResponse.__name__} expected multiple of "
                f"{FTResponse.EXPECTED_SIZE} bytes, "
                f"but {len(read_bytes)} bytes were given and it is not "
                f"a multiple of {FTResponse.EXPECTED_SIZE}."
            )
            raise ValueError(message)

        output = []
        for i in range(0, len(read_bytes), FTResponse.EXPECTED_SIZE):
            j = i + FTResponse.EXPECTED_SIZE
            sensor_response = FTResponse(read_bytes[i:j])
            output.append(sensor_response)

        return output

    def __init__(self, read_bytes: bytes) -> None:
        if len(read_bytes) != self.EXPECTED_SIZE:
            message = (
                f"{self.__class__.__name__} expected {self.EXPECTED_SIZE} bytes,"
                f"but only {len(read_bytes)} bytes were given."
            )
            raise ValueError(message)

        start_byte, end_byte = 0, 0

        start_byte, end_byte = end_byte, end_byte + 2
        data_type = np.dtype("uint16").newbyteorder(">")
        self.command_id = np.frombuffer(
            read_bytes[start_byte:end_byte], dtype=data_type
        ).tolist()[0]

        start_byte, end_byte = end_byte, end_byte + 8
        data_type = np.dtype("uint64").newbyteorder(">")
        self.timestamp_us = np.frombuffer(
            read_bytes[start_byte:end_byte], dtype=data_type
        ).tolist()[0]

        start_byte, end_byte = end_byte, end_byte + 3 * 4
        data_type = np.dtype("uint32").newbyteorder(">")
        self.status, self.sample_count, self.sample_id = np.frombuffer(
            read_bytes[start_byte:end_byte], dtype=data_type
        ).tolist()

        start_byte, end_byte = end_byte, end_byte + 6 * 4
        sample_bytes = read_bytes[start_byte:end_byte]
        sample_type = np.dtype("int32").newbyteorder(">")
        ft_sample = np.frombuffer(sample_bytes, dtype=sample_type) / 1e6
        self.ft_sample = ft_sample.tolist()

    def __str__(self) -> str:
        output = [
            f"command_id=0x{self.command_id:X}",
            f"timestamp_us={self.timestamp_us}",
            f"status=0x{self.status:X}",
            f"sample_id={self.sample_id}",
            f"sample_count={self.sample_count}",
            f"ft_sample={self.ft_sample}",
        ]

        return f"{self.__class__.__name__}({' '.join(output)})"

    def __repr__(self) -> str:
        return f"<{str(self)} at {hex(id(self))}>"

    @property
    def fx(self) -> int:
        return self.ft_sample[0]

    @property
    def fy(self) -> int:
        return self.ft_sample[1]

    @property
    def fz(self) -> int:
        return self.ft_sample[2]

    @property
    def tx(self) -> int:
        return self.ft_sample[3]

    @property
    def ty(self) -> int:
        return self.ft_sample[4]

    @property
    def tz(self) -> int:
        return self.ft_sample[5]


class FTPeaksResponse: ...


class FTPeaksResponse:
    """When transmitting feedback, the data sent can have any arbitrary size.
    The feedback has the following layout:

    [Ascii: SOH, STX] [2 bytes]\n
    [Payload Size]    [2 bytes]\n
    [Payload]         [N bytes]\n
    [Ascii: ETX, EOT] [2 bytes]\n

    NOTE: The transmission of multiple bytes follows Most Significant Byte
    (MSB) first.

    NOTE: The FTPeaksResponse is to be feed with the payload bytes.

    The layout of the payload bytes are as follows:

    [Command Id]        [2 bytes]\n
    [Timestamp Count]   [8 bytes]\n
    [Status]            [4 bytes]\n
    [Sample Count]      [4 bytes]\n
    [Sample Id]         [4 bytes]\n

    [Max Force X]       [4 bytes]\n
    [Max Force Y]       [4 bytes]\n
    [Max Force Z]       [4 bytes]\n
    [Max Torque X]      [4 bytes]\n
    [Max Torque Y]      [4 bytes]\n
    [Max Torque Z]      [4 bytes]\n.

    [Min Force X]       [4 bytes]\n
    [Min Force Y]       [4 bytes]\n
    [Min Force Z]       [4 bytes]\n
    [Min Torque X]      [4 bytes]\n
    [Min Torque Y]      [4 bytes]\n
    [Min Torque Z]      [4 bytes].
    """

    EXPECTED_SIZE = 2 + 8 + 3 * 4 + 2 * 6 * 4

    def factory(read_bytes: bytes) -> list[FTPeaksResponse]:
        """Used when receiving a buffer of samples"""

        if len(read_bytes) % FTPeaksResponse.EXPECTED_SIZE != 0:
            message = (
                f"{FTPeaksResponse.__name__} expected multiple of "
                f"{FTPeaksResponse.EXPECTED_SIZE} bytes, "
                f"but {len(read_bytes)} bytes were given and it is not "
                f"a multiple of {FTPeaksResponse.EXPECTED_SIZE}."
            )
            raise ValueError(message)

        output = []
        for i in range(0, len(read_bytes), FTPeaksResponse.EXPECTED_SIZE):
            j = i + FTPeaksResponse.EXPECTED_SIZE
            sensor_response = FTPeaksResponse(read_bytes[i:j])
            output.append(sensor_response)

        return output

    def __init__(self, read_bytes: bytes) -> None:
        if len(read_bytes) != self.EXPECTED_SIZE:
            message = (
                f"{self.__class__.__name__} expected {self.EXPECTED_SIZE} bytes,"
                f"but only {len(read_bytes)} bytes were given."
            )
            raise ValueError(message)

        start_byte, end_byte = 0, 0

        start_byte, end_byte = end_byte, end_byte + 2
        data_type = np.dtype("uint16").newbyteorder(">")
        self.command_id = np.frombuffer(
            read_bytes[start_byte:end_byte], dtype=data_type
        ).tolist()[0]

        start_byte, end_byte = end_byte, end_byte + 8
        data_type = np.dtype("uint64").newbyteorder(">")
        self.timestamp_us = np.frombuffer(
            read_bytes[start_byte:end_byte], dtype=data_type
        ).tolist()[0]

        start_byte, end_byte = end_byte, end_byte + 3 * 4
        data_type = np.dtype("uint32").newbyteorder(">")
        self.status, self.sample_count, self.sample_id = np.frombuffer(
            read_bytes[start_byte:end_byte], dtype=data_type
        ).tolist()

        start_byte, end_byte = end_byte, end_byte + 6 * 4
        sample_bytes = read_bytes[start_byte:end_byte]
        sample_type = np.dtype("int32").newbyteorder(">")
        ft_sample_max = np.frombuffer(sample_bytes, dtype=sample_type) / 1e6
        self.ft_sample_max = ft_sample_max.tolist()

        start_byte, end_byte = end_byte, end_byte + 6 * 4
        sample_bytes = read_bytes[start_byte:end_byte]
        sample_type = np.dtype("int32").newbyteorder(">")
        ft_sample_min = np.frombuffer(sample_bytes, dtype=sample_type) / 1e6
        self.ft_sample_min = ft_sample_min.tolist()

    def __str__(self) -> str:
        output = [
            f"command_id=0x{self.command_id:X}",
            f"timestamp_us={self.timestamp_us}",
            f"status=0x{self.status:X}",
            f"sample_id={self.sample_id}",
            f"sample_count={self.sample_count}",
            f"ft_sample_max={self.ft_sample_max}",
            f"ft_sample_min={self.ft_sample_min}",
        ]

        return f"{self.__class__.__name__}({' '.join(output)})"

    def __repr__(self) -> str:
        return f"<{str(self)} at {hex(id(self))}>"


class UInt32Response:
    """When transmitting feedback, the data sent can have any arbitrary size.
    The feedback has the following layout:

    [Ascii: SOH, STX] [2 bytes]\n
    [Payload Size]    [2 bytes]\n
    [Payload]         [N bytes]\n
    [Ascii: ETX, EOT] [2 bytes]\n

    NOTE: The transmission of multiple bytes follows Most Significant Byte
    (MSB) first.

    NOTE: The UInt32Response is to be feed with the payload bytes.

    The layout of the payload bytes is as follows:

    [Command Id]        [2 bytes]\n
    [Timestamp Count]   [8 bytes]\n
    [Status]            [4 bytes]\n
    [Value]             [4 bytes]\n
    """

    EXPECTED_SIZE = 2 + 8 + 2 * 4

    def __init__(self, read_bytes: bytes) -> None:
        if len(read_bytes) != self.EXPECTED_SIZE:
            message = (
                f"{self.__class__.__name__} expected  at least {self.EXPECTED_SIZE} bytes,"
                f"but only {len(read_bytes)} bytes were given."
            )
            raise ValueError(message)

        start_byte, end_byte = 0, 0

        start_byte, end_byte = end_byte, end_byte + 2
        data_type = np.dtype("uint16").newbyteorder(">")
        self.command_id = np.frombuffer(
            read_bytes[start_byte:end_byte], dtype=data_type
        ).tolist()[0]

        start_byte, end_byte = end_byte, end_byte + 8
        data_type = np.dtype("uint64").newbyteorder(">")
        self.timestamp_us = np.frombuffer(
            read_bytes[start_byte:end_byte], dtype=data_type
        ).tolist()[0]

        start_byte, end_byte = end_byte, end_byte + 2 * 4
        data_type = np.dtype("uint32").newbyteorder(">")
        self.status, self.value = np.frombuffer(
            read_bytes[start_byte:end_byte], dtype=data_type
        ).tolist()

    def __str__(self) -> str:
        output = [
            f"command_id=0x{self.command_id:X}",
            f"timestamp_us={self.timestamp_us}",
            f"status=0x{self.status:X}",
            f"value={self.value}",
        ]
        return f"{self.__class__.__name__}({' '.join(output)})"

    def __repr__(self) -> str:
        return f"<{str(self)} at {hex(id(self))}>"


class IpV4Response:
    """When transmitting feedback, the data sent can have any arbitrary size.
    The feedback has the following layout:

    [Ascii: SOH, STX] [2 bytes]\n
    [Payload Size]    [2 bytes]\n
    [Payload]         [N bytes]\n
    [Ascii: ETX, EOT] [2 bytes]\n

    NOTE: The transmission of multiple bytes follows Most Significant Byte
    (MSB) first.

    NOTE: The IpV4Response is to be feed with the payload bytes.

    The layout of the payload bytes is as follows:

    [Command Id]        [2 bytes]\n
    [Timestamp Count]   [8 bytes]\n
    [Status]            [4 bytes]\n
    [Value]             [4 bytes]\n
    """

    EXPECTED_SIZE = 2 + 8 + 2 * 4

    def __init__(self, read_bytes: bytes) -> None:
        if len(read_bytes) != self.EXPECTED_SIZE:
            message = (
                f"{self.__class__.__name__} expected {self.EXPECTED_SIZE} bytes,"
                f"but only {len(read_bytes)} bytes were given."
            )
            raise ValueError(message)

        start_byte, end_byte = 0, 0

        start_byte, end_byte = end_byte, end_byte + 2
        data_type = np.dtype("uint16").newbyteorder(">")
        self.command_id = np.frombuffer(
            read_bytes[start_byte:end_byte], dtype=data_type
        ).tolist()[0]

        start_byte, end_byte = end_byte, end_byte + 8
        data_type = np.dtype("uint64").newbyteorder(">")
        self.timestamp_us = np.frombuffer(
            read_bytes[start_byte:end_byte], dtype=data_type
        ).tolist()[0]

        start_byte, end_byte = end_byte, end_byte + 2 * 4
        data_type = np.dtype("uint32").newbyteorder(">")
        self.status, self.ipv4_raw = np.frombuffer(
            read_bytes[start_byte:end_byte], dtype=data_type
        ).tolist()

        self.ip_v4 = convert_int_to_ip_string(self.ipv4_raw)

    def __str__(self) -> str:
        output = [
            f"command_id=0x{self.command_id:X}",
            f"timestamp_us={self.timestamp_us}",
            f"status=0x{self.status:X}",
            f"ipv4={self.ip_v4}",
            f"ipv4_raw={self.ipv4_raw}",
        ]
        return f"{self.__class__.__name__}({' '.join(output)})"

    def __repr__(self) -> str:
        return f"<{str(self)} at {hex(id(self))}>"


class StringResponse:
    """When transmitting feedback, the data sent can have any arbitrary size.
    The feedback has the following layout:

    [Ascii: SOH, STX] [2 bytes]\n
    [Payload Size]    [2 bytes]\n
    [Payload]         [N bytes]\n
    [Ascii: ETX, EOT] [2 bytes]\n

    NOTE: The transmission of multiple bytes follows Most Significant Byte
    (MSB) first.

    NOTE: The StringResponse is to be feed with the payload bytes.

    The layout of the payload bytes is as follows:

    [Command Id]        [2 bytes]\n
    [Timestamp Count]   [8 bytes]\n
    [Status]            [4 bytes]\n
    [String]            [N-14 bytes]
    """

    EXPECTED_SIZE = 2 + 8 + 2 * 4

    def __init__(self, read_bytes: bytes) -> None:
        if len(read_bytes) <= self.EXPECTED_SIZE:
            message = (
                f"{self.__class__.__name__} expected at least {self.EXPECTED_SIZE} bytes,"
                f"but only {len(read_bytes)} bytes were given."
            )
            raise ValueError(message)

        start_byte, end_byte = 0, 0

        start_byte, end_byte = end_byte, end_byte + 2
        data_type = np.dtype("uint16").newbyteorder(">")
        self.command_id = np.frombuffer(
            read_bytes[start_byte:end_byte], dtype=data_type
        ).tolist()[0]

        start_byte, end_byte = end_byte, end_byte + 8
        data_type = np.dtype("uint64").newbyteorder(">")
        self.timestamp_us = np.frombuffer(
            read_bytes[start_byte:end_byte], dtype=data_type
        ).tolist()[0]

        start_byte, end_byte = end_byte, end_byte + 4
        data_type = np.dtype("uint32").newbyteorder(">")
        self.status = np.frombuffer(
            read_bytes[start_byte:end_byte], dtype=data_type
        ).tolist()[0]

        start_byte = end_byte
        self.string = read_bytes[start_byte:].decode("utf-8")

    def __str__(self) -> str:
        output = [
            f"command_id=0x{self.command_id:X}",
            f"timestamp_us={self.timestamp_us}",
            f"status=0x{self.status:X}",
            f'string="{self.string}"',
        ]
        return f"{self.__class__.__name__}({' '.join(output)})"

    def __repr__(self) -> str:
        return f"<{str(self)} at {hex(id(self))}>"
