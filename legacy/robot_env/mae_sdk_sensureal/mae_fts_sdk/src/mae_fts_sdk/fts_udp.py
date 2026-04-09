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


import logging
import socket
import time
from logging import Logger

import numpy as np
from robot_env.mae_sdk_sensureal.mae_sdk.src.mae_sdk import log_utils
from robot_env.mae_sdk_sensureal.mae_sdk.src.mae_sdk.communication import CommunicationInterface

from .fts_commands import FtsCommand
from .fts_constants import (
    FTS_DEFAULT_ARGUMENT_SIZE,
    FTS_RX_BUFFER_MAX_SIZE,
    FTS_TX_BYTE_ORDER,
    FTS_TX_REQUEST_MAX_BYTES_UDP,
)

__all__ = ["UdpCommunication"]


class UdpCommunication(CommunicationInterface):
    def __init__(
        self,
        ip_address: str,
        port: int,
        timeout_sec: float = 0.1,
        log_level: int | str = None,
    ) -> None:
        """Initializes the UdpCommunication instance.

        Args:
            ip_address (str):
                The IP address of the sensor to communicate with.
            port (int):
                The port number of the sensor to communicate with.
            timeout_sec (float, optional):
                The timeout duration in seconds for socket operations. Defaults to 0.1.
            log_level (int | str, optional):
                The logging level to set for the logger. Defaults to None.
        """

        super().__init__(timeout_sec, log_level)

        self._sensor_ip = ip_address
        self._sensor_port = port
        self._sensor_address = (ip_address, port)

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._socket.bind(("", 0))  # Bind to an ephemeral port chosen by the OS
        # local_port = self._socket.getsockname()[1]  # Get the port number assigned
        # print(f"Local port used for sending: {local_port}")

        # Ensuring blocking is set to false to minimize wait times.
        # See https://dnmtechs.com/pythons-socket-recv-behavior-for-non-blocking-sockets-with-timeout/
        self._socket.setblocking(False)

        self._socket.settimeout(timeout_sec)

        self.flush_receive()

    def disconnect(self):
        """UDP communication is connection less. However, calling this function
        will conveniently flush data in the receive buffer."""
        self.flush_receive()

    def flush_receive(self) -> None:
        """Flushes data from the receive buffer."""
        is_blocking = self._socket.getblocking()
        timeout = self._socket.gettimeout()
        self._socket.setblocking(False)

        try:
            while True:
                # Read and discard data from the socket buffer
                data, addr = self._socket.recvfrom(FTS_RX_BUFFER_MAX_SIZE)

        except BlockingIOError:
            # No more data to read, buffer is flushed
            pass

        self._socket.setblocking(is_blocking)
        self._socket.settimeout(timeout)

    def send_request_bytes(self, request_bytes: bytes) -> None:
        """Sends request to device.

        Args:
            request (bytes): Request to send.
        """
        self._socket.sendto(request_bytes, self._sensor_address)
        self.get_logger().debug(f"Sent {len(request_bytes)} bytes.")

    def send_request(
        self,
        command: FtsCommand,
        argument: int | np.ndarray[np.float32] | bytes = bytes(
            FTS_DEFAULT_ARGUMENT_SIZE
        ),
        pause=False,
        pause_on_debug=False,
    ) -> None:
        """Sends command to device.

        Args:
            command (FtsCommand):
                MAE FTS Command.
            argument (int | bytes | np.ndarray], optional):
                argument as big endian bytes, i.e. Most Significant Byte First
                (MSB). This argument must have at most 4 bytes. Defaults to
                bytes(FTS_DEFAULT_ARGUMENT_SIZE).
            pause (bool, optional): _description_. Defaults to False.
            pause_on_debug (bool, optional):
                If True and log level is `logging.DEBUG`, the code will pause
                and require user intervention prior to sending request to the
                device. Defaults to False.

        Raises:
            TypeError
            ValueError
        """
        if not isinstance(argument, (int, np.ndarray, bytes)):
            raise TypeError(
                f"Expected types for arguments are: (int, np.ndarray, bytes). "
                f"Got {type(argument)}"
            )

        if isinstance(argument, int):
            argument = argument.to_bytes(
                length=FTS_DEFAULT_ARGUMENT_SIZE,
                byteorder=FTS_TX_BYTE_ORDER,
                signed=False,
            )

        if isinstance(argument, np.ndarray):
            # Using `>` for big endian given network standard of using MSB.
            argument = argument.astype(np.float32).newbyteorder(">").tobytes()

        request_bytes = bytes()
        request_bytes += command.to_bytes()
        request_bytes += argument

        if len(request_bytes) > FTS_TX_REQUEST_MAX_BYTES_UDP:
            raise ValueError(
                f"Request data should have maximum size of {FTS_TX_REQUEST_MAX_BYTES_UDP}"
            )

        message = f"Sending {len(request_bytes)} bytes via UDP: Bytes["
        message += " ".join(f"{byte:02X}" for byte in request_bytes)
        message += "]."

        logger = self.get_logger()
        logger.debug(message)

        if (logger.isEnabledFor(logging.DEBUG) and pause_on_debug) or pause:
            input("(Press Enter to Continue ...)")

        self.send_request_bytes(request_bytes)

    def waits_response_bytes(self) -> bytes | None:
        """Waits response from the device.

        Returns:
            bytes | None: if successful, or None if timed-out or an error occurred.
        """

        try:
            while True:
                data, address = self._socket.recvfrom(FTS_RX_BUFFER_MAX_SIZE)
                if address == self._sensor_address:
                    return data

        except TimeoutError:
            return None
