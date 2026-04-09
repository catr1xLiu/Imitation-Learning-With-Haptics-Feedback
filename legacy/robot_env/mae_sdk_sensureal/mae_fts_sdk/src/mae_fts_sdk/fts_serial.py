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
import threading
import time

import numpy as np
import serial as pyserial
import serial.tools.list_ports
from robot_env.mae_sdk_sensureal.mae_sdk.src.mae_sdk import log_utils
from robot_env.mae_sdk_sensureal.mae_sdk.src.mae_sdk.communication import CommunicationInterface
from robot_env.mae_sdk_sensureal.mae_sdk.src.mae_sdk.log_utils import print_on_keyboard_exception

from .fts_commands import FtsCommand

# from .fts_communication import Communication
from .fts_constants import (
    FTS_DEFAULT_ARGUMENT_SIZE,
    FTS_TX_BUFFER_SIZE_SERIAL,
    FTS_TX_BYTE_ORDER,
    Ascii,
)

__all__ = [
    "SerialCommunication",
    "prompt_user_port_selection",
]


def _get_logger() -> logging.Logger:
    return log_utils.get_logger(__name__)


def prompt_user_port_selection(logging_level: int | str = None) -> str:
    original_logging_level = _get_logger().level
    if logging_level is not None:
        _get_logger().setLevel(logging_level)

    ports = pyserial.tools.list_ports.comports()
    port_id = 0 if len(ports) == 1 else None

    while len(ports) > 1:
        print("Available COM ports:")
        for i, port in enumerate(ports):
            print(f"{i} - {port.device}")

        print("Select COM port: ")
        try:
            port_id = int(input())
            if port_id < len(ports):
                return ports[port_id].device

            print("Invalid port selection.")

        except KeyboardInterrupt:
            print_on_keyboard_exception()

            _get_logger().setLevel(original_logging_level)  # Restore logging level
            return None

        except Exception:
            ...

    _get_logger().setLevel(original_logging_level)  # Restore logging level
    return ports[0].device


# TODO: Remove lock, since it is not needed in this case.
class SerialCommunication(CommunicationInterface):
    """SerialCommunication objects implement a lock, making them safe to be
    shared among multiple threads.
    """

    def __init__(
        self,
        port: int,
        timeout_sec: float = 0.1,
        log_level: int | str = None,
    ) -> None:
        """Constructor of SerialCommunication.
        Args:
            port (int): Serial Port.
            timeout_sec (int | float, optional):
                Serial interface timeout. Defaults to 0.1.
            log_level (int | str, optional): Defaults to None.
        """

        super().__init__(timeout_sec, log_level)

        self._connection = None
        self._port = port
        self._lock = threading.Lock()

    def __del__(self) -> None:
        if self.is_connected:
            self.flush_receive()
            self.flush_transmit()
            self.disconnect()

    @property
    def is_connected(self) -> bool:
        with self._lock:
            if self._connection is None:
                return False

            return self._connection.is_open

    def connect(self) -> None:
        with self._lock:
            self._connection = pyserial.Serial(
                port=self._port,
                baudrate=2000000,
                bytesize=pyserial.EIGHTBITS,
                parity=pyserial.PARITY_NONE,
                stopbits=pyserial.STOPBITS_ONE,
                timeout=self.timeout_sec,
                exclusive=True,
            )

        self.get_logger().info("Starting Connection.")

        i = 0
        while True:
            self.get_logger().info(f"Flushing {i}.")
            total_flushed = self.flush_receive()
            self.flush_transmit()
            time.sleep(0.5)

            self.get_logger().info(f"Total bytes flushed: {total_flushed}.")
            if total_flushed == 0:
                break

            i += 1

        if self.is_connected:
            self.get_logger().info("Connection Ready.")
        else:
            self.get_logger().info("Failed to Start Connection.")

    def disconnect(self) -> None:
        if not self.is_connected:
            return

        with self._lock:
            self._connection.close()
            self._connection = None

        self.get_logger().info("Connection is Closed.")

    def flush_receive(self) -> None:
        total_flushed = 0
        with self._lock:
            total_flushed += self._connection.in_waiting
            if self._connection.in_waiting > 0:
                self.get_logger().info(
                    f"Flushing Rx Buffer. "
                    f"Current bytes in buffer={self._connection.in_waiting}."
                )
                self._connection.reset_input_buffer()
                self.get_logger().info(
                    f"Rx Buffer Flushed. "
                    f"Current bytes in buffer={self._connection.in_waiting}."
                )
            else:
                self.get_logger().info("Nothing to flush with `reset_input_buffer`.")

            read_bytes = self._connection.read_all()
            total_flushed += len(read_bytes)
            if len(read_bytes) > 0:
                self.get_logger().info(
                    f"Flushed {len(read_bytes)} bytes with `read_all`."
                )

        return total_flushed

    def flush_transmit(self) -> None:
        with self._lock:
            self._connection.flush()
            self._connection.flushOutput()

    def send_request_bytes(self, request_bytes: bytes) -> None:
        """Sends request to device.

        Args:
            request_bytes (bytes): Request to send.
        """
        self._connection.write(request_bytes)
        self.get_logger().info(f"Sent {len(request_bytes)} bytes.")

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

        with self._lock:
            if not self._connection.is_open:
                raise RuntimeError("Connection not opened")

            if (
                not isinstance(argument, bytes)
                or len(argument) > FTS_DEFAULT_ARGUMENT_SIZE
            ):
                raise ValueError(
                    f"Arguments should have maximum size {FTS_DEFAULT_ARGUMENT_SIZE}"
                )

            # Padding and Reversing Byte Order as needed.
            if len(argument) < 4:
                argument = bytes(4 - len(argument)) + argument

            if FTS_TX_BYTE_ORDER == "little":
                argument = argument[::-1]

            request_bytes = bytes()
            request_bytes += Ascii.SOH.byte + Ascii.STX.byte
            request_bytes += command.to_bytes()
            request_bytes += argument
            request_bytes += Ascii.ETX.byte + Ascii.EOT.byte

            # Padding with Ascii EOT
            request_bytes = request_bytes.ljust(
                FTS_TX_BUFFER_SIZE_SERIAL, Ascii.EOT.byte
            )

            message = f"Sending {len(request_bytes)} bytes via Serial: Bytes["
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

        # NOTE: When transmitting response, the data sent can have any arbitrary size.
        # The response has the following layout:
        #
        # [Ascii: SOH, STX]    [2 bytes]
        # [Payload Size]       [2 bytes]
        # [Payload]            [N bytes]
        # [Ascii: ETX, EOT]    [2 bytes]
        #
        # NOTE: The transmission of multiple bytes follows Most Significant Byte
        # (MSB) first.

        with self._lock:
            byte_stream = bytes()
            offset_payload_size = 2
            offset_payload = 4

            while True:
                ######################################################################
                # Part 1 - Guaranteeing that SOH and STX are in the byte_stream.
                ######################################################################
                expected_bytes = Ascii.SOH.byte + Ascii.STX.byte

                if Ascii.SOH.byte not in byte_stream:
                    # If they are not, we read expecting ofr STX until timeout.
                    byte_stream = self._connection.read_until(expected=Ascii.STX.byte)
                    if len(byte_stream) < 2 or byte_stream[-2:] != expected_bytes:
                        # We break the loop if not able to read meaningful data.
                        breakpoint()
                        return None

                    # Otherwise, we reset byte_stream based on the new data
                    byte_stream = byte_stream[-2:]

                else:
                    # On the other hand, if SOH is already in the byte_stream,
                    # we just readjust byte_stream.
                    offset_soh = byte_stream.index(Ascii.SOH.byte)
                    byte_stream = byte_stream[offset_soh:]

                    # Then we check if it is followed by STX.
                    if byte_stream[:2] != expected_bytes:
                        # Otherwise, we consider that it was a glitch and that
                        # the current byte_stream is invalid.
                        # Hence, we reset the byte_stream and start over.
                        byte_stream = byte_stream[2:]
                        continue

                # NOTE: At this point, SOX and STX are guaranteed to be
                # in the beginning of the byte stream. However, they are not yet
                # fully validated, since we could had got SOX and STX out of a lucky
                # match with an incomplete payload data or some other data being
                # transferred.
                # NOTE: Let's assume that they are in fact our control characters,
                # for now. We will come back to this assumption later.

                ######################################################################
                # Part 2 - Extracting payload.
                ######################################################################

                # The next two bytes should contain the payload size.
                if len(byte_stream) < offset_payload:
                    byte_stream += self._connection.read(size=2)

                if len(byte_stream) < offset_payload:
                    # Break the loop if we timed-out and were not able to read the bytes.
                    breakpoint()
                    return None

                data_type = np.dtype("uint16").newbyteorder(">")
                payload_size = np.frombuffer(
                    byte_stream[offset_payload_size:offset_payload],
                    dtype=data_type,
                )[0]

                # Then we extract the payload.
                byte_stream += self._connection.read(size=payload_size + 2)
                offset_etx = offset_payload + payload_size
                total_transmission_size = offset_etx + 2
                if len(byte_stream) < total_transmission_size:
                    # Break the loop if we timed-out and were not able to read the bytes.
                    breakpoint()
                    return None

                payload_bytes = byte_stream[offset_payload:offset_etx]

                ######################################################################
                # Part 3 - Validating Transmission
                ######################################################################
                # As mentioned earlier, we assumed SOH and STX were in fact the begin
                # of the message. In order to confirm that, we need to check if we
                # have ETX and EOT at the end of the message.

                expected_bytes = Ascii.ETX.byte + Ascii.EOT.byte
                if byte_stream[offset_etx:total_transmission_size] != expected_bytes:
                    # Failing to do so, we discard the first two bytes and
                    # repeat the entire process.
                    byte_stream = byte_stream[2:]
                    continue

                ######################################################################
                # Part 4 - Parsing Payload
                ######################################################################

                return payload_bytes
