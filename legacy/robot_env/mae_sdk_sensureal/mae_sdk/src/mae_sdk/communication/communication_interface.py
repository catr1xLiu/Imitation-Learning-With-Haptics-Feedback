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

from .. import log_utils

__all__ = ["CommunicationInterface"]


class CommunicationInterface:
    def __init__(
        self,
        timeout_sec: int | float = 0.1,
        log_level: int | str = None,
    ):
        """Communication Interface

        Args:
            timeout_sec (float, optional): UDP interface timeout. Defaults to 0.1.
            log_level (int | str, optional): Defaults to None.
        """

        if not isinstance(timeout_sec, (int, float)):
            raise TypeError(
                f"Invalid timeout type: {type(timeout_sec)}. Expected int or float."
            )

        if timeout_sec < 0:
            raise ValueError(
                f"Invalid timeout value: {timeout_sec}. Expected non-negative value."
            )

        if not isinstance(log_level, (int, str, type(None))):
            raise TypeError(
                f"Invalid log level type: {type(log_level)}. Expected int or str."
            )

        self._timeout_sec = timeout_sec

        if log_level:
            self.get_logger().setLevel(log_level)

    def get_logger(self) -> logging.Logger:
        return log_utils.get_logger(self.__class__.__name__)

    @property
    def timeout_sec(self) -> int | float:
        return self._timeout_sec

    def connect(self) -> None: ...

    def disconnect(self) -> None: ...

    def flush_receive(self) -> None:
        """Flushes data from the receive buffer."""
        ...

    def flush_transmit(self) -> None:
        """Flushes data from the transmitting buffer."""
        ...

    def send_request_bytes(self, request_bytes: bytes) -> None:
        """Sends request to device.

        Args:
            request_bytes (bytes): Request to send.
        """
        raise NotImplementedError()

    def waits_response_bytes(self) -> bytes | None:
        """Waits for response from the device.

        Returns:
            bytes | None: Returns None when timed-out or an error occurred;
            otherwise, if successful, returns bytes.
        """
        raise NotImplementedError()
