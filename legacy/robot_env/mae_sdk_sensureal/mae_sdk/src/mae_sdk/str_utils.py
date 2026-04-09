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


def convert_ip_string_to_int(ip_address_v4_str: str) -> int:
    """
    Converts an IPv4 address from dotted decimal notation to a 32-bit integer.

    This function takes an IPv4 address as a string (e.g., "192.168.0.1") and
    converts it to a 32-bit integer. The most significant byte corresponds to the
    first octet, with conversion done in big-endian byte order.

    Example:
        "192.168.1.11" -> [192, 168, 1, 11] -> 0xC0A8010B -> 3232235787

    Args:
        ip_address_v4_str (str): The IPv4 address in dotted decimal notation.

    Returns:
        int: The 32-bit integer representation of the IPv4 address.

    Example:
        >>> convert_ip_string_to_int("192.168.0.1")
        3232235521
    """

    ip_address_v4_int = [int(octet) for octet in ip_address_v4_str.split(".")]
    ip_address_v4_int = np.array(ip_address_v4_int, dtype=np.dtype("uint8"))
    ip_address_v4_int = int.from_bytes(ip_address_v4_int.tobytes(), "big")
    return ip_address_v4_int


def convert_int_to_ip_string(ip_address_v4_int: int | np.uint32) -> str:
    """
    Converts a 32-bit integer to its IPv4 address in dotted decimal notation.

    This function takes a 32-bit integer representation of an IPv4 address and converts
    it back to its dotted string form (e.g., 3232235787 -> "192.168.1.11"). The integer
    is interpreted in big-endian byte order.

    Example:
        3232235787 -> 0xC0A8010B -> [192, 168, 1, 11] -> "192.168.1.11"

    Args:
        ip_address_v4_int (int): The 32-bit integer representation of the IPv4 address.

    Returns:
        str: The IPv4 address in dotted decimal notation.

    Example:
        >>> convert_int_to_ip_string(3232235787)
        "192.168.1.11"
    """

    if isinstance(ip_address_v4_int, int):
        ip_bytes = ip_address_v4_int.to_bytes(4, byteorder="big")

    elif isinstance(ip_address_v4_int, np.uint32):
        ip_bytes = ip_address_v4_int.newbyteorder(">").tobytes()
    else:
        raise ValueError("Invalid type.")

    ip_address_v4_str = ".".join(map(str, ip_bytes))
    return ip_address_v4_str
