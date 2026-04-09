#!/usr/bin/env python
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


SCRIPT_DESCRIPTION = """
This Python script shows how to communicate with the MAE Force-Torque Sensor
(FTS) whereas both Serial Port and UDP Protocol are supported as communication
interfaces.
"""


SCRIPT_EPILOG = """"""

import logging
from pathlib import Path

import mae_fts_sdk.src.mae_fts_sdk.fts_commands as mae
from mae_sdk.src.mae_sdk import log_utils

# CONFIG
FTS_IP_ADDRESS = "192.168.1.11"
FTS_PORT = 10547


AVAILABLE_FTS_COMMANDS = {str(key): key for key in mae.FTS_COMMAND_RESPONSE.keys()}
"""Intentionally use str(command) to keep the prefix FtsCommand.
This allow us to distinguish commands from other subsets that could exist."""

def run_fts_sdk_sample(
    *,
    ip_address: str,
    port: int,
    command: str,
    timeout=1,
):
    command = AVAILABLE_FTS_COMMANDS[command]
    log_level = logging.WARNING

    logger = log_utils.get_logger(f"{Path(__file__).stem}", log_level=log_level)

    communication_interface = mae.UdpCommunication(
        ip_address=ip_address,
        port=port,
        timeout_sec=timeout,
        log_level=log_level,
    )

    communication_interface.connect()

    confirmation = f"Sent: {command}."
    communication_interface.send_request(command)
    print(confirmation)

    # Some Commands don't have a response, hence response_type can be None.
    response_type = mae.FTS_COMMAND_RESPONSE[command]

    if response_type is not None:
        byte_stream = communication_interface.waits_response_bytes()
        if byte_stream is None:
            logger.error(f"No Response. Timed-out.")

        response = response_type(byte_stream)
        print(f"Received: {response}.")

    communication_interface.disconnect()

def main():
    run_fts_sdk_sample(ip_address=FTS_IP_ADDRESS, port=FTS_PORT, command="FtsCommand.STREAM_FT_START")


if __name__ == "__main__":
    main()
