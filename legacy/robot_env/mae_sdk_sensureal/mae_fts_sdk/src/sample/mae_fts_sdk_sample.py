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

import argparse
import logging
import time
from pathlib import Path

import argcomplete
import mae_fts_sdk as mae
import numpy as np
from mae_sdk import log_utils
from mae_sdk.log_utils import print_on_keyboard_exception
from mae_sdk.parser_utils import print_args
from mae_sdk.str_utils import convert_ip_string_to_int


AVAILABLE_FTS_COMMANDS = {str(key): key for key in mae.FTS_COMMAND_RESPONSE.keys()}
"""Intentionally use str(command) to keep the prefix FtsCommand.
This allow us to distinguish commands from other subsets that could exist."""


def run_fts_sdk_sample(
    *,
    mode: str,
    command: str,
    communication: str,
    ip_address: str,
    port: int,
    buffer_size: int = None,
    debug=False,
    sampling_period: int = None,
    new_ip_address: str = None,
    tool_argument: float = None,
    sample_count: int = None,
    timeout=1,
    verbose=False,
    **kwargs,
):
    command = AVAILABLE_FTS_COMMANDS[command]
    log_level = logging.WARNING

    if verbose:
        log_level = logging.INFO

    if debug:
        log_level = logging.DEBUG
        timeout = 999999999

    if new_ip_address:
        new_ip_address = convert_ip_string_to_int(new_ip_address)

    logger = log_utils.get_logger(f"{Path(__file__).stem}", log_level=log_level)

    communication_interface = None
    if communication == "serial":
        port = mae.prompt_user_port_selection(log_level)
        communication_interface = mae.SerialCommunication(
            port=port,
            timeout_sec=timeout,
            log_level=log_level,
        )

    if communication == "udp":
        communication_interface = mae.UdpCommunication(
            ip_address=ip_address,
            port=port,
            timeout_sec=timeout,
            log_level=log_level,
        )

    communication_interface.connect()

    iterations_left = 0
    if mode == "stream":
        iterations_left = sample_count if sample_count else 0
        argument = sample_count if sample_count else 0
        communication_interface.send_request(command, argument)
        print(f"Sent: {command} with argument={argument}.")

    try:
        while True:
            t0 = time.perf_counter_ns()

            if mode == "send_once" or mode == "send_loop":
                argument = 1
                if buffer_size is not None:
                    argument = buffer_size

                if sampling_period is not None:
                    argument = sampling_period

                if new_ip_address is not None:
                    argument = new_ip_address

                if tool_argument is not None:
                    argument = np.array([tool_argument], dtype=">f4").tobytes()

                confirmation = f"Sent: {command} with argument={argument}."
                communication_interface.send_request(command, argument)
                print(confirmation)

            # Some Commands don't have a response, hence response_type can be None.
            response_type = mae.FTS_COMMAND_RESPONSE[command]

            if response_type is not None:
                byte_stream = communication_interface.waits_response_bytes()
                if byte_stream is None:
                    logger.error(f"No Response. Timed-out.")
                    break

                response = response_type(byte_stream)
                t1 = time.perf_counter_ns()
                print(f"Received: {response}.")
                logger.info(f"Response Latency: {(t1-t0)/1e3}us.")

            else:
                t1 = time.perf_counter_ns()
                logger.info(f"Command Latency: {(t1-t0)/1e3}us.")

            if mode == "send_once":
                break

            elif mode == "stream" and iterations_left - 1 == 0:
                break

            iterations_left = iterations_left - 1

    except KeyboardInterrupt:
        print_on_keyboard_exception()

    except TimeoutError:
        print("Timeout Error: FTS device did not respond.")

    finally:
        if mode == "stream":
            communication_interface.send_request(mae.FtsCommand.STREAM_STOP)
            print(f"Sent: {mae.FtsCommand.STREAM_STOP}.")
            print("Stream Stopped Successfully.")

        communication_interface.disconnect()


def define_additional_arguments(parser):
    parser.add_argument(
        "--communication",
        choices=["serial", "udp"],
        default="udp",
        help="Choose a communication interface.",
    )

    parser.add_argument(
        "--timeout",
        "-t",
        type=int,
        help="Timeout in seconds",
        default=1,
    )

    parser.add_argument(
        "--debug",
        "-d",
        action="store_true",
        help="Turn on DEBUG mode",
        default=False,
    )

    parser.add_argument(
        "--ip_address",
        "--ip",
        help="IP v4 address. Default is 192.168.1.11",
        default="192.168.1.11",
    )

    parser.add_argument(
        "--port",
        help="Port. Default is 10547",
        type=int,
        default=10547,
    )

    parser.add_argument(
        "--verbose",
        "-v",
        "--info",
        action="store_true",
        help="Turn on verbose mode, a.k.a logging.INFO",
        default=False,
    )


def main():

    parser = argparse.ArgumentParser(
        description=SCRIPT_DESCRIPTION,
        epilog=SCRIPT_EPILOG,
    )
    subparser_modes = parser.add_subparsers(
        title="Available Modes",
        dest="mode",
        required=True,
    )

    ########################## Mode "Send Once" ##########################
    parser_mode_send_once = subparser_modes.add_parser(
        "send_once",
        help="Send a command to the sensor and wait for its response",
    )

    parser_mode_send_once_command = parser_mode_send_once.add_subparsers(
        metavar="[command]",
        dest="command",
        help="select on of the following API commands",
        required=True,
    )

    for command in sorted(AVAILABLE_FTS_COMMANDS.keys()):
        command_subparser = parser_mode_send_once_command.add_parser(
            command,
            help="",
        )

        mae.add_command_argument_to_parser(command, command_subparser)

        define_additional_arguments(command_subparser)

    ########################## Mode "Send Loop" ##########################
    parser_mode_send_loop = subparser_modes.add_parser(
        "send_loop",
        help="Send a command to the sensor and wait for its response."
        " This process is repeated indefinitely until the user presses CTRL+C",
    )

    parser_mode_send_loop_command = parser_mode_send_loop.add_subparsers(
        metavar="[command]",
        dest="command",
        help="select on of the following API commands",
        required=True,
    )

    for command in sorted(AVAILABLE_FTS_COMMANDS.keys()):
        command_subparser = parser_mode_send_loop_command.add_parser(
            command,
            help="",
        )

        mae.add_command_argument_to_parser(command, command_subparser)

        define_additional_arguments(command_subparser)

    ########################## Mode "Stream" ##########################
    parser_mode_stream = subparser_modes.add_parser(
        "stream",
        help="Send a stream command to the sensor and listen to the responses."
        " The start streaming command is sent only once."
        " Streaming is interrupted when the user presses CTRL+C",
    )

    parser_mode_stream_command = parser_mode_stream.add_subparsers(
        metavar="[command]",
        dest="command",
        help="select on of the following API commands",
        required=True,
    )

    for command in mae.FTS_STREAMING_COMMANDS:
        command_subparser = parser_mode_stream_command.add_parser(
            command,
            help="",
        )

        command_subparser.add_argument(
            "sample_count",
            nargs="?",
            type=int,
            help="Set sample_count to the number of samples to output."
            " If the sample_count is set to zero, the sensor continuously "
            " outputs until the user presses CTRL+C",
            default=0,
        )

        define_additional_arguments(command_subparser)

    ############################################################################

    argcomplete.autocomplete(parser)
    args = parser.parse_args()
    print_args(__file__, args)

    run_fts_sdk_sample(**vars(args))


if __name__ == "__main__":
    main()
