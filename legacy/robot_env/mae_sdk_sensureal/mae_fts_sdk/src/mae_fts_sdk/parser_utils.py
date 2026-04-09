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


import argparse


def add_command_argument_to_parser(
    command: str,
    command_subparser: argparse._ActionsContainer,
):
    """Add arguments based on the available commands to command subparser.

    Args:
        command (str):
            The command to add arguments for. This should be a FtsCommand as
            string value.
        command_subparser (argparse._ActionsContainer):
            The subparser to add the arguments to.
    """

    if command == "FtsCommand.SAMPLING_PERIOD_SET":
        command_subparser.add_argument(
            "sampling_period",
            type=int,
            help=f"New sampling period in microseconds",
        )
    # elif command == "FtsCommand.BUFFER_SIZE_SET":
    #     command_subparser.add_argument(
    #         "buffer_size",
    #         type=int,
    #         help=f"New buffer size",
    #     )
    elif command == "FtsCommand.IP_ADDRESS_SET":
        command_subparser.add_argument(
            "new_ip_address",
            type=str,
            help=f"New IP v4 address",
        )
    elif command == "FtsCommand.TOOL_TX_SET":
        command_subparser.add_argument(
            "tool_argument",
            type=float,
            help=f"Translation in X given in meters",
        )
    elif command == "FtsCommand.TOOL_TY_SET":
        command_subparser.add_argument(
            "tool_argument",
            type=float,
            help=f"Translation in Y given in meters",
        )
    elif command == "FtsCommand.TOOL_TZ_SET":
        command_subparser.add_argument(
            "tool_argument",
            type=float,
            help=f"Translation in Z given in meters",
        )
    elif command == "FtsCommand.TOOL_RX_SET":
        command_subparser.add_argument(
            "tool_argument",
            type=float,
            help=f"Rotation in X given in degrees",
        )
    elif command == "FtsCommand.TOOL_RY_SET":
        command_subparser.add_argument(
            "tool_argument",
            type=float,
            help=f"Rotation in Y given in degrees",
        )
    elif command == "FtsCommand.TOOL_RZ_SET":
        command_subparser.add_argument(
            "tool_argument",
            type=float,
            help=f"Rotation in Z given in degrees",
        )
