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
import os
from pathlib import Path

from .log_utils import print_on_keyboard_exception

__all__ = ["print_args"]


# class SmartFormatter(argparse.HelpFormatter):
#     """Formatter that kicks when detecting prefix."""

#     def _split_lines(self, text: str, width: int) -> list[str]:
#         if text.startswith("R|"):
#             return text[2:].splitlines()

#         # this is the RawTextHelpFormatter._split_lines
#         return argparse.HelpFormatter._split_lines(self, text, width)


def _is_debugging():
    debug_str = os.getenv("DEBUG")
    if (
        debug_str is None
        or debug_str == ""
        or debug_str == "False"
        or debug_str == "false"
        or int(debug_str) == 0
    ):
        return False

    return True


def print_args(script_file, args: argparse.Namespace, pause: bool = False) -> None:
    try:
        print(f"{Path(script_file).name} running with args={args}")

        if not pause or _is_debugging():
            print("")
            return

        if pause:
            input("(Press Enter to continue)\n")

    except KeyboardInterrupt:
        print_on_keyboard_exception()

        exit(0)
