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
import sys
import warnings

__LOGGERS__: dict[str, logging.Logger] = {}


# TODO: Remove this function.
def get_logger(name: str = "", log_level: int | str = None) -> logging.Logger:
    """Retrieves a `Logger` objects based on `name`. If not existing, it creates one.
    The Logger object is created and a custom log format is specified in this function.
    Whenever a `log_level` is defined, it will be set to the `Logger` object.

    Args:
        name (_type_): Name of the `Logger` object.
        log_level (int | str, optional): Log level to be set to the `Logger` object. Defaults to None.

    Returns:
        logging.Logger: Logger object with custom formatted logging message.
    """

    warnings.warn(
        "get_logger is deprecated and will be removed in the future.",
        DeprecationWarning,
    )

    if name in __LOGGERS__:
        logger = __LOGGERS__[name]

        if isinstance(log_level, (int, str)):
            logger.setLevel(log_level)

        # NOTE: this is meant to be deprecated and removed because log already
        # handles the uniqueness of the log instances, and I believe that there
        # is also a way to globally format logs.
        # NOTE: See also https://chatgpt.com/g/g-p-6764352b78d881918ff77c931c4887a3-mae-robotics/c/6846e4d3-1b9c-8000-8213-f7f695b9fb6b
        return logger

    # Create a new logger
    logger = logging.getLogger(name)
    __LOGGERS__[name] = logger

    if isinstance(log_level, (int, str)):
        logger.setLevel(log_level)

    # Create a console handler with a custom format
    console_handler = logging.StreamHandler()
    formatter = logging.Formatter(
        "[%(asctime)s][%(levelname)s][%(name)s %(filename)s:%(lineno)d]: %(message)s"
    )
    console_handler.setFormatter(formatter)

    # Add the handler to the logger
    logger.addHandler(console_handler)
    # logger.warning("Deprecated function")
    return logger


def apply_custom_logger_formatter(logger: logging.Logger) -> None:
    """Applies a custom console log format to the given logger.

    Args:
        logger (logging.Logger): _description_
    """

    formatter = logging.Formatter(
        # "[%(asctime)s][%(levelname)s][%(name)s %(filename)s:%(lineno)d]: %(message)s"
        "[%(asctime)s][%(levelname)s][%(filename)s:%(lineno)d]: %(message)s"
    )

    # Create a console handler with a custom format
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)

    # Add the handler to the logger
    logger.addHandler(console_handler)


def print_on_keyboard_exception():
    # sys._getframe(0) is this frame, (1) is the caller
    caller_name = sys._getframe(1).f_code.co_name
    print(f"\nKeyboardInterrupt: {caller_name} interrupted by user.")


if __name__ == "__main__":
    get_logger("TestingCase").info("Message 1.")
    get_logger("TestingCase").warning("Message 2.")
    get_logger("TestingCase").error("Message 3.")
    get_logger("TestingCase", log_level=logging.INFO).info("Message 4.")
    get_logger("TestingCase", log_level=logging.INFO).warning("Message 5.")
    get_logger("TestingCase", log_level=logging.INFO).error("Message 6.")
    print("Hello World")

    def test_caller():
        print_on_keyboard_exception()

    print_on_keyboard_exception()
    test_caller()
