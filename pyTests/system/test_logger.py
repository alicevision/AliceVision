"""
Collection of unit tests for the initialize_alicevision_logger function.
"""

import pytest

from pyalicevision import system as av

##################
### List of functions:
# - initialize_alicevision_logger(verboseLevel) => DONE
#   Sets the logging verbosity level for the AliceVision logger.
#   Valid levels: "fatal", "error", "warning", "info", "debug", "trace"
##################


def test_initialize_logger_info():
    """ Test initializing the logger with 'info' verbosity level. """
    # Should not raise any exception
    av.initialize_alicevision_logger("info")


def test_initialize_logger_debug():
    """ Test initializing the logger with 'debug' verbosity level. """
    av.initialize_alicevision_logger("debug")


def test_initialize_logger_warning():
    """ Test initializing the logger with 'warning' verbosity level. """
    av.initialize_alicevision_logger("warning")


def test_initialize_logger_error():
    """ Test initializing the logger with 'error' verbosity level. """
    av.initialize_alicevision_logger("error")


def test_initialize_logger_fatal():
    """ Test initializing the logger with 'fatal' verbosity level. """
    av.initialize_alicevision_logger("fatal")


def test_initialize_logger_trace():
    """ Test initializing the logger with 'trace' verbosity level. """
    av.initialize_alicevision_logger("trace")


def test_initialize_logger_change_level():
    """ Test changing the logger verbosity level multiple times. """
    av.initialize_alicevision_logger("info")
    av.initialize_alicevision_logger("debug")
    av.initialize_alicevision_logger("warning")
    # All calls should succeed without error
