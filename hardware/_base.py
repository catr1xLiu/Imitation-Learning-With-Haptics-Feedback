"""
hardware/_base.py — Abstract base class for all hardware drivers.

Every driver in this package inherits from HardwareDriver and must implement
connect(), disconnect(), and is_connected(). The base class enforces a consistent
lifecycle interface so that RobotEnv can treat all five devices uniformly and
the test suite can substitute mock implementations without branching.

Drivers must also be safe to call from multiple threads: their internal state
is protected by threading.Lock, and get_state() returns a copy of the latest
reading rather than a reference to the internal buffer.
"""
