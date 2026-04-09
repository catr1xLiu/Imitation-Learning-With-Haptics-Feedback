"""
hardware/__init__.py — Hardware driver package.

Exports the five hardware drivers and the shared state dataclasses.
Each driver runs a background thread at its native hardware rate and exposes
a thread-safe get_state() method. See hardware/_base.py for the common interface.
"""
