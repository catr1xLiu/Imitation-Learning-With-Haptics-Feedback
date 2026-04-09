"""
tests/test_haptic_decoding.py — Unit tests for Haply device input processing.

Tests that raw WebSocket JSON payloads from the Haply Inverse3 are decoded
correctly into HapticState, and that the delta computation in HapticDevice
produces the expected [dx, dy, dz, drx, dry, drz, gripper] action vectors.

No hardware required. A synthetic JSON payload mimicking the Haply WebSocket
protocol is fed directly to the parsing logic.

Reference: legacy/haply_data_collect_july23/teleop_controls/haply_reader/reader.py
    JSON parsing:        lines 175–219
Reference: legacy/haply_data_collect_july23/haply_barebones.py
    Delta computation:   lines 800–850 (approx, inside get_action())
"""
