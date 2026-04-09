"""
hardware/haptic.py — Haply Inverse3 haptic device driver.

Communicates with the Haply Inverse3 and Verse Grip over a local WebSocket
(ws://localhost:10001) managed by the Haply Haptic Composer software. A
background thread receives device state (cursor position, velocity, orientation,
buttons) at the WebSocket's native rate and writes it into a locked HapticState.

Optionally sends force feedback to the device via set_cursor_force commands on
the same WebSocket. Force feedback is gated off in env.py when config.SIM_MODE
is True, since there is no physical contact in simulation.

The Haply device is always real hardware regardless of SIM_MODE — it connects
physically to the host machine and its WebSocket is always available.

Reference: legacy/haply_data_collect_july23/teleop_controls/haply_reader/reader.py
    HapticReader class and connect_and_read loop: lines 144–268
    ForceFilter (low-pass + wall virtualisation): lines 14–142
Reference: legacy/haply_data_collect_july23/haply_barebones.py
    HapticVisualizer (delta computation):         line 663 onwards
"""
