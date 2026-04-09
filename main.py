"""
main.py — Program entry point.

Manages the application lifecycle: starts a background keyboard listener,
idles until the user initiates a session, runs RobotEnv for the duration of
the session, and handles save / discard on exit.

Key bindings:
    s       Start a new recording session
    e       End the session and save data to disk
    d       Discard the session without saving
    Esc     Quit the application

Usage:
    uv run main.py                  # real hardware
    SIM_MODE=true uv run main.py    # URSim (requires Docker container running)

Reference: legacy/run_haply.py
"""
