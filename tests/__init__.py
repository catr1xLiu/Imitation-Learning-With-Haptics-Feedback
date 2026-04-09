"""
tests/__init__.py — Test suite package.

Tests are split by concern:
    test_haptic_decoding    Unit tests for Haply JSON parsing and delta computation
    test_fts_transform      Unit tests for FTS coordinate frame transformation
    test_ur_commands        Unit tests for RTDE command formatting (mocked RTDE)
    test_env_step           Integration test: full control loop step against URSim

Run all tests with:
    uv run pytest

Run integration tests only (requires URSim running):
    uv run pytest tests/test_env_step.py
"""
