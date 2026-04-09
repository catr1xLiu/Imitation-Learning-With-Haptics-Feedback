"""
data/__init__.py — Data recording package.

Exports StepSnapshot (the per-step data container) and Recorder (the HDF5
writer). The control loop in env.py constructs a StepSnapshot each iteration
and passes it to the Recorder's queue; the Recorder serialises asynchronously.
"""
