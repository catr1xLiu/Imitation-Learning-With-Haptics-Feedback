import asyncio
import time
import numpy as np
from teleop_controls.haply_reader.reader import HapticReader

class VRPolicy:
    def __init__(self):
        self.haply_reader = HapticReader()
        self._state = {}

    async def _run_haptic_reader_loop(self):
        """Runs the HapticReader event loop in the background."""
        await self.haply_reader.connect_and_read()

    async def _update_internal_state(self, hz=50):
        last_read_time = time.time()

        while True:
            await asyncio.sleep(1 / hz)

            time_since_read = time.time() - last_read_time
            poses, buttons = self.haply_reader.get_device_state()

            if not poses:
                continue

            # Update poses and buttons states
            self._state["poses"] = poses
            self._state["buttons"] = buttons
            self._state["controller_on"] = time_since_read < 5

            # Log all relevant data
            print(f"State: {self._state}")
            print(f"Poses: {poses}")
            print(f"Buttons: {buttons}")
            print(f"Controller On: {self._state['controller_on']}")

            last_read_time = time.time()

    async def start(self):
        """Main entry point to run both loops."""
        # Start the HapticReader loop
        haptic_task = asyncio.create_task(self._run_haptic_reader_loop())

        # Start the state update loop
        state_update_task = asyncio.create_task(self._update_internal_state())

        # Wait for both tasks to complete (or run forever)
        await asyncio.gather(haptic_task, state_update_task)

# Main entry point for the event loop
if __name__ == "__main__":
    policy = VRPolicy()

    # Start the main async loop
    asyncio.run(policy.start())
