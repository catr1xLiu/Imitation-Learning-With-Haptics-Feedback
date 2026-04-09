from pynput import keyboard
import time
import sys
import threading

# Global variables to track the state
success = False
failure = False

def on_press(key):
    global success, failure  # Declare the variables as global
    
    try:
        if key.char == 's':  # When 's' is pressed, set success to True
            success = True
        elif key.char == 'f':  # When 'f' is pressed, set failure to True
            failure = True

    except AttributeError:
        pass  # Special keys do nothing

def on_release(key):
    global success, failure  # Declare the variables as global

    if key == keyboard.Key.esc:  # Stop listener when ESC is pressed
        return False

    try:
        if key.char == 's':  # When 's' is released, set success to False
            success = False
        elif key.char == 'f':  # When 'f' is released, set failure to False
            failure = False

    except AttributeError:
        pass  # Special keys do nothing

def start_listener():
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

# Start listener in a separate thread
listener_thread = threading.Thread(target=start_listener)
listener_thread.daemon = True  # Allow the program to exit even if the thread is still running
listener_thread.start()

# Infinite loop to check the states and display the changes
while True:
    print(f"Success: {success}, Failure: {failure}")
    time.sleep(1)  # Sleep to prevent flooding the display
