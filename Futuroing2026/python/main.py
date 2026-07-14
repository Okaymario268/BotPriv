import time

from arduino.app_utils import App

print("Futuroing2026 servo app running on UNO Q")


def loop():
    """This function is called repeatedly by the App framework."""
    # The servo logic runs on the MCU sketch; nothing to do here yet.
    time.sleep(10)


# See: https://docs.arduino.cc/software/app-lab/tutorials/getting-started/#app-run
App.run(user_loop=loop)
