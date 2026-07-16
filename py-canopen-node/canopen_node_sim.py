"""
CANopen slave node simulation.

Connects to the RH02 (gs_usb) adapter, loads a simulated device from an EDS 
file as a canopen LocalNode, boots it up over the bus, and logs all traffic 
to both the terminal and a file in real time.
"""

import time
import logging

import can
import canopen
import usb.core
import usb.backend.libusb1
import libusb_package

# --- gs_usb / libusb backend fix ---
# Monkey patch to access DLL for adapter (I think)
_backend = usb.backend.libusb1.get_backend(find_library=libusb_package.find_library)
import can.interfaces.gs_usb as gs_usb_mod
_original_find = usb.core.find
gs_usb_mod.usb.core.find = lambda *a, **kw: _original_find(*a, **{**kw, "backend": _backend})

# --- Config ---
NODE_ID = 2                    # must match the node ID configured in CM CANopen Configuration Studio
EDS_PATH = "basicDevice.eds"   # path to the trimmed EDS file
CHANNEL = "0"                  # gs_usb device index (see enumeration script from earlier)
BITRATE = 500000               # must match the CM module's configured bitrate
LOG_FILE = "canopen_traffic.log"

# --- Logging setup: everything goes to both the terminal and a file, live ---
logger = logging.getLogger("canopen_sim")
logger.setLevel(logging.DEBUG)

_fmt = logging.Formatter("%(asctime)s  %(message)s", "%H:%M:%S")

file_handler = logging.FileHandler(LOG_FILE, mode="a")
file_handler.setFormatter(_fmt)
logger.addHandler(file_handler)

console_handler = logging.StreamHandler()
console_handler.setFormatter(_fmt)
logger.addHandler(console_handler)


class TrafficLogger(can.Listener):
    """Logs every CAN frame the Notifier sees to the logger above."""

    def on_message_received(self, msg: can.Message):
        data_hex = msg.data.hex(" ").upper()
        logger.info(f"ID=0x{msg.arbitration_id:03X}  DLC={msg.dlc}  Data=[{data_hex}]")


def main():
    network = canopen.Network()

    # Add our listener to network.listeners BEFORE connect() -- connect() builds
    # a single can.Notifier from this list, so this is how you get a second
    # "tap" on the bus traffic without competing with canopen's own listener
    # for messages (two separate Notifiers on one bus would steal frames from
    # each other instead of both seeing everything).
    network.listeners.append(TrafficLogger())

    network.connect(bustype="gs_usb", channel=CHANNEL, bitrate=BITRATE)
    logger.info(f"Connected to {network.bus.channel_info}")

    node = canopen.LocalNode(NODE_ID, EDS_PATH)
    network.add_node(node)

    # NMT boot sequence, matching what a real device does:
    #   - Setting state to RESET moves the internal state to 0 (INITIALISING),
    #     which is what actually triggers canopen to transmit the 0x700+ID
    #     boot-up frame on the bus.
    #   - Setting state to PRE-OPERATIONAL immediately after puts the node in
    #     the state it should be in for the master to start talking to it.
    # Going straight to PRE-OPERATIONAL from a freshly-constructed node still
    # "works," but silently skips sending the boot-up frame, since that
    # message only fires on the transition *into* state 0.
    node.nmt.state = "RESET"
    node.nmt.state = "PRE-OPERATIONAL"

    logger.info(f"Node {NODE_ID} is up. Logging to {LOG_FILE}. Ctrl+C to stop.")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        network.disconnect()


if __name__ == "__main__":
    main()
