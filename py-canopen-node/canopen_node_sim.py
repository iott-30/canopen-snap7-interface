"""
CANopen slave node simulation.

Opens the RH02 (gs_usb) adapter, loads a simulated device from an EDS file
as a canopen LocalNode, boots it up over the bus, and logs all traffic to
both the terminal and a file in real time.
"""

import time
import logging
import threading

import can
import canopen
import usb.core
import usb.backend.libusb1
import libusb_package

# --- gs_usb / libusb backend fix (see earlier debugging session) ---
_backend = usb.backend.libusb1.get_backend(find_library=libusb_package.find_library)
import can.interfaces.gs_usb as gs_usb_mod
_original_find = usb.core.find
gs_usb_mod.usb.core.find = lambda *a, **kw: _original_find(*a, **{**kw, "backend": _backend})

# --- Config ---
NODE_ID = 2                    # must match the node ID configured in CM CANopen Configuration Studio
EDS_PATH = "basicDevice.eds"   # path to the trimmed EDS file
CHANNEL = "0"                  # gs_usb device index (see enumeration script from earlier)
BITRATE = 500000               # must match the CM module's configured bitrate
HEARTBEAT_MS = 500             # must match "Error Control Configuration" -> producer heartbeat time in CM Config Studio
MASTER_NODE_ID = 1             # the CM CANopen module's own node ID on this network
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


def log_event(message: str):
    """Log a device-level event (PDO state/changes, etc.), with a blank line
    before and after so it visually stands out from raw traffic frames in
    both the terminal and the log file."""
    for handler in (console_handler, file_handler):
        handler.stream.write("\n")
    logger.info(message)
    for handler in (console_handler, file_handler):
        handler.stream.write("\n")
        handler.flush()


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

    # Set the heartbeat producer time BEFORE the NMT transition below, not after.
    # The master starts monitoring node 2's heartbeat well before it gets around
    # to writing 0x1017 itself (partway through its own config sequence: device
    # type read, restore defaults, NMT reset, re-enter pre-op...). If we wait for
    # that write to start producing heartbeats, the master's consumer timeout
    # can elapse first, which is what an 0x8F02 "Heartbeat Consume error" EMCY
    # means. Setting it ourselves first means heartbeat is already running
    # before the master ever starts its clock.
    node.sdo[0x1017].raw = HEARTBEAT_MS

    # NMT boot sequence, matching what a real device does:
    #   - Setting state to RESET moves the internal state to 0 (INITIALISING),
    #     which is what actually triggers canopen to transmit the 0x700+ID
    #     boot-up frame on the bus.
    #   - Setting state to PRE-OPERATIONAL immediately after is what starts the
    #     heartbeat producer (using the 0x1017 value set above) and puts the
    #     node in the state the master expects to talk to it in.
    # Going straight to PRE-OPERATIONAL from a freshly-constructed node still
    # "works" for the heartbeat start, but silently skips sending the boot-up
    # frame, since that message only fires on the transition *into* state 0.
    node.nmt.state = "RESET"
    node.nmt.state = "PRE-OPERATIONAL"

    # --- PDO setup ---
    # Configured to match what CM Configuration Studio actually assigns
    # (confirmed from a captured successful boot): TPDO1 = COB-ID 0x180+id,
    # RPDO1 = COB-ID 0x200+id, both using the EDS's default 2-byte mapping
    # (sub1 + sub2). This is hardcoded rather than tracked live from the
    # master's SDO writes -- simple and correct as long as Configuration
    # Studio isn't changed to use different COB-IDs or mappings.
    TPDO1_COB_ID = 0x180 + NODE_ID
    RPDO1_COB_ID = 0x200 + NODE_ID

    tpdo1 = node.tpdo[1]
    tpdo1.clear()
    tpdo1.add_variable(0x6000, 1)  # sub1: the byte we're actually driving
    tpdo1.add_variable(0x6000, 2)  # sub2: left at its default (0x00)
    tpdo1.cob_id = TPDO1_COB_ID
    tpdo1.enabled = True

    rpdo1 = node.rpdo[1]
    rpdo1.clear()
    rpdo1.add_variable(0x6200, 1)  # sub1: what the PLC is driving
    rpdo1.add_variable(0x6200, 2)  # sub2
    rpdo1.cob_id = RPDO1_COB_ID
    rpdo1.enabled = True
    rpdo1.subscribe()  # register with the network so incoming frames on 0x202 actually reach us

    last_rpdo1 = [None, None]

    def on_rpdo1(pdo_map):
        nonlocal last_rpdo1
        current = [pdo_map[i].raw for i in range(2)]
        if last_rpdo1 == [None, None]:
            log_event(f"RPDO1 initial state: sub1=0b{current[0]:08b}  sub2=0b{current[1]:08b}")
        else:
            changes = [
                f"sub{i + 1}: 0b{last_rpdo1[i]:08b} -> 0b{current[i]:08b}"
                for i in range(2) if current[i] != last_rpdo1[i]
            ]
            if changes:
                log_event("RPDO1 changed - " + ", ".join(changes))
        last_rpdo1 = current

    rpdo1.add_callback(on_rpdo1)

    # In both captured logs, the master formally starts node 2 (targeted Start,
    # 01 02) roughly a second *before* it starts itself (01 01) and broadcasts
    # a final Start (01 00) -- and TPDO1 only gets picked up into the PLC's
    # process image after that second step. Since TPDO1 is event-driven with
    # no periodic resend (event timer = 0), a single transmission that lands
    # before the master's own Start is simply lost -- nothing will trigger us
    # to send it again. Watching the master's own heartbeat (node MASTER_NODE_ID)
    # for OPERATIONAL gives us a real signal to resend against, instead of
    # guessing at a delay.
    master_operational = threading.Event()

    def on_master_heartbeat(can_id, data, timestamp):
        if data == b"\x05":
            master_operational.set()

    network.subscribe(0x700 + MASTER_NODE_ID, on_master_heartbeat)

    logger.info(f"Node {NODE_ID} is up. Logging to {LOG_FILE}. Ctrl+C to stop.")

    # PDOs are only meaningful once the master brings us to OPERATIONAL, so
    # wait for that transition (there's no state-change callback in canopen's
    # NMT implementation, so this just polls) before sending TPDO1's first value.
    last_nmt_state = None
    tpdo1_initialized = False
    tpdo1_resent_after_master = False

    try:
        while True:
            current_state = node.nmt.state
            if current_state != last_nmt_state:
                logger.info(f"NMT state -> {current_state}")
                last_nmt_state = current_state

            if current_state == "OPERATIONAL" and not tpdo1_initialized:
                tpdo1[0].raw = 0b10010000
                log_event(
                    f"TPDO1 initial state: sub1=0b{tpdo1[0].raw:08b}  sub2=0b{tpdo1[1].raw:08b}"
                )
                tpdo1.transmit()
                tpdo1_initialized = True

            # Covers the observed ordering (node 2 started, TPDO1 sent, *then*
            # the master starts itself) by resending once we see confirmation
            # the master is actually ready, regardless of which order the two
            # Start events happened in.
            if tpdo1_initialized and master_operational.is_set() and not tpdo1_resent_after_master:
                tpdo1.transmit()
                log_event(
                    "Re-sent TPDO1 now that the master (node "
                    f"{MASTER_NODE_ID}) has confirmed OPERATIONAL, in case the "
                    "first transmission landed before its process image was ready."
                )
                tpdo1_resent_after_master = True

            time.sleep(0.2)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        network.disconnect()


if __name__ == "__main__":
    main()