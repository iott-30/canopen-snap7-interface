"""
CANopen slave node simulation.

Opens the RH02 (gs_usb) adapter, loads a simulated device from an EDS file
as a canopen LocalNode, boots it up over the bus, and logs traffic and
device events to both the terminal and a file in real time.
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


def ask_yes_no(prompt: str) -> bool:
    while True:
        choice = input(f"{prompt} (y/n): ").strip().lower()
        if choice in ("y", "n"):
            return choice == "y"
        print("Please enter 'y' or 'n'.")


show_traffic_in_console = ask_yes_no("Show raw CAN traffic in the console?")

# --- Logging setup ---
# `logger` (device/status events -- NMT state, PDO state/changes, startup
# messages) always goes to both the terminal and the file.
# `traffic_logger` (one line per raw CAN frame) always goes to the file, but
# only goes to the terminal if the user opted in above.
_fmt = logging.Formatter("%(asctime)s  %(message)s", "%H:%M:%S")

file_handler = logging.FileHandler(LOG_FILE, mode="a")
file_handler.setFormatter(_fmt)

console_handler = logging.StreamHandler()
console_handler.setFormatter(_fmt)

logger = logging.getLogger("canopen_sim")
logger.setLevel(logging.DEBUG)
logger.addHandler(file_handler)
logger.addHandler(console_handler)

traffic_logger = logging.getLogger("canopen_sim.traffic")
traffic_logger.setLevel(logging.DEBUG)
traffic_logger.propagate = False  # don't also run through logger's handlers (would double-print)
traffic_logger.addHandler(file_handler)
if show_traffic_in_console:
    traffic_logger.addHandler(console_handler)


class TrafficLogger(can.Listener):
    """Logs every CAN frame the Notifier sees to traffic_logger above."""

    def on_message_received(self, msg: can.Message):
        data_hex = msg.data.hex(" ").upper()
        traffic_logger.info(f"ID=0x{msg.arbitration_id:03X}  DLC={msg.dlc}  Data=[{data_hex}]")


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


def setup_tpdo(node, pdo_num, cob_id, mapping, initial_values, fmt):
    """Configure and enable a TPDO. `mapping` is a list of (index, subindex)
    tuples; `initial_values` is a same-length list of values to load and
    transmit once the node reaches OPERATIONAL. `fmt` formats a value for
    logging (e.g. binary for digital I/O, decimal for analog)."""
    pdo_map = node.tpdo[pdo_num]
    pdo_map.clear()
    for index, subindex in mapping:
        pdo_map.add_variable(index, subindex)
    pdo_map.cob_id = cob_id
    pdo_map.enabled = True
    return {
        "name": f"TPDO{pdo_num}",
        "map": pdo_map,
        "values": initial_values,
        "fmt": fmt,
        "sent": False,
        "resent": False,
    }


def setup_rpdo(node, network, pdo_num, cob_id, mapping, fmt):
    """Configure and enable an RPDO, subscribe it to the network, and attach
    a callback that logs the initial state and any subsequent value changes."""
    pdo_map = node.rpdo[pdo_num]
    pdo_map.clear()
    for index, subindex in mapping:
        pdo_map.add_variable(index, subindex)
    pdo_map.cob_id = cob_id
    pdo_map.enabled = True
    pdo_map.subscribe()  # register with the network so incoming frames actually reach us

    name = f"RPDO{pdo_num}"
    num_subs = len(mapping)
    last_values = [None] * num_subs

    def on_message(pdo_map):
        nonlocal last_values
        current = [pdo_map[i].raw for i in range(num_subs)]
        if last_values == [None] * num_subs:
            state_str = "  ".join(f"sub{i + 1}={fmt(v)}" for i, v in enumerate(current))
            log_event(f"{name} initial state: {state_str}")
        else:
            changes = [
                f"sub{i + 1}: {fmt(last_values[i])} -> {fmt(current[i])}"
                for i in range(num_subs) if current[i] != last_values[i]
            ]
            if changes:
                log_event(f"{name} changed - " + ", ".join(changes))
        last_values = current

    pdo_map.add_callback(on_message)
    return pdo_map


def fmt_bin8(v: int) -> str:
    return f"0b{v:08b}"


def fmt_decimal(v: int) -> str:
    return str(v)


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
    # COB-IDs and mappings are hardcoded to match what CM Configuration
    # Studio actually assigns (confirmed from captured traffic) rather than
    # tracked live from the master's SDO writes -- simple and correct as long
    # as Configuration Studio isn't changed to use different values.
    #
    # PDO1 pair: 8-bit digital I/O (2-byte mapping: sub1 + sub2 each)
    # PDO2 pair: 16-bit analog I/O (4 channels: sub1-sub4 each)
    tpdos = [
        setup_tpdo(
            node, 1, 0x180 + NODE_ID,
            mapping=[(0x6000, 1), (0x6000, 2)],
            initial_values=[0b10010000, 0x00],
            fmt=fmt_bin8,
        ),
        setup_tpdo(
            node, 2, 0x280 + NODE_ID,
            mapping=[(0x6401, 1), (0x6401, 2), (0x6401, 3), (0x6401, 4)],
            initial_values=[1234, 4567, 7654, 4321],
            fmt=fmt_decimal,
        ),
    ]

    setup_rpdo(
        node, network, 1, 0x200 + NODE_ID,
        mapping=[(0x6200, 1), (0x6200, 2)],
        fmt=fmt_bin8,
    )
    setup_rpdo(
        node, network, 2, 0x300 + NODE_ID,
        mapping=[(0x6411, 1), (0x6411, 2), (0x6411, 3), (0x6411, 4)],
        fmt=fmt_decimal,
    )

    # In both captured logs, the master formally starts node 2 (targeted Start,
    # 01 02) roughly a second *before* it starts itself (01 01) and broadcasts
    # a final Start (01 00) -- and TPDO data only gets picked up into the PLC's
    # process image after that second step. Since our TPDOs are event-driven
    # with no periodic resend (event timer = 0), a single transmission that
    # lands before the master's own Start is simply lost -- nothing triggers
    # us to send it again. Watching the master's own heartbeat (node
    # MASTER_NODE_ID) for OPERATIONAL gives us a real signal to resend
    # against, instead of guessing at a delay.
    master_operational = threading.Event()

    def on_master_heartbeat(can_id, data, timestamp):
        if data == b"\x05":
            master_operational.set()

    network.subscribe(0x700 + MASTER_NODE_ID, on_master_heartbeat)

    logger.info(f"Node {NODE_ID} is up. Logging to {LOG_FILE}. Ctrl+C to stop.")

    # PDOs are only meaningful once the master brings us to OPERATIONAL, so
    # wait for that transition (there's no state-change callback in canopen's
    # NMT implementation, so this just polls) before sending each TPDO's
    # first value.
    last_nmt_state = None

    try:
        while True:
            current_state = node.nmt.state
            if current_state != last_nmt_state:
                logger.info(f"NMT state -> {current_state}")
                last_nmt_state = current_state

            if current_state == "OPERATIONAL":
                for t in tpdos:
                    if not t["sent"]:
                        for i, value in enumerate(t["values"]):
                            t["map"][i].raw = value
                        state_str = "  ".join(
                            f"sub{i + 1}={t['fmt'](v)}" for i, v in enumerate(t["values"])
                        )
                        log_event(f"{t['name']} initial state: {state_str}")
                        t["map"].transmit()
                        t["sent"] = True

            # Covers the observed ordering (node 2 started, TPDOs sent, *then*
            # the master starts itself) by resending once we see confirmation
            # the master is actually ready, regardless of which order the two
            # Start events happened in.
            if master_operational.is_set():
                for t in tpdos:
                    if t["sent"] and not t["resent"]:
                        t["map"].transmit()
                        log_event(
                            f"Re-sent {t['name']} now that the master (node "
                            f"{MASTER_NODE_ID}) has confirmed OPERATIONAL, in case "
                            "the first transmission landed before its process "
                            "image was ready."
                        )
                        t["resent"] = True

            time.sleep(0.2)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        network.disconnect()


if __name__ == "__main__":
    main()