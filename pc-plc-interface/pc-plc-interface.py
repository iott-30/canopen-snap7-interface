import struct
import threading
import time
from typing import Optional

import snap7


# ---------------------------------------------------------------------------
# PLC connection settings
# ---------------------------------------------------------------------------

PLC_IP = "192.168.0.1"
PLC_RACK = 0
PLC_SLOT = 1

# CAN process image DBs (mapped via UDTs in TIA Portal)
INPUT_DB_NUMBER = 9    # MappedCanProcessImageInputs  (typeCanProcessImageInputs)
OUTPUT_DB_NUMBER = 1   # MappedCanProcessImageOutputs (typeCanProcessImageOutputs)

# Both DBs share the same UDT layout:
#   SlaveDigital1 : Byte @ offset 0
#   SlaveDigital2 : Byte @ offset 1
#   SlaveAnalog1  : Int  @ offset 2   (signed 16-bit, big-endian)
#   SlaveAnalog2  : Int  @ offset 4
#   SlaveAnalog3  : Int  @ offset 6
#   SlaveAnalog4  : Int  @ offset 8
PROCESS_IMAGE_SIZE = 10  # bytes

OUTPUT_ANALOG1_OFFSET = 2  # SlaveAnalogOutput1 in DB1

POLL_INTERVAL_SECONDS = 0.2


class CanInputImage:
    """Decoded view of one MappedCanProcessImageInputs UDT instance."""

    __slots__ = (
        "digital_input1",
        "digital_input2",
        "analog_input1",
        "analog_input2",
        "analog_input3",
        "analog_input4",
    )

    def __init__(
        self,
        digital_input1: int,
        digital_input2: int,
        analog_input1: int,
        analog_input2: int,
        analog_input3: int,
        analog_input4: int,
    ):
        self.digital_input1 = digital_input1
        self.digital_input2 = digital_input2
        self.analog_input1 = analog_input1
        self.analog_input2 = analog_input2
        self.analog_input3 = analog_input3
        self.analog_input4 = analog_input4

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, CanInputImage):
            return NotImplemented

        return (
            self.digital_input1 == other.digital_input1
            and self.digital_input2 == other.digital_input2
            and self.analog_input1 == other.analog_input1
            and self.analog_input2 == other.analog_input2
            and self.analog_input3 == other.analog_input3
            and self.analog_input4 == other.analog_input4
        )

    def __str__(self) -> str:
        return (
            f"Digital1={self.digital_input1:#04x} "
            f"Digital2={self.digital_input2:#04x} "
            f"Analog1={self.analog_input1} "
            f"Analog2={self.analog_input2} "
            f"Analog3={self.analog_input3} "
            f"Analog4={self.analog_input4}"
        )


def decode_input_image(data: bytes) -> CanInputImage:
    """Decode DB9 (MappedCanProcessImageInputs) into typed fields."""

    if len(data) != PROCESS_IMAGE_SIZE:
        raise ValueError(f"Expected {PROCESS_IMAGE_SIZE} bytes, received {len(data)}.")

    digital_input1, digital_input2 = data[0], data[1]
    analog_input1, analog_input2, analog_input3, analog_input4 = struct.unpack(
        ">4h", data[2:10]
    )

    return CanInputImage(
        digital_input1,
        digital_input2,
        analog_input1,
        analog_input2,
        analog_input3,
        analog_input4,
    )


def encode_int16(value: int) -> bytearray:
    """Encode one Siemens Int (signed 16-bit, big-endian)."""

    if not -32768 <= value <= 32767:
        raise ValueError(f"Int value out of range (-32768..32767): {value}")

    return bytearray(struct.pack(">h", value))


class PlcLink:
    """Thread-safe wrapper around a single snap7 client connection."""

    def __init__(self, ip: str, rack: int, slot: int):
        self._client = snap7.Client()
        self._lock = threading.Lock()
        self._ip = ip
        self._rack = rack
        self._slot = slot

    def connect(self) -> None:
        self._client.connect(self._ip, self._rack, self._slot)

        if not self._client.get_connected():
            raise ConnectionError("The S7 connection was not established.")

    def disconnect(self) -> None:
        if self._client.get_connected():
            self._client.disconnect()

    def read_input_image(self) -> CanInputImage:
        with self._lock:
            data = self._client.db_read(INPUT_DB_NUMBER, 0, PROCESS_IMAGE_SIZE)

        return decode_input_image(data)

    def write_analog_output1(self, value: int) -> None:
        data = encode_int16(value)

        with self._lock:
            self._client.db_write(OUTPUT_DB_NUMBER, OUTPUT_ANALOG1_OFFSET, data)


stop_event = threading.Event()


def polling_loop(plc: PlcLink) -> None:
    """Continuously read the CAN input DB and print changes."""

    last_seen: Optional[CanInputImage] = None

    while not stop_event.is_set():
        try:
            current = plc.read_input_image()
        except Exception as error:
            print(f"[input]  Read error: {error}")
            time.sleep(POLL_INTERVAL_SECONDS)
            continue

        if last_seen is None or current != last_seen:
            print(f"[input]  DB{INPUT_DB_NUMBER}: {current}")
            last_seen = current

        time.sleep(POLL_INTERVAL_SECONDS)


def input_prompt_loop(plc: PlcLink) -> None:
    """Prompt the user for new SlaveAnalogOutput1 values and write them."""

    print(
        f"Enter an integer (-32768..32767) to write to SlaveAnalogOutput1 "
        f"(DB{OUTPUT_DB_NUMBER}, offset {OUTPUT_ANALOG1_OFFSET}). Ctrl+C to exit.\n"
    )

    while not stop_event.is_set():
        try:
            raw = input("> ").strip()
        except EOFError:
            break

        if not raw:
            continue

        try:
            value = int(raw)
            plc.write_analog_output1(value)
        except ValueError as error:
            print(f"Invalid value: {error}")
            continue
        except Exception as error:
            print(f"[output] Write error: {error}")
            continue

        print(f"[output] DB{OUTPUT_DB_NUMBER}: SlaveAnalogOutput1 = {value}")


def main() -> None:
    plc = PlcLink(PLC_IP, PLC_RACK, PLC_SLOT)

    print(f"Connecting to PLC at {PLC_IP}...")
    plc.connect()
    print("Connected.\n")

    poll_thread = threading.Thread(target=polling_loop, args=(plc,), daemon=True)
    poll_thread.start()

    try:
        input_prompt_loop(plc)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        poll_thread.join(timeout=2.0)
        plc.disconnect()
        print("\nDisconnected.")


if __name__ == "__main__":
    main()