import random
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

INPUT_DB_NUMBER = 9    # MappedCanProcessImageInputs
OUTPUT_DB_NUMBER = 1   # MappedCanProcessImageOutputs

# ---------------------------------------------------------------------------
# DB layout
#
# NOTE: these offsets are calculated by hand from the UDT field order/sizes
# (S7 aligns 2-byte+ types to even byte offsets). TIA only finalizes real
# offsets once the DB is compiled -- confirm these against the compiled
# block's Offset column before trusting them.
# ---------------------------------------------------------------------------

# --- Input DB: double buffer, no handshake ---------------------------------
# BufferA @ 0  : Byte, Byte, Int, Int, Int, Int   (10 bytes)
# BufferB @ 10 : same layout                       (10 bytes)
# ActiveBuffer @ 20 : Byte
INPUT_BUFFER_SIZE = 10
INPUT_BUFFER_A_OFFSET = 0
INPUT_BUFFER_B_OFFSET = 10
INPUT_ACTIVE_BUFFER_OFFSET = 20
INPUT_TOTAL_SIZE = 21  # BufferA(10) + BufferB(10) + ActiveBuffer(1), read as one job

# --- Output DB: double buffer + handshake -----------------------------------
# BufferA @ 0  : Byte, Byte, Int, Int, Int, Int, UInt(RequestID)  (12 bytes)
# BufferB @ 12 : same layout                                       (12 bytes)
# ResponseID @ 24 : UInt
# ActiveBuffer @ 26 : Byte
OUTPUT_BUFFER_SIZE = 12
OUTPUT_BUFFER_A_OFFSET = 0
OUTPUT_BUFFER_B_OFFSET = 12
OUTPUT_RESPONSE_ID_OFFSET = 24
OUTPUT_ACTIVE_BUFFER_OFFSET = 26

POLL_INTERVAL_SECONDS = 0.2
ACK_TIMEOUT_SECONDS = 3.0
ACK_POLL_INTERVAL_SECONDS = 0.05
FLIP_VERIFY_ATTEMPTS = 8
FLIP_VERIFY_RETRY_DELAY = 0.10
PAYLOAD_VERIFY_ATTEMPTS = 8
PAYLOAD_VERIFY_RETRY_DELAY = 0.10

# Diagnostics: flip these to isolate whether failures are threading-related.
ENABLE_INPUT_POLLING = True
DEBUG_ACK_POLL = True


# ---------------------------------------------------------------------------
# Input side (CAN -> PC): decode + double-buffer-safe read
# ---------------------------------------------------------------------------


class CanInputImage:
    """Decoded view of one input buffer (SlaveDigitalInput/SlaveAnalogInput fields)."""

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
    if len(data) != INPUT_BUFFER_SIZE:
        raise ValueError(f"Expected {INPUT_BUFFER_SIZE} bytes, received {len(data)}.")

    digital_input1, digital_input2 = data[0], data[1]
    analog1, analog2, analog3, analog4 = struct.unpack(">4h", data[2:10])

    return CanInputImage(digital_input1, digital_input2, analog1, analog2, analog3, analog4)


# ---------------------------------------------------------------------------
# Output side (PC -> CAN): command payload + double buffer + handshake
# ---------------------------------------------------------------------------


class OutputCommand:
    """Mutable command state. Every send re-transmits the full struct, so
    fields you don't touch keep carrying their last-set value forward."""

    __slots__ = (
        "digital_output1",
        "digital_output2",
        "analog_output1",
        "analog_output2",
        "analog_output3",
        "analog_output4",
    )

    def __init__(self):
        self.digital_output1 = 0x0A
        self.digital_output2 = 0x0B
        self.analog_output1 = 0
        self.analog_output2 = 1234
        self.analog_output3 = 2345
        self.analog_output4 = 3456

    def encode(self, request_id: int) -> bytearray:
        return bytearray(
            struct.pack(
                ">BB4hH",
                self.digital_output1,
                self.digital_output2,
                self.analog_output1,
                self.analog_output2,
                self.analog_output3,
                self.analog_output4,
                request_id,
            )
        )


def next_request_id(previous_response_id: int) -> int:
    """Pick a new, nonzero transaction ID that isn't the currently-acked one."""

    candidate = random.randint(1, 65535)

    while candidate == previous_response_id:
        candidate = random.randint(1, 65535)

    return candidate


# ---------------------------------------------------------------------------
# PLC link
# ---------------------------------------------------------------------------


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

    def _read_bytes(self, db_number: int, offset: int, size: int) -> bytes:
        """Read a contiguous byte range while holding the PLC connection lock."""
        with self._lock:
            return bytes(self._client.db_read(db_number, offset, size))

    def _read_byte(self, db_number: int, offset: int) -> int:
        return self._read_bytes(db_number, offset, 1)[0]

    # --- Input: double-buffer-safe read -------------------------------

    def read_input_image(self) -> CanInputImage:
        """Read BufferA, BufferB, and ActiveBuffer in a single call.

        Because this is one contiguous read within the negotiated PDU size,
        the CPU firmware services it as one atomic job -- there's no gap
        between "check which buffer is active" and "read that buffer" for
        the PLC to flip in, unlike issuing those as separate round trips."""

        with self._lock:
            data = self._client.db_read(INPUT_DB_NUMBER, INPUT_BUFFER_A_OFFSET, INPUT_TOTAL_SIZE)

        active = data[INPUT_ACTIVE_BUFFER_OFFSET]

        if active == 0:
            payload = data[INPUT_BUFFER_A_OFFSET : INPUT_BUFFER_A_OFFSET + INPUT_BUFFER_SIZE]
        else:
            payload = data[INPUT_BUFFER_B_OFFSET : INPUT_BUFFER_B_OFFSET + INPUT_BUFFER_SIZE]

        return decode_input_image(payload)

    # --- Output: double-buffer write + handshake + flow control -------

    def _read_response_id(self) -> int:
        with self._lock:
            data = self._client.db_read(OUTPUT_DB_NUMBER, OUTPUT_RESPONSE_ID_OFFSET, 2)

        if DEBUG_ACK_POLL:
            print(f"    [ack-poll] raw bytes at offset {OUTPUT_RESPONSE_ID_OFFSET}: {data.hex()}")

        return struct.unpack(">H", data)[0]

    def _write_bytes_and_verify(
        self,
        db_number: int,
        offset: int,
        data: bytes,
        *,
        attempts: int,
        retry_delay: float,
        description: str,
    ) -> None:
        """Write a byte range and read it back until the bytes match.

        This is deliberately used for both the output payload and
        ActiveBuffer. A successful snap7 db_write call only tells us that
        the request completed without raising an exception; the read-back
        verifies that the PLC DB actually contains the bytes we intended to
        write before we proceed.
        """
        expected = bytes(data)

        for attempt in range(attempts):
            with self._lock:
                self._client.db_write(db_number, offset, bytearray(expected))

            confirmed = self._read_bytes(db_number, offset, len(expected))

            if confirmed == expected:
                return

            print(
                f"[output] {description} did not verify "
                f"(expected {expected.hex()}, read back {confirmed.hex()}); "
                f"retrying ({attempt + 1}/{attempts})."
            )
            time.sleep(retry_delay)

        raise RuntimeError(
            f"{description} could not be confirmed after {attempts} attempts. "
            f"Expected {expected.hex()}, but the PLC read back "
            f"{confirmed.hex()}."
        )

    def _write_active_buffer_and_verify(self, index: int) -> None:
        """Write ActiveBuffer and verify the byte actually landed in the PLC."""

        self._write_bytes_and_verify(
            OUTPUT_DB_NUMBER,
            OUTPUT_ACTIVE_BUFFER_OFFSET,
            bytes([index]),
            attempts=FLIP_VERIFY_ATTEMPTS,
            retry_delay=FLIP_VERIFY_RETRY_DELAY,
            description=f"ActiveBuffer flip to {index}",
        )

    def _write_output_payload_and_verify(self, offset: int, payload: bytes) -> None:
        """Write the complete inactive output buffer and verify every byte.

        The payload contains both the output values and RequestID. It is
        verified before ActiveBuffer is flipped, so the PLC cannot be told to
        consume a buffer whose contents have not been confirmed.
        """

        self._write_bytes_and_verify(
            OUTPUT_DB_NUMBER,
            offset,
            payload,
            attempts=PAYLOAD_VERIFY_ATTEMPTS,
            retry_delay=PAYLOAD_VERIFY_RETRY_DELAY,
            description=(
                f"output payload at DB{OUTPUT_DB_NUMBER}, offset {offset}"
            ),
        )

    def send_output_command(
        self,
        command: OutputCommand,
        wait_for_ack: bool = True,
        ack_timeout: float = ACK_TIMEOUT_SECONDS,
    ) -> int:
        """Write `command` into the inactive output buffer, flip
        ActiveBuffer (verifying the flip actually took effect), and
        (optionally) block until the PLC acknowledges it via ResponseID.
        Returns the RequestID that was sent."""

        active = self._read_byte(OUTPUT_DB_NUMBER, OUTPUT_ACTIVE_BUFFER_OFFSET)
        inactive_index = 1 if active == 0 else 0
        inactive_offset = OUTPUT_BUFFER_B_OFFSET if active == 0 else OUTPUT_BUFFER_A_OFFSET

        previous_response_id = self._read_response_id()
        request_id = next_request_id(previous_response_id)

        payload = command.encode(request_id)

        # Write the complete inactive buffer (payload + RequestID) and verify
        # it by reading the same bytes back. Only after the entire payload is
        # confirmed do we make that buffer active.
        self._write_output_payload_and_verify(inactive_offset, payload)

        # Flip last, and confirm it actually stuck before trusting it.
        self._write_active_buffer_and_verify(inactive_index)

        print(
            f"[output] Sent RequestID={request_id} -> "
            f"buffer {'A' if inactive_index == 0 else 'B'}"
        )

        if not wait_for_ack:
            return request_id

        deadline = time.monotonic() + ack_timeout

        while time.monotonic() < deadline:
            response_id = self._read_response_id()

            if response_id == request_id:
                print(f"[output] RequestID={request_id} acknowledged.")
                return request_id

            time.sleep(ACK_POLL_INTERVAL_SECONDS)

        raise TimeoutError(
            f"No PLC acknowledgment for RequestID {request_id} within {ack_timeout:.1f}s."
        )


# ---------------------------------------------------------------------------
# Runtime loops
# ---------------------------------------------------------------------------

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
    """Prompt the user for new SlaveAnalogOutput1 values and send them
    with flow control -- each send blocks until the PLC acknowledges it
    before the prompt returns."""

    command = OutputCommand()

    print(
        f"Enter an integer (-32768..32767) to write to SlaveAnalogOutput1 "
        f"(DB{OUTPUT_DB_NUMBER}). Ctrl+C to exit.\n"
    )

    while not stop_event.is_set():
        try:
            raw = input("> ").strip()
        except EOFError:
            break

        if not raw:
            continue

        try:
            command.analog_output1 = int(raw)
        except ValueError as error:
            print(f"Invalid value: {error}")
            continue

        try:
            plc.send_output_command(command, wait_for_ack=True)
        except TimeoutError as error:
            print(f"[output] {error}")
        except RuntimeError as error:
            print(f"[output] Write verification failed: {error}")
        except Exception as error:
            print(f"[output] Write error: {error}")


def main() -> None:
    plc = PlcLink(PLC_IP, PLC_RACK, PLC_SLOT)

    print(f"Connecting to PLC at {PLC_IP}...")
    plc.connect()
    print("Connected.\n")

    poll_thread = None
    if ENABLE_INPUT_POLLING:
        poll_thread = threading.Thread(target=polling_loop, args=(plc,), daemon=True)
        poll_thread.start()
    else:
        print("[diagnostic] Input polling thread disabled (ENABLE_INPUT_POLLING = False).\n")

    try:
        input_prompt_loop(plc)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        if poll_thread is not None:
            poll_thread.join(timeout=2.0)
        plc.disconnect()
        print("\nDisconnected.")


if __name__ == "__main__":
    main()
