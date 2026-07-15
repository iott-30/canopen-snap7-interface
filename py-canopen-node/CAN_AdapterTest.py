# Tests CAN to USB adapter to see if python and Windows can communicate with it.
# Start bus and output any traffic
# I think some extra configuration was needed to get the CAN library to find the libusb1 DLL.

import usb.backend.libusb1
import libusb_package
import can

backend = usb.backend.libusb1.get_backend(find_library=libusb_package.find_library)

import can.interfaces.gs_usb as gs_usb_mod
_original_find = usb.core.find
gs_usb_mod.usb.core.find = lambda *a, **kw: _original_find(*a, **{**kw, 'backend': backend})

bus = can.interface.Bus(channel='0', interface='gs_usb', bitrate=500000)
print("Bus opened OK")

msg = bus.recv(timeout=60.0)
print(msg)

bus.shutdown()   