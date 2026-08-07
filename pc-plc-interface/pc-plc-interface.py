import snap7

plc = snap7.Client()
plc.connect("192.168.0.1", 0, 1)

print(plc.get_connected())

