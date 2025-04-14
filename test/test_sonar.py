#!/usr/bin/env python3

from brping import Ping1D

myPing = Ping1D()
myPing.connect_serial("/dev/ttyUSB0", 115200)

print("OK")
