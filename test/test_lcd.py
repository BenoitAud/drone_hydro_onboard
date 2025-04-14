#!/usr/bin/env python3

# RUNS ON STARTUP

from RPLCD.i2c import CharLCD
from time import sleep

# Initialize LCD (Change 0x27 to 0x3F if necessary)
lcd = CharLCD('PCF8574', 0x27) # 0x27 for 16x2, 0x3F for 20x4

lcd.clear()
lcd.write_string("DRONE HYDROGRAPHIQUE")
lcd.write_string("--------DEMO--------")
lcd.write_string("                    ")
lcd.write_string("      REPER 3D      ")

print("OK")
