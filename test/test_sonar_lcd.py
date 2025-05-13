#!/usr/bin/env python3

# AVEC tC = 26 degC

from RPLCD.i2c import CharLCD
from time import sleep
from brping import Ping1D


# Célérité du son en fonction de la température
def ss(temperatureC):
    ss = 1404.3+4.7*temperatureC-0.04*temperatureC*temperatureC
    return(1000*ss) #unit = mm/s

myPing = Ping1D()
myPing.connect_serial("/dev/ttyUSB0", 115200)
myPing.initialize()
myPing.set_speed_of_sound(int(ss(26.0)))
#print("Sound velocity updated (mm/s): ", ss(tC))

lcd = CharLCD('PCF8574', 0x27) # 0x27 for 20x4
lcd.clear()

while True:
    try:
        data = myPing.get_distance()

        # erreur 1: le sonar ne renvoie pas de donnees. Tentative de reinitialisation!
        if data is None:
            print("NO DATA")
            print("REINITIALIZING")
            myPing.connect_serial("/dev/ttyUSB0", 115200)
            myPing.initialize()
            data = myPing.get_distance()
            
            if data is None:
                print("FAILED TWICE")
                i_failure += 1
            else:
                print("RESET WORKED")

        lcd.cursor_pos = (0, 0)
        lcd.write_string(f"DISTANCE: {data['distance']:.1f}mm          ")
        lcd.cursor_pos = (1, 0)
        lcd.write_string(f"CONFIANCE: {data['confidence']:.1f}%          ")
        lcd.cursor_pos = (3, 0)
        lcd.write_string("----DEMO PISCINE----")

    # Deuxieme issue: sonar ne repond plus.
    except serial.SerialException:
        print("SONAR NOT RESPONDING")
        continue