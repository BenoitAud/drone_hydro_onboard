# Ce script sert a l'acquisition des mesures de distance produites par le sonar Ping2.
# Il utilise un GPS branché sur USB, un capteur de température (célérité du son), ainsi que le sonar Ping2.
# Suppose que le sonar occupe le port /dev/ttyUSB0 avec un baud rate de 115200
# Suppose que le GPS occupe le port USB /dev/ttyACM0 avec un baud rate de 9600
# Suppose que le capteur de temperature est branché sur les pins GPIO (Ground, 3.3V et GPIO4)

import time as tiime # changement de nom pour éviter le conflits avec datetime.time
from brping import Ping1D # blue robotics library, téléchargement github requis
import sys
import os
import glob
from RPLCD.i2c import CharLCD
from datetime import datetime
from datetime import timedelta
from datetime import date
import serial
import yaml

# Convert Fahrenheit to Celsius
def tC_func(tF):
    return((tF-32)*5/9)

# Sound velocity depending on the temperature
def ss(temperatureC):
    ss = 1404.3+4.7*temperatureC-0.04*temperatureC*temperatureC
    return(1000*ss) #unit = mm/s

# INITIALIZATION

# Load parameters from param.yaml
with open("/home/dronehydro/main_ws/config/param.yaml", "r") as file:
    params = yaml.safe_load(file)

# Access parameters
duree = params.get("duree", 1) # Default to 1 minute if not found
tC = params.get("tC", 20) # Default to 20 degrees Celsius if not found
tF = params.get("tF", 60) # Default to 60 degrees Fahrenheit if not found
port_sonar = params.get("port_sonar", "/dev/ttyUSB0")  # Default to "/dev/ttyUSB0" if not found
port_gps = params.get("port_gps", "/dev/ttyACM0")      # Default to "/dev/ttyACM0" if not found
baud_rate_sonar = params.get("baud_rate_sonar", 115200)  # Default to 115200 if not found
baud_rate_gps = params.get("baud_rate_gps", 9600)  # Default to 9600 if not found
pin_button = params.get("pin_button", 15)  # Default to 15 if not found
path_logs = params.get("path_logs", "/home/dronehydro/main_ws/logs")  # Default to "/home/dronehydro/main_ws/logs" if not found

lcd = CharLCD('PCF8574', 0x27) # 0x27 for 20x4
lcd.clear()
lcd.cursor_pos = (0, 0)
lcd.write_string("----- REPER3D -----")

# Initialize sonar Ping2
myPing = Ping1D()
myPing.connect_serial(port_sonar, baud_rate_sonar)
if myPing.initialize() is False:
    lcd.cursor_pos = (1, 0)
    lcd.write_string("Sonar: INIT. FAILED ")
    print("Sonar initialization failed\n")
else:
    lcd.cursor_pos = (1, 0)
    lcd.write_string("Sonar: READY        ")
    print("Sonar initialized\n")

#   GPS PARAMETERS

# Open serial port
try:
    ser = serial.Serial(port_gps, baud_rate_gps)
    lcd.cursor_pos = (2, 0)
    lcd.write_string("GPS: READY          ")
    print("GPS initialized\n")
except serial.SerialException as e:
    lcd.cursor_pos = (2, 0)
    lcd.write_string("GPS: INIT. FAILED   ")
    print(f"Failed to open GPS port: {port_gps}. Error: {e}")
    ser = None  # Set to None to indicate failure

# Delete previous logs, create new one
try:
    os.remove(f"{path_logs}/GPS/GPS.txt")
except OSError:
    pass
print("Old GPS log deleted")

with open(f"{path_logs}/GPS/GPS.txt", "a+") as f:
    f.write("Nouvelle acquisition GPS \n")
    f.write("$, Heure (UTC), Latitude, Hemisphere Lat, Longitude, Hemisphere Long, quality indicator, Nb satellites. HDOP, Altitude, Unit, Geoidal separation, Unit, Age of diff. data, Difference ref station ID")
#All the columns NMEA format

# TEMPERATURE PARAMETERS

os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28*')[0]
temp_path = device_folder + '/w1_slave'

# ACQUISITION

# Initialisation des variables
i_failure = 0 # Sonar connection errors, max 5
lat_list = []
long_list = []
time_list = []
dist_list = []
default_lat = 0.000
default_long = 0.000

# Example initialization
time_init = datetime.now()
timeout = timedelta(minutes=duree)

# Main loop
while (datetime.now() - time_init) < timeout and i_failure < 6:
    time_current = datetime.now()
    elapsed_time = time_current - time_init

    # Lecture GPS
    line = ser.readline().decode("utf-8")
    words = line.split(',') #séparer tous les champs
    type_GPS = str(words[0])
    if type_GPS not in ["$GPGGA", "$GPGLL", "$GPRMC"]: #ne garder que les acquisitions de type gpgga
        #print("Not a GPGGA acquisiition, format is %s" %str(words[0]))
        continue
    elif type_GPS == "$GPGGA":
        print("Temps écoulé (s): ", elapsed_time.seconds) # mesure de l'avancement
        time = datetime.strptime(str(words[1]), "%H%M%S.%f")
        lat_current = str(words[2]+words[3])
        long_current = str(words[4]+words[5])
    elif type_GPS == "$GPGLL": #pas de données position => on prend les valeurs par défaut définies
        print("Temps écoulé (s): ", elapsed_time.seconds) # mesure de l'avancement
        time = datetime.strptime(str(words[5]), "%H%M%S.%f")
        lat_current = str(default_lat)
        long_current = str(default_long)
    elif type_GPS == "$GPRMC":
        print("Temps écoulé (s): ", elapsed_time.seconds) # mesure de l'avancement
        time = datetime.strptime(str(words[1]), "%H%M%S.%f")
        lat_current = str(words[3]+words[4])
        long_current = str(words[5]+words[6])
    try:
        time = time.replace(year = datetime.now().year, month = datetime.now().month, day = datetime.now().day) #avec le format nmea, on a que l'heure, pas la date!!
        # garder en mémoire les données
        with open(f"{path_logs}/GPS/GPS.txt", "a+") as f:
            f.write("%s\n" % line)
 
        time_list.append(time_current)
        lat_list.append(lat_current)
        long_list.append(long_current)
        chemin_success = "file_" + datetime.now().strftime("%Y-%m-%d2_%H%M%S") + ".txt"

        f = open(os.path.join(f"{path_logs}/sonar/", chemin_success), "a+") #fichier pour l'enregistrement des données
    
    except ValueError:
        print("Value Error on GPS")
        continue

    # update temperature from sensor
    with open(temp_path, "r") as temperature_file:
        lines = temperature_file.readlines()
        while lines[0].strip()[-3:] != 'YES':
            time.sleep(0.2)
            with open(temp_path, "r") as temperature_file_retry:
                lines = temperature_file_retry.readlines()
        
        temp_data = lines[1].split('t=')[-1]
        temp_c = float(temp_data) / 1000.0
        print(f"water temperature is : {temp_c:.2f} C")

    try:
        myPing.set_speed_of_sound(int(ss(tC)))
        print("Sound velocity updated (mm/s): ", ss(tC))
        # get the data
        data = myPing.get_distance()
        
        # erreur 1: le sonar ne renvoie pas de donnees. Tentative de reinitialisation!
        if data is None:
            lcd.cursor_pos = (1, 0)
            lcd.write_string("Sonar: NO DATA      ")
            print("No data")
            print("Reinitialisation")
            myPing.connect_serial(port_sonar, baud_rate_sonar)
            myPing.initialize()
            data = myPing.get_distance()
            
            if data is None:
                data = {"distance": -1, "confidence": -1}
                lcd.cursor_pos = (1, 0)
                lcd.write_string("Sonar: REINIT FAILED")
                print("Reinitialisation failed")
                i_failure += 1
            else:
                lcd.cursor_pos = (1, 0)
                lcd.write_string("Sonar: REINIT OK    ")
                print("Reinitialisation succeeded")

            distance_current = str(data["distance"])
            confidence_current = str(data["confidence"])
            dist_list.append(distance_current)

        distance_current = str(data["distance"])
        confidence_current = str(data["confidence"])
        print("Distance =" + distance_current + " mm \n")

        msg = "Depth = " + distance_current + " mm"
        lcd.cursor_pos = (3, 0)
        lcd.write_string(msg)

        msg_err = "Conf = " + confidence_current + " %"
        lcd.cursor_pos = (3, 8)
        lcd.write_string(msg)

        dist_list.append(distance_current)
        tiime.sleep(0.5)
        # TODO clear LCD?

        f.write("%(distance)s, %(date)s, %(lat)s, %(long)s, %(confidence)s\n" % {'distance': distance_current, 'date': time_current,'lat': lat_current, 'long': long_current, 'confidence': confidence_current})
        lcd.cursor_pos = (3, 0)
        lcd.write_string("---- File saved ----")
        print("Acquisition sauvegardée")
        f.close()
        print("fichier fermé")

    # Deuxieme issue: sonar ne repond plus.
    except serial.SerialException:
        print("Sonar ne repond plus")
        data = {"distance": -1, "confidence": -1}
        distance_current = str(data["distance"])
        confidence_current = str(data["confidence"])
        lcd.cursor_pos = (1, 0)
        lcd.write_string("Sonar: NO DATA      ")
        print("Acquisition failed")
        i_failure += 1
        f.write("%(distance)s, %(date)s, %(lat)s, %(long)s, %(confidence)s\n" % {'distance': distance_current, 'date': time_current,'lat':
        lat_current, 'long': long_current, 'confidence': confidence_current})
        lcd.cursor_pos = (3, 0)
        lcd.write_string("---- File saved ----")
        continue

#rebooter et relancer au bout de 5 erreurs
if i_failure > 5:
    chemin = "fail_" + str(time_current) + ".txt"
    print("Rebooting")
    lcd.cursor_pos = (3, 0)
    lcd.write_string("Rebooting...        ")
    tiime.sleep(0.5)
    #os.system("sudo reboot") TODO NECESSARY?

lcd.cursor_pos = (3, 0)
lcd.write_string("---- Done ----")
print("Fin de l'acquisition")