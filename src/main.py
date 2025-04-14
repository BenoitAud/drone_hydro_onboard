# TODO SETUP GITHUB

# Ce script sert a l'acquisition des mesures de distance produites par le sonar Ping2.
# Il utilise un GPS branché sur USB, un capteur de température (célérité du son), ainsi que le sonar Ping2.
# Suppose que le sonar occupe le port /dev/ttyUSB0 avec un baud rate de 115200
# Suppose que le GPS occupe le port USB /dev/ttyACM0 avec un baud rate de 9600
# Suppose que le capteur de temperature est branché sur les pins GPIO (Ground, 3.3V et GPIO4)
####################################################### Fonctions support -importation
###########################################################################
########

import time as tiime # changement de nom pour éviter le conflits avec datetime.time
from brping import Ping1D # blue robotics library, téléchargement github requis
import sys
import os
sys.path.append("/home/pi/SunFounder_SensorKit_for_RPi2/Python/") # bibliotheque pour utilisation capteurs pins GPIO (LCD et temperature)
import LCD1602
from datetime import datetime
from datetime import timedelta
from datetime import date
import serial

# Fonction de conversion F -> C de la température
def tC_func(tF):
    return((tF-32)*5/9)

# Célérité son en fonction de la température
def ss(temperatureC):
    ss = 1404.3+4.7*temperatureC-0.04*temperatureC*temperatureC
    return(1000*ss) #unit = mm/s

####################################################### Initialisation
###########################################################################
########
# Parametres initiaux
duree = 1 # Temps d'acquisition en minutes
tC = 20 #temperature de l'eau en degrés Celcius
# tF = 60 #temperature de l'eau en degrés Farenheit

# Initialize LCD screen
LCD1602.init(0x27, 1)

# Initialize sonar Ping2
myPing = Ping1D()
myPing.connect_serie("/dev/ttyUSB0", 115200)
if myPing.initialize() is False:
    LCD1602.write(0, 0, "Init. failed")
    print("L'initialisation a échoué")
else:
    print("Bravo petit canard, tu as réussi ton acquisition \n")
    LCD1602.write(0, 0, "Ready")

####################################################### Parametres pour le GPS ###################################################################################

# Parametrage -- Modifiable !!
port = "/dev/ttyACM0" # port USB du GPS, vérifier via la commande dmesg |grep tty
baud = 9600

# Ouvrir le port serie
ser = serial.Serial(port, baud)

# Suppression ancien fichier et creation d'un nouveau
try:
    os.remove("/home/pi/logs/GPS/GPS.txt")
except OSError:
    pass
print("Old GPS file removed")

with open("/home/pi/logs/GPS/GPS.txt", "a+") as f:
    f.write("Nouvelle acquisition GPS \n")
    f.write("$, Heure (UTC), Latitude, Hemisphere Lat, Longitude, Hemisphere Long, quality indicator, Nb satellites. HDOP, Altitude, Unit, Geoidal separation, Unit, Age of diff. data, Difference ref station ID")
#All the columns NMEA format

####################################################### Parametres pour la temperature ###################################################################################
input_folder = '/sys/bus/w1/devices'
output_folder = '/home/pi/logs/temperature'
global ds18b20
for i in os.listdir(input_folder):
    if i!='w1_bus_master1':
        ds18b20 = i
t
emp_addr = input_folder + '/' + ds18b20 + '/w1_slave'

####################################################### C'est parti pour une acquisition en continu !!! ###################################################################################
# Initialisation des variables
i_failure = 0 #nombre de fois que le sonar n'arrive pas a se connecter, garder en memoire pour un reboot apres 5 erreurs.
i_time = 0 # permet de definir le temps initial pour respect de la duree d'acquisition fixee par l'utilisateur
lat_list = []
long_list = []
time_list = []
dist_list = []
time_init = 0 #heure de début -- doit etre enregistré pour mesurer la duree de l'acquisition
time_current = 0 #temps courant
timeout = timedelta(minutes = duree) #duree prévue de l'acquisition
today = date.today()
default_lat = 0.000
default_long = 0.000

while time_current-time_init < timeout and i_failure < 6:
    tps_ecoule = time_current-time_init

    # Lecture GPS
    line = ser.readline().decode("utf-8")
    words = line.split(',') #séparer tous les champs
    type_GPS = str(words[0])
    if type_GPS not in ["$GPGGA", "$GPGLL", "$GPRMC"]: #ne garder que les acquisitions de type gpgga
        #print("Not a GPGGA acquisiition, format is %s" %str(words[0]))
        continue
    elif type_GPS == "$GPGGA":
        print("Temps écoulé (s): ", tps_ecoule.seconds) # mesure de l'avancement
        time = datetime.strptime(str(words[1]), "%H%M%S.%f")
        lat_current = str(words[2]+words[3])
        long_current = str(words[4]+words[5])
    elif type_GPS == "$GPGLL": #pas de données position => on prend les valeurs par défaut définies
        print("Temps écoulé (s): ", tps_ecoule.seconds) # mesure de l'avancement
        time = datetime.strptime(str(words[5]), "%H%M%S.%f")
        lat_current = str(default_lat)
        long_current = str(default_long)
    elif type_GPS == "$GPRMC":
        print("Temps écoulé (s): ", tps_ecoule.seconds) # mesure de l'avancement
        time = datetime.strptime(str(words[1]), "%H%M%S.%f")
        lat_current = str(words[3]+words[4])
        long_current = str(words[5]+words[6])
    try:
        time = time.replace(year = today.year, month = today.month, day = today.day) #avec le format nmea, on a que l'heure, pas la date!!
        if i_time == 0:
            time_init = time #définition du temps 0
            time_current = time
            i_time += 1
        else:
            time_current = time #définition du temps actuel
        # garder en mémoire les données
        with open("/home/pi/logs/GPS/GPS.txt", "a+") as f:
            f.write("%s\n" % line)
 
        time_list.append(time_current)
        lat_list.append(lat_current)
        long_list.append(long_current)
        chemin_success = "file_" + datetime.strftime("%Y-%m-%d_%H%M%S") + ".txt"

        f = open(os.path.join("/home/pi/ping-python/", chemin_success), "a+") #fichier pour l'enregistrement des données
    
    except ValueError:
        print("Value Error on GPS")
        continue

    # update temperature from sensor
    with open(temp_addr, "r") as temperature_file:
        text = temperature_file.read()
        i_tC = text.find('t=')
        tC = float(text[i_tC+2:])/1000
        #print("water temperature is : %f C" %tC)

    try:
        myPing.set_speed_of_sound(int(ss(tC)))
        print("Sound velocity updated (mm/s): ", ss(tC))
        # get the data
        data = myPing.get_distance()
        
        # erreur 1: le sonar ne renvoie pas de donnees. Tentative de reinitialisation!
        if data is None:
            LCD1602.write(0, 0, "No data")
            print("No data")
            print("Reinitialisation")
            myPing.connect_serie("/dev/ttyUSB0", 115200)
            myPing.initialize()
            data = myPing.get_distance()
            
            if data is None:
                data = {"distance": -1, "confidence": -1}
                LCD1602.write(0, 1, "Reinit failed")
                print("Reinitialisation failed")
                i_failure += 1
            else:
                LCD1602.write(0, 1, "Data received")
                print("Reinitialisation succeeded")

            distance_current = str(data["distance"])
            confidence_current = str(data["confidence"])
            dist_list.append(distance_current)

        distance_current = str(data["distance"])
        confidence_current = str(data["confidence"])
        print("Distance =" + distance_current + " mm \n")
        msg = "Depth = " + distance_current + " mm"
        LCD1602.write(0, 0, msg)
        msg_err = "Conf = " + confidence_current + " %"
        LCD1602.write(0, 1, msg_err)
        dist_list.append(distance_current)
        tiime.sleep(0.5)
        LCD1602.clear()

        f.write("%(distance)s, %(date)s, %(lat)s, %(long)s, %(confidence)s\n" % {'distance': distance_current, 'date': time_current,'lat': lat_current, 'long': long_current, 'confidence': confidence_current})
        LCD1602.write("Acquisition sauvée :)")
        print("Acquisition sauvegardée")
        f.close()
        print("fichier fermé")

    # Deuxieme issue: sonar ne repond plus.
    except serial.SerialException:
        print("Sonar ne repond plus")
        data = {"distance": -1, "confidence": -1}
        distance_current = str(data["distance"])
        confidence_current = str(data["confidence"])
        LCD1602.clear()
        LCD1602.write(0, 1, "Acqui failed")
        print("Acquisition failed")
        i_failure += 1
        f.write("%(distance)s, %(date)s, %(lat)s, %(long)s, %(confidence)s\n" % {'distance': distance_current, 'date': time_current,'lat':
        lat_current, 'long': long_current, 'confidence': confidence_current})
        LCD1602.write("Acquisition sauvée :)")
        continue

#rebooter et relancer au bout de 5 erreurs
if i_failure > 5:
    chemin = "fail_" + str(time_current) + ".txt"
    print("Rebooting")
    LCD1602.clear()
    LCD1602.write(0, 0, "Rebooting now")
    tiime.sleep(0.5)
    os.system("sudo reboot")

####################################################### Sauvegarde et fin
###########################################################################
########
LCD1602.clear()
LCD1602.write(0,0,"Fin")
print("Fin de l'acquisition")