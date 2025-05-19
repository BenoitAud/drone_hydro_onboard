# PROGRAMME D'ACQUISITION DE DONNÉES SONAR ET GPS

import time as tiime
from brping import Ping1D
import sys
import os
import glob
from RPLCD.i2c import CharLCD
from datetime import datetime, timedelta, date
import serial
import yaml
import subprocess
import threading
import RPi.GPIO as GPIO

# Add right after your imports section
def restart_program():
    """Reset variables and start a fresh acquisition cycle"""
    print("Restarting program for new acquisition cycle...")
    # Close any open resources before restart
    try:
        if 'gps_log_handle' in globals() and gps_log_handle:
            gps_log_handle.close()
        if 'sonar_log_handle' in globals() and sonar_log_handle:
            sonar_log_handle.close()
    except:
        pass
    
    # Execute the program again
    python = sys.executable
    os.execl(python, python, *sys.argv)

# ---------------------------
# Helpers for log file management
# ---------------------------

def _try_create_file(path, timeout=2):
    result = {"success": False, "error": None}
    def target():
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "a"):
                pass
            result["success"] = True
        except Exception as e:
            result["error"] = e
    t = threading.Thread(target=target, daemon=True)
    t.start()
    t.join(timeout)
    if t.is_alive() or not result["success"]:
        return False, result["error"]
    return True, None

def _ensure_file(path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "a"):
        pass
    return path

def create_log_file(filename, extension="txt", timestamp_format=None, timeout=2, subdir=""):
    fmt = timestamp_format or log_name_format or "%Y-%m-%d_%H-%M-%S"
    now = datetime.now()
    try:
        ts = now.strftime(fmt)
    except Exception as e:
        print(f"Error formatting timestamp ({e}), using fallback")
        ts = now.strftime("%Y-%m-%d_%H-%M-%S")
    base = f"{ts}_{filename}.{extension}"
    primary_path = os.path.join(logs_path, subdir, base)
    ok, err = _try_create_file(primary_path, timeout)
    if ok:
        return primary_path
    print(f"Primary log path failed ({err}), falling back…")
    fallback_path = os.path.join(fallback_logs_path, subdir, base)
    return _ensure_file(fallback_path)

# ---------------------------
# INITIALIZATION
# ---------------------------

# Load parameters from param.yaml
try:
    with open("/home/dronehydro/main_ws/config/param.yaml", "r") as file:
        params = yaml.safe_load(file)
except FileNotFoundError:
    print("ERROR: param.yaml not found. Please check the path.")
    params = {}

duree              = params.get("duree", 1)
tC                 = params.get("tC", 20)
tF                 = params.get("tF", 60)
port_sonar         = params.get("port_sonar", "/dev/ttyUSB0")
port_gps           = params.get("port_gps", "/dev/ttyACM0")
baud_rate_sonar    = params.get("baud_rate_sonar", 115200)
baud_rate_gps      = params.get("baud_rate_gps", 9600)
pin_button         = params.get("pin_button", 15)
logs_path          = params.get("logs_path", "/mnt/hdd/logs")
fallback_logs_path = params.get("fallback_logs_path", "/home/dronehydro/main_ws/logs")
log_name_format    = params.get("log_name_format", "%Y-%m-%d_%H-%M-%S")
restart_timeout    = params.get("restart_timeout", 5)
failure_threshold  = params.get("failure_threshold", 5)

# Set up the button
BUTTON_PIN = pin_button  # Using the value from param.yaml
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Verify primary log path; if inaccessible, immediately fallback
try:
    mount_result = subprocess.run(["mountpoint", "-q", logs_path], timeout=0.5, check=False)
    if mount_result.returncode != 0 or not (os.path.isdir(logs_path) and os.access(logs_path, os.W_OK)):
        print(f"WARNING: Primary log path '{logs_path}' is not accessible. Using fallback.")
        logs_path = fallback_logs_path
    print(f"INFO: Primary log path '{logs_path}' is accessible.")
except (subprocess.TimeoutExpired, OSError) as e:
    print(f"WARNING: {e}. Using fallback logs path: {fallback_logs_path}")
    logs_path = fallback_logs_path
    os.makedirs(logs_path, exist_ok=True)

# Initialize LCD
lcd = CharLCD('PCF8574', 0x27)  # 0x27 for 20x4
lcd.clear()
lcd.cursor_pos = (0, 0)
lcd.write_string("----- REPER3D ----- ")
lcd.cursor_pos = (1, 0)
lcd.write_string("Sonar: INITIALIZING ")
lcd.cursor_pos = (2, 0)
lcd.write_string("GPS:   INITIALIZING ")

# ---------------------------
# DEVICE INITIALIZATION
# ---------------------------

# Initialize sonar Ping2
myPing = Ping1D()
myPing.connect_serial(port_sonar, baud_rate_sonar)
if myPing.initialize() is False:
    lcd.cursor_pos = (1, 0)
    lcd.write_string("Sonar: INIT. FAILED ")
    print("Sonar initialization failed")
else:
    lcd.cursor_pos = (1, 0)
    lcd.write_string("Sonar: READY        ")
    print("Sonar initialized")

# Initialize GPS serial port
try:
    ser = serial.Serial(port_gps, baud_rate_gps)
    lcd.cursor_pos = (2, 0)
    lcd.write_string("GPS:   READY        ")
    print("GPS initialized")
except serial.SerialException as e:
    lcd.cursor_pos = (2, 0)
    lcd.write_string("GPS: INIT. FAILED   ")
    print(f"Failed to open GPS port: {port_gps}. Error: {e}")
    ser = None

# ---------------------------
# TEMPERATURE SENSOR INITIALIZATION
# ---------------------------

os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')
base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28*')[0]
temp_path = device_folder + '/w1_slave'

# Display waiting message
lcd.cursor_pos = (3, 0)
lcd.write_string("Ready, press button")
print("System ready. Waiting for button press...")

# Wait for button press
button_pressed = False
while not button_pressed:
    if GPIO.input(BUTTON_PIN) == GPIO.LOW:
        button_pressed = True
        lcd.cursor_pos = (3, 0)
        lcd.write_string("Starting...         ")
        print("Button pressed! Starting acquisition...")
        tiime.sleep(0.5)  # Debounce
    tiime.sleep(0.1)  # Reduce CPU usage while waiting

# CREATE LOG FILES ONLY AFTER BUTTON PRESS
# Define log subdirectories and ensure they exist
current_time = datetime.now()
try:
    formatted_timestamp = current_time.strftime(log_name_format)
except:
    print("WARNING: Invalid log_name_format in config. Using default format.")
    formatted_timestamp = current_time.strftime("%Y-%m-%d_%H-%M-%S")

gps_log_dir = os.path.join(logs_path, "GPS")
sonar_log_dir = os.path.join(logs_path, "SONAR")
os.makedirs(gps_log_dir, exist_ok=True)
os.makedirs(sonar_log_dir, exist_ok=True)

# Define log filenames with proper subdirectories and timestamp
gps_log_filename   = create_log_file("gps", "txt", subdir="GPS")
sonar_log_filename = create_log_file("sonar", "txt", subdir="SONAR")

# Open persistent file handles for logging
gps_log_handle   = open(gps_log_filename, "a+")
sonar_log_handle = open(sonar_log_filename, "a+")

# Write headers to the log files
gps_log_handle.write("$, Heure (UTC), Latitude, Hemisphere Lat, Longitude, Hemisphere Long, quality indicator, Nb satellites, HDOP, Altitude, Unit, Geoidal separation, Unit, Age of diff. data, Difference ref station ID\n")
gps_log_handle.flush()

sonar_log_handle.write("Distance, Date, Latitude, Longitude, Confidence\n")
sonar_log_handle.flush()

# Reset the initial time after button press
time_init = datetime.now()
timeout = timedelta(minutes=duree)

# ---------------------------
# FUNCTION DEFINITIONS
# ---------------------------

def tC_func(tF):
    return (tF - 32) * 5 / 9

def ss(temperatureC):
    sound_speed = 1404.3 + 4.7 * temperatureC - 0.04 * temperatureC * temperatureC
    return 1000 * sound_speed  # mm/s

# ---------------------------
# ACQUISITION LOOP
# TODO REVOIR LA LOGIQUE D'ACQUISITION DU GPS, ÊTRE SÛR QUE LES BONNES DONNÉES SONT RÉCOLTÉES
# ---------------------------

i_failure = 0       # Sonar connection errors, max 5

default_lat = 0.000
default_long = 0.000

current_lat = None
current_lon = None

# Before the acquisition loop, define a variable to track if early termination was requested
early_termination = False

# Modify the acquisition loop to check for button press
while (datetime.now() - time_init) < timeout and i_failure < 6 and not early_termination:
    # First check if button is pressed for early termination
    if GPIO.input(BUTTON_PIN) == GPIO.LOW:
        # Button pressed during acquisition, confirm with debounce
        tiime.sleep(0.2)  # Initial debounce
        if GPIO.input(BUTTON_PIN) == GPIO.LOW:  # Still pressed after debounce
            lcd.cursor_pos = (3, 0)
            lcd.write_string("Stopping...         ")
            print("Button pressed during acquisition. Stopping...")
            tiime.sleep(1)  # Give user visual feedback
            early_termination = True
            continue  # Skip to next iteration which will exit the loop

    time_current = datetime.now()
    elapsed_time = time_current - time_init

    # Read GPS line
    line = ser.readline().decode("utf-8")
    words = line.split(',')
    type_GPS = str(words[0])
    if type_GPS not in ["$GPGGA", "$GPGLL", "$GPRMC"]:
        continue
    elif type_GPS == "$GPGGA":
        #print("Temps écoulé (s): ", elapsed_time.seconds)
        try:
            time_val = datetime.strptime(str(words[1]), "%H%M%S.%f")
        except ValueError:
            continue
        current_lat = words[2] + words[3]
        current_lon = words[4] + words[5]
    elif type_GPS == "$GPGLL":
        #print("Temps écoulé (s): ", elapsed_time.seconds)
        try:
            time_val = datetime.strptime(str(words[5]), "%H%M%S.%f")
        except ValueError:
            continue
        current_lat = str(default_lat)
        current_lon = str(default_long)
    elif type_GPS == "$GPRMC":
        #print("Temps écoulé (s): ", elapsed_time.seconds)
        try:
            time_val = datetime.strptime(str(words[1]), "%H%M%S.%f")
        except ValueError:
            continue
        current_lat = words[3] + words[4]
        current_lon = words[5] + words[6]
    try:
        time_val = time_val.replace(year=time_current.year, month=time_current.month, day=time_current.day)
        gps_log_handle.write(f"{line}")
        gps_log_handle.flush()
    except ValueError:
        print("Value Error on GPS")
        continue

    # Update temperature from sensor
    with open(temp_path, "r") as temperature_file:
        lines = temperature_file.readlines()
        while lines[0].strip()[-3:] != 'YES':
            tiime.sleep(0.2)
            with open(temp_path, "r") as temperature_file_retry:
                lines = temperature_file_retry.readlines()
        temp_data = lines[1].split('t=')[-1]
        temp_c = float(temp_data) / 1000.0
        #print(f"water temperature is : {temp_c:.2f} C")

    try:
        myPing.set_speed_of_sound(int(ss(tC)))
        #print("Sound velocity updated (mm/s): ", ss(tC))
        data = myPing.get_distance()
        if data is None:
            lcd.cursor_pos = (1, 0)
            lcd.write_string("Sonar: NO DATA      ")
            print("No data; reinitialising sonar")
            myPing.connect_serial(port_sonar, baud_rate_sonar)
            myPing.initialize()
            data = myPing.get_distance()
            if data is None:
                data = {"distance": -1, "confidence": -1}
                lcd.cursor_pos = (1, 0)
                lcd.write_string("Sonar: REINIT FAILED ")
                print("Reinitialisation failed")
                i_failure += 1
            else:
                lcd.cursor_pos = (1, 0)
                lcd.write_string("Sonar: REINIT OK     ")
                print("Reinitialisation succeeded")
        distance_current = str(data["distance"])
        confidence_current = str(data["confidence"])
        #print("Distance = " + distance_current + " mm")

        msg = "Depth = " + distance_current + " mm"
        lcd.cursor_pos = (3, 0)
        lcd.write_string(msg.ljust(20))  # Ensure 20 chars
        
        # Add latitude and longitude to the log file
        if current_lat is not None and current_lon is not None:
            sonar_log_handle.write(f"{distance_current}, {time_current.isoformat()}, {current_lat}, {current_lon},  {confidence_current}\n")
        else:
            sonar_log_handle.write(f"{distance_current}, {time_current.isoformat()}, NA, NA, {confidence_current}\n")
            
        sonar_log_handle.flush()
        #tiime.sleep(0.5) # was here before, what for?

    except serial.SerialException:
        print("SONAR NOT RESPONDING")
        data = {"distance": -1, "confidence": -1}
        distance_current = str(data["distance"])
        confidence_current = str(data["confidence"])
        lcd.cursor_pos = (1, 0)
        lcd.write_string("Sonar: NO DATA      ")
        print("Acquisition failed")
        
        # Add latitude and longitude to the log file even in failure case
        if current_lat is not None and current_lon is not None:
            sonar_log_handle.write(f"{time_current.isoformat()}, {distance_current}, {current_lat}, {current_lon},  {confidence_current}\n")
        else:
            sonar_log_handle.write(f"{time_current.isoformat()}, {distance_current}, NA, NA, {confidence_current}\n")
            
        sonar_log_handle.flush()
        i_failure += 1
        print("SONAR FAILURE +1")
        continue

# After the loop, update the final message based on termination reason
if early_termination:
    completion_reason = "Stopped by user"
elif i_failure > 5:
    completion_reason = "Sonar failure"
    fail_path = os.path.join(logs_path, f"fail_{time_current.strftime('%Y-%m-%d_%H-%M-%S')}.txt")
    with open(fail_path, "w") as f_fail:
        f_fail.write("Sonar failure threshold exceeded; rebooting\n")
    print("Rebooting")
    lcd.cursor_pos = (3, 0)
    lcd.write_string("Rebooting...        ")
    tiime.sleep(0.5)
    # Uncomment the following line if a reboot is needed:
    # os.system("sudo reboot")
else:
    completion_reason = "Completed successfully"

lcd.cursor_pos = (3, 0)
lcd.write_string("- Done, logs saved -")
print(f"End of acquisition: {completion_reason}")
tiime.sleep(2)

# Wait for button press to restart
lcd.cursor_pos = (3, 0)
lcd.write_string("- Press to restart -")
print("Press button to start a new acquisition cycle...")

restart_requested = False
button_wait_start = datetime.now()
button_timeout = timedelta(minutes=restart_timeout)  # Use the parameter value

while not restart_requested and (datetime.now() - button_wait_start) < button_timeout:
    if GPIO.input(BUTTON_PIN) == GPIO.LOW:
        # Button pressed, confirm with debounce
        tiime.sleep(0.1)  # Debounce
        if GPIO.input(BUTTON_PIN) == GPIO.LOW:  # Still pressed after debounce
            restart_requested = True
            lcd.cursor_pos = (3, 0)
            lcd.write_string("Restarting...       ")
            print("Button pressed. Restarting for new acquisition...")
            tiime.sleep(1)  # Give user visual feedback
            restart_program()  # This will reset everything
    tiime.sleep(0.1)  # Reduce CPU usage

# If we get here, it means the button wasn't pressed within the timeout period
lcd.clear()
lcd.cursor_pos = (1, 0)
lcd.write_string("System idle         ")
lcd.cursor_pos = (2, 0)
lcd.write_string("Power can be turned ")
lcd.cursor_pos = (3, 0)
lcd.write_string("off safely.         ")
print("No restart requested within timeout period. System idle.")

# Final cleanup
GPIO.cleanup()