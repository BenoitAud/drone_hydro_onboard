import os
import glob
import time

os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28*')[0]
device_file = device_folder + '/w1_slave'

def read_temp_raw():
    with open(device_file, 'r') as f:
        lines = f.readlines()
    return lines

def read_temp():
    lines = read_temp_raw()
    while lines[0].strip()[-3:] != 'YES':
        time.sleep(0.2)
        lines = read_temp_raw()
    
    temp_data = lines[1].split('t=')[-1]
    temp_c = float(temp_data) / 1000.0
    return temp_c

while True:
    temperature = read_temp()
    print(f"Temperature: {temperature:.2f} °C")
    time.sleep(1)

