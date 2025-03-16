from bluet import *
from machine import Timer
from read import read_FSR_values
from time import sleep

setup_ble()


while True:
#     if is_connected:
#         ble.gatts_write(tx, read_FSR_values())
        
    sleep(0.0001)