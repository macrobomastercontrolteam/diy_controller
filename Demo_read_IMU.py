
import serial
import math
import time
import matplotlib.pyplot as plt
import keyboard
import numpy as np

# import threading
# import insruct

from imu_data import imu #raw data read

onboard_imu = imu("COM13")
    
if __name__=='__main__':
    
    while True:

        onboard_imu.DataRead()
        
        angle = onboard_imu.get_angle()
        
        


    