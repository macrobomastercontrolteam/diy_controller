
import serial
import math
import serial.tools.list_ports
import struct
import time
import matplotlib.pyplot as plt
import keyboard
import numpy as np

import threading

from diy_controller_code.imu_data import imu
from diy_controller_code.filters import imu_filter, calculation

read = imu()
filters = imu_filter()
cal = calculation()

def imu_update():
    global datahex
    while (1):
        
        # print("thread")
        if(SystemExit):
            break

    
if __name__=='__main__':
    
    ser = serial.Serial('COM3', 115200)
    
    #initialize
    a_x = [0.0,0.0]
    a_y = [0.0,0.0]
    a_z = [0.0,0.0]
    
    v_x = [0.0,0.0]
    v_y = [0.0,0.0]
    v_z = [0.0,0.0]
    
    
    timer = 500
    while(1):
        datahex = ser.read(33)
        read.DueData(datahex)
        
        # get acceleration, q
        acc = read.get_acc()
        q = read.get_q()
        angle = read.get_angle()
        gyro = read.get_gyro()
        
        #remove gravity
        acc = filters.remove_gravity1(acc,q)
        
        #check stable or not
        
        
        if(keyboard.is_pressed('q')):
            # print(acc[0])
            #deadzone and update acceleration
            stable = filters.is_stationary(gyro[0], gyro[1], gyro[2])
            
            imu_filter.zero_update(acc[0], a_x, stable)
            imu_filter.zero_update(acc[1], a_y, stable)
            imu_filter.zero_update(acc[2], a_z, stable)
            
            #calculate velocity
            if(stable):
                v_x.append(0.0)
                v_y.append(0.0)
                v_z.append(0.0)
                
                
            else:
                delat_v_x = cal.integral_trap(a_x)
                delat_v_y = cal.integral_trap(a_y)
                delat_v_z = cal.integral_trap(a_z)
                
                cal.addition(v_x, delat_v_x)
                cal.addition(v_y, delat_v_y)
                cal.addition(v_z, delat_v_z)
                # print(v_x[-1], v_y[-1],v_z[-1], end = "\r")
            
        
            
        else:
            print("waiting", end="\r")
            a_x.append(0.0)
            a_y.append(0.0)
            a_z.append(0.0)
            v_x.append(0.0)
            v_y.append(0.0)
            v_z.append(0.0)
            
        print(v_x[-1], v_y[-1],v_z[-1],"               " ,end = "\r")
        
        timer -=1
        
    cal.visulization(v_x,v_y,v_z)
        


    