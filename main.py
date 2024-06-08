
import serial
import math
import serial.tools.list_ports
import struct
import time
import matplotlib.pyplot as plt
import keyboard
import numpy as np

# import threading
# import insruct


from imu_data import imu #raw data read
from filters import imu_filter, calculation #filters and calculation
from crc_verify import CRC #communication security

read = imu("COM13")
filters = imu_filter()
cal = calculation()

crc = CRC()

def imu_update():
    global datahex
    while (1):
        
        # print("thread")
        if(SystemExit):
            break

    
if __name__=='__main__':
    
    send = serial.Serial('COM14', 115200)
    #raspberry pi
    # uart_com = serial.Serial('/dev/ttyUSB1', 115200) #uart connection
    # IMU = serial.Serial('/dev/ttyUSB0', 115200) #uart connection
    
    #initialize
    a_x = [0.0,0.0]
    a_y = [0.0,0.0]
    a_z = [0.0,0.0]
    
    v_x = [0.0,0.0]
    v_y = [0.0,0.0]
    v_z = [0.0,0.0]
    
    #communcation setup
    SOF = 0xA5 #Domain ID of the data 
    data_length = 30  #two byte
    # data_length = 0x1E  #two byte
    seq = 0x00 #packet sequence number
    #crc8 = 0x00 #require function here or not???
    
    #diy controller
    cmd_id = 0x0302 #two byte H
    
    timer = True
    while(timer):
        read.DataRead()
        
        # get acceleration, q
        acc = read.get_acc()
        q = read.get_q()
        angle = read.get_angle()
        gyro = read.get_gyro()
        
        # print(gyro,"\n")
        # print(angle,"\n")
        # print(gyro)
        
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
                
                bit_x = struct.pack("fff", v_x[-1], v_y[-1],v_z[-1])
                
                # print(bit_x )
            
        else:
            # print("waiting", end="\r")
            a_x.append(0.0)
            a_y.append(0.0)
            a_z.append(0.0)
            v_x.append(0.0)
            v_y.append(0.0)
            v_z.append(0.0)

        print(v_x[-1], v_y[-1],v_z[-1], angle[0],angle[1],angle[2] , "\n")        
        # timer -=1
        
        #seril communication
        
        #5 bit header
        head = struct.pack("<BHB", SOF, data_length, seq)
        
        
        # test.reset()
        head_length = 5
        
        #CRC8 add
        head = crc.append_CRC8_check_sum(head, 5)
        # append_CRC8_check_sum(head, 5)
        # print("head:", len(head))
        
        #temp length filling for now, change later
        command = 0x1111
        #ID 2 bytes + command 30 bytes
        #Send the latest calculated 3-axis velocity, angles, and custom command
        # print(v_x[-1])
        send_command = struct.pack("<HffffffHHH",cmd_id,v_x[-1],v_y[-1],v_z[-1],angle[0],angle[1],angle[2],command,0x0101,0x0101) 
        buff = head + send_command
        
        print(send_command)
        
        #crc16 add, use random from the table
        buff = crc.append_CRC16_check_sum(buff,39)
        # append_CRC16_check_sum(buff,39)
        # buff = buff + struct.pack("<H", crc16)
        
        #send data to C board through referee system, more details refer to read.md
        send.write(buff)
        

    #Graphing
    # cal.visulization(v_x,v_y,v_z)
        


    