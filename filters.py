import numpy as np
import math
import matplotlib.pyplot as plt

class imu_filter:
    
    def remove_gravity1(self, a, q):
    # Convert the quaternion to a rotation matrix
        q0, q1, q2, q3 = q[0], q[1], q[2], q[3]
        R = np.array([
            [1 - 2*q2**2 - 2*q3**2,   2*q1*q2 - 2*q3*q0,   2*q1*q3 + 2*q2*q0],
            [2*q1*q2 + 2*q3*q0,   1 - 2*q1**2 - 2*q3**2,   2*q2*q3 - 2*q1*q0],
            [2*q1*q3 - 2*q2*q0,   2*q2*q3 + 2*q1*q0,   1 - 2*q1**2 - 2*q2**2]
        ])

        # Rotate the acceleration vector from sensor frame to Earth frame
        acc_earth = np.dot(R, a)

        # Subtract gravity (assuming the z-axis is pointing upwards)
        gravity = np.array([0, 0, 1.0])
        acc = acc_earth - gravity

        return acc
    
    def remove_gravity2(acc, Angle):
        # 欧拉角（以弧度为单位）
        # print(acc[0],acc[1],acc[2])
        roll = Angle[0]*math.pi/180
        pitch = Angle[1]*math.pi/180
        yaw = Angle[2]*math.pi/180

        # 重力向量在传感器坐标系中的分量
        g_x = math.sin(pitch) * 1.04
        g_y = math.sin(roll) * 1.04
        g_z = math.cos(roll) * math.cos(pitch) * 1.03

        # 从加速度读数中减去重力分量
        acc[0] = acc[0] - g_x
        acc[1] = acc[1] - g_y
        acc[2] = acc[2] - g_z
        
    
    def zero_update(data,new_data, is_stable):
        lowerbound = 0
        upperbound = 2
        #bias = sum(buffer)/len(buffer)
        if (is_stable): 
            # print("stable")
            new_data.append(0.0)
        else:
        #data = data -bias
            difference = abs(data - new_data[-1])
            if (difference >  lowerbound) and (difference < upperbound):
                new_data.append(data)
                
                #print("not")
            else:
                #not moveing a lot
                new_data.append(new_data[-1]*0.7) 
    
    #stationary the velocity(?) if the value is too small
    def is_stationary(self, gyro_x, gyro_y, gyro_z):
        threshold = 15 #rotation sensitivity
        if abs(gyro_x) < threshold and abs(gyro_y) < threshold and abs(gyro_z) < threshold:
            return True
        else:
            return False

    
class calculation:
    
    def integral_trap(self,input):
        time_step = 0.001 #delta time
        # velocity = np.zeros_like(acc)
        n = len(input)
        for i in range(1, n):  
            change = (input[i-1] + input[i]) * time_step / 2
            # velocity[i] = velocity[i-1] + velocity_change
        return change
    
    def addition(self, base, change):
        base = base.append(round(base[-1]+change, 5))
    
    def visulization(self,v_x,v_y,v_z):
    
        plt.figure(1)
        
        plt.plot(v_x[:], marker='o', linestyle='-')
        plt.plot(v_y[:], marker='v', linestyle='-')
        plt.plot(v_z[:], marker='*', linestyle='-')
        plt.legend(["v_x", "v_y", "v_z"])
        plt.xlabel('time (s)')
        plt.ylabel('velocity')
        plt.title('velocity vs time')
        
        plt.show()