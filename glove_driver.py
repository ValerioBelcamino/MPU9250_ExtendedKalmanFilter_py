#!/usr/bin/env python3
import struct
import socket
import argparse
import os
import json, requests
from IMU_class import IMU, Quat_array, Acc_array, Gyro_array, Mag_array
import time
import numpy as np
import math
import quaternionic
from kalmanFilter import ExtendedKalmanFilter


def computeRPY(acc, mag):
    roll = math.atan2(acc[1], acc[2])
    # pitch = math.atan2(-acc[0], math.sqrt(acc[1]*acc[1] + acc[2]*acc[2]))
    pitch = math.asin(acc[0]/ (math.sqrt(acc[1]*acc[1] + acc[2]*acc[2] + acc[0]*acc[0])))
    yaw = math.atan2(mag[2]*math.sin(roll)-mag[1]*math.cos(roll), 
    mag[0]*math.cos(pitch)+mag[1]*math.sin(roll)*math.sin(pitch)*mag[2]*math.sin(pitch)*math.cos(roll))

    # math.atan2(coorectedMag[2]*math.sin(pitch)-coorectedMag[1]*math.cos(pitch),
    #  coorectedMag[0]*math.cos(roll)+coorectedMag[1]*math.sin(roll)*math.sin(pitch)*coorectedMag[2]*math.sin(roll)*math.cos(pitch))
    return roll, -pitch, yaw




def mag2acc_frame(vector3):
    vector3=vector3.reshape(3,1)
    result = np.matrix([[0, 1, 0], [1, 0, 0], [0, 0, -1]]) @ vector3
    return np.array([result[0,0], result[1,0], result[2,0]])

def correctMagnetometer(vector3):
    b = np.array([-12.252622, 53.44271, -60.0742])
    A_1 = np.array([[1.5846435, 0.028956, -0.015616], [0.028956, 1.5994587, -0.01484], [-0.01561694, -0.0148454, 1.5684942]])
    # subtract the hard iron offset
    xm_off = vector3[0]-b[0]
    ym_off  = vector3[1]-b[1]
    zm_off  = vector3[2]-b[2]
    
    #multiply by the inverse soft iron offset
    xm_cal = xm_off *  A_1[0,0] + ym_off *  A_1[0,1]  + zm_off *  A_1[0,2] 
    ym_cal = xm_off *  A_1[1,0] + ym_off *  A_1[1,1]  + zm_off *  A_1[1,2] 
    zm_cal = xm_off *  A_1[2,0] + ym_off *  A_1[2,1]  + zm_off *  A_1[2,2] 

    return np.asarray([xm_cal, ym_cal, zm_cal])

'''LA CLASSE CHE GESTISCE LA COMUNICAZIONE SOCKET E LA RICEZIONE DEI DATI'''
class ImuDriver:

    def __init__(self, udp_port, ip_addr, recording=False):
        self.udp_ip = ip_addr
        self.udp_port = udp_port
        self.recording = recording
        self.counter = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        with open("config/driver_config.txt") as f:
            lines = f.readlines()

        self.names = {}
        for line in lines:
            values = line.strip().split(" ")
            self.names[int(values[0])] = values[1]

        self.namelist = sorted([n for n in self.names.values() if 'wrist' not in n])
        self.connectedIMUs = {n:0 for n in self.namelist}
        self.msgCounter = 0

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.udp_ip, self.udp_port))
        sock.settimeout(10)

        self.ekf = ExtendedKalmanFilter(P=np.eye(7) * 0.1, Q = np.eye(7) * 0.00001, R = np.eye(4) * 0.001)
        self.g_bias = np.array([-1,-1,-1]) * math.pi/180

        x = None


        old_time = time.time()
        while True:
            try:
                data = sock.recv(29+6)  # (29) # buffer size is 1024 bytes
            except Exception as e:
                print("Socket Exception:", e)
                sock.close()
                print("Socket closed, returning...")
                exit()
            if not data:
                print("no data")
            else:

                ID = data[28+6]
                mux = int(ID/100)
                channel = int((ID-mux*100)/10)
                address = int(ID-mux*100-channel*10)

                quat = Quat_array(
                    struct.unpack('<f', data[0:4])[0],
                    struct.unpack('<f', data[4:8])[0],
                    struct.unpack('<f', data[8:12])[0],
                    struct.unpack('<f', data[12:16])[0])
                
                acc = Acc_array(
                    struct.unpack('<h', data[16:18])[0],
                    struct.unpack('<h', data[18:20])[0],
                    struct.unpack('<h', data[20:22])[0])

                gyro = Gyro_array(
                    struct.unpack('<h', data[22:24])[0],
                    struct.unpack('<h', data[24:26])[0],
                    struct.unpack('<h', data[26:28])[0])

                mag = Mag_array(
                    struct.unpack('<h', data[28:30])[0],
                    struct.unpack('<h', data[30:32])[0],
                    struct.unpack('<h', data[32:34])[0])

                imu = IMU(time.time(),
                        self.names[ID],
                        ID,
                        quat,
                        acc,
                        gyro,
                        mag)

                sensor_name = self.names[ID]

                tmp = (str(imu.time) + "," + 
                    str(imu.orientation.x) + "," + str(imu.orientation.y) + "," + str(imu.orientation.z) + "," + str(imu.orientation.w) + "," + 
                    str(imu.accelerometer.x) + "," + str(imu.accelerometer.y) + "," + str(imu.accelerometer.z) + "," +
                    str(imu.gyroscope.x) + "," + str(imu.gyroscope.y) + "," + str(imu.gyroscope.z) + "," +
                    str(imu.magnetometer.x) + "," + str(imu.magnetometer.y) + "," + str(imu.magnetometer.z) + "\n")

                folderpath = "C:\\Users\\valer\\OneDrive\\Desktop\\KalmanFilter\\Magnetometer_recordings"#"C:/recordings"
                try:
                    os.mkdir(folderpath)
                except FileExistsError:
                    pass

                if recording:
                    filename = os.path.join(folderpath, sensor_name+".txt")  
                    f = open(filename, "a+")
                    f.write(tmp)
                    f.close()
                
                # self.connectedIMUs[self.names[ID]] = 1
                # self.msgCounter += 1
                # if self.msgCounter == 100:
                #     self.connectedIMUs = {n:0 for n in self.namelist}
                #     self.msgCounter = 0
                # elif  self.msgCounter > 30:
                #     print(self.connectedIMUs, end='\r')

                gyro = np.array([imu.gyroscope.x, imu.gyroscope.y, imu.gyroscope.z]) * math.pi/180
                mag = np.array([imu.magnetometer.x, imu.magnetometer.y, imu.magnetometer.z])
                acc = np.array([imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z])
                correctedMag = mag2acc_frame( correctMagnetometer(mag))

                roll, pitch, yaw = computeRPY(acc, correctedMag)

                # roll = roll * 180 / math.pi
                # pitch = pitch * 180 / math.pi
                # yaw = yaw * 180 / math.pi
                # print(f"roll: {roll:0.3f}, pitch: {pitch:0.3f}, yaw: {yaw:0.3f}")

                quat = quaternionic.array.from_euler_angles(roll, pitch, yaw).ndarray
                if x is None:
                    x = np.array([quat[0], quat[1], quat[2], quat[3], self.g_bias[0], self.g_bias[1], self.g_bias[2]])
                    self.ekf.x = x
                
                x = self.ekf.predict(gyro, time.time()-old_time)
                
                # x = self.ekf.update(np.array([roll,pitch,yaw]))
                x = self.ekf.update(np.array([quat[0], quat[1], quat[2], quat[3]]))
                print(f"quat: {x[0]:.3f}, {x[1]:.3f}, {x[2]:.3f}, {x[3]:.3f}")
                old_time = time.time()
                # print(f"quat: {quat.w:.2f}, {quat.x:.2f}, {quat.y:.2f}, {quat.z:.2f}")
                # print(f'gyro: {gyro[0]}, {gyro[1]}, {gyro[2]}')


'''QUI SI ISTANZIA IL NODO ROS CHE NON CI SERVE PIÙ E SI LEGGONO I PARAMETRI PASSATI DA LINEA DI COMANDO (X ES. LA PORTA SU CUI APRIRE IL SOCKET)'''
def main(): 
    imu_driver = ImuDriver(2390, '130.251.13.116', recording=False)
    imu_driver.listener()


'''CHIAMIAMO IL MAIN()'''
if __name__ == '__main__':
    main()