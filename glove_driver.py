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
from filterpy.kalman import ExtendedKalmanFilter


def computeRPY(acc, mag):
    roll = math.atan2(acc[1], acc[2])
    pitch = math.atan2(-acc[0], math.sqrt(acc[1]*acc[1] + acc[2]*acc[2]))
    yaw = math.atan2(mag[2]*math.sin(pitch)-mag[1]*math.cos(pitch), 
    mag[0]*math.cos(roll)+mag[1]*math.sin(roll)*math.sin(pitch)*mag[2]*math.sin(roll)*math.cos(pitch))

    # math.atan2(coorectedMag[2]*math.sin(pitch)-coorectedMag[1]*math.cos(pitch),
    #  coorectedMag[0]*math.cos(roll)+coorectedMag[1]*math.sin(roll)*math.sin(pitch)*coorectedMag[2]*math.sin(roll)*math.cos(pitch))
    return roll, pitch, yaw

def F(quat, gyro, bias):
    g = gyro
    b = bias
    q = quat

    uno = -0.5*[ (g[0]-b[0])*q[1] + (g[1]-b[1])*q[2] + (g[2]-b[2])*q[3]]
    due = -0.5*[ (g[0]-b[0])*q[0] + (g[1]-b[1])*q[3] + (g[2]-b[2])*q[2]]
    tre = -0.5*[ (g[0]-b[0])*q[3] + (g[1]-b[1])*q[0] + (g[2]-b[2])*q[1]]
    quattro = -0.5*[ (g[0]-b[0])*q[2] + (g[1]-b[1])*q[1] + (g[2]-b[2])*q[0]]

    return np.array([uno, due, tre, quattro, 0, 0, 0])

def H(quat):
    q = quat

    dTheta_dq0 = (2*q[3]^2*q[1] - 4*q[3]*q[0]*q[2] - 2*q[1]*(x^2 + q[1]^2 + q[2]^2))/(q[3]^4 + x^4 + 8*q[3]*q[0]*q[1]*q[2] + 2*x^2*(q[1]^2 - q[2]^2) + 2*q[3]^2*(q[0]^2 - q[1]^2 + q[2]^2) + (q[1]^2 + q[2]^2)^2)
    dTheta_dq1 = (2*(q[3]^2*q[0] + 2*q[3]*q[1]*q[2] + q[0]*(q[0]^2 + q[1]^2 - q[2]^2)))/(q[3]^4 + q[0]^4 + 8*q[3]*q[0]*q[1]*q[2] + 2*q[0]^2 (q[1]^2 - q[2]^2) + 2*q[3]^2*(q[0]^2 - q[1]^2 + q[2]^2) + (q[1]^2 + q[2]^2)^2)
    dTheta_dq2 = (2*(q[3]^3 + 2*q[0]*q[1]*q[2] + q[3]*(q[0]^2 - q[1]^2 + q[2]^2)))/(q[3]^4 + q[0]^4 + 8*q[3]*q[0]*q[1]*q[2] + 2*q[0]^2*(q[1]^2 - q[2]^2) + 2*q[3]^2*(q[0]^2 - q[1]^2 + q[2]^2) + (q[1]^2 + q[2]^2)^2)
    dTheta_dq3 = (-2*(2*q[3]*q[0]*q[1] + q[3]^2*q[2] + q[2]*(-q[0]^2 + q[1]^2 + q[2]^2)))/(q[3]^4 + q[0]^4 + 8*q[3]*q[0]*q[1]*q[2] + 2*q[0]^2 (q[1]^2 - q[2]^2) + 2*q[3]^2*(q[0]^2 - q[1]^2 + q[2]^2) + (q[1]^2 + q[2]^2)^2)

    dPhi_dq0 = (2*q[2])/math.sqrt[1 - 4 (q[3]*q[1] - q[0]*q[2])^2]
    dPhi_dq1 = (-2*q[3])/math.sqrt[1 - 4 (q[3]*q[1] - q[0]*q[2])^2]
    dPhi_dq2 = (2*q[0])/math.sqrt[1 - 4 (q[3]*q[1] - q[0]*q[2])^2]
    dPhi_dq3 = (-2*q[1])/math.sqrt[1 - 4 (q[3]*q[1] - q[0]*q[2])^2]

    dTheta_dq0 = (-2*(q[3]^3 + 2*q[0]*q[1]*q[2] + q[3]*(q[0]^2 - q[1]^2 + q[2]^2)))/(q[3]^4 + q[0]^4 + 8*q[3]*q[0]*q[1]*q[2] + 2*q[0]^2*(q[1]^2 - q[2]^2) + 2*q[3]^2*(q[0]^2 - q[1]^2 + q[2]^2) + (q[1]^2 + q[2]^2)^2)
    dTheta_dq1 = (-2*(2*q[3]*q[0]*q[1] + q[3]^2*q[2] + q[2]*(-q[0]^2 + q[1]^2 + q[2]^2)))/(q[3]^4 + q[0]^4 + 8*q[3]*q[0]*q[1]*q[2] + 2*q[0]^2*(q[1]^2 - q[2]^2) + 2*q[3]^2*(q[0]^2 - q[1]^2 + q[2]^2) + (q[1]^2 + q[2]^2)^2)
    dTheta_dq2 = (-2*q[3]^2*q[1] + 4*q[3]*q[0]*q[2] + 2*q[1]*(q[0]^2 + q[1]^2 + q[2]^2))/(q[3]^4 + q[0]^4 + 8*q[3]*q[0]*q[1]*q[2] + 2*q[0]^2*(q[1]^2 - q[2]^2) + 2*q[3]^2*(q[0]^2 - q[1]^2 + q[2]^2) + (q[1]^2 + q[2]^2)^2)
    dTheta_dq3 = (2*(q[3]^2*q[0] + 2*q[3]*q[1]*q[2] + q[0]*(q[0]^2 + q[1]^2 - q[2]^2)))/(q[3]^4 + q[0]^4 + 8*q[3]*q[0]*q[1]*q[2] + 2*q[0]^2*(q[1]^2 - q[2]^2) + 2*q[3]^2*(q[0]^2 - q[1]^2 + q[2]^2) + (q[1]^2 + q[2]^2)^2)

    return np.array([[dTheta_dq0, dTheta_dq1, dTheta_dq2, dTheta_dq3,0,0,0],
                    [dPhi_dq0, dPhi_dq1, dPhi_dq2, dPhi_dq3,0,0,0], 
                    [dPsi_dq0, dPsi_dq1, dPsi_dq2, dPsi_dq3,0,0,0]])

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


                mag = np.array([imu.magnetometer.x, imu.magnetometer.y, imu.magnetometer.z])
                acc = np.array([imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z])
                correctedMag = mag2acc_frame( correctMagnetometer(mag))

                roll, pitch, yaw = computeRPY(acc, correctedMag)

                # roll = roll * 180 / math.pi
                # pitch = pitch * 180 / math.pi
                # yaw = yaw * 180 / math.pi
                # print(f"roll: {roll:0.3f}, pitch: {pitch:0.3f}, yaw: {yaw:0.3f}")

                quat = quaternionic.array.from_euler_angles(roll, pitch, yaw)
                print(f"quat: {quat.w:.2f}, {quat.x:.2f}, {quat.y:.2f}, {quat.z:.2f}")


'''QUI SI ISTANZIA IL NODO ROS CHE NON CI SERVE PIÙ E SI LEGGONO I PARAMETRI PASSATI DA LINEA DI COMANDO (X ES. LA PORTA SU CUI APRIRE IL SOCKET)'''
def main(): 
    imu_driver = ImuDriver(2390, '130.251.13.116', recording=False)
    imu_driver.listener()


'''CHIAMIAMO IL MAIN()'''
if __name__ == '__main__':
    main()