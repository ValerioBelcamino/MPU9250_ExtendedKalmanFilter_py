#!/usr/bin/env python3
from tkinter.font import names
import struct
import socket
import argparse
import os
import json, requests
import pickle
import keyboard
from IMU_class import IMU, Quaternion



'''LA CLASSE CHE GESTISCE LA COMUNICAZIONE SOCKET E LA RICEZIONE DEI DATI'''
class ImuDriver:

    def __init__(self):
        self.in_udp_ip = '130.251.13.124' #'130.251.13.158' #STATIC IP ETHERNET CABLE; '192.168.0.101' #Hard-coded static IP
        self.in_udp_port = 2395
        self.out_udp_ip = '130.251.13.124'
        self.out_udp_port = 2400

        self.msg = IMU('', Quaternion(0,0,0,0))

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

        self.in_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.in_sock.bind((self.in_udp_ip, self.in_udp_port))
        self.in_sock.settimeout(0.1)

        self.out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        print(f"IMU listener initialized on IP {self.in_udp_ip} with ports {self.in_udp_port=} and {self.out_udp_port=}")
        while True:
            if keyboard.is_pressed('shift+q'):
                break
            try:
                data = self.in_sock.recv(29+6)  # (29) # buffer size is 1024 bytes
            except socket.timeout as se:
                print("timeout, press shift+q to exit", se)
                continue
            if not data:
                print("no data")
            else:
                ID = data[28+6]


                mux = int(ID/100)
                channel = int((ID-mux*100)/10)
                address = int(ID-mux*100-channel*10)


                self.msg.orientation.x = struct.unpack('<f', data[0:4])[0]
                self.msg.orientation.y = struct.unpack('<f', data[4:8])[0]
                self.msg.orientation.z = struct.unpack('<f', data[8:12])[0]
                self.msg.orientation.w = struct.unpack('<f', data[12:16])[0]
                self.msg.sensor_name = self.names[ID] + "-Pose"


                tmp = (str(self.msg.sensor_name) + "," + 
                    str(self.msg.orientation.x) + "," + str(self.msg.orientation.y) + "," + str(self.msg.orientation.z) + "," + str(self.msg.orientation.w) + "\n")
                # print(tmp, end='\r')

                self.connectedIMUs[self.names[ID]] = 1
                self.msgCounter += 1
                if self.msgCounter == 100:
                    self.connectedIMUs = {n:0 for n in self.namelist}
                    self.msgCounter = 0
                elif  self.msgCounter > 30:
                    print(self.connectedIMUs, end='\r')

                serialized_msg = pickle.dumps(self.msg)
                self.out_sock.sendto(serialized_msg, (self.out_udp_ip, self.out_udp_port))
        
        in_sock.close()
        out_sock.close()


def main(): 
    imu_driver = ImuDriver()
    imu_driver.listener()


if __name__ == '__main__':
    main()

