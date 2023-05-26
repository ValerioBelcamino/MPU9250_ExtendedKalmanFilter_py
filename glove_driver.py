#!/usr/bin/env python3
import struct
import socket
import argparse
import os
import json, requests
from IMU_class import IMU, Quat_array, Acc_array, Gyro_array, Mag_array


'''LA CLASSE CHE GESTISCE LA COMUNICAZIONE SOCKET E LA RICEZIONE DEI DATI'''
class ImuDriver:

    def __init__(self, udp_port):
        # print(json.loads(requests.get("https://ip.seeip.org/jsonip?").text)["ip"])
        # ipaddr = json.loads(requests.get("https://ip.seeip.org/jsonip?").text)["ip"]
        ipaddr = '130.251.13.107'
        self.udp_ip = ipaddr #'130.251.13.158' #STATIC IP ETHERNET CABLE; '192.168.0.101' #Hard-coded static IP
        self.udp_port = udp_port
        # print(ipaddr)
        self.counter = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        with open("config/driver_config.txt") as f:
            lines = f.readlines()

        self.names = {}
        for line in lines:
            values = line.strip().split(" ")
            self.names[int(values[0])] = values[1]

        # print(sorted([n for n in self.names.values()]))
        self.namelist = sorted([n for n in self.names.values() if 'wrist' not in n])
        self.connectedIMUs = {n:0 for n in self.namelist}
        # print(self.connectedIMUs)
        self.msgCounter = 0


        '''CREAZIONE E CONNESSIONE AL SOCKET'''
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.udp_ip, self.udp_port))
        sock.settimeout(2)

        while True:
            '''LETTURA DA SOCKET DI UN PACCHETTO DI DIMENSIONE 29+6 BYTE'''
            try:
                data = sock.recv(29+6)  # (29) # buffer size is 1024 bytes
            except Exception as e:
                print("Socket Exception:", e)
                sock.close()
                print("Socket closed, returning...")
                exit()
            if not data:
                '''MESSAGGIO VUOTO'''
                print("no data")
            else:

                ''' IL MESSAGGIO NON È VUOTO E PROCEDIAMO A SPACCHETTARLO'''
                ID = data[28+6]
                # Fix to ensure topic is published under same name with both devices
                # print('ID: ', ID, ' \t - \t', self.names[ID])
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

                imu = IMU(Time.time(),
                        self.names[ID],
                        ID,
                        quat,
                        acc,
                        gyro,
                        mag)

                '''QUESTA VARIABILE CONTIENE IL NOME DEL SENSORE E VA USATO PER SMISTARE I MESSAGGI SU FILE DIVERSI (IDEALMENTE VOGLIAMO UNA CARTELLA /recordings AL CUI
                    INTERNO VENGANO CREATI 11 FILE DIVERSI, UNO PER OGNI SENSORE'''
                sensor_name = self.names[ID] + "-Pose"

                '''TMP CONTIENE UNA STRINGA CON TIMESTAMP, ORIENTAZIONE (X,Y,Z,W), ACCELERAZIONI (X,Y,Z) E MAGNETOMETRO (X,Y,Z) SEPARATE DA VIRGOLA'''
                tmp = (str(imu.time) + "," + 
                    str(imu.orientation.x) + "," + str(imu.orientation.y) + "," + str(imu.orientation.z) + "," + str(imu.orientation.w) + "," + 
                    str(imu.accelerometer.x) + "," + str(imu.accelerometer.y) + "," + str(imu.accelerometer.z) + "," +
                    str(imu.gyroscope.x) + "," + str(imu.gyroscope.y) + "," + str(imu.gyroscope.z) + "," +
                    str(imu.magnetometer.x) + "," + str(imu.magnetometer.y) + "," + str(imu.magnetometer.z) + "\n")

                '''QUI BISOGNA INSERIRE LA STAMPA SU FILE DELLA STRINGA TMP CHE CONTIENE TUTTE LE INFORMAZIONI CHE CI SERVONO'''
                
                #//////////////////////////////////////

                folderpath = "C:\\Users\\valer\\OneDrive\\Desktop\\KalmanFilter\\Magnetometer_recordings"#"C:/recordings"
                try:
                    os.mkdir(folderpath)
                except FileExistsError:
                    pass

                filename = os.path.join(folderpath, sensor_name+".txt")  
                f = open(filename, "a+")
                f.write(tmp)
                f.close()

                #/////////////////////////////////////

                
                self.connectedIMUs[self.names[ID]] = 1
                self.msgCounter += 1
                if self.msgCounter == 100:
                    self.connectedIMUs = {n:0 for n in self.namelist}
                    self.msgCounter = 0
                elif  self.msgCounter > 30:
                    print(self.connectedIMUs, end='\r')
                '''QUESTA È LA PARTE CHE COMUNICAVA CON ROS E NON CI SERVE PIÙ'''



'''QUI SI ISTANZIA IL NODO ROS CHE NON CI SERVE PIÙ E SI LEGGONO I PARAMETRI PASSATI DA LINEA DI COMANDO (X ES. LA PORTA SU CUI APRIRE IL SOCKET)'''
def main(): 
    imu_driver = ImuDriver(2390)
    imu_driver.listener()


'''CHIAMIAMO IL MAIN()'''
if __name__ == '__main__':
    main()