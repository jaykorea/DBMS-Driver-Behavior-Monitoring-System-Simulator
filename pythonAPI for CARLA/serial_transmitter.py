import sys
import os
import time
import numpy as np

# sys.path.append("~/carla-simulator/PythonAPI/examples/manual_control_copy")
import manual_control_v2 as mc

# ==============================================================================
# -- Serial transmitter---------------------------------------------------------
# ==============================================================================

from pySerialTransfer import pySerialTransfer as txfer
import serial.tools.list_ports

class SerialTransmitter:
    def __init__(self):
        self.vehicle_speed = 0.0
        self.vehicle_throttle = 0.0
        self.vehicle_accelX = 0.0
        self.vehicle_accelY = 0.0
        self.vehicle_lat = 0.0
        self.vehicle_lon = 0.0
        self.vehecle_rpm = 700.0

    def get_ports(self):

        ports = serial.tools.list_ports.comports()
    
        return ports

    def findPort(self, portsFound):
    
        commPort = 'None'
        numConnection = len(portsFound)
        strPort = []
    
        for i in range(0,numConnection):
            port = portsFound[i]
            strPort.append(str(port))
            print(strPort)

        # if 'MCU' port 'ACM0' in strPort select one:
        for s in strPort:
            if "ACM0" in s:
                selectedPort = s
                splitPort = selectedPort.split(' ')
                commPort = splitPort[0]
                print(commPort)
        
        return commPort  

    def connectToMCU(self):
        
        self.foundPorts = self.get_ports()
        connectPort = self.findPort(self.foundPorts)

        if connectPort != 'None':
            self.link = txfer.SerialTransfer(connectPort, 115200)
            print('Connected to ' + connectPort)
            self.link.open()
            time.sleep(4)

        else:
            print('Connection Issue!')
            self.link.close()
    
    # def statusbar(self):
    #     img = np.full((480, 640, 3), (255, 255, 255), np.uint8)
    #     #_text1 = str(self.temp_vehicle_speed)
    #     # text2 = "%3.0f" % self.temp_vehicle_throttle
    #     # text3 = "%2.2f " % self.temp_vehicle_accelX
    #     # text4 = "%2.2f " % self.temp_vehicle_accelY
    #     # text5 = "%2.6f " % self.temp_vehicle_lat
    #     # text6 = "%3.6f " % self.temp_vehicle_lon
    #     #cv2.putText(img,  _text1 (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    #     # cv2.putText(img,  text2, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    #     # cv2.putText(img,  text3, (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    #     # cv2.putText(img,  text4, (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    #     # cv2.putText(img,  text5, (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    #     # cv2.putText(img,  text6, (10, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    #     cv2.imshow("Status", img)
    #     cv2.waitKey()


    def get_data(self):      
        self.vehicle_speed = mc.temp_vehicle_speed
        self.vehicle_throttle = mc.temp_vehicle_throttle
        self.vehicle_rpm = mc.temp_vehicle_rpm
        self.vehicle_accelX = mc.temp_vehicle_accelX
        self.vehicle_accelY = mc.temp_vehicle_accelY
        self.vehicle_lat = mc.temp_vehicle_lat
        self.vehicle_lon = mc.temp_vehicle_lon

    def printdata(self):
        print('Speed : %dkm/h ' % self.vehicle_speed)
        print('Throttle : %3.0f ' % self.vehicle_throttle)
        print('Rpm : %3.0f ' % self.vehicle_rpm)
        print('AccelX : %2.2f ' % self.vehicle_accelX)
        print('AccelY : %2.2f ' % self.vehicle_accelY)
        print('Latitude : %3.2f ' % self.vehicle_lat)
        print('Longitude : %3.3f ' % self.vehicle_lon)

    def transmit_data(self):     
        sendsize = 0
        
        sendsize = self.link.tx_obj(self.vehicle_speed, start_pos=sendsize)
        sendsize = self.link.tx_obj(self.vehicle_throttle, start_pos=sendsize)
        sendsize = self.link.tx_obj(self.vehicle_rpm, start_pos=sendsize)
        sendsize = self.link.tx_obj(self.vehicle_accelX, start_pos=sendsize)
        sendsize = self.link.tx_obj(self.vehicle_accelY, start_pos=sendsize)
        sendsize = self.link.tx_obj(self.vehicle_lat, start_pos=sendsize)
        sendsize = self.link.tx_obj(self.vehicle_lon, start_pos=sendsize)
        self.link.send(sendsize)
        time.sleep(0.2)


                # while not self.link.available():
        #         if self.link.status < 0:
        #             if self.link.status == txfer.CRC_ERROR:
        #                 print('ERROR: CRC_ERROR')
        #             elif self.link.status == txfer.PAYLOAD_ERROR:
        #                 print('ERROR: PAYLOAD_ERROR')
        #             elif self.link.status == txfer.STOP_BYTE_ERROR:
        #                 print('ERROR: STOP_BYTE_ERROR')
        #             else:
        #                 print('ERROR: {}'.format(self.link.status))

if __name__ == '__main__':
    try:
        st=SerialTransmitter()
        st.connectToMCU()
        while True:
            st.get_data()
            st.printdata()
            st.transmit_data()
    except KeyboardInterrupt:
        st.link.close()
        print('\nCancelled by user. Bye!!!')
