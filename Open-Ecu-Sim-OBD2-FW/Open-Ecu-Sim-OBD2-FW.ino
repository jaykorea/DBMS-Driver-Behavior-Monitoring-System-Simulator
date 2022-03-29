
/*
Open-Ecu-Sim-OBD2-FW 

Info
The goal of this project is to create an open source OBD-2 ECU simulator for OBD-2 reader, testing ,training and development.

Hardware
The hardware needed is a Arduino Uno and a MCP2515 or MCP25625 shield or controller, with a DB9 to OBD-2 female connector.

Build Depends
MCP_CAN Library
https://github.com/coryjfowler/MCP_CAN_lib

Usage
Connect to PC using the USB cable supplied with you Arduino
Run the Open-Ecu-Sim-OBD2-GUI to connect and control the simulation 


Copyright (C) 2020  Rick Spooner https://github.com/spoonieau

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#include <Arduino.h>
#include "mcp_can.h"
#include <SPI.h>

#include "SerialTransfer.h"
SerialTransfer myTransfer;
typedef struct STRUCT {
  float temp_vehicle_speed;
  float temp_vehicle_throttle;
  float temp_vehicle_rpm;
  float temp_vehicle_accelX;
  float temp_vehicle_accelY;
  float temp_vehicle_lat;
  float temp_vehicle_lon;
} DataStruct;

DataStruct ds;

bool transmit_flag = false;
//typedef struct STRUCT2 {
//  float temp2_vehicle_speed;
//  float temp2_vehicle_throttle;
//  float temp2_vehicle_rpm;
//  float temp2_vehicle_accelX;
//  float temp2_vehicle_accelY;
//  float temp2_vehicle_lat;
//  float temp2_vehicle_lon;
//} DataStruct2;
//
//DataStruct2 ds2;

//char query_cmd[] = "at+qd";

//char Serial_arr[50];
//char gyro_buf[] = "";
//char gyro_buf1[] = "";
//char gyro_buf2[] = "";


// Set INT to pin 2
#define CAN0_INT 2 //Defines which board you are using

// Set CS to pin 10 for the CAB-BUS sheild, will need to check you sheilds documentation for correct pin.                                     
MCP_CAN CAN0(53); //Defines which board you are using                                

//Default reply ECU ID
#define REPLY_ID 0x7E8 

//=================================================================
//Setup Needed Vars
//=================================================================

//Current Firmware Version
char FW_Version[] = "0.10";  

// Incoming CAN-BUS message
long unsigned int canId = 0x000;

// This is the length of the incoming CAN-BUS message
unsigned char len = 0;

// This the eight byte buffer of the incoming message data payload
unsigned char buf[8];

//char str[20];
String canMessageRead = "";

// MIL on and DTC Present 
bool MIL = true;

//Stored Vechicle VIN 
unsigned char vehicle_Vin[18] = "1WK58FB1111111111"; //"KNABX511BCT012366" testing VIN

//Stored Calibration ID
unsigned char calibration_ID[18] = "FW00116MHZ1111111";

//Stored ECU Name
unsigned char ecu_Name[19] = "OPENECUSIM";

//OBD standards https://en.wikipedia.org/wiki/OBD-II_PIDs#Service_01_PID_1C
int obd_Std = 11;

//Fuel Type Coding https://en.wikipedia.org/wiki/OBD-II_PIDs#Fuel_Type_Coding
int fuel_Type = 7;

//Default PID values
unsigned int engine_Coolant_Temperature  = 80;
unsigned int intake_Manifold_Pressure = 10;
unsigned int engine_Load = 18;
float engine_Rpm  =  2000;
unsigned int vehicle_Speed = 100;
int timing_Advance  = 10;
int intake_Temp = 25;
unsigned int throttle_Position = 8;
unsigned int mass_Air_Flow_Rate =  3;
unsigned int ctrlModuleVoltage = 14;
static float runTime;
unsigned int fuel_Level = 50;
float AFR = 1.0;
float relative_throttle_Position = 8;
int ambient_Air_Temp = 30;
float commanded_Throttle_Actuator = 8;
unsigned int relative_Pedal_Position = 8;
int oil_Temp = 60;

int ledState = LOW;

uint16_t recSize = 0;

//=================================================================
//Init CAN-BUS and Serial
//=================================================================

void setup() {
   pinMode(LED_BUILTIN, OUTPUT);
    
   Serial.begin(115200);
   Serial3.begin(115200);
   Serial2.begin(115200);
   myTransfer.begin(Serial);
   delay(100);
   Serial3.println("*********************************************************");
   Serial3.println("*          Open-Ecu-Sim-OBD2-FW " + String(FW_Version) + "                    *");

START_CAN_INIT:

//Try and init your CAN-BUS sheild. 
//You will have to check your crystal oscillator on your CAN-BUS sheild and change 'MCP_16MHZ' accordingly.    
  if (CAN_OK == CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ))
  {
    Serial3.println("CAN BUS Shield init ok!");
    CAN0.setMode(MCP_NORMAL);
    //CAN0.setMode(MCP_LOOPBACK); 
    pinMode(CAN0_INT, INPUT); 
  }
  else
  {
    Serial3.println("ERROR!!!! CAN-BUS Shield init fail");
    Serial3.println("ERROR!!!! Will try to init CAN-BUS shield again");
    delay(100);
    goto START_CAN_INIT;
  }
}

void loop()
{
    if(myTransfer.available() > 0)
  {
    // use this variable to keep track of how many
    // bytes we've processed from the receive buffer
    recSize = 0;
//    Serial_arr[0] = {'/0'};

    recSize = myTransfer.rxObj(ds, recSize);

//    ds2.temp2_vehicle_speed = ds.temp_vehicle_speed/(float)1;
//    ds2.temp2_vehicle_throttle = ds.temp_vehicle_throttle/(float)1;
//    ds2.temp2_vehicle_rpm = ds.temp_vehicle_rpm/(float)1;
//    ds2.temp2_vehicle_accelX = ds.temp_vehicle_accelX/(float)100;
//    ds2.temp2_vehicle_accelY = ds.temp_vehicle_accelY/(float)100;
//    ds2.temp2_vehicle_lat = ds.temp_vehicle_lat/(float)1000;
//    ds2.temp2_vehicle_lon = ds.temp_vehicle_lon/(float)1000;

    Serial3.print(ds.temp_vehicle_speed);
    Serial3.print(" ");
    Serial3.print(ds.temp_vehicle_throttle);
    Serial3.print(" ");
    Serial3.print(ds.temp_vehicle_rpm);
    Serial3.print(" ");
    Serial3.print(ds.temp_vehicle_accelX);
    Serial3.print(" ");
    Serial3.print(ds.temp_vehicle_accelY);
    Serial3.print(" ");
    Serial3.print(ds.temp_vehicle_lat);
    Serial3.print(" ");
    Serial3.print(ds.temp_vehicle_lon);
    Serial3.print(" ");    
    Serial3.print(" | ");
    Serial3.println(" ");

//    recSize = myTransfer.rxObj(Serial_arr, recSize);
//    Serial3.println(Serial_arr);
    
//    dtostrf(ds.temp_vehicle_accelX, 4, 2, gyro_buf1);
//    dtostrf(ds.temp_vehicle_accelY, 4, 2, gyro_buf2);
//    sprintf(gyro_buf, "%s,%s", gyro_buf1, gyro_buf2);
  }


  if (Serial2.available() > 0 )
  {
    String cmdString = "at+qd";
    String recString = Serial2.readStringUntil('\n');
    int index = recString.indexOf('\r');
    String parsedString = recString.substring(0, index);
    Serial3.println(parsedString);
    if (parsedString.equals(cmdString))
    {
      //Serial2.println(gyro_buf);
      Serial2.print(ds.temp_vehicle_accelX);
      Serial2.print(",");
      Serial2.print(ds.temp_vehicle_accelY);
      Serial2.print('<');
//      Serial3.println(gyro_buf);
    }
  }


//=================================================================
//Random PID Values - Used if want in stand alone mode... Testing
//=================================================================
  engine_Coolant_Temperature = random(80, 90);
  intake_Manifold_Pressure = random(30, 101);
  engine_Load = random(0, 100);
  engine_Rpm = ds.temp_vehicle_rpm;
  vehicle_Speed = ds.temp_vehicle_speed;
  timing_Advance = random(-20, 35);
  intake_Temp = random(10, 20);
  throttle_Position = random(10, 80);
  mass_Air_Flow_Rate = 0;//random(2, 30);
  ctrlModuleVoltage = random(12,14);
  runTime = millis() / 1000;
  fuel_Level = random(50, 60);
  AFR = (random(100,150)/ (float)100.0);
  relative_throttle_Position = random(0, 100);
  ambient_Air_Temp = random(10, 20);
  commanded_Throttle_Actuator = ds.temp_vehicle_throttle;
  relative_Pedal_Position = random(10, 80);
  oil_Temp = random(70, 95); // Set the values as similar as engine's running


//=================================================================
//Define ECU Supported PID's
//=================================================================

  // Define the set of PIDs for MODE01 you wish you ECU to support.  For more information, see:
  // https://en.wikipedia.org/wiki/OBD-II_PIDs#Mode_1_PID_00
  //
  // PID 0x01 (1) - Monitor status since DTCs cleared. (Includes malfunction indicator lamp (MIL) status and number of DTCs.)
  // |   PID 0x05 (05) - Engine Coolant Temperature
  // |   |      PID 0x0C (12) - Engine RPM
  // |   |      |PID 0x0D (13) - Vehicle speed
  // |   |      ||PID 0x0E (14) - Timing advance
  // |   |      |||PID 0x0F (15) - Intake air temperature
  // |   |      ||||PID 0x10 (16) - MAF Air Flow Rate
  // |   |      |||||            PID 0x1C (28) - OBD standards this vehicle conforms to
  // |   |      |||||            |                              PID 0x51 (58) - Fuel Type
  // |   |      |||||            |                              |
  // v   V      VVVVV            V                              v
  // 10001000000111110000:000000010000000000000:0000000000000000100
  // Converted to hex, that is the following four byte value binary to hex
  // 0x881F0000 0x00 PID 01 -20
  // 0x02000000 0x20 PID 21 - 40
  // 0x04000000 0x40 PID 41 - 60

  // Next, we'll create the bytearray that will be the Supported PID query response data payload using the four bye supported pi hex value
  // we determined above (0x081F0000):

  //                               0x06 - additional meaningful bytes after this one (1 byte Service Mode, 1 byte PID we are sending, and the four by Supported PID value)
  //                                |    0x41 - This is a response (0x40) to a service mode 1 (0x01) query.  0x40 + 0x01 = 0x41
  //                                |     |    0x00 - The response is for PID 0x00 (Supported PIDS 1-20)
  //                                |     |     |    0x88 - The first of four bytes of the Supported PIDS value
  //                                |     |     |     |    0x1F - The second of four bytes of the Supported PIDS value
  //                                |     |     |     |     |    0x00 - The third of four bytes of the Supported PIDS value
  //                                |     |     |     |     |      |   0x00 - The fourth of four bytes of the Supported PIDS value
  //                                |     |     |     |     |      |    |    0x00 - OPTIONAL - Just extra zeros to fill up the 8 byte CAN message data payload)
  //                                |     |     |     |     |      |    |     |
  //                                V     V     V     V     V      V    V     V
  byte mode1Supported0x00PID[8] = {0x06, 0x41, 0x00, 0x88, 0x1F, 0x00, 0x00, 0x00};
  byte mode1Supported0x20PID[8] = {0x06, 0x41, 0x20, 0x02, 0x00, 0x00, 0x00, 0x00};
  byte mode1Supported0x40PID[8] = {0x06, 0x41, 0x40, 0x04, 0x00, 0x00, 0x00, 0x00};

  // Define the set of PIDs for MODE09 you wish you ECU to support.
  // As per the information on bitwise encoded PIDs (https://en.wikipedia.org/wiki/OBD-II_PIDs#Mode_1_PID_00)
  // Our supported PID value is:
  //
  //  PID 0x02 - Vehicle Identification Number (VIN)
  //  | PID 0x04 (04) - Calibration ID
  //  | |     PID 0x0C (12) - ECU NAME
  //  | |     |
  //  V V     V
  // 01010000010  // Converted to hex, that is the following four byte value binary to hex
  // 0x28200000 0x00 PID 01-11

  // Next, we'll create the bytearray that will be the Supported PID query response data payload using the four bye supported pi hex value
  // we determined above (0x28200000):

  //                               0x06 - additional meaningful bytes after this one (1 byte Service Mode, 1 byte PID we are sending, and the four by Supported PID value)
  //                                |    0x41 - This is a response (0x40) to a service mode 1 (0x01) query.  0x40 + 0x01 = 0x41
  //                                |     |    0x00 - The response is for PID 0x00 (Supported PIDS 1-20)
  //                                |     |     |    0x28 - The first of four bytes of the Supported PIDS value
  //                                |     |     |     |    0x20 - The second of four bytes of the Supported PIDS value
  //                                |     |     |     |     |    0x00 - The third of four bytes of the Supported PIDS value
  //                                |     |     |     |     |      |   0x00 - The fourth of four bytes of the Supported PIDS value
  //                                |     |     |     |     |      |    |    0x00 - OPTIONAL - Just extra zeros to fill up the 8 byte CAN message data payload)
  //                                |     |     |     |     |      |    |     |
  //                                V     V     V     V     V      V    V     V
  byte mode9Supported0x00PID[8] = {0x06, 0x49, 0x00, 0x28, 0x28, 0x00, 0x00, 0x00};

//=================================================================
//Vars to help build msg
//=================================================================

  ////Build setting return msg
  byte obd_Std_Msg[8] = {4, 65, 0x1C, (byte)(obd_Std)};
  byte fuel_Type_Msg[8] = {4, 65, 0x51, (byte)(fuel_Type)};

  //Work out eng RPM
  float rpm_Val = engine_Rpm * 4;
  unsigned int rpm_A = (long)rpm_Val / 256;
  unsigned int rpm_B = (long)rpm_Val % 256;

  //Work out MAF values
  float maf_Val = mass_Air_Flow_Rate * 100;
  unsigned int maf_A = (long)maf_Val / 256;
  unsigned int maf_B = (long)maf_Val;

  //Work out runTime values
  unsigned int runTime_A = (long)runTime /256 ;
  unsigned int runTime_B = (long)runTime;

  //Work out ctrlModule values
  float ctrlModuleVoltage_Val = ctrlModuleVoltage * 1000;
  unsigned int ctrlModuleVoltage_A = (long)ctrlModuleVoltage_Val / 256;
  unsigned int ctrlModuleVoltage_B = (long)ctrlModuleVoltage_Val;
  
  
  //Work out AFR values
  float AFR_Val = AFR * (65536/2);
  unsigned int AFR_A = (long)AFR_Val / 256;
  unsigned int AFR_B = (long)AFR_Val;
  
  

  //Build sensor return msg
  byte engine_Coolant_Temperature_Msg[8] = {3, 65, 0x05, (byte)(engine_Coolant_Temperature + 40)};
  byte intake_Manifold_Pressure_Msg[8] = {3, 65, 0x0B, (byte)(intake_Manifold_Pressure)};
  byte engine_Load_Msg[8] = {3, 65, 0x04, (byte)(engine_Load*2.55)};
  byte engine_Rpm_Msg[8] = {4, 65, 0x0C, (byte)rpm_A, (byte)rpm_B};
  byte vehicle_Speed_Msg[8] = {3, 65, 0x0D, (byte)(vehicle_Speed)};
  byte timing_Advance_Msg[8] = {3, 65, 0x0E, (byte)((timing_Advance + 64) * 2)};
  byte intake_Temp_Msg[8] = {3, 65, 0x0F, (byte)(intake_Temp + 40)};
  byte throttle_Position_Msg[8] = {3, 65, 0x11, (byte)(throttle_Position*2.55)};
  byte mass_Air_Flow_Rate_Msg[8] = {4, 65, 0x10, (byte)maf_A, (byte)maf_B};
  byte engine_runTime_Msg[8] = {4, 65, 0x1F, (byte)runTime_A, (byte)runTime_B};
  byte dist_with_MIL_Msg[8] = {4, 65, 0x21, (byte)runTime_A, (byte)runTime_B};
  byte fuel_Level_Msg[8] = {3, 65, 0x2F, (byte)(fuel_Level*2.55)};
  byte dist_since_codes_cleared_Msg[8] = {4, 65, 0x31, (byte)runTime_A, (byte)runTime_B}; 
  byte ctrl_Module_Msg[8] = {4, 65, 0x42, (byte)ctrlModuleVoltage_A, (byte)ctrlModuleVoltage_B};
  byte AFR_Msg[8] = {4, 65, 0x44, (byte)AFR_A, (byte)AFR_B};
  byte relative_throttle_Position_Msg[8] = {3, 65, 0x45, (byte)(relative_throttle_Position*2.55)};
  byte ambient_Air_Temp_Msg[8] = {3, 65, 0x46, (byte)(ambient_Air_Temp + 40)};
  byte commanded_Throttle_Actuator_Msg[8] = {3, 65, 0x4C, (byte)(commanded_Throttle_Actuator*2.55)};
  byte relative_Pedal_Position_Msg[8] = {3, 65, 0x5A, (byte)(relative_Pedal_Position*2.55)};
  byte oil_Temp_Msg[8] = {3, 65, 0x5C, (byte)(oil_Temp + 40)};
  
  //Serial return message
  String reply;

//=================================================================
//Handel Recived CAN-BUS frames from service tool
//=================================================================
  //if(CAN_MSGAVAIL == CAN0.checkReceive())
  if (!digitalRead(CAN0_INT))
  {
    CAN0.readMsgBuf(&canId, &len, buf);
    //https://en.wikipedia.org/wiki/OBD-II_PIDs#CAN_(11-bit)_bus_format
    
    Serial3.print("Received: "); Serial3.print(canId, HEX); Serial3.print(",");

    for (int i = 0; i < 3; i++)
    {
      canMessageRead = canMessageRead + buf[i] + ",";
      delay(1);
    }
    Serial3.println(canMessageRead);

//=================================================================
//Return CAN-BUS Messages - SUPPORTED PID's 
//=================================================================

    if (canMessageRead == "2,1,0,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, mode1Supported0x00PID);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)mode1Supported0x00PID));
      Serial3.println("Reply: " + reply);
      reply = "";
    }

    else if (canMessageRead == "2,1,32,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, mode1Supported0x20PID);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)mode1Supported0x20PID));
      Serial3.println("Reply: " + reply);
      reply = "";
    }

    else if (canMessageRead == "2,1,64,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, mode1Supported0x40PID);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)mode1Supported0x40PID));
      Serial3.println("Reply: " + reply);
      reply = "";
    }

    else if (canMessageRead == "2,9,0,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, mode9Supported0x00PID);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)mode9Supported0x00PID));
      Serial3.println("Reply: " + reply);
      reply = "";
    }
    
//=================================================================
//Return CAN-BUS Messages - RETURN PID VALUES - SENSORS 
//=================================================================    

    //Engine Load
    else if (canMessageRead == "2,1,4,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, engine_Load_Msg);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)engine_Load_Msg));
      Serial3.println("Reply: " + reply);
      reply = "";
    }

     
    
    //Engine Coolant
    else if (canMessageRead == "2,1,5,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, engine_Coolant_Temperature_Msg);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)engine_Coolant_Temperature_Msg));
      Serial3.println("Reply: " + reply);
      reply = "";
    }
     
    
    //Intake Manifold Pressure
    else if (canMessageRead == "2,1,11,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, intake_Manifold_Pressure_Msg);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)intake_Manifold_Pressure_Msg));
      Serial3.println("Reply: " + reply);
      reply = "";
    }    
     
    
    //Rpm
    else if (canMessageRead == "2,1,12,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, engine_Rpm_Msg);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)engine_Rpm_Msg));
      Serial3.println("Reply: " + reply);
      reply = "";
    }
     
    
    //Speed
    else if (canMessageRead == "2,1,13,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, vehicle_Speed_Msg);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)vehicle_Speed_Msg));
      Serial3.println("Reply: " + reply);
      reply = "";
    }
     
    
  /*  //Timing Adv
    if (canMessageRead == "2,1,14,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, timing_Advance_Msg);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)timing_Advance_Msg));
      Serial.println("Reply: " + reply);
      reply = "";
    } */

    //Intake Tempture
    else if (canMessageRead == "2,1,15,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, intake_Temp_Msg);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)intake_Temp_Msg));
      Serial3.println("Reply: " + reply);
      reply = "";
    }
     
    
    //MAF
    else if (canMessageRead == "2,1,16,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, mass_Air_Flow_Rate_Msg);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)mass_Air_Flow_Rate_Msg));
      Serial3.println("Reply: " + reply);
      reply = "";
    }
     
        
    //Throttle Position 1byte
    else if (canMessageRead == "2,1,17,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, throttle_Position_Msg);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)throttle_Position_Msg));
      Serial3.println("Reply: " + reply);
      reply = "";
    }
     
       
    //OBD standard
    else if (canMessageRead == "2,1,28,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, obd_Std_Msg);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)obd_Std_Msg));
      Serial3.println("Reply: " + reply);
      reply = "";
    } 

    //Engine RunTime 2bytes
    else if (canMessageRead == "2,1,31,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, engine_runTime_Msg);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)engine_runTime_Msg));
      Serial3.println("Reply: " + reply);
      reply = "";
    }
     
    
    //Dist with MIL
    else if (canMessageRead == "2,1,33,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, dist_with_MIL_Msg);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)dist_with_MIL_Msg));
      Serial3.println("Reply: " + reply);
      reply = "";
    }
     
    
    //Fuel Level
    else if (canMessageRead == "2,1,47,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, fuel_Level_Msg);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)fuel_Level_Msg));
      Serial3.println("Reply: " + reply);
      reply = "";
    }
     
    
    //Dist Since Codes Cleared
    else if (canMessageRead == "2,1,49,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, dist_since_codes_cleared_Msg);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)dist_since_codes_cleared_Msg));
      Serial3.println("Reply: " + reply);
      reply = "";
    }    
     
    
    //Fuel Type Coding
    else if (canMessageRead == "2,1,58,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, fuel_Type_Msg);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)fuel_Type_Msg));
      Serial3.println("Reply: " + reply);
      reply = "";
    }
     
    
    //control module voltage
    else if (canMessageRead == "2,1,66,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, ctrl_Module_Msg);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)ctrl_Module_Msg));
      Serial3.println("Reply: " + reply);
      reply = "";
    } 
     
    
    //AFR
    else if (canMessageRead == "2,1,68,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, AFR_Msg);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)AFR_Msg));
      Serial3.println("Reply: " + reply);
      reply = "";
    }
     
    
    //Relative Throttle Position
    else if (canMessageRead == "2,1,69,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, relative_throttle_Position_Msg);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)relative_throttle_Position_Msg));
      Serial3.println("Reply: " + reply);
      reply = "";
    }
     
    
    //Ambient Air Temp
    else if (canMessageRead == "2,1,70,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, ambient_Air_Temp_Msg);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)ambient_Air_Temp_Msg));
      Serial3.println("Reply: " + reply);
      reply = "";
    }
     
    
    //Commanded Throttle Actuator
    else if (canMessageRead == "2,1,76,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, commanded_Throttle_Actuator_Msg);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)commanded_Throttle_Actuator_Msg));
      Serial3.println("Reply: " + reply);
      reply = "";
    }

    //Fuel Type
    else if (canMessageRead == "2,1,81,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, fuel_Type_Msg);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)fuel_Type_Msg));
      Serial3.println("Reply: " + reply);
      reply = "";
    }
     
    
    //Relative Pedal Position
    else if (canMessageRead == "2,1,90,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, relative_Pedal_Position_Msg);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)relative_Pedal_Position_Msg));
      Serial3.println("Reply: " + reply);
      reply = "";
    }
     
    
    //Oil Temp
    else if (canMessageRead == "2,1,92,")
    {
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, oil_Temp_Msg);
      reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char*)oil_Temp_Msg));
      Serial3.println("Reply: " + reply);
      reply = "";
    }     


//=================================================================
//Return CAN-BUS Messages - RETURN PID VALUES - DATA 
//=================================================================

    //VIN
    else if (canMessageRead == "2,9,2,")
    {
      unsigned char frame1[8] = {16, 20, 73, 2, 1, vehicle_Vin[0], vehicle_Vin[1], vehicle_Vin[2]};
      unsigned char frame2[8] = {33, vehicle_Vin[3], vehicle_Vin[4], vehicle_Vin[5], vehicle_Vin[6], vehicle_Vin[7], vehicle_Vin[8], vehicle_Vin[9]};
      unsigned char frame3[8] = {34, vehicle_Vin[10], vehicle_Vin[11], vehicle_Vin[12], vehicle_Vin[13], vehicle_Vin[14], vehicle_Vin[15], vehicle_Vin[16]};

      CAN0.sendMsgBuf(REPLY_ID, 0, 8, frame1);
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, frame2);
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, frame3);
    }
    
    //CAL ID
    else if (canMessageRead == "2,9,4,")
    {
       unsigned char frame1[8] = {16, 20, 73, 4, 1, calibration_ID[0], calibration_ID[1], calibration_ID[2]};
       unsigned char frame2[8] = {33, calibration_ID[3], calibration_ID[4], calibration_ID[5], calibration_ID[6], calibration_ID[7], calibration_ID[8], calibration_ID[9]};
       unsigned char frame3[8] = {34, calibration_ID[10], calibration_ID[11], calibration_ID[12], calibration_ID[13], calibration_ID[14], calibration_ID[15], calibration_ID[16]};

      CAN0.sendMsgBuf(REPLY_ID, 0, 8, frame1);
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, frame2);
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, frame3);
    }

    //ECU NAME
    else if (canMessageRead == "2,9,10,")
    {

      unsigned char frame1[8] = {10, 14, 49, 10, 01, ecu_Name[0], ecu_Name[1], ecu_Name[2]};
      unsigned char frame2[8] = {21, ecu_Name[3], ecu_Name[4], ecu_Name[5], ecu_Name[6], ecu_Name[7], ecu_Name[8], ecu_Name[9]};
      unsigned char frame3[8] = {22, ecu_Name[10], ecu_Name[11], ecu_Name[12], ecu_Name[13], ecu_Name[14], ecu_Name[15], ecu_Name[16]};
      unsigned char frame4[8] = {23, ecu_Name[17], ecu_Name[18]};

      CAN0.sendMsgBuf(REPLY_ID, 0, 8, frame1);
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, frame2);
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, frame3);
      CAN0.sendMsgBuf(REPLY_ID, 0, 8, frame4);
    }

//=================================================================
//Return CAN-BUS Messages - RETURN PID VALUES - DTC
//=================================================================

    //DTC
    else if (canMessageRead == "1,3,0,")
    {
      if (MIL)
      {
        unsigned char DTC[] = {6, 67, 1, 2, 23, 0, 0, 0}; //P0217
        CAN0.sendMsgBuf(REPLY_ID, 0, 8, DTC);
        reply = String(String(REPLY_ID, HEX) + ",0,8, " + String((char*)DTC));
        Serial3.println("Reply: " + reply);
        reply = "";
      }
      else
      {
        unsigned char DTC[] = {6, 67, 0, 0, 0, 0, 0, 0}; //No Stored DTC
        CAN0.sendMsgBuf(REPLY_ID, 0, 8, DTC);
        reply = String(String(REPLY_ID, HEX) + ",0,8, " + String((char*)DTC));
        Serial3.println("Reply: " + reply);
        reply = "";
      }
    }

    //DTC Clear 
    else if (canMessageRead == "1,4,0,")
    {
      MIL = false;
    }

    else
    {
      Serial3.println("AT Command not matched...");
      delay(10);
    }
      Blink();
    }
    delay(5);
     canMessageRead = "";
  }

void Blink() {
  if (ledState == LOW) {
    ledState = HIGH;
  } else {
    ledState = LOW;
  }
  digitalWrite(LED_BUILTIN, ledState);
}
