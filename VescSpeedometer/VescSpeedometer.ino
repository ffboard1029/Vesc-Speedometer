/*
 * 
MIT License

Copyright (c) 2018 ffboard1029

Credit to: Adafruit Industries (adafruit.com)
           Benjamin Vedder (vedder.se)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */


#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_FeatherOLED_Custom.h>
#include <bluefruit.h>
#include <crc.h>

#define VBAT_PIN          (A7)
#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider

//for calculating speed from rpm
#define DISPLAY_IMPERIAL_UNITS true   //true for mph on display, false for kmh (can still be toggled with the button)
#define WHEEL_DIAMETER     90    //in mm EVEN WHEN USING IMPERIAL UNITS!!!!
#define GEAR_RATIO         1.0f  //1 for hub motors  Motor Teeth / Wheel Teeth for belt motors

#define COMM_GET_VALUES   4   //vesc uart command to get values

Adafruit_FeatherOLED_Custom  oled = Adafruit_FeatherOLED_Custom();

BLEClientDis  clientDis;
BLEClientUart clientUart;
bool _imperial;
bool _sendNewRequest;

int readVBAT(void) 
{
  int raw;

  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  // Let the ADC settle
  delay(1);

  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(VBAT_PIN);

  // Set the ADC back to the default settings
  analogReference(AR_DEFAULT);
  analogReadResolution(10);

  return raw;
}

void setup() {
  Serial.begin(9600);
  
  _imperial = DISPLAY_IMPERIAL_UNITS;
  
  bleCentralSetup();

  // Setup the OLED display
  oled.init();
  oled.clearDisplay();

  pinMode(VBAT_PIN, INPUT);
  readVBAT();
  oled.setUnits(_imperial);

  _sendNewRequest = true;
  
  oled.display();
}

void loop() {
  
  if ( Bluefruit.Central.connected() )
  {
    // Not discovered yet
    if ( clientUart.discovered() )
    {
      oled.setConnected(true);
      if ( _sendNewRequest)
      {
        char buff[6 + 1] = {0};
        if(buildGetValuesPacket(buff, 6))
        {
          clientUart.print( buff );
          _sendNewRequest = false;
        }
      }
    }
    {
      oled.setConnected(false);
    }
  }
  else
  {
    oled.setConnected(false);
  }
    // Get a raw ADC reading
  int vbat_raw = readVBAT();

  // Convert the raw value to compensated mv, taking the resistor-
  // divider into account (providing the actual LIPO voltage)
  // ADC range is 0..3000mV and resolution is 12-bit (0..4095),
  // VBAT voltage divider is 2M + 0.806M, which needs to be added back
  float vbat_mv = (float)vbat_raw * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP;

  //drawSpeed(0.0);
  //setBTConnected(true);

  oled.setSpeed(52.7f);
  oled.setBattery(vbat_mv/1000);
  oled.display();

  delay(500);
}

float rpmToSpeed(float rpm)
{
  float circ = WHEEL_DIAMETER * 3.1415926535; //pi
  float mmph = GEAR_RATIO * 60 * circ;
  if(_imperial)
  {
    return mmph / 1609340.0f;  //mm to miles
  }
  else
  {
    return mmph / 1000000.0f;  
  }
}

bool buildGetValuesPacket(char * buff, int len)
{
  if(len < 6)
  {
    return false;
  }
  buff[0] = 2;  //2 for short packets, 3 for packets over 256 in length
  buff[1] = 1;  //packet length, one byte for short packets 2 bytes for long packets
  buff[2] = COMM_GET_VALUES;  //the packet data

  //checksum
  short checksum = crc16((unsigned char *)buff + 2, 1);
  buff[3] = (char)(checksum >> 8);
  buff[4] = (char)(checksum & 0xFF);
  buff[5] = 3; //stop byte
  return true;
}

/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

void bleCentralSetup()
{
  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);
  
  Bluefruit.setName("Bluefruit52 Central");

  // Configure DIS client
  clientDis.begin();

  // Init BLE Central Uart Serivce
  clientUart.begin();
  clientUart.setRxCallback(bleuart_rx_callback);

  // Increase Blink rate to different from PrPh advertising mode
  Bluefruit.setConnLedInterval(250);

    // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Don't use active scan
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);                   // // 0 = Don't stop scanning after n seconds
}

/**
 * Callback invoked when scanner pick up an advertising data
 * @param report Structural advertising data
 */
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Check if advertising contain BleUart service
  if ( Bluefruit.Scanner.checkReportForService(report, clientUart) )
  {
    Serial.print("BLE UART service detected. Connecting ... ");

    // Connect to device with bleuart service in advertising
    Bluefruit.Central.connect(report);
  }
}

/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void connect_callback(uint16_t conn_handle)
{
  Serial.println("Connected");

  Serial.print("Dicovering DIS ... ");
  if ( clientDis.discover(conn_handle) )
  {
    Serial.println("Found it");
    char buffer[32+1];
    
    // read and print out Manufacturer
    memset(buffer, 0, sizeof(buffer));
    if ( clientDis.getManufacturer(buffer, sizeof(buffer)) )
    {
      Serial.print("Manufacturer: ");
      Serial.println(buffer);
    }

    // read and print out Model Number
    memset(buffer, 0, sizeof(buffer));
    if ( clientDis.getModel(buffer, sizeof(buffer)) )
    {
      Serial.print("Model: ");
      Serial.println(buffer);
    }

    Serial.println();
  }  

  Serial.print("Discovering BLE Uart Service ... ");

  if ( clientUart.discover(conn_handle) )
  {
    Serial.println("Found it");

    Serial.println("Enable TXD's notify");
    clientUart.enableTXD();

    Serial.println("Ready to receive from peripheral");
  }else
  {
    Serial.println("Found NONE");
    
    // disconect since we couldn't find bleuart service
    Bluefruit.Central.disconnect(conn_handle);
  }  
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  
  Serial.println("Disconnected");
}

bool foundStart = false;
int vescPacketLen = 0;
int totalBytesRead = 0;
bool foundRPM = false;

/**
 * Callback invoked when uart received data
 * @param uart_svc Reference object to the service where the data 
 * arrived. In this example it is clientUart
 */
void bleuart_rx_callback(BLEClientUart& uart_svc)
{
  uint8_t vescResponseData[64] = {0};
  Serial.print("[RX]: ");
  
  while ( uart_svc.available() )
  {
    
    int bytesRead = uart_svc.read(vescResponseData, 64);
    Serial.print("bytes read: ");
    Serial.print(bytesRead);
    if(!foundStart)
    {
      if(vescResponseData[0] == 2)
      {
        Serial.print(" len:");
        vescPacketLen = vescResponseData[1];
        Serial.print(vescPacketLen);
        foundStart = true;
      }
      else if(vescResponseData[0] == 3)
      {
        vescPacketLen = vescResponseData[1] << 8 + vescResponseData[2];
        foundStart = true;
      }
      else
      {
        Serial.print(" not vesc data?? or some other error...... :/");
        totalBytesRead += bytesRead;
        continue;
      }
    }

    if(foundStart)
    {
      int headerLen = 2;
      if(vescPacketLen > 256)
      {
        headerLen = 3;
      }
      if(totalBytesRead + bytesRead < headerLen + 23) // byte location for rpm
      {
        totalBytesRead += bytesRead;
         continue;
      }
      //location of rpm is 23 bytes into the actual data packet, and is 4 bytes long
      else if (totalBytesRead + bytesRead >= headerLen + 23 && !foundRPM)
      {
        foundRPM = true;
        int index = headerLen + 23 - totalBytesRead;
        Serial.print(index);
        Serial.print(" ");
        int rpm = (int)(vescResponseData[index++]) << 24 + (int)(vescResponseData[index++]) << 16 + 
                  (int)(vescResponseData[index++]) << 8 + (int)(vescResponseData[index]);
        Serial.print(rpm);
        Serial.print(" rpm ");
        Serial.print(rpmToSpeed(rpm));
        Serial.print(" mph");
        //oled.setSpeed(rpmToSpeed(rpm);
      }
      else if(totalBytesRead + bytesRead >= vescPacketLen + headerLen + 3)
      {
        Serial.print(" end? ");
        int index = vescPacketLen + headerLen + 2 - totalBytesRead;
        //found the end data
        if(vescResponseData[index] == 3)
        {
          Serial.print(" foundEND ");
          //_sendNewRequest = true;
          foundStart = false;
          totalBytesRead = 0;
          vescPacketLen = 0;
          foundRPM = false;
          continue;
        }
        else
        {
          //keep going?? this should be an error case so huh...
          totalBytesRead += bytesRead;
          continue;
        }
      }
    }
    totalBytesRead += bytesRead;
  }
  Serial.println();
  
}
