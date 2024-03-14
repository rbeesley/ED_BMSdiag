// CAN Receive Example by Cory Fowler
// modified with CS constant, 2016 MyLab-odyssey

#include <mcp_can.h>
#include <SPI.h>

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];

#define CS 10                //!< chip select pin of MCP2515 CAN-Controller 
MCP_CAN CAN0(CS);                               

void setup()
{
  Serial.begin(115200);
  
  pinMode(CS, OUTPUT);
  pinMode(8, OUTPUT);       //!< disable chip select of SD card reader (commonly pin 8)
  digitalWrite(8, HIGH);
  
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");
  
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(2, INPUT);                            // Setting pin 2 for /INT input
  
  Serial.println("MCP2515 Library Receive Example...");
}

void loop()
{
    if(!digitalRead(2))                         // If pin 2 is low, read receive buffer
    {
      CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

      Serial.print("ID: ");                     // Print the message ID.
      Serial.print(rxId, HEX);

      Serial.print("  Data: ");
      for(int i = 0; i<len; i++)                // Print each byte of the data.
      {
        if(rxBuf[i] < 0x10)                     // If data byte is less than 0x10, add a leading zero.
        {
          Serial.print("0");
        }
        Serial.print(rxBuf[i], HEX);
        Serial.print(" ");
      }

      Serial.println();
    }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/