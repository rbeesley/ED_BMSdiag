//--------------------------------------------------------------------------------//
// ED BMSdiag, v0.21
// Retrieve battery diagnostic data from your smart electric drive EV.
//
// (c) 2016 by MyLab-odyssey
//
// Licensed under "MIT License (MIT)", see license file for more information.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER OR CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
// ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//--------------------------------------------------------------------------------//

//#define DO_DEBUG_UPDATE
#define RAW_VOLTAGES 0           // use RAW ADC values or calc offset
#define VERBOSE 1                // VERBOSE mode will output individual cell data

#ifndef DO_DEBUG_UPDATE
#define DEBUG_UPDATE(...)
#else
#define DEBUG_UPDATE(...) Serial.print(__VA_ARGS__)
#endif

#include <mcp_can.h>
#include <SPI.h>
#include <Timeout.h>
#include <Average.h>

#define DATALENGTH 456
#define CELLVOLTAGE_OFFSET 90

#define SPACER F("-----------------------------------------")

long unsigned int rxID;
unsigned char len = 0;
unsigned char rxLength = 0;
unsigned char rxBuf[8];
char payload[100];

unsigned int data[DATALENGTH];

Average <long> CellVoltage(93);
Average <int> CellCapacity(93);  

typedef struct {
  unsigned int Volts_min;
  unsigned int Volts_mean;
  unsigned int Volts_max;
  int VoltsOffset;

  unsigned int Cap_min_As;
  unsigned int Cap_mean_As;
  unsigned int Cap_max_As;
  
  unsigned long HVoff_time;
  float Cap_meas_quality;
  float Cap_combined_quality;
  int LastMeas_days;
  int SOH;
  
  float SOC;
  float CAP;
  float HV;
  
  unsigned int HVcontactState;
  long HVcontactCyclesLeft;
  long HVcontactCyclesMax;
  
} BatteryDiag_t;

BatteryDiag_t BattDiag;

int i = 0; int n = 0;
boolean fDiagOK = 0;

CTimeout CAN_Timeout(10000); 

unsigned long timer = 0;

unsigned char rqInit[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
unsigned char rqFlowControl[8] = {0x30, 0x08, 0x14, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

unsigned char rqBattADCref[8] = {0x03, 0x22, 0x02, 0x07, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned char rqBattVolts[8] = {0x03, 0x22, 0x02, 0x08, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned char rqBattCapacity[8] = {0x03, 0x22, 0x03, 0x10, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned char rqBattHVContactorCyclesLeft[8] = {0x03, 0x22, 0x03, 0x0B, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned char rqBattHVContactorMax[8] = {0x03, 0x22, 0x03, 0x0C, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned char rqBattHVContactorState[8] = {0x03, 0x22, 0xD0, 0x00, 0xFF, 0xFF, 0xFF, 0xFF};

byte rqFC_length = 8;                            // Interval to send flow control (rqFC) 

#define CS     10                                // CS-Pin CAN-Controller
#define CS_SD  8                                 // CS for SD card, if you plan to use a logger...

MCP_CAN CAN0(CS);                                // Set CS pin

void setup()
{  
  Serial.begin(115200);
  while (!Serial); // while the serial stream is not open, do nothing:
  Serial.println();
  
  pinMode(CS, OUTPUT);
  pinMode(CS_SD, OUTPUT);
  digitalWrite(CS_SD, HIGH);
  
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters enabled.
  if(CAN0.begin(MCP_STD, CAN_500KBPS, MCP_16MHZ) == CAN_OK) DEBUG_UPDATE(F("MCP2515 Init Okay!!\r\n"));
  else DEBUG_UPDATE(F("MCP2515 Init Failed!!\r\n"));
  
  clearCAN_Filter();
    
  digitalWrite(CS, HIGH);
  
  // Setting pin 2 for input, low if CAN messages are received
  pinMode(2, INPUT); 
  
  Serial.println(SPACER); 
  Serial.println(F("--- ED Battery Management Diagnostics ---"));
  Serial.println(SPACER);
  
  Serial.println(F("Connect to OBD port - Waiting for CAN-Bus "));
  do {
    Serial.print(F("."));
    delay(1000);
  } while (digitalRead(2));
  Serial.println(F("CONNECTED"));
}

void WaitforSerial() {
  Serial.println(F("Press a key to start query..."));
  while (!Serial.available()) {}                  // Wait for serial
}

void clearSerialBuffer() {
  do {                                            // Clear serial input buffer
      delay(10);
  } while (Serial.read() >= 0);
}

void clearCAN_Filter(){
  CAN0.init_Mask(0, 0, 0x00000000);
  CAN0.init_Mask(1, 0, 0x00000000);
  delay(100);
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.
}

void setCAN_Filter(unsigned long filter){
  filter = filter << 16;
  CAN0.init_Mask(0, 0, 0x07FF0000);
  CAN0.init_Mask(1, 0, 0x07FF0000);
  CAN0.init_Filt(0, 0, filter);
  CAN0.init_Filt(1, 0, filter);
  CAN0.init_Filt(2, 0, filter);
  CAN0.init_Filt(3, 0, filter);
  CAN0.init_Filt(4, 0, filter);
  CAN0.init_Filt(5, 0, filter);
  delay(100);
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.
}

unsigned int Request_Diagnostics(unsigned char* rqQuery){  
  // Reset Timeout-Timer
  CAN_Timeout.Reset();
  
  //Disable SD card
  digitalWrite(CS_SD, HIGH);
  
  //--- Diag Request Message ---
  DEBUG_UPDATE(F("Send Diag Request\n\r"));
  // send data: Request diagnostics data
  CAN0.sendMsgBuf(0x7E7, 0, 8, rqQuery);
  //wait for response of first frame
  return Get_RequestResponse();
}

unsigned int Get_RequestResponse(){ 
    
    unsigned int items = 0;
    byte FC_length = rqFlowControl[1];    
    boolean fDataOK = false;
    
    do{
      //--- Read Frames ---
      if(!digitalRead(2))                         // If pin 2 is low, read receive buffer
      {
        do{
          CAN0.readMsgBuf(&rxID, &len, rxBuf);    // Read data: len = data length, buf = data byte(s)       
          //Serial.println(rxBuf[0], HEX);
          if (rxID == 0x7EF) { 
            if(rxBuf[0] < 0x10) {
              if((rxBuf[1] != 0x7F)) {  
                for (i = 0; i<len; i++) {         // read data bytes, offset +1, 1 to 7
                    data[i] = rxBuf[i+1];       
                }
                DEBUG_UPDATE(F("SF reponse: "));
                DEBUG_UPDATE(rxBuf[0] & 0x0F); DEBUG_UPDATE("\n\r");
                items = 0;
                fDataOK = true;
              } else if (rxBuf[3] == 0x78) {
                DEBUG_UPDATE(F("pending reponse...\n\r"));
              } else {
                DEBUG_UPDATE(F("ERROR\n\r"));
              }
            }
            if ((rxBuf[0] & 0xF0) == 0x10){
              items = (rxBuf[0] & 0x0F)*256 + rxBuf[1]; // six data bytes already read (+ two type and length)
              for (i = 0; i<len; i++) {                 // read data bytes, offset +1, 1 to 7
                  data[i] = rxBuf[i+1];       
              }
              // send rqFC: Request for more data
              CAN0.sendMsgBuf(0x7E7, 0, 8, rqFlowControl);
              DEBUG_UPDATE(F("Resp, i:"));
              DEBUG_UPDATE(items - 6); DEBUG_UPDATE("\n\r");
              fDataOK = Read_FC_Response(items - 6);
            } 
          }     
        } while(!digitalRead(2) && !CAN_Timeout.Expired(false) && !fDataOK);
      }
    } while (!CAN_Timeout.Expired(false) && !fDataOK);

    if (fDataOK) {
      return (items + 7) / 7;
      DEBUG_UPDATE(F("success!\n\r"));
    } else {
      DEBUG_UPDATE(F("Event Timeout!\n\r")); 
      ClearReadBuffer(); 
      return 0; 
    } 
}

boolean Read_FC_Response(int items){   
    //Set wait timer
    CAN_Timeout.Reset();
    
    int n = 7;
    int FC_count = 0;
    byte FC_length = rqFlowControl[1];
    
    do{
      //--- Read Frames ---
      if(!digitalRead(2))                         // If pin 2 is low, read receive buffer
      {
        do{
          CAN0.readMsgBuf(&rxID, &len, rxBuf);    // Read data: len = data length, buf = data byte(s)       
          if((rxBuf[0] & 0xF0) == 0x20){
            FC_count++;
            items = items - len + 1;
            for(i = 0; i<len; i++) {              // copy each byte of the data
              if ((n < (DATALENGTH - 6)) && (i < 7)){
                data[n+i] = rxBuf[i+1];
              }       
            }
            //--- FC counter -> then send Flow Control Message ---
            if (FC_count % FC_length == 0 && items > 0) {
              // send rqFC: Request for more data
              CAN0.sendMsgBuf(0x7E7, 0, 8, rqFlowControl);
              DEBUG_UPDATE(F("FCrq\n\r"));
   
            }
            n = n + 7;
          }      
        } while(!digitalRead(2) && !CAN_Timeout.Expired(false) && items > 0);
      }
    } while (!CAN_Timeout.Expired(false) && items > 0);
    if (!CAN_Timeout.Expired(false)) {
      fDiagOK = 1;
      DEBUG_UPDATE(F("Items left: ")); DEBUG_UPDATE(items); DEBUG_UPDATE("\n\r");
      DEBUG_UPDATE(F("FC count: ")); DEBUG_UPDATE(FC_count); DEBUG_UPDATE("\n\r");
    } else {
      fDiagOK = 0;
      DEBUG_UPDATE(F("Event Timeout!\n\r"));
    } 
    ClearReadBuffer();    
    return fDiagOK;
}

void PrintReadBuffer(unsigned int items) {
  Serial.println(items);
  for(int i = 0; i < items; i++) {
      Serial.print(F("Data: "));
      for(int n = 0; n < 7; n++)               // Print each byte of the data.
      {
        if(data[n + 7 * i] < 0x10)             // If data byte is less than 0x10, add a leading zero.
        {
          Serial.print(F("0"));
        }
        Serial.print(data[n + 7 * i], HEX);
        Serial.print(" ");
      }
      Serial.println();
  }
}

void ClearReadBuffer(){
  if(!digitalRead(2)) {                        // still messages? pin 2 is low, clear read receive buffer
    for (int i = 1; i <= 2; i++) {
      CAN0.readMsgBuf(&rxID, &len, rxBuf);
    }
    DEBUG_UPDATE(F("Buffer cleared!\n\r"));
  }
}

void ReadCellCapacity(unsigned int data_in[], unsigned int highOffset, unsigned int length){
  for(int n = 0; n < (length * 2); n = n + 2){
    CellCapacity.push(data_in[n + highOffset] * 256 + data_in[n + highOffset + 1]);
  }
}

void ReadCellVoltage(unsigned int data_in[], unsigned int highOffset, unsigned int length){
  for(int n = 0; n < (length * 2); n = n + 2){
    CellVoltage.push(data_in[n + highOffset] * 256 + data_in[n + highOffset + 1]);
  }
}

void ReadDiagWord(unsigned int data_out[], unsigned int data_in[], unsigned int highOffset, unsigned int length){
  for(int n = 0; n < (length * 2); n = n + 2){
    data_out[n/2] = data_in[n + highOffset] * 256 + data_in[n + highOffset + 1];
    //Serial.print("#"); Serial.print((n/2)+1); Serial.print(": ");
    //Serial.println(data[n + highOffest] * 256 + data[n+5]);
    //Serial.print(data[n+4],HEX); Serial.println(data[n+5],HEX);
  }
}

boolean getBatteryCapacity(boolean debug_verbose) {
  unsigned int items = Request_Diagnostics(rqBattCapacity);
  if(items){
    if (debug_verbose) {
      PrintReadBuffer(items);
    }   
    ReadCellCapacity(data,25,93);

    BattDiag.HVoff_time = (unsigned long) data[5] * 65535 + data[6] * 256 + data[7];
    ReadDiagWord(&BattDiag.Cap_min_As,data,21,1);
    ReadDiagWord(&BattDiag.Cap_mean_As,data,23,1);
    ReadDiagWord(&BattDiag.Cap_max_As,data,17,1);
    BattDiag.SOH = data[14];
    BattDiag.LastMeas_days = data[428];
    unsigned int value;
    ReadDiagWord(&value,data,429,1);
    BattDiag.Cap_meas_quality = value / 65535.0;
    ReadDiagWord(&value,data,425,1);
    BattDiag.Cap_combined_quality = value / 65535.0;
    return true;
  } else {
    return false;
  }
}

boolean getBatteryVoltage(boolean debug_verbose) {
  unsigned int items;
  items = Request_Diagnostics(rqBattVolts);
  if(items){
    if (debug_verbose) {
      PrintReadBuffer(items);
    }   
    ReadCellVoltage(data,4,93);
    return true;
  } else {
    return false;
  }
}

boolean getBatteryVoltageMean(boolean debug_verbose) {
  unsigned int items;
  items = Request_Diagnostics(rqBattVolts);
  if(items){
    if (debug_verbose) {
      PrintReadBuffer(items);
    }   
    ReadCellVoltage(data,4,93);
    return true;
  } else {
    return false;
  }
}

boolean getBatteryADCref(boolean debug_verbose) {
  unsigned int items;
  items = Request_Diagnostics(rqBattADCref);
  if(items){
    if (debug_verbose) {
      PrintReadBuffer(items);
    }   
    ReadDiagWord(&BattDiag.Volts_mean,data,8,1);
    
    ReadDiagWord(&BattDiag.Volts_min,data,6,1);
    BattDiag.Volts_min += 1500;
    ReadDiagWord(&BattDiag.Volts_max,data,4,1);
    BattDiag.Volts_max += 1500;
    
    if (RAW_VOLTAGES) {
      BattDiag.VoltsOffset = 0;
    } else {
      BattDiag.VoltsOffset = (int) CellVoltage.mean() - BattDiag.Volts_mean;
    }
    
    return true;
  } else {
    return false;
  }
}

boolean getHVcontactorState(boolean debug_verbose) {
  unsigned int items;
  boolean fValid = false;
  items = Request_Diagnostics(rqBattHVContactorCyclesLeft);
  if(items){
    if (debug_verbose) {
      PrintReadBuffer(items);
    }   
    BattDiag.HVcontactCyclesLeft = (long) data[4] * 65536 + data[5] * 256 + data[6]; 
    fValid = true;
  } else {
    fValid = false;
  }
  items = Request_Diagnostics(rqBattHVContactorMax);
  if(items){
    if (debug_verbose) {
      PrintReadBuffer(items);
    }   
    BattDiag.HVcontactCyclesMax = (long) data[4] * 65536 + data[5] * 256 + data[6]; 
    fValid = true;
  } else {
    fValid = false;
  }
  items = Request_Diagnostics(rqBattHVContactorState);
  if(items){
    if (debug_verbose) {
      PrintReadBuffer(items);
    }   
    BattDiag.HVcontactState = (unsigned int) data[3]; 
    fValid = true;
  } else {
    fValid = false;
  }
  return fValid;
}

boolean ReadSOC() {
  setCAN_Filter(0x518);
  CAN_Timeout.Reset();
  
  if(!digitalRead(2)) {    
    do {    
    CAN0.readMsgBuf(&rxID, &len, rxBuf); 
      if (rxID == 0x518 ) {
        BattDiag.SOC = (float) rxBuf[7] / 2;      //soc = soc / 2;
        return true;
      }
    } while (!CAN_Timeout.Expired(false));
  }
  return false;
}

boolean ReadHV() {
  setCAN_Filter(0x448);
  CAN_Timeout.Reset();
  
  float HV;
  if(!digitalRead(2)) {    
    do {    
    CAN0.readMsgBuf(&rxID, &len, rxBuf); 
      if (rxID == 0x448 ) {
        HV = ((float)rxBuf[6]*256 + (float)rxBuf[7]);
        HV = HV / 10.0;
        BattDiag.HV = HV;
        return true;
      }
    } while (!CAN_Timeout.Expired(false));
  }
  return false;
}

void loop()
{       
   boolean fOK = false;
   
   //Wait for start via serial terminal
   WaitforSerial();
   clearSerialBuffer();
   Serial.println(SPACER);
   delay(500);
     
   fOK = ReadSOC();
   fOK = ReadHV();
   
   setCAN_Filter(0x7EF); 
   Serial.print(F("Read Voltages..."));
   if (fOK = getBatteryVoltage(false)) {
     Serial.println(F("OK"));
   } else {
     Serial.println(F("FAIL"));
   }
   Serial.print(F("Read Capacity..."));
   if (fOK = getBatteryCapacity(false)) {
     Serial.println(F("OK"));
   } else {
     Serial.println(F("FAIL"));
   }
   Serial.print(F("Read ADCref. ..."));
   if (fOK = getBatteryADCref(false)) {
     Serial.println(F("OK"));
   } else {
     Serial.println(F("FAIL"));
   }
   Serial.print(F("Read HV state..."));
   if (fOK = getHVcontactorState(false)) {
     Serial.println(F("OK"));
   } else {
     Serial.println(F("FAIL"));
   }
    
   if (fOK) {
      digitalWrite(CS, HIGH);
      Serial.println(SPACER);
      Serial.print(F("SOC     : ")); Serial.print(BattDiag.SOC,1);
      Serial.print(F(" %,  HV: ")); Serial.print(BattDiag.HV,1); Serial.println(F(" V"));
      Serial.println(SPACER);
      Serial.print(F("CV mean : ")); Serial.print(BattDiag.Volts_mean); Serial.print(F(" mV"));
      Serial.print(F(", ∆ CV: ")); Serial.print(BattDiag.Volts_max - BattDiag.Volts_min); Serial.println(F(" mV"));
      Serial.print(F("CV min  : ")); Serial.print(BattDiag.Volts_min); Serial.println(F(" mV"));
      Serial.print(F("CV max  : ")); Serial.print(BattDiag.Volts_max); Serial.println(F(" mV"));
      Serial.println(SPACER);
      Serial.print(F("CAP mean: ")); Serial.print(BattDiag.Cap_mean_As); Serial.print(F(" As/10, ")); Serial.print(BattDiag.Cap_mean_As / 360.0,1); Serial.println(F(" Ah"));
      Serial.print(F("CAP min : ")); Serial.print(BattDiag.Cap_min_As); Serial.print(F(" As/10, ")); Serial.print(BattDiag.Cap_min_As / 360.0,1); Serial.println(F(" Ah"));
      Serial.print(F("CAP max : ")); Serial.print(BattDiag.Cap_max_As); Serial.print(F(" As/10, ")); Serial.print(BattDiag.Cap_max_As / 360.0,1); Serial.println(F(" Ah"));
      Serial.print(F("Last measurement: ")); Serial.print(BattDiag.LastMeas_days); Serial.println(F(" day(s)"));
      Serial.println(SPACER);
      if (BattDiag.HVcontactState == 0x02) {
        Serial.println(F("HV contactor state ON"));
      } else if (BattDiag.HVcontactState == 0x00) {
        Serial.print(F("HV contactor state OFF"));
        Serial.print(F(", for: ")); Serial.print(BattDiag.HVoff_time); Serial.println(F(" s"));
      }
      Serial.print(F("HV contactor cycles left: ")); Serial.println(BattDiag.HVcontactCyclesLeft);
      Serial.print(F("           of max cycles: ")); Serial.println(BattDiag.HVcontactCyclesMax);
      Serial.println(SPACER);
      Serial.print(F("Meas. quality         : ")); Serial.println(BattDiag.Cap_meas_quality,3);
      Serial.print(F("Meas. quality combined: ")); Serial.println(BattDiag.Cap_combined_quality,3);
      Serial.println(SPACER);
      
      if (VERBOSE) {
        Serial.println(F("# ;mV  ;As/10"));
        for(n = 0; n < 93; n++){
          if (n < 9) Serial.print(F("0"));
          Serial.print(n+1); Serial.print(F(";")); Serial.print(CellVoltage.get(n) - BattDiag.VoltsOffset); Serial.print(F(";")); Serial.println(CellCapacity.get(n));
        }
        Serial.println(SPACER);
        Serial.print(F("CV mean : ")); Serial.print(CellVoltage.mean() - BattDiag.VoltsOffset,0); Serial.print(F(" mV"));
        Serial.print(F(", ∆ CV: ")); Serial.print(CellVoltage.maximum() - CellVoltage.minimum()); Serial.println(F(" mV"));
        Serial.print(F("CV min  : ")); Serial.print(CellVoltage.minimum() - BattDiag.VoltsOffset); Serial.println(F(" mV"));
        Serial.print(F("CV max  : ")); Serial.print(CellVoltage.maximum() - BattDiag.VoltsOffset); Serial.println(F(" mV"));
        Serial.println(SPACER);
      }   
   } else {
      Serial.println(F("Measurement failed"));
      fOK = false;
   } 
   
}
