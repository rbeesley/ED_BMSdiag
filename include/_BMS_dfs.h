//--------------------------------------------------------------------------------
// (c) 2015-2017 by MyLab-odyssey
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
//--------------------------------------------------------------------------------
//! \file    BMS_dfs.h
//! \brief   Definitions and structures for the BMS module.
//! \date    2017-December
//! \author  MyLab-odyssey
//! \version 1.0.7
//--------------------------------------------------------------------------------
#ifndef BMS_DFS_H
#define BMS_DFS_H
#include <Arduino.h>

//Definitions for BMS
#define DATALENGTH 238
#define CELLCOUNT 93
#define RAW_VOLTAGES 0           //!< Use RAW values or calc ADC offset voltage
#define IQR_FACTOR 1.5           //!< Factor to define Outliners-Range, 1.5 for suspected outliners, 3 for definitive outliners

//Easter Egg from the HAL Laboratories in Urbana, Illinois
#define myVIN "WMEEJ9AA7EK737739" //example: enter your VIN to get a reminder from a paranoid, holonomic brain
//#define myVIN ""                //empty myVIN will disable easter egg ;-)

//Data structure soft-/hardware-revision
typedef struct {
  byte rev[3];                   //!< year, week, patchlevel
} Revision_t;

//Data structure for statistics (min, mean, max values)
template<typename T>
struct Stats{
  uint16_t min;                  //!< minimum
  byte p25_out_count;            //!< count of datasets below p25, including mininal value
  uint16_t p25;                  //!< 25th percentile
  T mean;                        //!< average, 
  uint16_t median;               //!< 50th percentile
  uint16_t p75;                  //!< 75th percentile
  byte p75_out_count;            //!< count of datasets above p75, including maximum value
  uint16_t max;                  //!< maximum
};

//BMS data structure
typedef struct {   
  Stats<uint16_t> ADCCvolts;     //!< average cell voltage in mV, no offset
                                 //!< minimum and maximum cell voltages in mV, add offset +1500
  int16_t ADCvoltsOffset;        //!< calculated offset between RAW cell voltages and ADCref, about 90mV
  
  Stats<uint16_t> Cap_As;        //!< cell capacity statistics from BMS measurement cycle
  float Cap_meas_quality;        //!< some sort of estimation factor??? after measurement cycle
  float Cap_combined_quality;    //!< some sort of estimation factor??? constantly updated
  uint16_t LastMeas_days;        //!< days elapsed since last successful measurement
  
  Stats<float> Cvolts;           //!< calculated statistics from individual cell voltage query              
  int16_t CV_min_at;             //!< cell number with voltage mininum in pack
  int16_t CV_max_at;             //!< cell number with voltage maximum in pack
  float Cvolts_stdev;            //!< calculated standard deviation (populated)
  
  Stats<float> Ccap_As;          //!< cell capacity statistics calculated from individual cell data
  int16_t CAP_min_at;            //!< cell number with capacity mininum in pack
  int16_t CAP_max_at;            //!< cell number with capacity maximum in pack
  
  int16_t CapInit;               //!< battery initial capacity (As/10), at a certain temperature maybe 45 degC
  int16_t CapLoss;               //!< battery capacity loss (x/1000) in %, reflects aging (distance related?)
  
  unsigned long HVoff_time;      //!< HighVoltage contactor off time in seconds
  unsigned long HV_lowcurrent;   //!< counter time of no current, reset e.g. with PLC heater or driving
  uint16_t OCVtimer;             //!< counter time in seconds to reach OCV state
  
  byte Day;                      //!< day of battery final testing
  byte Month;                    //!< month of battery final testing
  byte Year;                     //!< year of battery final testing

  byte ProdDay;                  //!< day of battery production
  byte ProdMonth;                //!< month of battery production
  byte ProdYear;                 //!< year of battery production

  Revision_t sw;                 //!< soft-revision
  Revision_t hw;                 //!< hardware-revision
  
  byte hour;                     //!< time in car: hour
  byte minutes;                  //!< time in car: minutes
  
  float SOC;                     //!< State of Charge, as reported by vehicle dash
  byte SOH;                      //!< Flag showing if degraded cells are found, or battery failure present 
  uint16_t realSOC;              //!< The internal SOC value in % (x/10)
    
  int16_t Amps;                  //!< battery current in ampere (x/32) reported by by BMS
  float Amps2;                   //!< battery current in ampere read by live data on CAN or from BMS
  float Power;                   //!< power as product of voltage and amps in kW
  
  float HV;                      //!< total voltage of HV system in V
  float LV;                      //!< 12V onboard voltage / LV system
  byte LV_DCDC_amps;             //!< current of DC/DC LV system, not 12V battery!
  
  unsigned long ODO;             //!< Odometer count
  
  int16_t Temps[13];             //!< three temperatures per battery unit (1 to 3)
                                 //!< + max, min, mean and coolant-in temperatures
  uint16_t Isolation;            //!< Isolation in DC path, resistance in kOhm
  uint16_t DCfault;              //!< Flag to show DC-isolation fault
  
  byte HVcontactState;           //!< contactor state: 0 := OFF, 2 := ON
  long HVcontactCyclesLeft;      //!< counter related to ON/OFF cyles of the car
  long HVcontactCyclesMax;       //!< static, seems to be maxiumum of contactor cycles 
  byte UnknownCounter[3];        //!< some incremental counter - for what?
  char BattVIN[18];              //!< VIN stored in BMS
  char CarVIN[18];               //!< VIN stored in ECU
  boolean fHAL = false;
} BatteryDiag_t; 

const PROGMEM byte rqBattHWrev[4]                 = {0x03, 0x22, 0xF1, 0x50};  // 3 bytes (Y-2000, week, PL)
const PROGMEM byte rqBattSWrev[4]                 = {0x03, 0x22, 0xF1, 0x51};  // 3 bytes (Y-2000, week, PL)
const PROGMEM byte rqBattVIN[4]                   = {0x03, 0x22, 0xF1, 0x90};  // 17 (compressible)
const PROGMEM byte rqBattTemperatures[4]          = {0x03, 0x22, 0x02, 0x01};  // 17 bytes
const PROGMEM byte rqBattModuleTemperatures[4]    = {0x03, 0x22, 0x02, 0x02};  // 63 bytes
const PROGMEM byte rqBattHVstatus[4]              = {0x03, 0x22, 0x02, 0x04};  // 5-6 bytes
const PROGMEM byte rqBattADCref[4]                = {0x03, 0x22, 0x02, 0x07};  // 6 bytes max, min, mean [4,6,8]
const PROGMEM byte rqBattVolts[6]                 = {0x03, 0x22, 0x02, 0x08, 28, 57}; // 93*2 raw voltage (starting at 4) typical range is +/- 8 (0xfc1-0xfc9)
const PROGMEM byte rqBattIsolation[4]             = {0x03, 0x22, 0x02, 0x09};  // 3 bytes, 2 for isolation [45], 1 for flags [6]
const PROGMEM byte rqBattAmps[4]                  = {0x03, 0x22, 0x02, 0x03};  //
const PROGMEM byte rqBattDate[4]                  = {0x03, 0x22, 0x03, 0x04};  // 3 bytes [46] -> Factory Acceptance Date
const PROGMEM byte rqBattProdDate[4]              = {0x03, 0x22, 0xF1, 0x8C};  // 6 bytes data [49] -> ASCII format? 14 bytes raw data
const PROGMEM byte rqBattCapacity[6]              = {0x03, 0x22, 0x03, 0x10, 31, 59}; // 430 bytes raw; 207 used; 93*2 raw capacity (start at 25) + 3 HVOffTime [57] + 3 lowCurrent [9-11] + 2 OCVtimer [12-13] + 1 SOH [14] + 6 min, max, mean (21, 17, 23) + 2 lastMeasurementDays [224] + 2 measurementQual [226] + 2 overallQuality [222]
// 0x45c0 - 0x4762 range on cabrio
// 
// 0x4512 - 0x45fb on peadpod as range, was a 0x44c6, 0x45fb,  
// 0x4697 on peapod was maybe highest ever? (not in this data dump, but was in 22 03 18) 
// Give 10 bits to capacity and 6 bits to voltage differential?
const PROGMEM byte rqBattHVContactorCyclesLeft[4] = {0x03, 0x22, 0x03, 0x0B};  // 3 bytes raw [46] - seemed absent on cabrio and peapod
const PROGMEM byte rqBattHVContactorMax[4]        = {0x03, 0x22, 0x03, 0x0C};  // 3 bytes raw [46] - seemed absent on cabrio and peapod
const PROGMEM byte rqBattHVContactorState[4]      = {0x03, 0x22, 0xD0, 0x00};  // 1 byte [3]

const PROGMEM byte rqCarVIN[4]                   = {0x02, 0x09, 0x02, 0x00};   //

//Experimental readouts
const PROGMEM byte rqBattCapInit[4]               = {0x03, 0x22, 0x03, 0x05};  //
const PROGMEM byte rqBattCapLoss[4]               = {0x03, 0x22, 0x03, 0x09};  //
const PROGMEM byte rqBattUnknownCounter[4]        = {0x03, 0x22, 0x01, 0x01};  //

#endif // of #ifndef BMS_DFS_H
