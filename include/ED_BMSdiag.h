//--------------------------------------------------------------------------------
// (c) 2015-2018 by MyLab-odyssey
// (c) 2017-2020 by Jim Sokoloff
// (c) 2024 by Ryan Beesley
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
//! \file    ED_BMSdiag.h
//! \brief   Definitions and structures for the main program ED_BMSdiag.c
//! \date    2024-March
//! \author  Ryan Beesley
//! \version 1.0.9
//--------------------------------------------------------------------------------
#pragma once

#ifndef _ED_BMSDIAG_H_
#define _ED_BMSDIAG_H_

#define CS_PIN        9         //!< chip select pin of MCP2515 CAN-Controller
#define SD_CS_PIN     4         //!< CS for SD card, if you plan to use a logger...
#define CAN_INT_PIN   2         //!< INT pin on the OBD-II/CAN Shield

#define VERBOSE       1         //!< VERBOSE mode will output individual cell data
#define BOXPLOT       1         //!< Visualize cell statistics as boxplot
#define EXPDATA       1         //!< EXPDATA mode will output experimental / NOT VERIFIED data
#define HELP          1         //!< HELP menu active
#define ECHO          1         //!< local ECHO of CLI
#define NLG6TEST      1         //!< Test if the NLG6 fast charger is installed, 
                                //!< set zero to speed up startup with standard OBL!!!

#include <mcp_can.h>
#include <Timeout.h>
#include <Cmd.h>
#include "canDiag.h"

//Global definitions
char* const PROGMEM version = (char *) "1.0.9";
#define FAILURE F("* Measurement failed *")
#define ALL_OK F("* All measurements captured *")
#define MSG_OK F("OK")
#define MSG_FAIL F("F")
#define MSG_DOT F(".")

//Menu levels
typedef enum {MAIN, subBMS, subNLG6, subOBL, subCS} submenu_t;

//deviceStatus struct to store menu settings
typedef struct {
  submenu_t menu = MAIN;
  uint16_t timer = 30;
  bool logging = false;
  bool logFile = false;
  uint16_t logCount = 0;
  bool initialDump = true;
  bool experimental = false;
  bool cardPresent = false;
} deviceStatus_t;

enum {EE_Signature = 0, EE_InitialDumpAll, EE_logging, EE_logFile, EE_logInterval, EE_Experimental};
const byte kMagicSignature = 0xAA;

void ReadGlobalConfig(deviceStatus_t *config, bool force_write = false);
boolean getNLG6data();
void ReadCANtraffic_BMS(byte *selected, byte len);
boolean getBMSdata(byte *selected, byte len);
boolean getCLSdata();

extern deviceStatus_t myDevice;
extern CTimeout CAN_Timeout;
extern CTimeout CLI_Timeout;
extern CTimeout LOG_Timeout;
extern canDiag DiagCAN;
extern BatteryDiag_t BMS;
extern ChargerDiag_t NLG6;
extern CoolingSub_t CLS;
extern MCP_CAN CAN0;

#endif
