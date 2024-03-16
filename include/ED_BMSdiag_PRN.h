//--------------------------------------------------------------------------------
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
//! \file    ED_BMSdiag_PRN.h
//! \brief   Declarations of functions for serial printing the datasets
//! \date    2024-March
//! \author  Ryan Beesley
//! \version 1.0.9
//--------------------------------------------------------------------------------
#pragma once

#ifndef _ED_BMSDIAG_PRN_H_
#define _ED_BMSDIAG_PRN_H_

void printWelcomeScreen();
void printSplashScreen();
void printBMSall();
void printNLG6all();
void printCLSall();
void printRPT();
void printBMStemperatures();
void printNLG6temperatures();
void printBMS_CellVoltages();
void printNLG6_Status();
void PrintSPACER();

#endif
