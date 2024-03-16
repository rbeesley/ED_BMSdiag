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
//! \file    ED_BMSdiag_CLI.h
//! \brief   Declaration of functions for the Command Line Interface (CLI) menu system
//! \date    2024-March
//! \author  Ryan Beesley
//! \version 1.0.9
//--------------------------------------------------------------------------------
#pragma once

#ifndef _ED_BMSDIAG_CLI_H_
#define _ED_BMSDIAG_CLI_H_

#include <memory>

#include <Arduino.h>

boolean nlg6_installed();
boolean test_BattVIN();
void setupMenu();
void init_cmd_prompt();
void get_all (uint8_t arg_cnt, char **args);
void logdata();
void log_file();
void help(uint8_t arg_cnt, char **args);
void main_menu (uint8_t arg_cnt, char **args);
void show_splash(uint8_t arg_cnt, char **args);
void get_temperatures (uint8_t arg_cnt, char **args);
void get_voltages (uint8_t arg_cnt, char **args);
void bms_sub (uint8_t arg_cnt, char **args);
void cs_sub (uint8_t arg_cnt, char **args);
void nlg6_sub (uint8_t arg_cnt, char **args);
void obl_sub (uint8_t arg_cnt, char **args);
void get_rpt (uint8_t arg_cnt, char **args);
void set_logging(uint8_t arg_cnt, char **args);
void set_log_file(uint8_t arg_cnt, char **args);
void show_info(uint8_t arg_cnt, char **args);
void reset_factory_defaults(uint8_t arg_cnt, char **args);
void set_initial_dump(uint8_t arg_cnt, char **args);
void set_experimental(uint8_t arg_cnt, char **args);

#endif