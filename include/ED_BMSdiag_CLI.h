#pragma once

#ifndef _ED_BMSDIAG_CLI_H_
#define _ED_BMSDIAG_CLI_H_

#include <Arduino.h>

boolean nlg6_installed();
boolean test_BattVIN();
void setupMenu();
void init_cmd_prompt();
void get_all (uint8_t arg_cnt, char **args);
void logdata();
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
void show_info(uint8_t arg_cnt, char **args);
void reset_factory_defaults(uint8_t arg_cnt, char **args);
void set_initial_dump(uint8_t arg_cnt, char **args);
void set_experimental(uint8_t arg_cnt, char **args);

#endif