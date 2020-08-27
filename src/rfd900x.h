#ifndef RFD900X_H
#define RFD900X_H

#include <Arduino.h>

// platformio doesn't seem to have F(), but has FPSTR and PSTR
#define F(string_literal) (FPSTR(PSTR(string_literal)))

void r900x_initiate_serials(void);
void r900x_setup(bool reflash);
void r900x_attempt_factory_reset(void);
int r900x_getparams(String filename, bool factory_reset_first);
bool r900x_saveparams(String filename);
int r900x_savesingle_param_and_verify(String prefix, String ParamID, String ParamVAL); 
int r900x_savesingle_param_and_verify_more(String prefix, String ParamID, String ParamVAL, bool save_and_reboot); 
int r900x_readsingle_param(String prefix, String ParamID);
uint8_t r900x_rssi_percentage(uint8_t mav_rssi_109);

#endif