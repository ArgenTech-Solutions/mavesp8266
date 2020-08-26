#ifndef RFD900X_H
#define RFD900X_H

// platformio doesn't seem to have F(), but has FPSTR and PSTR
#define F(string_literal) (FPSTR(PSTR(string_literal)))

void r900x_initiate_serials(void);
void r900x_setup(bool reflash);
void r900x_attempt_factory_reset(void);

#endif