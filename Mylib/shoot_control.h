#ifndef _SHOOT_CONTROL_H_
#define _SHOOT_CONTROL_H_
#include "stdint.h"

extern int rub_flag;//s2-1λ����״̬
extern int shoot_flag;//s2-2λ����״̬

void BLDC_control(uint8_t s2, uint8_t press_r);
void Fire(uint8_t s2, uint8_t press_l);

#endif
