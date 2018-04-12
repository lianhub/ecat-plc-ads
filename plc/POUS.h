#ifndef __POUS_H
#define __POUS_H

#include "accessor.h"
#include "iec_std_lib.h"

// PROGRAM MY_PROGRAM
// Data part
typedef struct {
  // PROGRAM Interface - IN, OUT, IN_OUT variables

  // PROGRAM private variables - TEMP, private and located variables
  __DECLARE_LOCATED(BOOL,BUTTON)
  __DECLARE_LOCATED(BOOL,LAMP)
  TOF T0;
  __DECLARE_LOCATED(UINT,VAR1)
  __DECLARE_LOCATED(UINT,VAR2)

} MY_PROGRAM;

void MY_PROGRAM_init__(MY_PROGRAM *data__, BOOL retain);
// Code part
void MY_PROGRAM_body__(MY_PROGRAM *data__);
#endif //__POUS_H
