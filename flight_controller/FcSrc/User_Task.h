#ifndef __USER_TASK_H
#define __USER_TASK_H

#include "SysConfig.h"

extern int takeoff_if;
extern int takeoff_ready;

void UserTask_OneKeyCmd(void);
void Duoji_Set(int place);
void Duoji_top_set(void);

#endif
