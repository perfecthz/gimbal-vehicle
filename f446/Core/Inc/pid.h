#ifndef __PID_H
#define __PID_H
#include "main.h"



#define PID_H (float)(1320/(float)(600))//PID参数转化

typedef struct
{
	__IO float Proportion;
	__IO float Integral;
	__IO float Derivative;
	
	__IO float Error;//E(k)	
	__IO float PrevError;//E(k-1)
	__IO float LastError;//E(k-2)
} IncPID_Struct;

void IncPID_Init(IncPID_Struct *PID, float P, float I, float D);
float Incremental_PID(IncPID_Struct *PID, float NextPoint, float SetPoint);
#endif
