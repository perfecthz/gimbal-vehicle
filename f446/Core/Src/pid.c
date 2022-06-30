
#include "pid.h"
#include "main.h"

extern IncPID_Struct PID_M1;
extern IncPID_Struct PID_M2;
extern IncPID_Struct PID_M3;
extern IncPID_Struct PID_M4;

void IncPID_Init(IncPID_Struct *PID, float P, float I, float D)
{
	PID->Proportion = P;
	PID->Integral  =  I;
	PID->Derivative = D;
	
	PID->Error = 0;//E(k)	
	PID->PrevError = 0;//E(k-1)
	PID->LastError = 0;//E(k-2)
}



float Incremental_PID(IncPID_Struct *PID, float NextPoint, float SetPoint)
{
  PID->Error = SetPoint - NextPoint;
    
  float Value = PID->Proportion * (PID->Error - PID->PrevError) + PID->Integral * PID->Error+\
  PID->Derivative * (PID->Error-2 * PID->PrevError + PID->LastError);
    
  PID->LastError = PID->PrevError;
  PID->PrevError = PID->Error;
  return Value; 
}

