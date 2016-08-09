#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

typedef struct _PIDdata {

	//pid factors
	volatile float Kp;
	volatile float Ki;
	volatile float Kd;

	//pid terms
	volatile float Perr;
	volatile float Ierr;
	volatile float Derr;

	//pid terms limits
	volatile float Perrmin;
	volatile float Perrmax;
	volatile float Ierrmin;
	volatile float Ierrmax;

} PIDdata, *ptrPIDdata;


//functions
extern void PID_Init(ptrPIDdata pPd);
extern void PID_SetPID(ptrPIDdata pPd, float pidP, float pidI, float pidD);
extern void PID_SetLimitsPerr(ptrPIDdata pPd, float Perr_min, float Perr_max);
extern void PID_SetLimitsIerr(ptrPIDdata pPd, float Ierr_min, float Ierr_max);
extern void PID_ResetIerr(ptrPIDdata pPd);
extern float PID_Update(ptrPIDdata pPd, float setpoint, float input);

#endif
