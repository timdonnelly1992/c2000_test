
//#############################################################################
//
// FILE:    CRD25DA12N-FMC_shared.h (Wolfspeed CRD25DA12N-FMC Shared Header)
//
// TITLE:   CRD25DA12N-FMC 25 kW Controller Code
//
// AUTHOR:  WOLFSPEED
//
// PURPOSE:
//  Header of functions/variables that are shared between the primary CPU in
//  the CRD25DA12N-FMC reference design and its Control Law Accelerator (CLA).
//
//#############################################################################
//
// Header Guards
//
#ifndef WSSHARED
#define WSSHARED

//
// Include Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "libraries/CLAmath/CLAmath.h"
#include "wsinclude/wsdefine.h"

//
// Shared Variables
//
extern float modulationFactor;
extern float angleStep;
extern float epwmAngles[NUMPHASES];
extern uint16_t epwmTimebasePeriod;
extern float electricalAngle;
extern float phaseCurrentU;
extern float phaseCurrentV;
extern float phaseCurrentW;
extern float phaseVoltageU;
extern float phaseVoltageV;
extern float phaseVoltageW;
extern float dcBusVoltageMeas;
extern float idRefCurrent;
extern float iqRefCurrent;
extern float currentKp;
extern float currentKi;
extern uint16_t currentControlEnable;
extern uint16_t controlMode;
extern uint16_t angleSourceSelect;
extern float controlPeriodSec;
extern float pllKp;
extern float pllKi;
extern float pllOmegaNom;
extern float idCurrentMeas;
extern float iqCurrentMeas;
extern float idCurrentErr;
extern float iqCurrentErr;
extern float vdCommand;
extern float vqCommand;
extern float idIntegrator;
extern float iqIntegrator;
extern float vdVoltageMeas;
extern float vqVoltageMeas;
extern float vdVoltageRef;
extern float vqVoltageRef;
extern float vdVoltageErr;
extern float vqVoltageErr;
extern float voltageKp;
extern float voltageKi;
extern float vdVoltageIntegrator;
extern float vqVoltageIntegrator;
extern float idRefFromVoltageLoop;
extern float iqRefFromVoltageLoop;
extern float vdcVoltageRef;
extern float vdcVoltageErr;
extern float dcVoltageKp;
extern float dcVoltageKi;
extern float vdcVoltageIntegrator;
extern float idRefFromDcVoltageLoop;
extern float pllTheta;
extern float pllOmega;
extern float pllVq;
extern float pllIntegrator;
extern float pllFreqErrHz;
extern uint16_t pllLockCounter;
extern uint16_t pllLocked;
extern uint16_t pllError;

//
// Function Prototypes
//
__interrupt void Cla1Task1();

#endif
//
// End of file
//
