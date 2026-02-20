
//#############################################################################
//
// FILE:    wscan.h (Wolfspeed Controller Area Network - WS CAN)
//
// TITLE:   CRD25DA12N-FMC 25 kW Controller Code
//
// AUTHOR:  WOLFSPEED
//
// PURPOSE:
//  Controller Area Network (CAN) interface functions. Supports communicating
//  with the Wolfspeed CRD25DA12N-FMC reference design to remotely control
//  parameters such as switching frequency and dead time.
//
//#############################################################################

//
// Header Guards
//
#ifndef WSCAN
#define WSCAN

//
// Included Files
//
#include "wsgpio.h"
#include "wssensor.h"
#include "wsdefine.h"

//
// Initialize Variables. All Variables Stored Elsewhere, Just Need Access.
//
extern float switchingFreq;
extern float modulationFactor;
extern uint16_t deadTime;
extern uint16_t fundamentalFreq;
extern float sensorRms[NUMAVGSENSORS];

//
// Function Prototypes
//
void sendCanControls(void);
uint16_t validateIndividualOperatingParameter(uint16_t val, uint16_t valMin, uint16_t valMax);
void receiveCanControls(uint16_t *packetData);
void receiveCanAdditional(uint16_t *packetData);
void sendCanTemperature(float ntcTemp);
void sendCanCurrent(float *currentRms);
void sendCanVoltage(float *voltageRms, float voltageDc);
void sendCanSensors();

#endif
//
// End of file
//
