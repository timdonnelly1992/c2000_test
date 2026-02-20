
//#############################################################################
//
// FILE:    wssensor.h (Wolfspeed Sensors - WS Sensor)
//
// TITLE:   CRD25DA12N-FMC 25 kW Controller Code
//
// AUTHOR:  WOLFSPEED
//
// PURPOSE:
//  Read/control sensors attached to analog-to-digital converter (ADC) pin
//  functions. Supports reading/controlling various sensors such as
//  thermistors, current sensors, and voltage sensors.
//
//#############################################################################

//
// Header Guards
//
#ifndef WSSENSOR
#define WSSENSOR

//
// Include Files
//
#include "driverlib.h"
#include "board.h"
#include "math.h"
#include "wsinclude/wsdefine.h"

//
// Globals
//
extern float sensor2Sums[NUMAVGSENSORS];
extern uint16_t rmsIndex;
extern uint16_t rmsAvgBins;
extern uint16_t bufferIndex;
extern float currentUBuffer[BUFFERSIZE];
extern float currentVBuffer[BUFFERSIZE];
extern float currentWBuffer[BUFFERSIZE];
extern float sensorRms[NUMAVGSENSORS];

//
// Conversions
//
float convertAdcToCurrent(uint16_t adcVal);
float convertAdcToVoltageDc(uint16_t adcVal);
float convertAdcToVoltageAc(uint16_t adcVal);
float convertCurrentToAdc(uint16_t currentVal);
float convertAdcToModuleTemp(uint16_t currentVal);

//
// Currents
//
float getCurrentU(void);
float getCurrentV(void);
float getCurrentW(void);

//
// Voltages
//
float getVoltageU(void);
float getVoltageV(void);
float getVoltageW(void);
float getVoltageDc(void);

//
// Temperatures
//
uint16_t getModuleTempC(void);
uint16_t getModuleTempK(void);

//
// Spare ADCs
//
uint16_t getSpareAdc1(void);
uint16_t getSpareAdc2(void);

//
// Resolver
//
float getResolver(void);

//
// Force Current ADCs
//
void forceCurrentSocU(void);
void forceCurrentSocV(void);
void forceCurrentSocW(void);
void forceCurrentSocs(void);
void forceCurrentSocsWait(void);
void forceCurrentSocByNumber(uint16_t sensorNum);

//
// Force Voltage ADCs
//
void forceVoltageSocU(void);
void forceVoltageSocV(void);
void forceVoltageSocW(void);
void forceVoltageSocDC(void);
void forceVoltageSocs(void);
void forceVoltageSocsWait(void);
void forceVoltageSocByNumber(uint16_t sensorNum);

//
// Force Temperature ADCs
//
void forceNtcModuleSoc(void);
void forceNtcModuleSocWait(void);

//
// Force Spare ADCs
//
void forceSpareAdc1Soc(void);
void forceSpareAdc2Soc(void);
void forceSpareAdcSocs(void);
void forceSpareAdcSocsWait(void);

//
// Force Grouped ADCs
//
void forceCurrentVoltageSocs(void);
void forceCurrentVoltageSocsWait(void);
void forceAllSocs(void);

//
// Configure Offset Correction and Overcurrent Faults
//
uint16_t getOvercurrentTripValue(void);
void setOvercurrentTripValue(uint16_t setVal);
void clearCurrentSensorOffsetCorrection(void);
void clearVoltageSensorOffsetCorrection(void);
void clearSensorOffsetCorrection(void);
void setSensorOffsetCorrection(void);
void setOvercurrentProtection(uint16_t overcurrentVal);

//
// Store Sensor Values
//
void storeCurrentSensors(void);
void storeCurrentSensorsAlternate(uint16_t sensorNum);

//
// Math Functions
//
void clearArrayFloat(uint16_t arrSize, float *arr);
void calculateSensorRms();

#endif
//
// End of file
//
