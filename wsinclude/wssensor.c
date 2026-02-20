
//#############################################################################
//
// FILE:    wssensor.c (Wolfspeed Sensors - WS Sensor)
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
// Included Files
//
#include "wssensor.h"

//
// Initialize Variables
//
uint16_t overcurrentTripCurrent     = DEFAULTOVERCURRENT;

//*****************************************************************************
//
//! Convert ADC value to current value (A).
//!
//! \param adcVal is the measured ADC value (12-bit, 0-4095).
//!
//! \return Current (A) value after applying board-specific gain and offset.
//
//*****************************************************************************
float convertAdcToCurrent(uint16_t adcVal) {
    return 0.0302808303*adcVal - 62.0;
}

//*****************************************************************************
//
//! Convert ADC value to voltage value (A). Apply gain/offset associate with
//! the DC voltage feedback circuit.
//!
//! \param adcVal is the measured ADC value (12-bit, 0-4095).
//!
//! \return Current (A) value after applying board-specific gain and offset.
//
//*****************************************************************************
float convertAdcToVoltageDc(uint16_t adcVal) {
    return 0.32234432234*adcVal;
}

//*****************************************************************************
//
//! Convert ADC value to voltage value (A). Apply gain/offset associate with
//! the AC voltage feedback circuit.
//!
//! \param adcVal is the measured ADC value (12-bit, 0-4095).
//!
//! \return Current (A) value after applying board-specific gain and offset.
//
//*****************************************************************************
float convertAdcToVoltageAc(uint16_t adcVal) {
    return 0.3741147741*adcVal - 766.0;
}

//*****************************************************************************
//
//! Get Phase U current ADC register and convert to corresponding current
//! value (A). Use post-processing block (PPB) to remove and hall-effect
//! current sensor offset in hardware.
//!
//! \return Current (A) value after applying board-specific gain and offset.
//
//*****************************************************************************
float getCurrentU() {
    return convertAdcToCurrent(ADC_readPPBResult(myADC2_RESULT_BASE, IU_PPB));
}

//*****************************************************************************
//
//! Get Phase V current ADC register and convert to corresponding current
//! value (A). Use post-processing block (PPB) to remove and hall-effect
//! current sensor offset in hardware.
//!
//! \return Current (A) value after applying board-specific gain and offset.
//
//*****************************************************************************
float getCurrentV() {
    return convertAdcToCurrent(ADC_readPPBResult(myADC2_RESULT_BASE, IV_PPB));
}

//*****************************************************************************
//
//! Get Phase W current ADC register and convert to corresponding current
//! value (A). Use post-processing block (PPB) to remove and hall-effect
//! current sensor offset in hardware.
//!
//! \return Current (A) value after applying board-specific gain and offset.
//
//*****************************************************************************
float getCurrentW() {
    return convertAdcToCurrent(ADC_readPPBResult(myADC2_RESULT_BASE, IW_PPB));
}

//*****************************************************************************
//
//! Get Phase U voltage ADC register and convert to corresponding voltage
//! value (V).
//!
//! \return Voltage (V) value after applying board-specific gain and offset.
//
//*****************************************************************************
float getVoltageU() {
    return convertAdcToVoltageAc(ADC_readResult(myADC1_RESULT_BASE, myADC1_VU_MEAS));
}

//*****************************************************************************
//
//! Get Phase V voltage ADC register and convert to corresponding voltage
//! value (V).
//!
//! \return Voltage (V) value after applying board-specific gain and offset.
//
//*****************************************************************************
float getVoltageV() {
    return convertAdcToVoltageAc(ADC_readResult(myADC1_RESULT_BASE, myADC1_VV_MEAS));
}

//*****************************************************************************
//
//! Get Phase W voltage ADC register and convert to corresponding voltage
//! value (V).
//!
//! \return Voltage (V) value after applying board-specific gain and offset.
//
//*****************************************************************************
float getVoltageW() {
    return convertAdcToVoltageAc(ADC_readResult(myADC1_RESULT_BASE, myADC1_VW_MEAS));
}

//*****************************************************************************
//
//! Get DC voltage ADC register and convert to corresponding voltage value (V).
//!
//! \return Voltage (V) value after applying board-specific gain and offset.
//
//*****************************************************************************
float getVoltageDc() {
    return convertAdcToVoltageDc(ADC_readResult(myADC1_RESULT_BASE, myADC1_VDC_MEAS));
}

//*****************************************************************************
//
//! Start ADC conversion for all current measurements (Phase U, V, W currents).
//!
//! \return None
//
//*****************************************************************************
void forceCurrentSocs() {
    forceCurrentSocU();
    forceCurrentSocV();
    forceCurrentSocW();
}

//*****************************************************************************
//
//! Start ADC conversion for phase U current. This function does not wait for
//! conversions to be completed. Recommended procedure to use this: Ensure
//! last SOC sets an ADC interrupt flag in SysConfig. Wait for status flag
//! after this function call.
//!
//! \return None
//
//*****************************************************************************
void forceCurrentSocU() {
    ADC_forceSOC(myADC2_BASE, myADC2_IU_MEAS);    // Phase U Current ADC
}

//*****************************************************************************
//
//! Start ADC conversion for phase V current. This function does not wait for
//! conversions to be completed. Recommended procedure to use this: Ensure
//! last SOC sets an ADC interrupt flag in SysConfig. Wait for status flag
//! after this function call.
//!
//! \return None
//
//*****************************************************************************
void forceCurrentSocV() {
    ADC_forceSOC(myADC2_BASE, myADC2_IV_MEAS);    // Phase V Current ADC
}

//*****************************************************************************
//
//! Start ADC conversion for phase W current. This function does not wait for
//! conversions to be completed. Recommended procedure to use this: Ensure
//! last SOC sets an ADC interrupt flag in SysConfig. Wait for status flag
//! after this function call.
//!
//! \return None
//
//*****************************************************************************
void forceCurrentSocW() {
    ADC_forceSOC(myADC2_BASE, myADC2_IW_MEAS);    // Phase W Current ADC
}

//*****************************************************************************
//
//! Start ADC conversion for one current sensor. Select sensor using sensorNum
//! input value.
//!
//! \param sensorNum selects which current sensor SOC to begin (0 = U; 1 = V;
//!     2 = W)
//!
//! \return None
//
//*****************************************************************************
void forceCurrentSocByNumber(uint16_t sensorNum) {
    if (sensorNum == SENSORU) {
        forceCurrentSocU();
    } else if (sensorNum == SENSORV) {
        forceCurrentSocV();
    } else if (sensorNum == SENSORW) {
        forceCurrentSocW();
    }
}

//*****************************************************************************
//
//! Start ADC conversion for all voltage measurements (Phase U, V, W and DC
//! voltages). This function does not wait for conversions to be completed.
//! Recommended procedure to use this: Ensure last SOC sets an ADC interrupt
//! flag in SysConfig. Wait for status flag after this function call.
//!
//! \return None
//
//*****************************************************************************
void forceVoltageSocs() {
    forceVoltageSocDC();
    forceVoltageSocU();
    forceVoltageSocV();
    forceVoltageSocW();
}

//*****************************************************************************
//
//! Start ADC conversion for phase U voltage. This function does not wait for
//! conversions to be completed. Recommended procedure to use this: Ensure
//! last SOC sets an ADC interrupt flag in SysConfig. Wait for status flag
//! after this function call.
//!
//! \return None
//
//*****************************************************************************
void forceVoltageSocU() {
    ADC_forceSOC(myADC1_BASE, myADC1_VU_MEAS);    // Phase U Voltage ADC
}

//*****************************************************************************
//
//! Start ADC conversion for phase V voltage. This function does not wait for
//! conversions to be completed. Recommended procedure to use this: Ensure
//! last SOC sets an ADC interrupt flag in SysConfig. Wait for status flag
//! after this function call.
//!
//! \return None
//
//*****************************************************************************
void forceVoltageSocV() {
    ADC_forceSOC(myADC1_BASE, myADC1_VV_MEAS);    // Phase V Voltage ADC
}

//*****************************************************************************
//
//! Start ADC conversion for phase W voltage. This function does not wait for
//! conversions to be completed. Recommended procedure to use this: Ensure
//! last SOC sets an ADC interrupt flag in SysConfig. Wait for status flag
//! after this function call.
//!
//! \return None
//
//*****************************************************************************
void forceVoltageSocW() {
    ADC_forceSOC(myADC1_BASE, myADC1_VW_MEAS);    // Phase W Voltage ADC
}

//*****************************************************************************
//
//! Start ADC conversion for DC voltage. This function does not wait for
//! conversions to be completed. Recommended procedure to use this: Ensure
//! last SOC sets an ADC interrupt flag in SysConfig. Wait for status flag
//! after this function call.
//!
//! \return None
//
//*****************************************************************************
void forceVoltageSocDC() {
    ADC_forceSOC(myADC1_BASE, myADC1_VDC_MEAS);   // DC Voltage ADC
}

//*****************************************************************************
//
//! Start ADC conversion for one voltage sensor. Select sensor using sensorNum
//! input value.
//!
//! \param sensorNum selects which current sensor SOC to begin (0 = U; 1 = V;
//!     2 = W; 3 = DC)
//!
//! \return None
//
//*****************************************************************************
void forceVoltageSocByNumber(uint16_t sensorNum) {
    if (sensorNum == SENSORU) {
        forceVoltageSocU();
    } else if (sensorNum == SENSORV) {
        forceVoltageSocV();
    } else if (sensorNum == SENSORW) {
        forceVoltageSocW();
    } else if (sensorNum == SENSORDC) {
        forceVoltageSocDC();
    }
}

//*****************************************************************************
//
//! Start ADC conversion for module substrate NTC measurement. This function
//! does not wait for conversions to be completed. Recommended procedure to
//! use this: Ensure last SOC sets an ADC interrupt flag in SysConfig. Wait
//! for status flag after this function call.
//!
//! \return None
//
//*****************************************************************************
void forceNtcModuleSoc() {
    ADC_forceSOC(myADC0_BASE, myADC0_NTC);
}

//*****************************************************************************
//
//! Start ADC conversion for all current and voltage measurements (Phase U, V,
//! W voltages and currents. DC voltage). This function does not wait for
//! conversions to be completed.
//!
//! \return None
//
//*****************************************************************************
void forceCurrentVoltageSocs() {
    forceCurrentSocs();
    forceVoltageSocs();
}

//*****************************************************************************
//
//! Start ADC conversion for all sensors. This function does not wait for
//! conversions to be completed.
//!
//! \return None
//
//*****************************************************************************
void forceAllSocs() {
    forceCurrentVoltageSocs();
    forceNtcModuleSoc();
    forceSpareAdcSocs();
}

//*****************************************************************************
//
//! Start ADC conversion for all current and voltage measurements (Phase U, V,
//! W voltages and currents. DC voltage). Wait for the appropriate ADC
//! conversion complete flags. Do not use the forceCurrentSocsWait and
//! forceVoltageSocsWait so that both ADCs can begin simultaneously.
//!
//! \return None
//
//*****************************************************************************
void forceCurrentVoltageSocsWait() {
    forceCurrentSocs();
    forceVoltageSocs();
    while(ADC_getInterruptStatus(myADC2_BASE, ADC_INT_NUMBER1) == false) {}
        ADC_clearInterruptStatus(myADC2_BASE, ADC_INT_NUMBER1);
    while(ADC_getInterruptStatus(myADC1_BASE, ADC_INT_NUMBER1) == false) {}
            ADC_clearInterruptStatus(myADC1_BASE, ADC_INT_NUMBER1);
}

//*****************************************************************************
//
//! Start ADC conversion for all current measurements (Phase U, V, W currents).
//! Wait for the appropriate ADC conversion complete flag.
//!
//! \return None
//
//*****************************************************************************
void forceCurrentSocsWait() {
    forceCurrentSocs();
    while(ADC_getInterruptStatus(myADC2_BASE, ADC_INT_NUMBER1) == false) {}
        ADC_clearInterruptStatus(myADC2_BASE, ADC_INT_NUMBER1);
}

//*****************************************************************************
//
//! Start ADC conversion for all voltage measurements (Phase U, V, W and DC
//! voltages). Wait for the appropriate ADC conversion complete flag.
//!
//! \return None
//
//*****************************************************************************
void forceVoltageSocsWait() {
    forceVoltageSocs();
    while(ADC_getInterruptStatus(myADC1_BASE, ADC_INT_NUMBER1) == false) {}
        ADC_clearInterruptStatus(myADC1_BASE, ADC_INT_NUMBER1);
}

//*****************************************************************************
//
//! Start ADC conversion for module substrate NTC measurement. Wait for the
//! appropriate ADC conversion complete flag.
//!
//! \return None
//
//*****************************************************************************
void forceNtcModuleSocWait() {
    forceNtcModuleSoc();
    while(ADC_getInterruptStatus(myADC0_BASE, ADC_INT_NUMBER2) == false) {}
        ADC_clearInterruptStatus(myADC0_BASE, ADC_INT_NUMBER2);
}

//*****************************************************************************
//
//! Get the stored overcurrent value which will fault the system.
//!
//! \return Overcurrent trip value in [A].
//
//*****************************************************************************
uint16_t getOvercurrentTripValue() {
    return overcurrentTripCurrent;
}

//*****************************************************************************
//
//! Set the stored overcurrent value which will fault the system.
//!
//! \param setVal is the new overcurrent value in [A] which will fault the
//!     system.
//!
//! \return None.
//
//*****************************************************************************
void setOvercurrentTripValue(uint16_t setVal) {
    overcurrentTripCurrent = setVal;
}

//*****************************************************************************
//
//! Clear any ADC post-processing block (PPB) offset correction values of the
//! current sensors.
//!
//! \return None
//
//*****************************************************************************
void clearCurrentSensorOffsetCorrection() {
    ADC_setPPBCalibrationOffset(myADC2_BASE, IU_PPB, 0);
    ADC_setPPBCalibrationOffset(myADC2_BASE, IV_PPB, 0);
    ADC_setPPBCalibrationOffset(myADC2_BASE, IW_PPB, 0);
}

//
//! Clear any ADC post-processing block (PPB) offset correction values of the
//! AC voltage sensors. DC voltage is not referenced to a mid-point voltage,
//! so offset correction is less critical.
//!
//! \return None
//
//*****************************************************************************
void clearVoltageSensorOffsetCorrection() {
    ADC_setPPBCalibrationOffset(myADC1_BASE, VU_PPB, 0);
    ADC_setPPBCalibrationOffset(myADC1_BASE, VV_PPB, 0);
    ADC_setPPBCalibrationOffset(myADC1_BASE, VW_PPB, 0);
}

//*****************************************************************************
//
//! Clear any ADC post-processing block (PPB) offset correction values of the
//! current and voltage sensors.
//!
//! \return None
//
//*****************************************************************************
void clearSensorOffsetCorrection() {
    clearCurrentSensorOffsetCorrection();
    clearVoltageSensorOffsetCorrection();
}

//*****************************************************************************
//
//! Measure zero-point of ADCs. With ADCs not energized (no phase current/
//! voltage, measure ADCs to determine fundamental offset. Average the offset
//! over multiple measurements. Store offset in ADC post-processing block (PBB)
//! to be removed in all future ADC measurements.
//!
//! \return None
//
//*****************************************************************************
void setSensorOffsetCorrection() {
    //
    // Initialize Variables
    //
    int16_t sensorSums[7] = {0,0,0,0,0,0,0};
    uint16_t loopCal;

    //
    // Ensure Previous PPB Calibrations will Not Interfere with Calibration
    //
    clearSensorOffsetCorrection();

    //
    // Read ADCs multiple Times to Determine Average Offset
    //
    for (loopCal = 0; loopCal < ZEROCALSIZE; loopCal++) {
        //
        // Force ADC SOCs
        //
        forceCurrentVoltageSocsWait();

        //
        // Phase Current & Voltage ADCs Should be Referenced to ADC range
        // midpoint. DC Voltage ADC Referenced to 0.
        //
        sensorSums[0] += (ADC_readResult(myADC2_RESULT_BASE, myADC2_IU_MEAS) - ADCMIDVALUE);    // Phase U Current
        sensorSums[1] += (ADC_readResult(myADC2_RESULT_BASE, myADC2_IV_MEAS) - ADCMIDVALUE);    // Phase V Current
        sensorSums[2] += (ADC_readResult(myADC2_RESULT_BASE, myADC2_IW_MEAS) - ADCMIDVALUE);    // Phase W Current
        sensorSums[3] += (ADC_readResult(myADC1_RESULT_BASE, myADC1_VU_MEAS) - ADCMIDVALUE);    // Phase U Voltage
        sensorSums[4] += (ADC_readResult(myADC1_RESULT_BASE, myADC1_VV_MEAS) - ADCMIDVALUE);    // Phase V Voltage
        sensorSums[5] += (ADC_readResult(myADC1_RESULT_BASE, myADC1_VW_MEAS) - ADCMIDVALUE);    // Phase W Voltage
        sensorSums[6] += (ADC_readResult(myADC1_RESULT_BASE, myADC1_VDC_MEAS));                 // Phase DC Voltage

        //
        // Create Delay Before Repeating ADC Measurements
        //
        DEVICE_DELAY_US(1000);
    }

    //
    // Apply Corrections in Post Processing block. DC voltage is reference to
    // 0, so offset correction is less critical.
    //
    ADC_setPPBCalibrationOffset(myADC2_BASE, IU_PPB, (int16_t)(sensorSums[0]/((int16_t)(ZEROCALSIZE))));
    ADC_setPPBCalibrationOffset(myADC2_BASE, IV_PPB, (int16_t)(sensorSums[1]/((int16_t)(ZEROCALSIZE))));
    ADC_setPPBCalibrationOffset(myADC2_BASE, IW_PPB, (int16_t)(sensorSums[2]/((int16_t)(ZEROCALSIZE))));
    ADC_setPPBCalibrationOffset(myADC1_BASE, VU_PPB, (int16_t)(sensorSums[3]/((int16_t)(ZEROCALSIZE))));
    ADC_setPPBCalibrationOffset(myADC1_BASE, VV_PPB, (int16_t)(sensorSums[4]/((int16_t)(ZEROCALSIZE))));
    ADC_setPPBCalibrationOffset(myADC1_BASE, VW_PPB, (int16_t)(sensorSums[5]/((int16_t)(ZEROCALSIZE))));
}

//*****************************************************************************
//
//! Convert current value (A) to ADC value
//!
//! \param Current value (A)
//!
//! \return ADC value (0-4095) after applying board-specific gain and offset
//
//*****************************************************************************
float convertCurrentToAdc(uint16_t currentVal) {
    return 33.02419355*currentVal + 2047.5;
}

//*****************************************************************************
//
//! Set overcurrent trip value of post-processing block (PPB) of current-
//! measuring ADCs.
//!
//! \param overcurrentVal is the trip current [A] where the PPB will trip an
//! interrupt if reached. This is a peak value (not RMS).
//!
//! \return None.
//
//*****************************************************************************
void setOvercurrentProtection(uint16_t overcurrentVal) {
    //
    // Convert Provided Positive Trip Current Limit into Usable ADC Value
    //
    uint16_t adcCurrentHi = convertCurrentToAdc(overcurrentVal);

    //
    // Prevent Non-Realizable Trip Limits
    //
    if (adcCurrentHi > ADCMAXVALUE)
        adcCurrentHi = ADCMAXVALUE;

    //
    // Calculate Negative Current Trip Limit ADC Value
    //
    uint16_t adcCurrentLo = ADCMAXVALUE - adcCurrentHi;

    //
    // Set Trip Limits in Post-Processing Block
    //
    ADC_setPPBTripLimits(myADC2_BASE, IU_PPB, adcCurrentHi, adcCurrentLo);
    ADC_setPPBTripLimits(myADC2_BASE, IV_PPB, adcCurrentHi, adcCurrentLo);
    ADC_setPPBTripLimits(myADC2_BASE, IW_PPB, adcCurrentHi, adcCurrentLo);

    //
    // Store Setpoint to be Access by Other Functions (i.e. CAN Message)
    //
    setOvercurrentTripValue(overcurrentVal);
}

//*****************************************************************************
//
//! Convert ADC value to module substrate temperature (C).
//!
//! \param adcVal is the measured ADC value (12-bit, 0-4095).
//!
//! \return Temperature (C) value after applying board-specific gain and offset.
//
//*****************************************************************************
float convertAdcToModuleTemp(uint16_t adcVal) {
    return -0.000000011377*adcVal*adcVal*adcVal + 0.000053839643*adcVal*adcVal - 0.126005551097*adcVal + 178.243484658716;
}

//*****************************************************************************
//
//! Get module substrate temperature sensor ADC register and convert to the
//! corresponding temperature value (C).
//!
//! \return Temperature (C) value after applying board-specific gain and offset.
//
//*****************************************************************************
uint16_t getModuleTempC() {
    return convertAdcToModuleTemp(ADC_readResult(myADC0_RESULT_BASE, myADC0_NTC));
}

//*****************************************************************************
//
//! Get module substrate temperature sensor ADC register and convert to the
//! corresponding temperature value (K).
//!
//! \return Temperature (K) value after converting C-to-K.
//
//*****************************************************************************
uint16_t getModuleTempK() {
    return getModuleTempC() + 273.15;
}

//*****************************************************************************
//
//! Start ADC conversion for the spare analog-to-digital conversion inputs.
//! This function does not wait for conversions to be completed. Recommended
//! procedure to use this: Ensure last SOC sets an ADC interrupt flag in
//! SysConfig. Wait for status flag after this function call.
//!
//! \return None
//
//*****************************************************************************
void forceSpareAdcSocs() {
    forceSpareAdc1Soc();
    forceSpareAdc2Soc();
}

//*****************************************************************************
//
//! Start ADC conversion for the first spare analog-to-digital conversion
//! input. This function does not wait for conversions to be completed.
//! Recommended procedure to use this: Ensure last SOC sets an ADC interrupt
//! flag in SysConfig. Wait for status flag after this function call.
//!
//! \return None
//
//*****************************************************************************
void forceSpareAdc1Soc() {
    ADC_forceSOC(myADC0_BASE, myADC0_SPARE_ADC1);
}

//*****************************************************************************
//
//! Start ADC conversion for the second spare analog-to-digital conversion
//! input. This function does not wait for conversions to be completed.
//! Recommended procedure to use this: Ensure last SOC sets an ADC interrupt
//! flag in SysConfig. Wait for status flag after this function call.
//!
//! \return None
//
//*****************************************************************************
void forceSpareAdc2Soc() {
    ADC_forceSOC(myADC0_BASE, myADC0_SPARE_ADC2);
}

//*****************************************************************************
//
//! Start ADC conversion for the spare analog-to-digital conversion inputs.
//! Wait for the appropriate ADC conversion complete flag.
//!
//! \return None
//
//*****************************************************************************
void forceSpareAdcSocsWait() {
    forceSpareAdcSocs();
    while(ADC_getInterruptStatus(myADC0_BASE, ADC_INT_NUMBER3) == false) {}
        ADC_clearInterruptStatus(myADC0_BASE, ADC_INT_NUMBER3);
}

//*****************************************************************************
//
//! Get the measured bit value (0-4095) of Spare ADC1 (A0).
//!
//! \return measured bit value (0-4095) of Spare ADC1 (A0).
//
//*****************************************************************************
uint16_t getSpareAdc1(void) {
    return ADC_readResult(myADC0_RESULT_BASE, myADC0_SPARE_ADC1);
}

//*****************************************************************************
//
//! Get the measured bit value (0-4095) of Spare ADC2 (A1).
//!
//! \return measured bit value (0-4095) of Spare ADC2 (A1).
//
//*****************************************************************************
uint16_t getSpareAdc2(void) {
    return ADC_readResult(myADC0_RESULT_BASE, myADC0_SPARE_ADC2);
}

//*****************************************************************************
//
//! Get resolver measurement. Currently not implemented.
//!
//! \return 0 until resolver feedback is implemented.
//
//*****************************************************************************
float getResolver() {
    return 0.0;
}

//*****************************************************************************
//
//! Get current measurements, store values in buffer array, and cumulatively
//! add the values squared to running sum-of-squares.
//!
//! \return None.
//
//*****************************************************************************
void storeCurrentSensors() {
    //
    // Get Current Sensor Measurements
    //
    float Iu    = getCurrentU();
    float Iv    = getCurrentV();
    float Iw    = getCurrentW();

    //
    // Store Measurements in Buffer
    //
    currentUBuffer[bufferIndex] = Iu;
    currentVBuffer[bufferIndex] = Iv;
    currentWBuffer[bufferIndex] = Iw;

    //
    // Cumulatively Add Sensor Value Squared
    // (Used for RMS calculations)
    //
    if (rmsIndex <= rmsAvgBins) {
        sensor2Sums[INDEXCURRENTU]  += Iu*Iu;
        sensor2Sums[INDEXCURRENTV]  += Iv*Iv;
        sensor2Sums[INDEXCURRENTW]  += Iw*Iw;
        rmsIndex++;
    }

    //
    // Increment Buffer Index; Reset If Exceeding Max Value
    //
    bufferIndex++;
    if (bufferIndex >= BUFFERSIZE)
        bufferIndex = 0;
}

//*****************************************************************************
//
//! Get single current measurement, store values in buffer array, and
//! cumulatively add the value squared to running sum-of-squares. This function
//! is used to reduce ADC compute resources by alternating which current sensor
//! is measured (instead of measuring all the current sensors every time).
//!
//! \param sensorNum selects which current sensor to store in the buffer begin
//!     (0 = U; 1 = V; 2 = W; 3 = DC)
//!
//! \return None.
//
//*****************************************************************************
void storeCurrentSensorsAlternate(uint16_t sensorNum) {
    //
    // Initialize Variables
    //
    float Isensor; uint16_t indexSensor;
    //
    // Get Relevant Sensor Value and Store to Buffer
    //
    if (sensorNum == SENSORU) {             // Phase U Current
        Isensor                     = getCurrentU();
        currentUBuffer[bufferIndex] = Isensor;
        indexSensor                 = INDEXCURRENTU;
    } else if (sensorNum == SENSORV) {      // Phase V Current
        Isensor                     = getCurrentV();
        currentVBuffer[bufferIndex] = Isensor;
        indexSensor                 = INDEXCURRENTV;
    } else if (sensorNum == SENSORW) {      // Phase W Current
        Isensor                     = getCurrentW();
        currentWBuffer[bufferIndex] = Isensor;
        indexSensor                 = INDEXCURRENTW;
    }

    //
    // Cumulatively Add the Squared Value to Sum
    //
    if (rmsIndex < rmsAvgBins) {
        sensor2Sums[indexSensor] += Isensor * Isensor;
    }

    //
    // Increment Index Counter Only After 1 Phase (Arbitrarily Selected W)
    // (incrementing on every phase will result in buffer skips)
    //
    if (sensorNum == SENSORW) {
        bufferIndex++;
        if (rmsIndex < rmsAvgBins) {
            rmsIndex++;
        }
    }

    //
    // Reset Buffer Index If Exceeding Max Value
    //
    if (bufferIndex >= BUFFERSIZE)
        bufferIndex = 0;
}

//*****************************************************************************
//
//! Calculates the root-mean-squared value of sensor measurements. The array
//! sensor2Sums should contain the cumulative sum of squared measured sensor
//! values. This function calculates the root of the cumulative sum and divides
//! by the number of elements (mean). The calculated RMS values are stored in
//! sensorRms. Cumulative sum array, sensor2Sums, is cleared.
//!
//! \return None
//
//*****************************************************************************
void calculateSensorRms() {
    uint16_t i;
    for (i = 0; i < NUMAVGSENSORS; i++)
        sensorRms[i] = (float)sqrtf((float)sensor2Sums[i] / (float)rmsAvgBins);
    rmsIndex = 0;
    clearArrayFloat(NUMAVGSENSORS,sensor2Sums);
}

//*****************************************************************************
//
//! Set all values in a float array to 0.
//!
//! \return None
//
//*****************************************************************************
void clearArrayFloat(uint16_t arrSize, float *arr) {
    uint16_t i;
    for (i = 0; i < arrSize; i++)
        arr[i] = 0;
}

//
// End of file
//
