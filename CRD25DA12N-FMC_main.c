
//#############################################################################
//
// FILE:    CRD25DA12N-FMC_main.c
//
// TITLE:   CRD25DA12N-FMC 25 kW Controller Code
//
// AUTHOR:  WOLFSPEED
//
// PURPOSE:
//  This software is designed for the evaluation of the Wolfspeed
//  CRD25DA12N-FMC Three-Phase Inverter Reference Design using the Wolfspeed
//  CCB021M12FM3T six-pack SiC power module. It supports open-loop PWM and
//  multiple closed-loop operating modes using CPU + CLA cooperation:
//      (1) open-loop three-phase sine PWM,
//      (2) dq current control,
//      (3) dq AC-voltage + current cascade control,
//      (4) active-rectifier DC-bus voltage + current cascade control.
//  Angle reference can be selected between open-loop phase advance and PLL.
//
// REVISION HISTORY:
// V1.0.0   Initial Release
// V2.0.0   Updated to support V2.0 hardware. Added CLA & more CAN feedback/
//          control.
//
//#############################################################################

//
// Included Files
//
#include <CRD25DA12N-FMC.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SCI_RX_DEBUG 0u
#define SCI_RX_RING_SIZE 128u
#define SCI_MENU_BUILD_TAG "SCI_MENU_DEBUG_2026_02_15_A"
#define SCI_MENU_ONLY_MODE 0u
#define SCI_PERIODIC_STATUS_ENABLE 0u
#define SCI_CONSOLE_BAUD 115200u

#ifndef mySCIA_BASE
#define mySCIA_BASE SCIA_BASE
#endif

//
// Local function prototypes
//
static void initSciStatusPort(void);
static void sciWriteString(const char *msg);
static long scaleToMilli(float value);
static long scaleToDeci(float value);
static void sendSciStatusReport(void);
static void sendSciSnapshot(void);
static void sciPrintMenu(void);
static void handleSciLine(const char *line);
static void pollSciConsole(void);
static uint16_t sciConsoleInputPending(void);
static void sciDebugRxByte(uint16_t ch);
static void sciDebugRxStatus(uint16_t status);

//
// Main
//
void main(void) {
    uint16_t statusReportTicks = 0u;
    uint16_t canTxTicks = 0u;

    //
    // Initializes system control, device clock, and peripherals
    //
    Device_init();
    Interrupt_initModule();
    Interrupt_initVectorTable();
    Board_init();
    initSciStatusPort();
    sciWriteString("\r\n[BOOT] SCI init complete\r\n");

    //
    //  Initialize parameters, Disable Sync, Configure EPWMs
    //
    initOperatingParameters();
    electricalAngle          = epwmAngles[0];
    phaseCurrentU            = 0.0f;
    phaseCurrentV            = 0.0f;
    phaseCurrentW            = 0.0f;
    phaseVoltageU            = 0.0f;
    phaseVoltageV            = 0.0f;
    phaseVoltageW            = 0.0f;
    dcBusVoltageMeas         = 0.0f;
    idRefCurrent             = 0.0f;
    iqRefCurrent             = 0.0f;
    currentKp                = 0.05f;
    currentKi                = 0.0f;
    currentControlEnable     = 0u;
    controlMode              = 0u;
    angleSourceSelect        = 0u;
    controlPeriodSec         = 1.0f / switchingFreq;
    pllKp                    = 0.5f;
    pllKi                    = 25.0f;
    pllOmegaNom              = PI2 * 60.0f;
    idCurrentMeas            = 0.0f;
    iqCurrentMeas            = 0.0f;
    idCurrentErr             = 0.0f;
    iqCurrentErr             = 0.0f;
    vdCommand                = 0.0f;
    vqCommand                = 0.0f;
    idIntegrator             = 0.0f;
    iqIntegrator             = 0.0f;
    vdVoltageMeas            = 0.0f;
    vqVoltageMeas            = 0.0f;
    vdVoltageRef             = 0.0f;
    vqVoltageRef             = 0.0f;
    vdVoltageErr             = 0.0f;
    vqVoltageErr             = 0.0f;
    voltageKp                = 0.01f;
    voltageKi                = 0.0f;
    vdVoltageIntegrator      = 0.0f;
    vqVoltageIntegrator      = 0.0f;
    idRefFromVoltageLoop     = 0.0f;
    iqRefFromVoltageLoop     = 0.0f;
    vdcVoltageRef            = 700.0f;
    vdcVoltageErr            = 0.0f;
    dcVoltageKp              = 0.01f;
    dcVoltageKi              = 0.0f;
    vdcVoltageIntegrator     = 0.0f;
    idRefFromDcVoltageLoop   = 0.0f;
    manualRelayCommand       = 0u;
    manualRelay1Command      = 0u;
    manualRelay2Command      = 0u;
    pllTheta                 = epwmAngles[0];
    pllOmega                 = pllOmegaNom;
    pllVq                    = 0.0f;
    pllIntegrator            = 0.0f;
    pllFreqErrHz             = 0.0f;
    pllLockCounter           = 0u;
    pllLocked                = 0u;
    pllError                 = 0u;
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    initEpwm();

    //
    // Enable sync and clock to PWM
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Assign the interrupt service routines to ePWM interrupts and enable
    //
    Interrupt_register(INT_EPWM1, &epwm1ISR);
    Interrupt_enable(INT_EPWM1);

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    //
    // Apply offset correction to sensors and set overcurrent protection
    //
    setSensorOffsetCorrection();
    setOvercurrentProtection(DEFAULTOVERCURRENT);

    //
    // Initialize Variables
    //
    uint16_t rxMsgData[8];

    //
    // Green LED Always On
    //
    enableOnBoardGreenLed();
    sciWriteString("\r\n[BOOT] Control init complete\r\n");
    sciWriteString("SCI console ready. Type 'h' + Enter for menu.\r\n");
    sciWriteString("BUILD TAG: " SCI_MENU_BUILD_TAG "\r\n");
    sciPrintMenu();

    //
    // Loop Indefinitely
    //
    while(1) {
        pollSciConsole();
#if SCI_MENU_ONLY_MODE
        DEVICE_DELAY_US(100);
#else
        DEVICE_DELAY_US(1000);

        //
        // Update relay outputs from manual commands
        //
        setRelay1(manualRelay1Command);
        setRelay2(manualRelay2Command);

        //
        // Check for Controller CAN Message. If Present, Decode and Update Controller
        //
        if (CAN_readMessage(myCAN0_BASE, CANOBJECTRXCONTROL, rxMsgData)) {
            receiveCanControls(rxMsgData);
            updateClockCycleParameters();
            controlPeriodSec = 1.0f / switchingFreq;
            setAllEpwm();
        }

        if (++canTxTicks >= 100u) {
            canTxTicks = 0u;
            sendCanControls();
            sendCanSensors();
            toggleOnBoardYellowLed();
        }

        if (++statusReportTicks >= 100u) {
#if SCI_PERIODIC_STATUS_ENABLE
            if (sciConsoleInputPending() != 0u) {
                statusReportTicks = 100u;
            } else {
                statusReportTicks = 0u;
                sendSciStatusReport();
            }
#else
            statusReportTicks = 0u;
#endif
        }
#endif
    }
}

static void initSciStatusPort(void) {
    mySCIA_init();
    SCI_disableFIFO(mySCIA_BASE);
    SCI_resetChannels(mySCIA_BASE);
    SCI_setConfig(mySCIA_BASE, DEVICE_LSPCLK_FREQ, SCI_CONSOLE_BAUD,
                  SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | SCI_CONFIG_PAR_NONE);
    SCI_enableModule(mySCIA_BASE);
}

static void sciWriteString(const char *msg) {
    while (*msg != '\0') {
        SCI_writeCharBlockingNonFIFO(mySCIA_BASE, (uint16_t)(*msg));
        msg++;
    }
}

static long scaleToMilli(float value) {
    if (value >= 0.0f)
        return (long)(value * 1000.0f + 0.5f);
    else
        return (long)(value * 1000.0f - 0.5f);
}

static long scaleToDeci(float value) {
    if (value >= 0.0f)
        return (long)(value * 10.0f + 0.5f);
    else
        return (long)(value * 10.0f - 0.5f);
}

static void sendSciStatusReport(void) {
    char txBuf[512];
    int n;
    uint16_t relay1;
    uint16_t relay2;

    relay1 = getRelay1();
    relay2 = getRelay2();

    n = snprintf(txBuf, sizeof(txBuf),
                 "mode=%u,ang=%u,relayCmd=%u,r1Cmd=%u,r2Cmd=%u,r1=%u,r2=%u,pllLock=%u,pllErr=%u,ctrlEn=%u,"
                 "vdc_dV=%ld,id_mA=%ld,iq_mA=%ld,idRef_mA=%ld,iqRef_mA=%ld,"
                 "vdCmd_dV=%ld,vqCmd_dV=%ld,vdMeas_dV=%ld,vqMeas_dV=%ld,"
                 "vdRef_dV=%ld,vqRef_dV=%ld,pllFErr_mHz=%ld,pllVq_mV=%ld\r\n",
                 (unsigned int)controlMode,
                 (unsigned int)angleSourceSelect,
                 (unsigned int)manualRelayCommand,
                 (unsigned int)manualRelay1Command,
                 (unsigned int)manualRelay2Command,
                 (unsigned int)relay1,
                 (unsigned int)relay2,
                 (unsigned int)pllLocked,
                 (unsigned int)pllError,
                 (unsigned int)currentControlEnable,
                 scaleToDeci(dcBusVoltageMeas),
                 scaleToMilli(idCurrentMeas),
                 scaleToMilli(iqCurrentMeas),
                 scaleToMilli(idRefCurrent),
                 scaleToMilli(iqRefCurrent),
                 scaleToDeci(vdCommand),
                 scaleToDeci(vqCommand),
                 scaleToDeci(vdVoltageMeas),
                 scaleToDeci(vqVoltageMeas),
                 scaleToDeci(vdVoltageRef),
                 scaleToDeci(vqVoltageRef),
                 scaleToMilli(pllFreqErrHz),
                 scaleToMilli(pllVq));

    if (n > 0)
        sciWriteString(txBuf);
}

static void sendSciSnapshot(void) {
    char txBuf[320];
    int n;

    n = snprintf(txBuf, sizeof(txBuf),
                 "\r\n--- SNAPSHOT ---\r\n"
                 "mode=%u angleSource=%u pllLock=%u pllErr=%u\r\n"
                 "relayCmd=%u relay1Cmd=%u relay2Cmd=%u relay1=%u relay2=%u\r\n"
                 "modCmd_milli=%ld\r\n"
                 "id_mA=%ld iq_mA=%ld idRef_mA=%ld iqRef_mA=%ld\r\n"
                 "vdCmd_dV=%ld vqCmd_dV=%ld\r\n"
                 "vdMeas_dV=%ld vqMeas_dV=%ld vdRef_dV=%ld vqRef_dV=%ld\r\n"
                 "vdc_dV=%ld vdcRef_dV=%ld\r\n"
                 "pllFreqErr_mHz=%ld pllVq_mV=%ld\r\n"
                 "----------------\r\n",
                 (unsigned int)controlMode,
                 (unsigned int)angleSourceSelect,
                 (unsigned int)pllLocked,
                 (unsigned int)pllError,
                 (unsigned int)manualRelayCommand,
                 (unsigned int)manualRelay1Command,
                 (unsigned int)manualRelay2Command,
                 (unsigned int)getRelay1(),
                 (unsigned int)getRelay2(),
                 scaleToMilli(modulationFactor),
                 scaleToMilli(idCurrentMeas),
                 scaleToMilli(iqCurrentMeas),
                 scaleToMilli(idRefCurrent),
                 scaleToMilli(iqRefCurrent),
                 scaleToDeci(vdCommand),
                 scaleToDeci(vqCommand),
                 scaleToDeci(vdVoltageMeas),
                 scaleToDeci(vqVoltageMeas),
                 scaleToDeci(vdVoltageRef),
                 scaleToDeci(vqVoltageRef),
                 scaleToDeci(dcBusVoltageMeas),
                 scaleToDeci(vdcVoltageRef),
                 scaleToMilli(pllFreqErrHz),
                 scaleToMilli(pllVq));

    if (n > 0)
        sciWriteString(txBuf);
}

typedef enum {
    SCI_MENU_HOME = 0,
    SCI_MENU_WAIT_MODE,
    SCI_MENU_WAIT_IDREF,
    SCI_MENU_WAIT_IQREF,
    SCI_MENU_WAIT_RELAY1,
    SCI_MENU_WAIT_RELAY2,
    SCI_MENU_WAIT_ANGLE_SOURCE,
    SCI_MENU_WAIT_VDREF,
    SCI_MENU_WAIT_VQREF,
    SCI_MENU_WAIT_VDCREF,
    SCI_MENU_WAIT_DUTY
} SciMenuState;

static SciMenuState gSciMenuState = SCI_MENU_HOME;

static void sciPrintMenu(void) {
    sciWriteString("\r\n=== MAIN MENU (" SCI_MENU_BUILD_TAG ") ===\r\n");
    sciWriteString("0: Show menu/home\r\n");
    sciWriteString("1: Change control mode (0=open,1=Iq/Id,2=Vdq+Idq,3=Vdc+Idq)\r\n");
    sciWriteString("2: Change current refs (Id/Iq in A)\r\n");
    sciWriteString("3: Set Relay1 (0=open,1=close)\r\n");
    sciWriteString("4: Set Relay2 (0=open,1=close)\r\n");
    sciWriteString("5: Set angle source (0=open-loop angle,1=PLL)\r\n");
    sciWriteString("6: Change voltage refs (Vd/Vq in V)\r\n");
    sciWriteString("7: Change DC-bus voltage ref (Vdc in V)\r\n");
    sciWriteString("8: Print snapshot now\r\n");
    sciWriteString("9: Set open-loop duty command (0.0 to 1.0)\r\n");
    sciWriteString("A: Set open-loop duty command (0.0 to 1.0)\r\n");
    sciWriteString("S: Print snapshot now\r\n");
    sciWriteString("h: Show menu\r\n");
    sciWriteString("> ");
}

static void handleSciLine(const char *line) {
    long intVal;
    float floatVal;

    if (line == NULL) {
        sciWriteString("> ");
        return;
    }

    while ((*line == ' ') || (*line == '\t')) {
        line++;
    }

    if (line[0] == '\0') {
        sciWriteString("> ");
        return;
    }

    if ((line[0] == 'h') || (line[0] == 'H') || (line[0] == '0')) {
        gSciMenuState = SCI_MENU_HOME;
        sciPrintMenu();
        return;
    }

    if (gSciMenuState == SCI_MENU_HOME) {
        switch (line[0]) {
            case '1':
                gSciMenuState = SCI_MENU_WAIT_MODE;
                sciWriteString("\r\nEnter control mode [0..3]: ");
                break;
            case '2':
                gSciMenuState = SCI_MENU_WAIT_IDREF;
                sciWriteString("\r\nEnter Id reference (A): ");
                break;
            case '3':
                gSciMenuState = SCI_MENU_WAIT_RELAY1;
                sciWriteString("\r\nEnter Relay1 command [0/1]: ");
                break;
            case '4':
                gSciMenuState = SCI_MENU_WAIT_RELAY2;
                sciWriteString("\r\nEnter Relay2 command [0/1]: ");
                break;
            case '5':
                gSciMenuState = SCI_MENU_WAIT_ANGLE_SOURCE;
                sciWriteString("\r\nEnter angle source [0=open-loop,1=PLL]: ");
                break;
            case '6':
                gSciMenuState = SCI_MENU_WAIT_VDREF;
                sciWriteString("\r\nEnter Vd reference (V): ");
                break;
            case '7':
                gSciMenuState = SCI_MENU_WAIT_VDCREF;
                sciWriteString("\r\nEnter Vdc reference (V): ");
                break;
            case '8':
                sendSciSnapshot();
                sciWriteString("> ");
                break;
            case '9':
                gSciMenuState = SCI_MENU_WAIT_DUTY;
                sciWriteString("\r\nEnter duty command [0.0..1.0]: ");
                break;
            case 's':
            case 'S':
                sendSciSnapshot();
                sciWriteString("> ");
                break;
            case 'a':
            case 'A':
                gSciMenuState = SCI_MENU_WAIT_DUTY;
                sciWriteString("\r\nEnter duty command [0.0..1.0]: ");
                break;
            default:
                sciWriteString("\r\nUnknown command. Type 'h' for menu.\r\n> ");
                break;
        }
        return;
    }

    switch (gSciMenuState) {
        case SCI_MENU_WAIT_MODE:
            intVal = strtol(line, NULL, 10);
            if ((intVal < 0) || (intVal > 3))
                sciWriteString("\r\nInvalid mode. Use 0..3.\r\n");
            else {
                setControlMode((uint16_t)intVal);
                sciWriteString("\r\nMode updated.\r\n");
            }
            gSciMenuState = SCI_MENU_HOME;
            sciWriteString("> ");
            break;

        case SCI_MENU_WAIT_IDREF:
            floatVal = strtof(line, NULL);
            setCurrentControllerReferences(floatVal, iqRefCurrent);
            gSciMenuState = SCI_MENU_WAIT_IQREF;
            sciWriteString("\r\nEnter Iq reference (A): ");
            break;

        case SCI_MENU_WAIT_IQREF:
            floatVal = strtof(line, NULL);
            setCurrentControllerReferences(idRefCurrent, floatVal);
            sciWriteString("\r\nCurrent references updated.\r\n");
            gSciMenuState = SCI_MENU_HOME;
            sciWriteString("> ");
            break;

        case SCI_MENU_WAIT_RELAY1:
            intVal = strtol(line, NULL, 10);
            manualRelay1Command = (intVal == 0) ? 0u : 1u;
            manualRelayCommand = (manualRelay1Command == manualRelay2Command) ? manualRelay1Command : 0u;
            sciWriteString("\r\nRelay1 command updated.\r\n");
            gSciMenuState = SCI_MENU_HOME;
            sciWriteString("> ");
            break;

        case SCI_MENU_WAIT_RELAY2:
            intVal = strtol(line, NULL, 10);
            manualRelay2Command = (intVal == 0) ? 0u : 1u;
            manualRelayCommand = (manualRelay1Command == manualRelay2Command) ? manualRelay1Command : 0u;
            sciWriteString("\r\nRelay2 command updated.\r\n");
            gSciMenuState = SCI_MENU_HOME;
            sciWriteString("> ");
            break;

        case SCI_MENU_WAIT_ANGLE_SOURCE:
            intVal = strtol(line, NULL, 10);
            setAngleSourceMode((intVal == 0) ? 0u : 1u);
            sciWriteString("\r\nAngle source updated.\r\n");
            gSciMenuState = SCI_MENU_HOME;
            sciWriteString("> ");
            break;

        case SCI_MENU_WAIT_VDREF:
            floatVal = strtof(line, NULL);
            setVoltageControllerReferences(floatVal, vqVoltageRef);
            gSciMenuState = SCI_MENU_WAIT_VQREF;
            sciWriteString("\r\nEnter Vq reference (V): ");
            break;

        case SCI_MENU_WAIT_VQREF:
            floatVal = strtof(line, NULL);
            setVoltageControllerReferences(vdVoltageRef, floatVal);
            sciWriteString("\r\nVoltage references updated.\r\n");
            gSciMenuState = SCI_MENU_HOME;
            sciWriteString("> ");
            break;

        case SCI_MENU_WAIT_VDCREF:
            floatVal = strtof(line, NULL);
            setDcVoltageControllerReference(floatVal);
            sciWriteString("\r\nDC-bus voltage reference updated.\r\n");
            gSciMenuState = SCI_MENU_HOME;
            sciWriteString("> ");
            break;

        case SCI_MENU_WAIT_DUTY:
            floatVal = strtof(line, NULL);
            if (floatVal < 0.0f)
                floatVal = 0.0f;
            if (floatVal > 1.0f)
                floatVal = 1.0f;
            modulationFactor = floatVal;
            sciWriteString("\r\nOpen-loop duty command updated.\r\n");
            gSciMenuState = SCI_MENU_HOME;
            sciWriteString("> ");
            break;

        default:
            gSciMenuState = SCI_MENU_HOME;
            sciWriteString("\r\nMenu reset.\r\n> ");
            break;
    }
}

static void pollSciConsole(void) {
    static char rxLine[64];
    static uint16_t rxIdx = 0u;
    static uint16_t lastTerminatorWasCR = 0u;
    uint16_t ch;
    uint16_t rxStatus;
    char c;

    rxStatus = SCI_getRxStatus(mySCIA_BASE);
    if ((rxStatus & SCI_RXSTATUS_ERROR) != 0u) {
#if SCI_RX_DEBUG
        sciDebugRxStatus(rxStatus);
#endif
        mySCIA_init();
        rxIdx = 0u;
        lastTerminatorWasCR = 0u;
    }

    while (SCI_isDataAvailableNonFIFO(mySCIA_BASE)) {
        ch = SCI_readCharNonBlocking(mySCIA_BASE);
#if SCI_RX_DEBUG
        sciDebugRxByte(ch);
#endif
        c = (char)(ch & 0xFFu);

        if ((c == '\r') || (c == '\n')) {
            if ((c == '\n') && (lastTerminatorWasCR != 0u)) {
                lastTerminatorWasCR = 0u;
                continue;
            }

            lastTerminatorWasCR = (c == '\r') ? 1u : 0u;
            sciWriteString("\r\n");
            rxLine[rxIdx] = '\0';
            handleSciLine(rxLine);
            rxIdx = 0u;
        } else if ((c == '\b') || (c == 127)) {
            lastTerminatorWasCR = 0u;
            if (rxIdx > 0u) {
                rxIdx--;
                sciWriteString("\b \b");
            }
        } else if ((c >= 32) && (c <= 126)) {
            lastTerminatorWasCR = 0u;
            if (rxIdx < (sizeof(rxLine) - 1u)) {
                rxLine[rxIdx++] = c;
                SCI_writeCharBlockingNonFIFO(mySCIA_BASE, (uint16_t)c);
            }
        }
    }
}

static uint16_t sciConsoleInputPending(void) {
    return SCI_isDataAvailableNonFIFO(mySCIA_BASE) ? 1u : 0u;
}

static void sciDebugRxByte(uint16_t ch) {
    char buf[32];
    char printable = '.';
    uint16_t byteVal = (ch & 0x00FFu);

    if ((byteVal >= 32u) && (byteVal <= 126u)) {
        printable = (char)byteVal;
    }

    snprintf(buf, sizeof(buf), "\r\n[SCI RX] 0x%02X '%c'\r\n", (unsigned)byteVal, printable);
    sciWriteString(buf);
}

static void sciDebugRxStatus(uint16_t status) {
    char buf[64];
    snprintf(buf, sizeof(buf), "\r\n[SCI RX ERROR] RXST=0x%02X (resetting SCI)\r\n",
             (unsigned)(status & 0x00FFu));
    sciWriteString(buf);
}

void setCurrentControllerMode(uint16_t enable) {
    currentControlEnable = (enable == 0u) ? 0u : 1u;
    controlMode = (enable == 0u) ? 0u : 1u;
}

void setCurrentControllerReferences(float idRef, float iqRef) {
    idRefCurrent = idRef;
    iqRefCurrent = iqRef;
}

void setCurrentControllerGains(float kp, float ki) {
    if (kp < 0.0f)
        kp = 0.0f;
    if (ki < 0.0f)
        ki = 0.0f;

    currentKp = kp;
    currentKi = ki;
}

void setControlMode(uint16_t mode) {
    if (mode > 3u)
        mode = 3u;

    controlMode = mode;
    currentControlEnable = (mode == 0u) ? 0u : 1u;

    // Active rectifier mode always requires PLL-referenced angle.
    if (mode == 3u)
        angleSourceSelect = 1u;
}

void setVoltageControllerReferences(float vdRef, float vqRef) {
    vdVoltageRef = vdRef;
    vqVoltageRef = vqRef;
}

void setVoltageControllerGains(float kp, float ki) {
    if (kp < 0.0f)
        kp = 0.0f;
    if (ki < 0.0f)
        ki = 0.0f;

    voltageKp = kp;
    voltageKi = ki;
}

void setDcVoltageControllerReference(float vdcRef) {
    if (vdcRef < 0.0f)
        vdcRef = 0.0f;
    vdcVoltageRef = vdcRef;
}

void setDcVoltageControllerGains(float kp, float ki) {
    if (kp < 0.0f)
        kp = 0.0f;
    if (ki < 0.0f)
        ki = 0.0f;

    dcVoltageKp = kp;
    dcVoltageKi = ki;
}

void setAngleSourceMode(uint16_t mode) {
    if (controlMode == 3u) {
        angleSourceSelect = 1u;
        return;
    }
    angleSourceSelect = (mode == 0u) ? 0u : 1u;
}

void setPllGains(float kp, float ki) {
    if (kp < 0.0f)
        kp = 0.0f;
    if (ki < 0.0f)
        ki = 0.0f;

    pllKp = kp;
    pllKi = ki;
}

void setPllNominalFrequency(float gridFreqHz) {
    if (gridFreqHz < 10.0f)
        gridFreqHz = 10.0f;
    if (gridFreqHz > 1000.0f)
        gridFreqHz = 1000.0f;

    pllOmegaNom = PI2 * gridFreqHz;
}

//*****************************************************************************
//
//! EPWM1 interrupt service routine (ISR) runs once per switching period.
//! Samples phase currents/voltages and DC-link voltage for control and
//! telemetry. Current samples are also stored for RMS statistics.
//!
//! \return None.
//
//*****************************************************************************
__interrupt void epwm1ISR(void)
{
    uint16_t latestIndex;

    forceCurrentSocsWait();
    storeCurrentSensors();
    forceVoltageSocsWait();

    latestIndex = (bufferIndex == 0) ? (BUFFERSIZE - 1) : (bufferIndex - 1);
    phaseCurrentU = currentUBuffer[latestIndex];
    phaseCurrentV = currentVBuffer[latestIndex];
    phaseCurrentW = currentWBuffer[latestIndex];
    phaseVoltageU = getVoltageU();
    phaseVoltageV = getVoltageV();
    phaseVoltageW = getVoltageW();
    dcBusVoltageMeas = getVoltageDc();

    //
    // Clear INT flag for this timer and Acknowledge
    //
    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}

//*****************************************************************************
//
//! After Control Law Accelerator (CLA) Task 1 runs, increment the phase angle
//! for the sine PWM (SPWM) reference.
//!
//! \return None.
//
//*****************************************************************************
__interrupt void cla1Isr1 () {
    //
    // Increment Phase Angle
    //
    uint16_t i;
    for (i = 0; i < NUMPHASES; i++) {
        epwmAngles[i] += angleStep;
        if (epwmAngles[i] > PI2)
            epwmAngles[i] -= PI2;
    }
    if (angleSourceSelect == 0u)
        electricalAngle = epwmAngles[0];

    //
    // Acknowledge the end-of-task interrupt for task 1
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);
}

//
// End of file
//
