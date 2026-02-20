
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
#include <string.h>

#define SCI_RX_DEBUG 0u
#define SCI_MENU_BUILD_TAG "SCI_MENU_DEBUG_2026_02_20_A"
#define SCI_MENU_ONLY_MODE 0u
#define SCI_PERIODIC_STATUS_ENABLE 0u
#define SCI_CONSOLE_BAUD 115200u
#define SCI_TX_TIMEOUT 120000u

#ifndef mySCIA_BASE
#define mySCIA_BASE SCIA_BASE
#endif

//
// Local function prototypes - SCI
//
static void initSciStatusPort(void);
static void sciWriteChar(char c);
static void sciWriteString(const char *msg);
static void sciWriteHexByte(uint16_t val);
static void sciWriteLong(long val);
static void sciWriteUint(uint16_t val);
static void sciWriteLabel(const char *label, long val);
static uint16_t sciStringLength(const char *s);
static uint16_t sciCountLongChars(long val);
static void sciWriteSpaces(uint16_t count);
static void sciWriteTwoCols(const char *leftLabel, long leftVal,
                            const char *rightLabel, long rightVal);
static long scaleToMilli(float value);
static long scaleToDeci(float value);
static uint16_t sciParseInt(const char *s, long *out);
static uint16_t sciParseFloat(const char *s, float *out);
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
    // Apply offset correction to sensors and set overcurrent protection before
    // enabling PWM interrupts/global interrupts (avoids ADC-flag contention
    // with epwm1ISR during calibration).
    //
    setSensorOffsetCorrection();
    setOvercurrentProtection(DEFAULTOVERCURRENT);

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
                sendSciSnapshot();
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

static void sciWriteChar(char c) {
    volatile uint32_t timeout = SCI_TX_TIMEOUT;
    while (!SCI_isSpaceAvailableNonFIFO(mySCIA_BASE)) {
        if (--timeout == 0u)
            return;
    }
    SCI_writeCharNonBlocking(mySCIA_BASE, (uint16_t)c);
}

static void sciWriteString(const char *msg) {
    while (*msg != '\0') {
        sciWriteChar(*msg);
        msg++;
    }
}

static void sciWriteHexByte(uint16_t val) {
    const char hex[] = "0123456789ABCDEF";
    sciWriteChar(hex[(val >> 4u) & 0x0Fu]);
    sciWriteChar(hex[val & 0x0Fu]);
}

static void sciWriteLong(long val) {
    char buf[12];
    char *p = buf + sizeof(buf) - 1;
    unsigned long uv;
    uint16_t neg = 0u;

    *p = '\0';
    if (val < 0) {
        neg = 1u;
        uv = (unsigned long)(-val);
    } else {
        uv = (unsigned long)val;
    }
    if (uv == 0u) {
        *(--p) = '0';
    } else {
        while (uv > 0u) {
            *(--p) = '0' + (char)(uv % 10u);
            uv /= 10u;
        }
    }
    if (neg)
        *(--p) = '-';
    sciWriteString(p);
}

static void sciWriteUint(uint16_t val) {
    sciWriteLong((long)val);
}

static void sciWriteLabel(const char *label, long val) {
    sciWriteString(label);
    sciWriteLong(val);
}

static uint16_t sciStringLength(const char *s) {
    uint16_t len = 0u;
    while (s[len] != '\0')
        len++;
    return len;
}

static uint16_t sciCountLongChars(long val) {
    uint16_t chars = 1u;
    unsigned long uv;

    if (val < 0) {
        chars++;
        uv = (unsigned long)(-val);
    } else {
        uv = (unsigned long)val;
    }

    while (uv >= 10u) {
        uv /= 10u;
        chars++;
    }

    return chars;
}

static void sciWriteSpaces(uint16_t count) {
    while (count-- > 0u)
        sciWriteChar(' ');
}

static void sciWriteTwoCols(const char *leftLabel, long leftVal,
                            const char *rightLabel, long rightVal) {
    const uint16_t col2Start = 28u;
    uint16_t used = sciStringLength(leftLabel) + sciCountLongChars(leftVal);

    sciWriteLabel(leftLabel, leftVal);
    if (used < col2Start)
        sciWriteSpaces((uint16_t)(col2Start - used));
    else
        sciWriteSpaces(3u);
    sciWriteLabel(rightLabel, rightVal);
    sciWriteString("\r\n");
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

static uint16_t sciParseInt(const char *s, long *out) {
    long result = 0;
    uint16_t neg = 0u;
    uint16_t digits = 0u;

    while ((*s == ' ') || (*s == '\t'))
        s++;
    if (*s == '-') {
        neg = 1u;
        s++;
    } else if (*s == '+') {
        s++;
    }
    while ((*s >= '0') && (*s <= '9')) {
        result = result * 10 + (*s - '0');
        s++;
        digits++;
    }
    if (digits == 0u)
        return 0u;
    while ((*s == ' ') || (*s == '\t'))
        s++;
    if (*s != '\0')
        return 0u;
    *out = neg ? -result : result;
    return 1u;
}

static uint16_t sciParseFloat(const char *s, float *out) {
    float result = 0.0f;
    float frac = 0.0f;
    float divisor = 1.0f;
    uint16_t neg = 0u;
    uint16_t digits = 0u;
    uint16_t hasDot = 0u;

    while ((*s == ' ') || (*s == '\t'))
        s++;
    if (*s == '-') {
        neg = 1u;
        s++;
    } else if (*s == '+') {
        s++;
    }
    while (((*s >= '0') && (*s <= '9')) || ((*s == '.') && (hasDot == 0u))) {
        if (*s == '.') {
            hasDot = 1u;
        } else {
            digits++;
            if (hasDot) {
                divisor *= 10.0f;
                frac += (float)(*s - '0') / divisor;
            } else {
                result = result * 10.0f + (float)(*s - '0');
            }
        }
        s++;
    }
    if (digits == 0u)
        return 0u;
    while ((*s == ' ') || (*s == '\t'))
        s++;
    if (*s != '\0')
        return 0u;
    result += frac;
    *out = neg ? -result : result;
    return 1u;
}

static void sendSciSnapshot(void) {
    long pllFreqHzRounded;

    pllFreqHzRounded = (long)((pllOmega / PI2) + ((pllOmega >= 0.0f) ? 0.5f : -0.5f));

    sciWriteString("\r\n--- SNAPSHOT ---\r\n");

    sciWriteTwoCols("mode=", (long)controlMode,
                    "angleSource=", (long)angleSourceSelect);
    sciWriteTwoCols("relay1Cmd=", (long)manualRelay1Command,
                    "relay2Cmd=", (long)manualRelay2Command);

    sciWriteString("modCmd_milli="); sciWriteLong(scaleToMilli(modulationFactor));
    sciWriteString("\r\n");

    sciWriteTwoCols("idRef_mA=", scaleToMilli(idRefCurrent),
                    "iqRef_mA=", scaleToMilli(iqRefCurrent));
    sciWriteTwoCols("id_mA=", scaleToMilli(idCurrentMeas),
                    "iq_mA=", scaleToMilli(iqCurrentMeas));
    sciWriteTwoCols("vdRef_dV=", scaleToDeci(vdVoltageRef),
                    "vqRef_dV=", scaleToDeci(vqVoltageRef));
    sciWriteTwoCols("vdMeas_dV=", scaleToDeci(vdVoltageMeas),
                    "vqMeas_dV=", scaleToDeci(vqVoltageMeas));
    sciWriteTwoCols("vdcRef_dV=", scaleToDeci(vdcVoltageRef),
                    "vdc_dV=", scaleToDeci(dcBusVoltageMeas));
    sciWriteTwoCols("pllLock=", (long)pllLocked,
                    "pllHz=", pllFreqHzRounded);

    sciWriteString("----------------\r\n");
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
    sciWriteString("8: Set open-loop duty command (0.0 to 1.0)\r\n");
    sciWriteString("S: Print snapshot now\r\n");
    sciWriteString("h: Show menu\r\n");
    sciWriteString("(type q to cancel any sub-prompt)\r\n");
    sciWriteString("> ");
}

static uint16_t sciIsCancelCommand(const char *line) {
    if (line[0] == '\0')
        return 1u;
    if (((line[0] == 'q') || (line[0] == 'Q') ||
         (line[0] == 'x') || (line[0] == 'X')) &&
        ((line[1] == '\0') || (line[1] == ' ') || (line[1] == '\t')))
        return 1u;
    return 0u;
}

static void handleSciLine(const char *line) {
    long intVal;
    float floatVal;

    if (line == NULL) {
        sciWriteString("> ");
        return;
    }

    while ((*line == ' ') || (*line == '\t'))
        line++;

    //
    // Home/help always works regardless of state
    //
    if ((line[0] == 'h') || (line[0] == 'H') || (line[0] == '0')) {
        gSciMenuState = SCI_MENU_HOME;
        sciPrintMenu();
        return;
    }

    //
    // Empty line at home prompt — just reprint prompt
    //
    if ((gSciMenuState == SCI_MENU_HOME) && (line[0] == '\0')) {
        sciWriteString("> ");
        return;
    }

    //
    // Cancel out of any sub-prompt with q/Q/x/X or empty Enter
    //
    if ((gSciMenuState != SCI_MENU_HOME) && sciIsCancelCommand(line)) {
        gSciMenuState = SCI_MENU_HOME;
        sciWriteString("\r\nCancelled.\r\n> ");
        return;
    }

    //
    // Home menu command dispatch
    //
    if (gSciMenuState == SCI_MENU_HOME) {
        switch (line[0]) {
            case '1':
                gSciMenuState = SCI_MENU_WAIT_MODE;
                sciWriteString("\r\nEnter control mode [0..3] (q=cancel): ");
                break;
            case '2':
                gSciMenuState = SCI_MENU_WAIT_IDREF;
                sciWriteString("\r\nEnter Id reference in A (q=cancel): ");
                break;
            case '3':
                gSciMenuState = SCI_MENU_WAIT_RELAY1;
                sciWriteString("\r\nEnter Relay1 command [0/1] (q=cancel): ");
                break;
            case '4':
                gSciMenuState = SCI_MENU_WAIT_RELAY2;
                sciWriteString("\r\nEnter Relay2 command [0/1] (q=cancel): ");
                break;
            case '5':
                gSciMenuState = SCI_MENU_WAIT_ANGLE_SOURCE;
                sciWriteString("\r\nEnter angle source [0=open-loop,1=PLL] (q=cancel): ");
                break;
            case '6':
                gSciMenuState = SCI_MENU_WAIT_VDREF;
                sciWriteString("\r\nEnter Vd reference in V (q=cancel): ");
                break;
            case '7':
                gSciMenuState = SCI_MENU_WAIT_VDCREF;
                sciWriteString("\r\nEnter Vdc reference in V (q=cancel): ");
                break;
            case '8':
                gSciMenuState = SCI_MENU_WAIT_DUTY;
                sciWriteString("\r\nEnter duty command [0.0..1.0] (q=cancel): ");
                break;
            case 's':
            case 'S':
                sendSciSnapshot();
                sciWriteString("> ");
                break;
            default:
                sciWriteString("\r\nUnknown command. Type 'h' for menu.\r\n> ");
                break;
        }
        return;
    }

    //
    // Sub-prompt value entry
    //
    switch (gSciMenuState) {
        case SCI_MENU_WAIT_MODE:
            if (sciParseInt(line, &intVal) == 0u || intVal < 0 || intVal > 3) {
                sciWriteString("\r\nInvalid. Use 0..3.\r\n");
            } else {
                setControlMode((uint16_t)intVal);
                sciWriteString("\r\nMode updated.\r\n");
            }
            gSciMenuState = SCI_MENU_HOME;
            sciWriteString("> ");
            break;

        case SCI_MENU_WAIT_IDREF:
            if (sciParseFloat(line, &floatVal) == 0u) {
                sciWriteString("\r\nInvalid input.\r\n");
                gSciMenuState = SCI_MENU_HOME;
                sciWriteString("> ");
            } else {
                setCurrentControllerReferences(floatVal, iqRefCurrent);
                gSciMenuState = SCI_MENU_WAIT_IQREF;
                sciWriteString("\r\nEnter Iq reference in A (q=cancel): ");
            }
            break;

        case SCI_MENU_WAIT_IQREF:
            if (sciParseFloat(line, &floatVal) == 0u) {
                sciWriteString("\r\nInvalid input.\r\n");
            } else {
                setCurrentControllerReferences(idRefCurrent, floatVal);
                sciWriteString("\r\nCurrent references updated.\r\n");
            }
            gSciMenuState = SCI_MENU_HOME;
            sciWriteString("> ");
            break;

        case SCI_MENU_WAIT_RELAY1:
            if (sciParseInt(line, &intVal) == 0u) {
                sciWriteString("\r\nInvalid input.\r\n");
            } else {
                manualRelay1Command = (intVal == 0) ? 0u : 1u;
                manualRelayCommand = (manualRelay1Command == manualRelay2Command) ? manualRelay1Command : 0u;
                sciWriteString("\r\nRelay1 command updated.\r\n");
            }
            gSciMenuState = SCI_MENU_HOME;
            sciWriteString("> ");
            break;

        case SCI_MENU_WAIT_RELAY2:
            if (sciParseInt(line, &intVal) == 0u) {
                sciWriteString("\r\nInvalid input.\r\n");
            } else {
                manualRelay2Command = (intVal == 0) ? 0u : 1u;
                manualRelayCommand = (manualRelay1Command == manualRelay2Command) ? manualRelay1Command : 0u;
                sciWriteString("\r\nRelay2 command updated.\r\n");
            }
            gSciMenuState = SCI_MENU_HOME;
            sciWriteString("> ");
            break;

        case SCI_MENU_WAIT_ANGLE_SOURCE:
            if (sciParseInt(line, &intVal) == 0u) {
                sciWriteString("\r\nInvalid input.\r\n");
            } else {
                setAngleSourceMode((intVal == 0) ? 0u : 1u);
                sciWriteString("\r\nAngle source updated.\r\n");
            }
            gSciMenuState = SCI_MENU_HOME;
            sciWriteString("> ");
            break;

        case SCI_MENU_WAIT_VDREF:
            if (sciParseFloat(line, &floatVal) == 0u) {
                sciWriteString("\r\nInvalid input.\r\n");
                gSciMenuState = SCI_MENU_HOME;
                sciWriteString("> ");
            } else {
                setVoltageControllerReferences(floatVal, vqVoltageRef);
                gSciMenuState = SCI_MENU_WAIT_VQREF;
                sciWriteString("\r\nEnter Vq reference in V (q=cancel): ");
            }
            break;

        case SCI_MENU_WAIT_VQREF:
            if (sciParseFloat(line, &floatVal) == 0u) {
                sciWriteString("\r\nInvalid input.\r\n");
            } else {
                setVoltageControllerReferences(vdVoltageRef, floatVal);
                sciWriteString("\r\nVoltage references updated.\r\n");
            }
            gSciMenuState = SCI_MENU_HOME;
            sciWriteString("> ");
            break;

        case SCI_MENU_WAIT_VDCREF:
            if (sciParseFloat(line, &floatVal) == 0u) {
                sciWriteString("\r\nInvalid input.\r\n");
            } else {
                setDcVoltageControllerReference(floatVal);
                sciWriteString("\r\nDC-bus voltage reference updated.\r\n");
            }
            gSciMenuState = SCI_MENU_HOME;
            sciWriteString("> ");
            break;

        case SCI_MENU_WAIT_DUTY:
            if (sciParseFloat(line, &floatVal) == 0u) {
                sciWriteString("\r\nInvalid input.\r\n");
            } else {
                if (floatVal < 0.0f)
                    floatVal = 0.0f;
                if (floatVal > 1.0f)
                    floatVal = 1.0f;
                modulationFactor = floatVal;
                sciWriteString("\r\nDuty command updated.\r\n");
            }
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
        initSciStatusPort();
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
                sciWriteChar(c);
            }
        }
    }
}

static uint16_t sciConsoleInputPending(void) {
    return SCI_isDataAvailableNonFIFO(mySCIA_BASE) ? 1u : 0u;
}

static void sciDebugRxByte(uint16_t ch) {
    uint16_t byteVal = (ch & 0x00FFu);
    char printable = '.';

    if ((byteVal >= 32u) && (byteVal <= 126u))
        printable = (char)byteVal;

    sciWriteString("\r\n[SCI RX] 0x");
    sciWriteHexByte(byteVal);
    sciWriteString(" '");
    sciWriteChar(printable);
    sciWriteString("'\r\n");
}

static void sciDebugRxStatus(uint16_t status) {
    sciWriteString("\r\n[SCI RX ERROR] RXST=0x");
    sciWriteHexByte(status & 0x00FFu);
    sciWriteString(" (resetting SCI)\r\n");
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
