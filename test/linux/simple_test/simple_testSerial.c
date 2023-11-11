/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <assert.h>
#include <stdio.h>  //1
#include <string.h> //6
#include <inttypes.h>
#include <stdlib.h> //2
#include <stdbool.h>
#include <sys/time.h> //3
#include <unistd.h>   //4
#include <sched.h>    //5
#include <time.h>     //8
#include <pthread.h>  //9
#include <math.h>     //10
#include <inttypes.h> //11

#include "ethercat.h"

//Included headers for serial port
//Credits: https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
//https://jason19970210.medium.com/raspberry-pi-4-with-multiple-uart-interface-4eac75f74d7c


#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()


// #pragma comment(lib,"ws2_32.lib") //Winsock Library

/*
#ifdef _WIN32
#include <Windows.h>
#endif
*/

typedef signed char INT8, *PINT8;
typedef signed short INT16, *PINT16;
typedef signed int INT32, *PINT32;
// typedef signed __int64      INT64, * PINT64;
typedef unsigned char UINT8, *PUINT8;
typedef unsigned short UINT16, *PUINT16;
typedef unsigned int UINT32, *PUINT32;
// typedef unsigned __int64    UINT64, * PUINT64;

#define stack64k (64 * 1024)

#define EC_TIMEOUTMON 500

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "27015"

#define ENCODER_RESOLUTION_IN_BITS 20
#define GEAR_RATIO 10
#define MOTOR_TORQUE_RATING 5.46

// There is some confusion regarding this value.
// Used in 2 places:
// OSAL Sleep Cycle: Here the value works as it is
// for calculation: the value needs to be 1/10th

// Max value at which it worked (when multiple was 10000): 8
// 7: 9.22
// 6: 8.02
// 5: 6.59
// 4: 5.51
// 3: 4.41
// 1: 1.51

#define SCAN_INTERVAL_IN_MSEC 2 // FBUS.SAMPLEPERIOD
#define SCAN_INTERVAL_IN_USEC SCAN_INTERVAL_IN_MSEC * 1000
#define SOCKET_SCAN_CYCLES 20

#define SOCKET_SERVER_APP_START_INIT 1
#define SOCKET_SERVER_CREATE_SOCKET_TO_LISTEN_FOR_CLIENT 2
#define SOCKET_SERVER_CREATE_SOCKET_FAILURE_ACTION 3
#define SOCKET_SERVER_SETUP_LISTENING_SOCKET 4
#define SOCKET_SERVER_LISTENING_SOCKET_SETUP_FAILURE_ACTION 5
#define SOCKET_SERVER_LISTEN_FOR_CLIENT_SOCKET 6
#define SOCKET_SERVER_LISTEN_FOR_CLIENT_SOCKET_FAILURE_ACTION 7
#define SOCKET_SERVER_ACCEPT_CLIENT_SOCKET 8
#define SOCKET_SERVER_GET_DATA_FROM_CLIENT 9
#define SOCKET_SERVER_ANALYZE_QUERY 10
#define SOCKET_SERVER_SEND_STATUS 11
#define SOCKET_SERVER_CLOSE_SOCKET 12

#define STATE_MACHINE_STAT_UNKNOWN 0
#define STATE_MACHINE_STAT_NOT_RDY_TO_SWITCH_ON 1
#define STATE_MACHINE_STAT_SWITCH_ON_DISABLED 2
#define STATE_MACHINE_STAT_RDY_TO_SWITCH_ON 3
#define STATE_MACHINE_STAT_SWITCHED_ON 4
#define STATE_MACHINE_STAT_OPERATION_ENABLED 5
#define STATE_MACHINE_STAT_FAULT 6
#define STATE_MACHINE_STAT_FAULT_REACTION_ACTIVE 7
#define STATE_MACHINE_STAT_QUICK_STOP_ACTIVE 8
#define STATE_MACHINE_STAT_SWITCH_ON_DISABLED_WITH_VE 9
#define STATE_MACHINE_STAT_SWITCH_ON_DISABLED_WITH_INTERNAL_LIMIT_ACTIVE 10

#define CTL_WD_SHUT_DOWN 1
#define CTL_WD_SWITCH_ON 2
#define CTL_WD_DISABLE_VTG 3
#define CTL_WD_QUICK_STOP 4
#define CTL_WD_DISABLE_OPN 5
#define CTL_WD_ENABLE_OPN 6
#define CTL_WD_ENABLE_IP 7
#define CTL_WD_FAULT_RESET 8
#define CTL_WD_STOP 9
#define CTL_WD_RESET_VE 10
#define CTL_WD_NEW_SET_POINT 11
#define CTL_WD_RESET_SET_POINT 12
#define CTL_WD_PP_MODE 13

#define BIT0 1
#define BIT1 2
#define BIT2 4
#define BIT3 8
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80
#define BIT8 0x100
#define BIT9 0x200
#define BIT10 0x400
#define BIT11 0x800
#define BIT12 0x1000
#define BIT13 0x2000
#define BIT14 0x4000
#define BIT15 0x8000

#define OPN_MODE_PROFILE_VELOCITY 1  // Available for Panasonic
#define OPN_MODE_INTERPOLATED_POSN 2 // Not Available for Panasonic
#define OPN_MODE_HOMING_MODE 3       // Available for Panasonic
#define OPN_MODE_PROFILE_POSN 4      //
#define OPN_MODE_PROFILE_TORQUE 5
#define OPN_MODE_CYCLIC_SYNCH_POSN 6
// Modes added for Panasonic
#define OPN_MODE_VELOCITY 7
#define OPN_MODE_CYCLIC_SYNCH_VELOCITY 8
#define OPN_MODE_CYCLIC_SYNCH_TORQUE 9

#define SERVO_OPN_EXPECTED 1
#define SERVO_OPN_NOT_EXPECTED 0

#define CMD_STAT_UNKNOWN 0
#define CMD_STAT_LOADED 1
#define CMD_STAT_ACCEPTED 2
#define CMD_STAT_COMPLETED_POSN 3
#define CMD_STAT_COMPLETED_TQ 4
#define CMD_STAT_ERR 5
#define CMD_STAT_REJECTED 6

// Register Definitions

// Mode setting
#define REG_DRV_OPMODE 0x35B4
#define REG_MODE_OF_OPN 0x6060

// Registers Associated with Profile Position Mode
#define REG_PROF_POSN_MODE_TARGET_POSN 0x607A
#define REG_PROF_POSN_MODE_SW_POSN_LIMIT 0x607D
#define REG_PROF_POSN_MODE_VELOCITY 0x6081
#define REG_PROF_POSN_MODE_ACCELERATION 0x6083
#define REG_PROF_POSN_MODE_DECELRATION 0x6084

#define REG_PROF_POSN_MODE_MAX_PROFILE_VELOCITY 0x607F
#define REG_PROF_POSN_MODE_MAX_MOTOR_SPEED 0X6080
#define REG_PROF_POSN_MODE_MAX_ACCELERATION 0X60C5
#define REG_PROF_POSN_MODE_MAX_DECELERATION 0X60C6
#define REG_PROF_POSN_MODE_MAX_TORQUE 0x6072

#define DRV_OPMODE_CURRENT_MODE 0
#define DRV_OPMODE_VELOCITY_MODE 1
#define DRV_OPMODE_POSITION_MODE 2

// Profile Position Mode SPecific
// #define PPMNEW_SETPOINT_ENABLE								*(ec_slave[0].outputs + 1) |= (BIT4)
// #define PPMNEW_SETPOINT_ENABLE_RELEASE						*(ec_slave[0].outputs + 1) &= ~(BIT4)
// #define PPMNEW_CHANGE_SET_IMMEDIATELY						*(ec_slave[0].outputs + 1) |= (BIT5)

// #define PPMNEW_RELATIVE_MODE								*(ec_slave[0].outputs + 1) |= (BIT6)
// #define PPMNEW_ABSOLUTE_MODE								*(ec_slave[0].outputs + 1) &= ~(BIT6)

#define CW 1  // Full Marks for being right handed
#define ACW 0 // Zero Marks for being left handed.... Indian Preconcieved notions become predefined definitions!!

#define MOTION_TIMEOUT_VAL 90000
#define BLOCK_ROTOR_DECLARE_CNT 1000 // This constant will be used for the number of samples for which the rotor consequtively has not achieved its desired position!

#define MANUFACTURER_ID 0x0000066F
#define PRODUCT_ID 0x60380008 // This ID is product specific and can be found in the specific device datasheet.

#define INPUT_OFFSET_ERRCODE 0
#define INPUT_OFFSET_STATUSWORD 2
#define INPUT_OFFSET_MODE_OF_OPN_DISP 4
#define INPUT_OFFSET_POSN_ACTUAL_VALUE 5
#define INPUT_OFFSET_VEL_ACTUAL_VALUE 9
#define INPUT_OFFSET_TQ_ACTUAL_VALUE 13

#define OUTPUT_OFFSET_CTLWD 0
#define OUTPUT_OFFSET_MODE_OF_OPN 2
#define OUTPUT_OFFSET_PROFILE_VELOCITY 3
#define OUTPUT_OFFSET_MAX_TQ 7
#define OUTPUT_OFFSET_TARGET_POSN 9
#define OUTPUT_OFFSET_TARGET_TQ 13

#define NSEC_PER_SEC 1000000000
#define MSEC_PER_SEC 1000000

#define MILLION 1000000
#define HUNDRED_THOU 100000
#define TEN_THOU 10000
#define THOU 1000
#define HUNDRED 100

char IOmap[4096];
OSAL_THREAD_HANDLE thread1, thread2;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

// IANET: 261122
volatile int64 rtcnt;

uint16 uiGFlag = 0;
uint32 uiLoopCntr = 0;
uint32 uiDly = 0;
uint32 uiStepsReqd = 0;

uint32 uiInitStatus = 0;

uint32 uiRdyToSwitchOn = 0;       // Bit 0
uint32 uiSwitchedOn = 0;          // Bit 1
uint32 uiOpnEnabled = 0;          // Bit 2
uint32 uiFault = 0;               // Bit 3
uint32 uiVtgEnabled = 0;          // Bit 4
uint32 uiQuickStop = 0;           // Bit 5
uint32 uiSwitchOnDisable = 0;     // Bit 6
uint32 uiWarning = 0;             // Bit 7
uint32 uiRemote = 0;              // Bit 9
uint32 uiTargetReached = 0;       // Bit 10
uint32 uiInternalLimitActive = 0; // Bit 11
uint32 uiInterpolationActive = 0; // Bit 12

uint32 u32val;
uint16 slave;

// Operational Variables
int16 uiSelectedMode = 0;
uint16 uiErrorFlag = 0;
uint16 uiSetPtAck = 0;
uint16 uiDriveStatus = 0;
uint16 uiDriveEnableStat = 0;
uint16 uiFaultFlag = 0;
uint16 uiPosnSetFlag = 0;
uint8 uiDriveStatWd;
uint16 uiProfileParamSetFlag = 0;
uint32 uiInterpolationEnableFlag = 0;
uint32 uiTargetReachedFlag = 0;
uint32 uiSetMotionFlag = 0;
uint16 uiStatusResetCntr = 0;
union
{
    uint16 hl;
    struct
    {
        uint8 l;
        uint8 h;
    } split;
} uiMaxTorque, uiStatusWd, uiTqOffset;

union
{
    int16 hl;
    struct
    {
        uint8 l;
        uint8 h;
    } split;
} iTqActual;




union
{
    uint32 hl;
    struct
    {
        uint8 ll;
        uint8 lh;
        uint8 hl;
        uint8 hh;
    } split;
} uiSecondFeedbackPosn, uiVelocity, uiVelCmdValue;

union
{
    int32 hl;
    struct
    {
        uint8 ll;
        uint8 lh;
        uint8 hl;
        uint8 hh;
    } split;
} iPosActualValue, iDesiredPositionVal, iFinalDesiredPosnVal;

union
{
    uint16 hl;
    struct
    {
        uint8 l;
        uint8 h;
    } split;
} uiCtlWd;

union
{
    uint16 hl;
    struct
    {
        uint8 l;
        uint8 h;
    } split;
} iErrCode;

uint8 ui8ModesOfOpnDisplay;

int32 iBuffPosValue;
uint32 uiBlockedRotorCntr;
uint32 uiBlockedRotorFlag;

int64 i64BuffVal;

uint64 uiEncoderCntsToMove;

// Application Specific
// uint8 ccwEnable = 0;
uint16 u16DirnCntr = 0;
uint16 u16MotionTimeOutCntr = 0;

FILE *fpIn;
FILE *fpOut;
uint32 uiDesiredDegreeOfRtn;
uint32 uiDesiredDirectionOfRtn;
uint32 uiDesiredRPM;
float fDesiredTq;
uint32 uiDesiredStatus;
uint32 uiActualPosn;
uint32 uiActualTq;
int32 iFinalPosnDesired;

uint32 uiRotationOffset;

uint32 uiModifyCmdStatusFlag = 0;
uint32 ui2MsecCntr = 0;
uint32 uiCyclesCntr = 0;
uint64 ui64Cntr = 0;

uint32 ui32StepsExecuted = 0;
uint32 ui32StepsProgramed = 0;

uint32 ui32RtThreadSpeedCntr = 0;
uint32 ui32RtThreadOvFlowCntr = 0;

// Socket Server related variables
uint32 uiServerSocketStatus = SOCKET_SERVER_APP_START_INIT;

// WSADATA wsaData;

int iResult;
u_long iMode = 0;
// UINT_PTR ListenSocket = INVALID_SOCKET;	//Changed to UINT_PTR from SOCKET because compiler was throwing an error
// UINT_PTR ClientSocket = INVALID_SOCKET;

// struct addrinfo* result = NULL;
// struct addrinfo hints;

int iSendResult;
char recvbuf[DEFAULT_BUFLEN];
int recvbuflen = DEFAULT_BUFLEN;

bool sktConnectedToClient = false;
uint8_t transferArr[10];

int iSocketElapsedCntr = 0;
int iMotionCompletedFlag = 0;

int getReasonForLossOfOperationalMode = 0;
int scanCntr = 0;

uint16 uiDelMeStatus = 0;

//
int64 gl_delta;
int64 toff;
int dorun = 0;
int64 gl_deltaArr[1000];
int gl_deltaCntr = 0;
int gl_deltaPrintFlag = 0;

struct timespec myTime;

int diffTimerSyncAchievedFlag = 0;
int diffTimerSyncAchievedCntr = 0;
int dataRdyForXtraction = 0;

int delMeOpnCntr = 0;
int delMeOpnExpectedFlag = 0;
int MotorOpnStatus = 0;

void fillMotionParams(uint32 uirProfileVelocity, int16 irMaxTq, int32 irTgtPosn);
uint16 setLimitingTorqueValue(float frDesiredTorque, float frMotorMaxTorque, uint16 uirGearRatio);
void updateStatus(uint16 uirStatus);
void printStatus(uint16 uirDriveStat);
void modifyControlWord(/*uint16 uirSlave,*/ uint16 uirDesiredStat /*, uint8 uiOffset*/);
// void setProfilePositionParameters(uint16 uirSlave, int32 irTargetPosn, uint32 uirProfileVelocity, uint32 uirProfileAcceleration, uint32 uirProfileDeceleration);
// void modifyInterpolatedPositionCommandValue(int32 irDesiredPosn, float frMaxTq, int32 irTargetVel);
void modifyLatchControlWordValue(uint16 uirLatchCtlWd /*, uint8 uirOffset*/);
// void modifyDigitalOutputValue(uint16 uirDigitalOp, uint8 uirOffset);
void DriveEnable();
void StopDrive();
void ShutDownDrive();
void setOutputPDO(uint16 slave, uint16 uirOutputPDO);
void setInputPDO(uint16 slave, uint16 uirInputPDO);
void setLimitValues(uint16 uirSlave);
void setDCModeSetup(uint16 uirSlave);
void setFlexibleOutputPDO(uint16 uirSlave);
void setFlexibleInputPDO(uint16 uirSlave);
unsigned char checkInterPolationDone();
void drv_SetOperationMode(uint16 slave, int32 irSelectedMode);
void resetDesiredTqAndDegOfRtn();
void GetDesiredTqAndDegOfRtn(void);
unsigned int ReadInteger(FILE *fp, unsigned int uiOffsetFromBegin);
void StoreInteger(FILE *fp, unsigned int IntegerToStore);
float ReadFloat(FILE *fp, unsigned int uiOffsetFromBegin);
void StoreFloat(FILE *fp, float floatToStore);
UINT32 ValidateInteger(UINT32 *uirIntegerToValidate, UINT32 uirIntegerMinVal, UINT32 uirIntegerMaxVal, UINT32 uirFillValIfInvalid);
UINT32 ValidateFloat(float *frFloatToValidate, float frFloatMinVal, float frFloatMaxVal, float frFillValIfInvalid);
void SetActualTqAndPosn();
// void SetCommandStatus(UINT32 uirCmdStatus);
void ConvertDegreeOfRotationToCount(uint32 uirRPM, uint32 uirScanIntervalInMsec, uint32 uirDegreesToRotate);
void socketServerAction();
char **str_split(char *a_str, const char a_delim);
void SocketSendResponse(char *);
int32 calculateFinalDesiredPosn(uint32 ui32rDesRtn);
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime);
void add_timespec(struct timespec *ts, int64 addtime);
void extractFlexibleInputPDO_data(void);

// Code taken from https://stackoverflow.com/questions/9210528/split-string-with-delimiters-in-c
// by user hmjd
/*
char** str_split(char* a_str, const char a_delim)
{
    char** result = 0;
    size_t count = 0;
    char* tmp = a_str;
    char* last_comma = 0;
    char delim[2];
    delim[0] = a_delim;
    delim[1] = 0;

    // Count how many elements will be extracted.
    while (*tmp)
    {
        if (a_delim == *tmp)
        {
            count++;
            last_comma = tmp;
        }
        tmp++;
    }

    // Add space for trailing token.
    count += last_comma < (a_str + strlen(a_str) - 1);

    // Add space for terminating null string so caller
    //   knows where the list of returned strings ends.
    count++;

    result = malloc(sizeof(char*) * count);

    if (result)
    {
        size_t idx = 0;
        char* token = strtok(a_str, delim);

        while (token)
        {
            assert(idx < count);
            *(result + idx++) = _strdup(token);
            token = strtok(0, delim);
        }
        assert(idx == count - 1);
        *(result + idx) = 0;
    }

    return result;
}
*/

// IANET: 261122
void fillMotionParams(uint32 uirProfileVelocity, int16 irMaxTq, int32 irTgtPosn)
{
    union
    {
        uint32 hl;
        struct
        {
            uint8 ll;
            uint8 lh;
            uint8 hl;
            uint8 hh;
        } split;
    } uiLclProfileVelocity;

    union
    {
        int32 hl;
        struct
        {
            uint8 ll;
            uint8 lh;
            uint8 hl;
            uint8 hh;
        } split;
    } iLclTgtPosn;

    union
    {
        int16 hl;
        struct
        {
            uint8 l;
            uint8 h;
        } split;
    } iLclMaxTq;

    uiLclProfileVelocity.hl = uirProfileVelocity;
    iLclMaxTq.hl = irMaxTq;
    iLclTgtPosn.hl = irTgtPosn;

    *(ec_slave[0].outputs + (OUTPUT_OFFSET_MAX_TQ)) = iLclMaxTq.split.l;
    *(ec_slave[0].outputs + (OUTPUT_OFFSET_MAX_TQ + 1)) = iLclMaxTq.split.h;

    *(ec_slave[0].outputs + (OUTPUT_OFFSET_TARGET_TQ)) = iLclMaxTq.split.l;
    *(ec_slave[0].outputs + (OUTPUT_OFFSET_TARGET_TQ + 1)) = iLclMaxTq.split.h; 

    *(ec_slave[0].outputs + (OUTPUT_OFFSET_PROFILE_VELOCITY)) = uiLclProfileVelocity.split.ll;
    *(ec_slave[0].outputs + (OUTPUT_OFFSET_PROFILE_VELOCITY + 1)) = uiLclProfileVelocity.split.lh;
    *(ec_slave[0].outputs + (OUTPUT_OFFSET_PROFILE_VELOCITY + 2)) = uiLclProfileVelocity.split.hl;
    *(ec_slave[0].outputs + (OUTPUT_OFFSET_PROFILE_VELOCITY + 3)) = uiLclProfileVelocity.split.hh;

    *(ec_slave[0].outputs + (OUTPUT_OFFSET_TARGET_POSN)) = iLclTgtPosn.split.ll;
    *(ec_slave[0].outputs + (OUTPUT_OFFSET_TARGET_POSN + 1)) = iLclTgtPosn.split.lh;
    *(ec_slave[0].outputs + (OUTPUT_OFFSET_TARGET_POSN + 2)) = iLclTgtPosn.split.hl;
    *(ec_slave[0].outputs + (OUTPUT_OFFSET_TARGET_POSN + 3)) = iLclTgtPosn.split.hh;
    iDesiredPositionVal.hl = iLclTgtPosn.hl;
}

uint16 setLimitingTorqueValue(float frDesiredTorque, float frMotorMaxTorque, uint16 uirGearRatio)
{
    uint16 uilclValToSet = 0;
    float flclValToSet;
    float flclMaxTorque = frMotorMaxTorque * (float)uirGearRatio;
    // printf("\n flclMaxTorque: %f\n ", flclMaxTorque);
    if (frDesiredTorque <= flclMaxTorque) // Indicates that a valid torque value has been entered
    {
        flclValToSet = (frDesiredTorque * 1000) / flclMaxTorque;
        if (ui32StepsExecuted > 2500)
        {
            printf("\tTQ Val Set: %f", flclValToSet);
        }
        uilclValToSet = (uint16)flclValToSet;
    }
    // printf("\n uilclValToSet: %d\n ", uilclValToSet);
    return uilclValToSet;
}

void updateStatus(uint16 uirStatus)
{
    // uint16 lclStat;
    // lclStat = 0;
    int32 statUpdated = 0;

    if ((uirStatus & (BIT6 + BIT3 + BIT2 + BIT1 + BIT0)) == 0)
    {
        uiDriveStatus = STATE_MACHINE_STAT_NOT_RDY_TO_SWITCH_ON;
        statUpdated = 1;
    }

    if ((uirStatus & (BIT6 + BIT3 + BIT2 + BIT1 + BIT0)) == BIT6)
    {
        uiDriveStatus = STATE_MACHINE_STAT_SWITCH_ON_DISABLED;
        statUpdated = 1;
    }

    if ((uirStatus & (BIT6 + BIT5 + BIT3 + BIT2 + BIT1 + BIT0)) == (BIT5 + BIT0))
    {
        uiDriveStatus = STATE_MACHINE_STAT_RDY_TO_SWITCH_ON;
        statUpdated = 1;
    }

    if ((uirStatus & (BIT6 + BIT5 + BIT3 + BIT2 + BIT1 + BIT0)) == (BIT5 + BIT1 + BIT0))
    {
        uiDriveStatus = STATE_MACHINE_STAT_SWITCHED_ON;
        statUpdated = 1;
    }

    if ((uirStatus & (BIT6 + BIT5 + BIT3 + BIT2 + BIT1 + BIT0)) == (BIT5 + BIT2 + BIT1 + BIT0))
    {
        uiDriveStatus = STATE_MACHINE_STAT_OPERATION_ENABLED;
        statUpdated = 1;
        // printf(" OE:%x inop:%d", uirStatus, inOP);

        // Delete this section later
        uiDelMeStatus = uirStatus;
    }

    if ((uirStatus & (BIT6 + BIT5 + BIT3 + BIT2 + BIT1 + BIT0)) == (BIT2 + BIT1 + BIT0))
    {
        printf("QS:%x", uirStatus);
        uiDriveStatus = STATE_MACHINE_STAT_QUICK_STOP_ACTIVE;
        statUpdated = 1;
    }

    if ((uirStatus & (BIT6 + BIT5 + BIT3 + BIT2 + BIT1 + BIT0)) == (BIT3 + BIT2 + BIT1 + BIT0))
    {
        uiDriveStatus = STATE_MACHINE_STAT_FAULT_REACTION_ACTIVE;
        statUpdated = 1;
    }

    if ((uirStatus & (BIT6 + BIT3 + BIT2 + BIT1 + BIT0)) == (BIT3))
    {
        uiDriveStatus = STATE_MACHINE_STAT_FAULT;
        statUpdated = 1;
    }

    if (statUpdated == 0)
    {
        uiDriveStatus = STATE_MACHINE_STAT_UNKNOWN;
    }

    if ((uirStatus & BIT7) == BIT7)
        uiErrorFlag = 1;
    else
        uiErrorFlag = 0;

    if ((uirStatus & BIT12) == BIT12)
        uiSetPtAck = 1;
    else
        uiSetPtAck = 0;

    if (uiInterpolationEnableFlag == 1)
    {
        if ((uirStatus & BIT12) == BIT12)
            uiInterpolationActive = 1;
        else
            uiInterpolationActive = 0;
    }

    if ((uirStatus & BIT10) == BIT10)
        uiTargetReachedFlag = 1;
    else
        uiTargetReachedFlag = 0;
}

void printStatus(uint16 uirDriveStat)
{
    unsigned int uiDesiredStat;
    if (uiErrorFlag == 1)
    {
        printf(" Error Detected  ");
        uiDesiredStat = CTL_WD_FAULT_RESET;
        modifyControlWord(uiDesiredStat);
    }

    switch (uirDriveStat)
    {
    case STATE_MACHINE_STAT_UNKNOWN:
        printf("Unknown               \n");
        break;
    case STATE_MACHINE_STAT_NOT_RDY_TO_SWITCH_ON:
        printf("Not Ready to Switch On\n");
        break;
    case STATE_MACHINE_STAT_SWITCH_ON_DISABLED:
        printf("Switch On Disabled    \n");
        break;
    case STATE_MACHINE_STAT_SWITCH_ON_DISABLED_WITH_INTERNAL_LIMIT_ACTIVE:
        printf("Switch On Disabled  IntLimAct \n");
        break;
    case STATE_MACHINE_STAT_RDY_TO_SWITCH_ON:
        printf("Ready to Switch On   \n");
        break;
    case STATE_MACHINE_STAT_SWITCHED_ON:
        printf("Switched On          \n");
        break;
    case STATE_MACHINE_STAT_OPERATION_ENABLED:
        // printf("OE\n");
        break;
    case STATE_MACHINE_STAT_FAULT:
        printf("Fault Detected       \n");
        break;
    case STATE_MACHINE_STAT_FAULT_REACTION_ACTIVE:
        printf("Fault Reaction Active\n");
        break;
    case STATE_MACHINE_STAT_QUICK_STOP_ACTIVE:
        printf("Quick Stop Active    \n");
        break;
    case STATE_MACHINE_STAT_SWITCH_ON_DISABLED_WITH_VE:
        printf("Switch ON disabled with VE\n");
        break;
    default:
        printf("Unknown              \n");
        break;
    }
    scanCntr++;
    printf("PAV: %x MdOfOpn:%d status:%x Pde:%x ipa: %d SC: %d ErCd: %X CW: %x MOS: %d Tq: %d\r", iPosActualValue.hl, ui8ModesOfOpnDisplay, uiStatusWd.hl, iDesiredPositionVal.hl, uiInterpolationActive, scanCntr, iErrCode.hl, uiCtlWd.hl, MotorOpnStatus, iTqActual.hl /*uiTargetReachedFlag*/);
}

void modifyControlWord(uint16 uirDesiredStat)
{

    // uiCtlWd.hl = 0;
    uint16 uiLclUpdateProfileVelocity = 0;

    switch (uirDesiredStat)
    {
    case CTL_WD_SHUT_DOWN:
        uiCtlWd.hl = 6;
        uiOpnEnabled = 0;
        break;

    case CTL_WD_SWITCH_ON:
        uiCtlWd.hl = 0x07;
        uiOpnEnabled = 0;

        break;

    case CTL_WD_DISABLE_VTG:
        uiCtlWd.hl = 0;
        uiOpnEnabled = 0;

        break;

    case CTL_WD_QUICK_STOP:
        uiCtlWd.split.h = 0;
        uiCtlWd.split.l |= BIT1;
        uiCtlWd.split.l &= ~BIT2;
        uiOpnEnabled = 0;

        break;

    case CTL_WD_DISABLE_OPN:
        uiOpnEnabled = 0;
        break;

    case CTL_WD_ENABLE_OPN:
        uiCtlWd.hl = 0x0F;
        if (uiOpnEnabled == 0)
            uiOpnEnabled = 1;
        break;

    case CTL_WD_PP_MODE:
        uiCtlWd.hl = 0x1F;
        break;
    case CTL_WD_ENABLE_IP:
        uiCtlWd.split.h = 0;
        uiCtlWd.split.l |= (BIT4 + BIT3 + BIT2 + BIT1 + BIT0);
        uiOpnEnabled = 0;

        break;

    case CTL_WD_FAULT_RESET:
        uiCtlWd.split.h = 0;
        uiCtlWd.split.l |= BIT7;
        uiOpnEnabled = 0;

        break;

    case CTL_WD_STOP:
        uiCtlWd.split.h = 0;
        uiCtlWd.split.l |= BIT8;
        uiOpnEnabled = 0;

        break;

    case CTL_WD_NEW_SET_POINT:
        uiCtlWd.hl = (uiCtlWd.hl | BIT4);
        uiLclUpdateProfileVelocity = 1;
        // Remove the following statements later! Done for compilation only!
        if (uiLclUpdateProfileVelocity == 1)
            uiLclUpdateProfileVelocity = 0;
        break;

    case CTL_WD_RESET_SET_POINT:
        uiCtlWd.hl &= ~BIT4;
        break;

    default:
        break;
    }
    modifyLatchControlWordValue(uiCtlWd.hl);
}

void setSoftwarePositionLimit(uint16 uirSlave, int32 irSW_PosnLimit1, int32 irSW_PosnLimit2)
{
    // Software position limit
    uint32 uilclRetval, ui32Val;
    uint16 ui16Val;
    uilclRetval = 0;

    // Setting 0 to these values will result in disabling of the software limit
    //  pg. 106 of SX-DSV03242 R6 document
    uilclRetval += ec_SDOwrite(uirSlave, REG_PROF_POSN_MODE_SW_POSN_LIMIT, 0x01, FALSE, sizeof(irSW_PosnLimit1), &irSW_PosnLimit1, EC_TIMEOUTRXM); // Make this and the following command 0 to disable sw position limit, refer pg. 95 of the Ethercat document
    uilclRetval += ec_SDOwrite(uirSlave, REG_PROF_POSN_MODE_SW_POSN_LIMIT, 0x02, FALSE, sizeof(irSW_PosnLimit2), &irSW_PosnLimit2, EC_TIMEOUTRXM);

    ui32Val = 55924060; // 32 rev/min for calculations see #Panasonic/Servo/encoder
    uilclRetval += ec_SDOwrite(uirSlave, REG_PROF_POSN_MODE_MAX_PROFILE_VELOCITY, 0x00, FALSE, sizeof(ui32Val), &ui32Val, EC_TIMEOUTRXM);

    ui32Val = 3500; // Since 10-30 rpm is allowed. and gear ratio is 10.
    uilclRetval += ec_SDOwrite(uirSlave, REG_PROF_POSN_MODE_MAX_MOTOR_SPEED, 0x00, FALSE, sizeof(ui32Val), &ui32Val, EC_TIMEOUTRXM);

    ui32Val = 0xF000000;//2796202; // #Panasonic/Servo/encoder
    uilclRetval += ec_SDOwrite(uirSlave, REG_PROF_POSN_MODE_MAX_ACCELERATION, 0x00, FALSE, sizeof(ui32Val), &ui32Val, EC_TIMEOUTRXM);

    ui32Val = 0xC000000; //2796202; // #Panasonic/Servo/encoder
    uilclRetval += ec_SDOwrite(uirSlave, REG_PROF_POSN_MODE_MAX_DECELERATION, 0x00, FALSE, sizeof(ui32Val), &ui32Val, EC_TIMEOUTRXM);

    ui16Val = 0x04;
    uilclRetval += ec_SDOwrite(uirSlave, 0x3015, 0x00, FALSE, sizeof(ui16Val), &ui16Val, EC_TIMEOUTRXM);


    ui16Val = 50; // 1000
    uilclRetval += ec_SDOwrite(uirSlave, REG_PROF_POSN_MODE_MAX_TORQUE, 0x00, FALSE, sizeof(ui16Val), &ui16Val, EC_TIMEOUTRXM);
}

void modifyLatchControlWordValue(uint16 uirLatchCtlWd)
{
    union
    {
        uint16 hl;
        struct
        {
            uint8 l;
            uint8 h;
        } split;
    } uiSplitVar;

    uiSplitVar.hl = uirLatchCtlWd;

    *(ec_slave[0].outputs + OUTPUT_OFFSET_CTLWD) = uiSplitVar.split.l;
    *(ec_slave[0].outputs + (OUTPUT_OFFSET_CTLWD + 1)) = uiSplitVar.split.h;
}

void DriveEnable()
{
    uint16 uiDesiredStat = 0;
    uint32 uiLclDoNothingFlag = 0;
    // int size;
    // int retval;
    // int32 i32val;
    /*union {
        uint16 hl;
        struct {
            uint8 l;
            uint8 h;
        }split;
    }uiMaxTq;*/
    // retval = 0;
    switch (uiDriveStatus)
    {

    case STATE_MACHINE_STAT_SWITCH_ON_DISABLED: // Nothing can be done over Ethercat to change the state!
        // printf("A");
        if (getReasonForLossOfOperationalMode == 1)
        {
            getReasonForLossOfOperationalMode = 0;
            // printf("StatusJustBeforeLoss:%x\n", uiDelMeStatus);
        }
        uiDesiredStat = CTL_WD_SHUT_DOWN;
        // uiDesiredStat = CTL_WD_SWITCH_ON;
        break;
        /*case STATE_MACHINE_STAT_SWITCH_ON_DISABLED_WITH_INTERNAL_LIMIT_ACTIVE:
            uiMaxTq.hl = 1000; //Resolution is 0.1% so 1000 = 100%
            *(ec_slave[0].outputs + OUTPUT_OFFSET_MAX_TQ) = uiMaxTq.split.h;
            *(ec_slave[0].outputs + (OUTPUT_OFFSET_MAX_TQ + 1)) = uiMaxTq.split.l;
            break;
            */
    case STATE_MACHINE_STAT_RDY_TO_SWITCH_ON:
        // printf("B");
        uiDesiredStat = CTL_WD_SWITCH_ON;
        break;
    case STATE_MACHINE_STAT_SWITCHED_ON:
        // printf("C");
        uiDesiredStat = CTL_WD_ENABLE_OPN;
        break;
    case STATE_MACHINE_STAT_OPERATION_ENABLED:
        // printf("X");
        // uiDesiredStat = CTL_WD_PP_MODE;
        // uiLclDoNothingFlag = 1;
        // getReasonForLossOfOperationalMode = 1;

        /*
        uiMaxTorque.hl = 100;

        *(ec_slave[0].outputs + OUTPUT_OFFSET_TARGET_TQ) = uiMaxTorque.split.l; //3
        *(ec_slave[0].outputs + (OUTPUT_OFFSET_TARGET_TQ + 1)) = uiMaxTorque.split.h; //4

        *(ec_slave[0].outputs + OUTPUT_OFFSET_MAX_TQ) = uiMaxTorque.split.l; //5
        *(ec_slave[0].outputs + (OUTPUT_OFFSET_MAX_TQ + 1)) = uiMaxTorque.split.h; //6

        *(ec_slave[0].outputs + OUTPUT_OFFSET_TARGET_POSN) = iPosActualValue.split.ll; //7
        *(ec_slave[0].outputs + (OUTPUT_OFFSET_TARGET_POSN + 1)) = iPosActualValue.split.lh; //8
        *(ec_slave[0].outputs + (OUTPUT_OFFSET_TARGET_POSN + 2)) = iPosActualValue.split.hl; //9
        *(ec_slave[0].outputs + (OUTPUT_OFFSET_TARGET_POSN + 3)) = iPosActualValue.split.hh; //10


        //Max motor speed
        *(ec_slave[0].outputs + 11) = 0;
        *(ec_slave[0].outputs + 12) = 0;
        *(ec_slave[0].outputs + 13) = 0;
        *(ec_slave[0].outputs + 14) = 0;

        //Target velocity
        *(ec_slave[0].outputs + 15) = 0;
        *(ec_slave[0].outputs + 16) = 0;
        *(ec_slave[0].outputs + 17) = 0;
        *(ec_slave[0].outputs + 18) = 0;

        //uiDesiredStat = CTL_WD_NEW_SET_POINT;


        */

        // Step 1: Set the target position (607A)
        // Step 2:
        /*if (uiInterpolationEnableFlag == 1)
        {
            uiDesiredStat = CTL_WD_ENABLE_IP;
        }
        else
        {
            uiDesiredStat = CTL_WD_ENABLE_OPN;
        }*/
        break;

    case STATE_MACHINE_STAT_QUICK_STOP_ACTIVE:
    case STATE_MACHINE_STAT_NOT_RDY_TO_SWITCH_ON: // The drive is booting up Wait!
                                                  // printf("E1");
        break;

    case STATE_MACHINE_STAT_FAULT:
        // printf("F");
        uiDesiredStat = CTL_WD_FAULT_RESET;
        break;

    case STATE_MACHINE_STAT_SWITCH_ON_DISABLED_WITH_VE:
        // printf("G");
        uiDesiredStat = CTL_WD_DISABLE_VTG; // CTL_WD_SWITCH_ON;//CTL_WD_RESET_VE;

        break;

    case STATE_MACHINE_STAT_UNKNOWN:
    case STATE_MACHINE_STAT_FAULT_REACTION_ACTIVE:
    default:
        // printf("H");
        uiDesiredStat = CTL_WD_SWITCH_ON;
        break;
    }
    if (uiLclDoNothingFlag == 0)
    {
        modifyControlWord(uiDesiredStat);
    }

    //*(ec_slave[0].outputs + 1) = 0;
}

void StopDrive()
{
    uint16 uiDesiredStat = CTL_WD_STOP;
    modifyControlWord(uiDesiredStat);
}

void ShutDownDrive()
{
    uint16 uiDesiredStat = CTL_WD_SHUT_DOWN;
    modifyControlWord(uiDesiredStat);
}

void setOutputPDO(uint16 slave, uint16 uirOutputPDO)
{
    uint8 u8val;
    uint16 u16val;
    int retval = 0;
    // Refer Pg. 36 of AKD Ethercat Communication Edition K, December 2014
    // Output Bytes in the frame are set by this
    u8val = 0;
    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM); // Mapping Section is cleared
    u16val = uirOutputPDO;                                                                   // Desired Value presently is thought to be 1722
    retval += ec_SDOwrite(slave, 0x1c12, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    u8val = 1;
    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
}

void setInputPDO(uint16 slave, uint16 uirInputPDO)
{
    // Refer Pg. 36 of AKD Ethercat Communication Edition K, December 2014
    // Input Bytes in the frame are set by this
    uint8 u8val;
    uint16 u16val;
    int retval = 0;
    u8val = 0;
    retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM); // Mapping Section is cleared
    u16val = uirInputPDO;
    retval += ec_SDOwrite(slave, 0x1c13, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    u8val = 1;
    retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
}

// Panasonic Specific
void setDCModeSetup(uint16 uirSlave)
{
    int retval = 0;
    uint16 u16val;
    uint8 u8val;
    // uint32 u32val;
    // int32 i32val;
    // int16 i16val;
    // Refer Pg. 7 of the Ethercat document of Panasonic
    u16val = 2;
    retval += ec_SDOwrite(uirSlave, 0x1C32, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);

    u32val = 10000000;
    retval += ec_SDOwrite(uirSlave, 0x1C32, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    u16val = 2;
    retval += ec_SDOwrite(uirSlave, 0x1C33, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);

    u32val = 0;
    retval += ec_SDOwrite(uirSlave, 0x1C33, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    u8val = 1;
    retval += ec_SDOwrite(uirSlave, 0x6060, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
}

void setFlexibleOutputPDO(uint16 uirSlave)
{

    // Remember: Set Values PDO cannot exceed 22 bytes
    uint8 u8val;
    uint16 u16val;
    uint32 u32val;
    int retval = 0;
    u8val = 0;

    // Mapping Section is cleared
    retval += ec_SDOwrite(uirSlave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    // Clear Output PDO's
    ec_SDOwrite(uirSlave, 0x1600, 0, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    ec_SDOwrite(uirSlave, 0x1601, 0, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    ec_SDOwrite(uirSlave, 0x1602, 0, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    ec_SDOwrite(uirSlave, 0x1603, 0, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    // Map the fields here using the following logic:
    // The meaning of the data (for example 0x60410010 in the mapping of 0x1A00 sub 1) is as follows:
    // 0x6041 is the index of the DS402 status word
    // 0x00 is the subindex of the DS402 status word
    // 0x10 is the number of bits for this entry, i. e. 16 bits or 2 bytes.
    u32val = 0x60400010; // Controlword
    ec_SDOwrite(uirSlave, 0x1600, 1, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    u32val = 0x60600008; // Modes of operation
    ec_SDOwrite(uirSlave, 0x1600, 2, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    u32val = 0x60810020; // Profile Velocity
    ec_SDOwrite(uirSlave, 0x1600, 3, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    u32val = 0x60720010; // Max Torque
    ec_SDOwrite(uirSlave, 0x1600, 4, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    u32val = 0x607A0020; // Target Position
    ec_SDOwrite(uirSlave, 0x1600, 5, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    u32val = 0x60710010; // Target Torque
    ec_SDOwrite(uirSlave, 0x1600, 6, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    // 1600.0 ==> Inform the number of variables there.
    u8val = 6; // Since 1600 maps only 6 Variables
    ec_SDOwrite(uirSlave, 0x1600, 0, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    // Map PDO 1600 to RxPDO Assign
    // This makes the 1600 PDO mapping acitve
    // Set the number of PDO's to 1
    // 1C12.0 ==> Number of assigned PDOs
    u8val = 1;
    ec_SDOwrite(uirSlave, 0x1C12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    u16val = 0x1600;
    ec_SDOwrite(uirSlave, 0x1C12, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
}

void setFlexibleInputPDO(uint16 uirSlave)
{

    // Remember: Actual Values PDO cannot exceed 32 bytes
    uint8 u8val;
    uint16 u16val;
    uint32 u32val;
    int retval = 0;
    u8val = 0;

    // Mapping Section is cleared
    retval += ec_SDOwrite(uirSlave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    // Clear Input PDO's
    ec_SDOwrite(uirSlave, 0x1A00, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    ec_SDOwrite(uirSlave, 0x1A01, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    ec_SDOwrite(uirSlave, 0x1A02, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    ec_SDOwrite(uirSlave, 0x1A03, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    // Map the fields here using the following logic:
    // The meaning of the data (for example 0x60410010 in the mapping of 0x1A00 sub 1) is as follows:
    // 0x6041 is the index of the DS402 status word
    // 0x00 is the subindex of the DS402 status word
    // 0x10 is the number of bits for this entry, i. e. 16 bits or 2 bytes.

    // Error code 603F0010h
    u32val = 0x603F0010;
    ec_SDOwrite(uirSlave, 0x1A00, 1, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    // Statusword 60410010h
    u32val = 0x60410010;
    ec_SDOwrite(uirSlave, 0x1A00, 2, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    // Modes of operation display 60610008h
    u32val = 0x60610008;
    ec_SDOwrite(uirSlave, 0x1A00, 3, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    // Position actual value 60640020h
    u32val = 0x60640020;
    ec_SDOwrite(uirSlave, 0x1A00, 4, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    // Velocity actual value 606C0020h
    u32val = 0x606C0020;
    ec_SDOwrite(uirSlave, 0x1A00, 5, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    // Torque actual value 60770010h
    u32val = 60770010;
    ec_SDOwrite(uirSlave, 0x1A00, 6, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    // Enter the number of PDO's in each variable

    // printf("\n Enable PDO 0x1A00");
    u8val = 6; // Since it maps 2 variables
    ec_SDOwrite(uirSlave, 0x1A00, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    // Set the number of PDO's to
    u8val = 0x01;
    ec_SDOwrite(uirSlave, 0x1C13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    u16val = 0x1A00;
    ec_SDOwrite(uirSlave, 0x1C13, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
}

unsigned char checkInterPolationDone()
{
    unsigned char ucRetVar;
    ucRetVar = 0;

    return ucRetVar;
}

void drv_SetOperationMode(uint16 slave, int32 irSelectedMode)
{
    int32 iOperatingMode = 0;
    // uint8 u16ErrCntr = 0;
    int8 iModeOfOpnParam = 0; // This variable value is being determined on the pg. 45 of AKD Ethercat Communication Edition K, December 2014
    // int retval = 0;
    uint8 uiRetArr[10];
    // int l = sizeof(uiRetArr) - 1;
    memset(&uiRetArr, 0, 10);

    switch (irSelectedMode)
    {
    case OPN_MODE_PROFILE_VELOCITY:
        iModeOfOpnParam = 3;
        iOperatingMode = 1;
        break;
    case OPN_MODE_INTERPOLATED_POSN:
        iModeOfOpnParam = 7;
        iOperatingMode = 2;
        uiInterpolationEnableFlag = 1;
        break;
    case OPN_MODE_HOMING_MODE:
        iModeOfOpnParam = 6;
        iOperatingMode = 2;
        break;
    case OPN_MODE_PROFILE_POSN:
        iModeOfOpnParam = 1;
        iOperatingMode = 2;
        break;
    case OPN_MODE_PROFILE_TORQUE:
        iModeOfOpnParam = 4;
        iOperatingMode = 1;
        break;
    case OPN_MODE_CYCLIC_SYNCH_POSN:
        iModeOfOpnParam = 8;
        iOperatingMode = 1;
        break;
    case OPN_MODE_VELOCITY:
        iModeOfOpnParam = 2;
        break;
    case OPN_MODE_CYCLIC_SYNCH_VELOCITY:
        iModeOfOpnParam = 9;
        break;
    case OPN_MODE_CYCLIC_SYNCH_TORQUE:
        iModeOfOpnParam = 10;
        break;
    default:
        break;
    }
    // printf("\n\n\n\n\n\n\n\n\n\n Value of iModeOfOpnParam: %d", iModeOfOpnParam);
    printf("Operational mode: %d", iOperatingMode);

    if (iModeOfOpnParam != 0)
    {
        // REG_MODE_OF_OPN = 0x6060
        ec_slave[0].outputs[OUTPUT_OFFSET_MODE_OF_OPN] = iModeOfOpnParam;
        // retval += ec_SDOwrite(slave, REG_MODE_OF_OPN, 0x00, FALSE, sizeof(iModeOfOpnParam), &iModeOfOpnParam, EC_TIMEOUTRXM);
    }
    else
    {
        printf("FAILED TO SET THE MODE.... NOT SUPPORTED!!!");
    }
    printf("\n Slave param: %d", slave);
}

unsigned int ReadInteger(FILE *fp, unsigned int uiOffsetFromBegin)
{
    unsigned int readInt;
    fseek(fp, uiOffsetFromBegin, SEEK_SET);
    fread(&readInt, sizeof(readInt), 1, fp);
    return readInt;
}

void StoreInteger(FILE *fp, unsigned int IntegerToStore)
{
    fwrite(&IntegerToStore, sizeof(IntegerToStore), 1, fp);
}

float ReadFloat(FILE *fp, unsigned int uiOffsetFromBegin)
{
    float readFloat;
    fseek(fp, uiOffsetFromBegin, SEEK_SET);
    fread(&readFloat, sizeof(readFloat), 1, fp);
    return readFloat;
}

void StoreFloat(FILE *fp, float floatToStore)
{
    fwrite(&floatToStore, sizeof(floatToStore), 1, fp);
}

// Comment
void resetDesiredTqAndDegOfRtn()
{
    uiDesiredDegreeOfRtn = 0;
    fDesiredTq = 0;
    uiDesiredStatus = 0;
    uiModifyCmdStatusFlag = 1;
    uiDesiredDirectionOfRtn = 0;
    uiDesiredRPM = 0;
    printf("Initial setting completed...\n");
}

void GetDesiredTqAndDegOfRtn()
{
    /*
    //To convert string to integer use atoi function
    //To convert string to float use atof function
    //strok to extract the string
    char cmdStr[] = "CTD";
    char cmdNoActionStr[] = "CTR";
    char** tokens;

    printf("Original String from client: %s", recvbuf);

    if (strncmp(recvbuf, cmdStr, 3) == 0) {
        printf("CTD Rcvd\n");
        tokens = str_split(recvbuf, ',');
        if (tokens) {
            int lclCntr;
            //printf("This is the no. of splits: %d", *(tokens + lclCntr));
            for (lclCntr = 0; *(tokens + lclCntr); lclCntr++) {
                switch (lclCntr) {
                case 0:
                    break;
                case 1:
                    uiDesiredDegreeOfRtn = atoi(*(tokens + lclCntr));
                    printf("uiDesiredDegreeOfRtn: %d\n", uiDesiredDegreeOfRtn);
                    break;
                case 2:
                    fDesiredTq = (float)(atof(*(tokens + lclCntr)));
                    printf("fDesiredTq : %f\n", fDesiredTq);
                    break;
                case 3:
                    uiDesiredRPM = atoi(*(tokens + lclCntr));
                    printf("uiDesiredRPM: %d\n", uiDesiredRPM);
                    break;
                case 4:
                    uiDesiredDirectionOfRtn = atoi(*(tokens + lclCntr));
                    printf("uiDesiredDirectionOfRtn: %d\n", uiDesiredDirectionOfRtn);
                    break;
                case 5:
                    uiDesiredStatus = atoi(*(tokens + lclCntr));
                    printf("uiDesiredStatus : %d\n", uiDesiredStatus);
                    break;
                default:
                    break;
                }
                free(*(tokens + lclCntr));
            }
            free(tokens);
        }
        ValidateInteger(&uiDesiredStatus, 0, 6, 0);
        if (uiDesiredStatus != 0)
            uiModifyCmdStatusFlag = 1;
    }
    else {
        if (strncmp(recvbuf, cmdNoActionStr, 3) == 0) {
            printf("...No action desired");
        }
        else {
            printf("Invalid Cmd received");
        }
    }
    */
}

void SetActualTqAndPosn()
{

    if (uiDesiredStatus == CMD_STAT_COMPLETED_POSN)
    {
        uiStatusResetCntr++;
        if (uiStatusResetCntr > 3)
        {
            uiStatusResetCntr = 0;
            uiDesiredStatus = CMD_STAT_UNKNOWN;
        }
    }

    char stringToTx[100];
    printf("\nActPosn: %d, ACT_TQ:%d, DesStat: %d", uiActualPosn, uiActualTq, uiDesiredStatus);
    sprintf(stringToTx, "STP,%d,%d,%d", uiActualPosn, uiActualTq, uiDesiredStatus);
    printf("\n Calling fn SocketSendResponse...");
    // SocketSendResponse(stringToTx);
}

/*
void SetCommandStatus(UINT32 uirCmdStatus)
{
    unsigned int uilclErrNo = 0;
    unsigned int uiPrintCntr = 0;
    char txStr[100];
    sprintf(txStr, "SCS,%d", uirCmdStatus);
    printf("\n StatUpdated: %d", uirCmdStatus);
}
*/

void ConvertDegreeOfRotationToCount(uint32 uirRPM, uint32 uirScanIntervalInMsec, uint32 uirDegreesToRotate)
{
    float fLclVar;
    float fDegreesPerScanInterval = 0;
    // Refer Sheet 2 of SOEM Calculations Sheet 2 in SOEM-Calculations folder
    // Step 1: Convert RPM to RPS
    fLclVar = (float)uirRPM / 60;
    printf("\n Step 1: %f", fLclVar);
    // Step 2: Convert RPS to Degrees in One Second
    fLclVar = fLclVar * 360;
    printf("\n Step 2: %f", fLclVar);

    // Step 3: Convert Degrees in One Second to Degrees in scan Interval
    fLclVar = fLclVar * ((float)uirScanIntervalInMsec / 1000);
    fDegreesPerScanInterval = fLclVar;
    printf("\n Step 3: %f", fLclVar);

    // Step 4: Count to be traversed in 1 Scan interval
    fLclVar = (float)(((fLclVar * pow(2, ENCODER_RESOLUTION_IN_BITS) * GEAR_RATIO)) / 360);
    uiRotationOffset = (uint32)fLclVar;
    printf("\n Step 4: %f", fLclVar);

    // Step 5: Steps required for completing the motion
    ui32StepsExecuted = 0;
    ui32StepsProgramed = (int32)((float)uirDegreesToRotate / fDegreesPerScanInterval);
    ui32StepsProgramed = ui32StepsProgramed /* + (int)(0 * (double)ui32StepsProgramed)*/;
    // Debug: Reducing for debug purposes
    // ui32StepsProgramed *= 2;
    // uiRotationOffset = uiRotationOffset / 1000;

    printf("\n********************** Steps Programed: %d", ui32StepsProgramed);
    printf("\n**************** Rotation Offset: %d", uiRotationOffset);
}

UINT32 ValidateInteger(UINT32 *uirIntegerToValidate, UINT32 uirIntegerMinVal, UINT32 uirIntegerMaxVal, UINT32 uirFillValIfInvalid)
{
    UINT32 uiRetVar = 1;
    if (*uirIntegerToValidate < uirIntegerMinVal)
    {
        //*uirIntegerToValidate = uirFillValIfInvalid;
        uiRetVar = 0;
    }
    if (*uirIntegerToValidate > uirIntegerMaxVal)
    {
        //*uirIntegerToValidate = uirFillValIfInvalid;
        uiRetVar = 0;
    }
    if (uiRetVar == 0)
    {
        *uirIntegerToValidate = uirFillValIfInvalid;
    }
    return uiRetVar;
}

UINT32 ValidateFloat(float *frFloatToValidate, float frFloatMinVal, float frFloatMaxVal, float frFillValIfInvalid)
{
    UINT32 uiRetVar = 1;
    if (*frFloatToValidate < frFloatMinVal)
    {
        //*frFloatToValidate = frFillValIfInvalid;
        uiRetVar = 0;
    }
    if (*frFloatToValidate > frFloatMaxVal)
    {
        //*frFloatToValidate = frFillValIfInvalid;
        uiRetVar = 0;
    }

    if (uiRetVar == 0)
    {
        *frFloatToValidate = frFillValIfInvalid;
    }
    return uiRetVar;
}

int32 calculateFinalDesiredPosn(uint32 ui32rDesRtn)
{
    int32 retVal;
    printf("Total Steps:%d", ui32StepsProgramed);
    printf("Rotation Offset:%d", uiRotationOffset);

    uint32 uLclTotalSteps = ui32StepsProgramed * uiRotationOffset;
    if (ui32rDesRtn == CW)
    {
        retVal = iPosActualValue.hl - uLclTotalSteps;
    }
    else
    {
        retVal = iPosActualValue.hl + uLclTotalSteps;
    }

    return retVal;
}

int PanasonicSetup(uint16 slave)
{
    int retval;
    // uint8 u8val;

    // uint32 uiRet32Arr[10];
    printf("\n This is the slave parameter: %d", slave);
    // int l32 = sizeof(uiRet32Arr) - 1;
    // memset(&uiRet32Arr, 0, 10);
    retval = 0;
    // u8val = 0;
    setDCModeSetup(slave);
    setSoftwarePositionLimit(slave, 0, 0);
    setFlexibleOutputPDO(slave);
    setFlexibleInputPDO(slave);

    while (EcatError)
        printf("%s", ec_elist2string());

    printf("Panasonic Servo slave %d set, retval = %d\n", slave, retval);
    return 1;
}

/* most basic RT thread for process data, just does IO transfer */

int64 cntr = 0;

OSAL_THREAD_FUNC_RT RTthread()
{
    struct timespec ts;
    struct timespec tleft;
    int ht;
    int64 cycletime;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    ht = (ts.tv_nsec / 1000000) + 1; // round to nearest ms
    ts.tv_nsec = ht * 1000000;
    if (ts.tv_nsec >= NSEC_PER_SEC)
    {
        ts.tv_sec++;
        ts.tv_nsec -= NSEC_PER_SEC;
    }
    cycletime = SCAN_INTERVAL_IN_USEC * 1000; // cycletime in ns
    toff = 0;
    dorun = 0;
    ec_send_processdata();

    while (1)
    {
        add_timespec(&ts, cycletime + toff);
        /* wait to cycle start */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);

        if (wkc < expectedWKC)
        {
            // printf("\n Need to handle ERROR. Erroneous data received!");
        }
        else
        {
            dataRdyForXtraction = 1;
            delMeOpnCntr++;
            if (delMeOpnCntr > 5000)
            {
                delMeOpnExpectedFlag = 1;
                delMeOpnCntr = 0;
            }
        }
    }
}

OSAL_THREAD_FUNC ecatcheck()
{
    int slave;
    while (1)
    {
        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                // printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state > EC_STATE_NONE)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n", slave);
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[slave].state == EC_STATE_NONE)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (ec_slave[slave].state == EC_STATE_NONE)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n", slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(SCAN_INTERVAL_IN_MSEC * 10000);
    }
}

void simpletest(char *ifname)
{ // 1
    // int j;
    int32 oloop, iloop, slc;
    // int32 chk, i;
    // int32 chkCntr, wkc_count;
    // uint32 mmResult;
    // chkCntr = 0;
    needlf = FALSE;
    inOP = FALSE;
    int chk = 400;
    //   int delMeIteration = 0;
    //  int size;
    //   int32 i32val;
    //    int16 i16val;

    int iomap_size;

    uint16_t sync_mode;
    uint32_t cycle_time;
    uint32_t minimum_cycle_time;
    uint32_t sync0_cycle_time;
    int ret = 0, l;
    int lclDlyCntr = 0;
    int32 modifiedPAV = 0;

    struct
    {
        uint32 uiProfileVelocity;
        int16 iMaxTq;
        int32 iTgtPosn;
    } motionParams;

    printf("Starting simple test\n");
    resetDesiredTqAndDegOfRtn();

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    { // 2
        printf("ec_init on %s succeeded.\n", ifname);
        /* find and auto-config slaves */

        if (ec_config_init(FALSE) > 0)
        { // 3
            printf("%d slaves found and configured.\n", ec_slavecount);

            if ((ec_slavecount >= 1)) // Earlier it was only >1, thus this loop was not getting executed!
            {                         // 4
                for (slc = 1; slc <= ec_slavecount; slc++)
                { // 5

                    // Panasonic Ethercat Servo Motor
                    if ((ec_slave[slc].eep_man == MANUFACTURER_ID) && (ec_slave[slc].eep_id == PRODUCT_ID))
                    { // 6
                        printf("It gives us great pleasure to inform you that Panasonic Ethercat Servo has been found!!!\n");
                        printf("Found %s at position %d\n", ec_slave[slc].name, slc);
                        // link slave specific setup to preop->safeop hook
                        ec_slave[1].PO2SOconfig = &PanasonicSetup;

                    } //\6
                }     //\5
            }         //\4
            else
            { // 4
                printf("Could not enter the if loop\n");
            } //\4
            iomap_size = ec_config_map(&IOmap);
            printf("SOEM IOMap size: %d\n", iomap_size); // Returns 36
            ec_configdc();                               // Configure distributed Clocks
            // ec_dcsync0(1, TRUE, 2000000U, 0);

            printf("Slaves mapped, state to SAFE_OP.\n");
            /* wait for all slaves to reach SAFE_OP state */
            if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4) == EC_STATE_SAFE_OP)
            { // 4
                printf("Slave is in safe operational mode.\n");
            } //\4

            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0))
                oloop = 1; // If less than 8 bits assign 1 byte
            if (oloop > 8)
                oloop = 8; // Not allowed more than 8 bytes

            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0))
                iloop = 1; // If less than 8 bits assign 1 byte
            if (iloop > 8)
                iloop = 8; // Not allowed more than 8 bytes
            printf("oloop: %d, iloop: %d \n", oloop, iloop);

            printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

            ec_slave[0].state = EC_STATE_OPERATIONAL;
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);

            printf("Request operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;

            // request OP state for all slaves
            ec_writestate(0);
            // wait for all slaves to reach OP state
            do
            { // 4
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL)); //\6

            if (ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) != EC_STATE_OPERATIONAL)
            { // 4
                fprintf(stderr, "OPERATIONAL state not set, exiting\n");
            } //\4
            else
            {
                ec_readstate();

                // ideally should run a loop for checking the number of slaves
                // taking shortcut since there is only 1 slave in our project
                printf("\nSlave: 1\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
                       ec_slave[1].name, ec_slave[1].Obits, ec_slave[1].Ibits,
                       ec_slave[1].state, ec_slave[1].pdelay, ec_slave[1].hasdc);
                if (ec_slave[1].hasdc)
                    printf(" DCParentport:%d\n", ec_slave[1].parentport);
                printf(" Activeports:%d.%d.%d.%d\n", (ec_slave[1].activeports & 0x01) > 0,
                       (ec_slave[1].activeports & 0x02) > 0,
                       (ec_slave[1].activeports & 0x04) > 0,
                       (ec_slave[1].activeports & 0x08) > 0);
                printf(" Configured address: %4.4x\n", ec_slave[1].configadr);

                l = sizeof(sync_mode);
                // ideally 1st parameter should be filled programatically. Another shortcut!
                ret += ec_SDOread(1, 0x1c32, 0x01, FALSE, &l, &sync_mode, EC_TIMEOUTRXM);
                l = sizeof(cycle_time);
                ret += ec_SDOread(1, 0x1c32, 0x02, FALSE, &l, &cycle_time, EC_TIMEOUTRXM);
                l = sizeof(minimum_cycle_time);
                ret += ec_SDOread(1, 0x1c32, 0x05, FALSE, &l, &minimum_cycle_time, EC_TIMEOUTRXM);
                l = sizeof(sync0_cycle_time);
                ret += ec_SDOread(1, 0x1c32, 0x0a, FALSE, &l, &sync0_cycle_time, EC_TIMEOUTRXM);

                // Write the limiting parameters for PP Mode

                printf("PDO syncmode %02x, cycle time %d ns (min %d), sync0 cycle time %d ns, ret = %d\n", sync_mode, cycle_time, minimum_cycle_time, sync0_cycle_time, ret);

                printf("\n*********************Finished configuration successfully\n");

                // Cycle worker thread (in our case RTthread) needs to be started here.
                osal_thread_create_rt(&thread1, stack64k * 2, &RTthread, 0); //&ctime was being passed here in the original code. scan time was being passed as an argument. Our scan time is fixed to 2 ms so no need to send the parameter
                osal_thread_create(&thread2, stack64k * 4, &ecatcheck, 0);

                printf("\n*********Thread Created************");

                // start the acyclic part here
                while (1)
                {

                    if (dataRdyForXtraction == 1)
                    {
                        // printf("\n AA: %d", dataRdyForXtraction );
                        dataRdyForXtraction = 0;
                        extractFlexibleInputPDO_data();
                        DriveEnable();
                    }

                    if (delMeOpnExpectedFlag == 1)
                    {
                        if (MotorOpnStatus == 0)
                        {
                            MotorOpnStatus = 1;
                            delMeOpnExpectedFlag = 0;
                        }
                    }

                    if (MotorOpnStatus == 1)
                    {

                        printf("\n Operate Motor");
                        modifiedPAV = (iPosActualValue.hl /*& 0x80FFFFFF*/);
                        printf("\nOriginal PAV: %x\n",iPosActualValue.hl);
                        printf("Modified PAV: %x\n",modifiedPAV);
                        motionParams.iTgtPosn = modifiedPAV - 83886080;
                        motionParams.uiProfileVelocity = 0x8E38C0;
                        motionParams.iMaxTq = 1000; // 20%
                        fillMotionParams(motionParams.uiProfileVelocity, motionParams.iMaxTq, motionParams.iTgtPosn);
                        printf("Current Position: %d\n", iPosActualValue.hl);
                        printf("Desired Position: %d\n", motionParams.iTgtPosn);

                        MotorOpnStatus = 2;
                        lclDlyCntr = scanCntr + 200;
                    }

                    if(MotorOpnStatus == 2){
                        
                        if(scanCntr > lclDlyCntr){
                            modifyControlWord(CTL_WD_NEW_SET_POINT);
                            printf("\n\n\n\n\n**************SET POINT*************\n");
                            printf("PAV: %x MdOfOpn:%d status:%x Pde:%x ipa: %d SC: %d ErCd: %X CW: %x MOS: %d\n", iPosActualValue.hl , ui8ModesOfOpnDisplay, uiStatusWd.hl, iDesiredPositionVal.hl, uiInterpolationActive, scanCntr, iErrCode.hl, uiCtlWd.hl, MotorOpnStatus /*uiTargetReachedFlag*/);
                            MotorOpnStatus = 3;
                            lclDlyCntr = scanCntr + 200;
                        }
                    }

                    if(MotorOpnStatus == 3){
                        if (scanCntr > lclDlyCntr)
                        {
                            if ((uiStatusWd.hl & BIT12) != 0)
                            {
                                MotorOpnStatus = 4;
                                modifyControlWord(CTL_WD_RESET_SET_POINT);
                                printf("\n\n\n\n\n**************RESET POINT*************\n");
                                printf("PAV: %x MdOfOpn:%d status:%x Pde:%x ipa: %d SC: %d ErCd: %X CW: %x MOS: %d\n", iPosActualValue.hl , ui8ModesOfOpnDisplay, uiStatusWd.hl, iDesiredPositionVal.hl, uiInterpolationActive, scanCntr, iErrCode.hl, uiCtlWd.hl, MotorOpnStatus /*uiTargetReachedFlag*/);

                                lclDlyCntr = scanCntr + 200;
                            }
                            else
                            {
                                printf("\n Status wd: %x", uiStatusWd.hl);
                            }
                        }
                    }

                    if (MotorOpnStatus == 4){
                        if (scanCntr > lclDlyCntr){
                            if((uiStatusWd.hl & BIT10) == 0){
                                MotorOpnStatus = 5;
                                printf("\n\n\n\n\n**************Target Reached RESET*************");
                                printf("PAV: %x MdOfOpn:%d status:%x Pde:%x ipa: %d SC: %d ErCd: %X CW: %x MOS: %d\n", iPosActualValue.hl , ui8ModesOfOpnDisplay, uiStatusWd.hl, iDesiredPositionVal.hl, uiInterpolationActive, scanCntr, iErrCode.hl, uiCtlWd.hl, MotorOpnStatus /*uiTargetReachedFlag*/);
                                lclDlyCntr = scanCntr + 200;
                            }
                        }
                    }

                    if (MotorOpnStatus == 5)
                    {
                        if (scanCntr > lclDlyCntr)
                        {
                            if((uiStatusWd.hl & BIT10) != 0) 
                            {
                                MotorOpnStatus = 0;
                                printf("\n\n\n\n\n**************Target Reached*************");
                                printf("PAV: %x MdOfOpn:%d status:%x Pde:%x ipa: %d SC: %d ErCd: %X CW: %x MOS: %d\n", iPosActualValue.hl , ui8ModesOfOpnDisplay, uiStatusWd.hl, iDesiredPositionVal.hl, uiInterpolationActive, scanCntr, iErrCode.hl, uiCtlWd.hl, MotorOpnStatus /*uiTargetReachedFlag*/);
                            }
                        }
                        // if((uiStatusWd.hl && BIT12) == 0){
                        //     MotorOpnStatus = 5;
                        //     printf("\n\n\n\n\n**************SET POINT ACK Reset*************");
                        // }
                    }
                }
            }

        } //\2
    }     //\3

} //\1

int main(int argc, char *argv[])
{
    printf("Endurance 2\n");
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");
    printf("argc:%d \n", argc);

    if (argc > 1)
    {
        // start cyclic part
        simpletest(argv[1]);
    }
    else
    {
        ec_adaptert *adapter = NULL;
        printf("Usage: simple_test ifname1\nifname = eth0 for example\n");

        printf("\nAvailable adapters:\n");
        adapter = ec_find_adapters();
        while (adapter != NULL)
        {
            printf("    - %s  (%s)\n", adapter->name, adapter->desc);
            adapter = adapter->next;
        }
        ec_free_adapters(adapter);
    }

    printf("End program\n");
    return (0);
}

// Functions added from red_test
/* PI calculation to get linux time synced to DC time */
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
    static int64 integral = 0;
    int64 delta;
    /* set linux sync point 50us later than DC sync, just as example */
    delta = (reftime - 50000) % cycletime;
    if (delta > (cycletime / 2))
    {
        delta = delta - cycletime;
    }
    if (delta > 0)
    {
        integral++;
    }
    if (delta < 0)
    {
        integral--;
    }
    *offsettime = -(delta / 100) - (integral / 20);
    gl_delta = delta;
    if ((gl_delta < 1000))
    {
        diffTimerSyncAchievedCntr++;
        if (diffTimerSyncAchievedCntr > 1000)
        {
            diffTimerSyncAchievedFlag = 1;
            printf("\n Sync Achieved..");
        }
        printf("\n Sync: %d", diffTimerSyncAchievedCntr);
    }
}

/* add ns to timespec */
void add_timespec(struct timespec *ts, int64 addtime)
{
    int64 sec, nsec;

    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;
    if (ts->tv_nsec >= NSEC_PER_SEC)
    {
        nsec = ts->tv_nsec % NSEC_PER_SEC;
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
        ts->tv_nsec = nsec;
    }
}

void extractFlexibleInputPDO_data()
{
    iErrCode.split.l = *(ec_slave[0].inputs + INPUT_OFFSET_ERRCODE);
    iErrCode.split.h = *(ec_slave[0].inputs + (INPUT_OFFSET_ERRCODE + 1));

    uiStatusWd.split.l = *(ec_slave[0].inputs + INPUT_OFFSET_STATUSWORD);
    uiStatusWd.split.h = *(ec_slave[0].inputs + (INPUT_OFFSET_STATUSWORD + 1));
    

    iPosActualValue.split.ll = *(ec_slave[0].inputs + INPUT_OFFSET_POSN_ACTUAL_VALUE);
    iPosActualValue.split.lh = *(ec_slave[0].inputs + INPUT_OFFSET_POSN_ACTUAL_VALUE + 1);
    iPosActualValue.split.hl = *(ec_slave[0].inputs + INPUT_OFFSET_POSN_ACTUAL_VALUE + 2);
    iPosActualValue.split.hh = *(ec_slave[0].inputs + INPUT_OFFSET_POSN_ACTUAL_VALUE + 3);
    //iPosActualValue.split.hh = *(ec_slave[0].inputs + INPUT_OFFSET_POSN_ACTUAL_VALUE + 3);

    ui8ModesOfOpnDisplay = *(ec_slave[0].inputs + INPUT_OFFSET_MODE_OF_OPN_DISP);

    iTqActual.split.l = *(ec_slave[0].inputs + INPUT_OFFSET_TQ_ACTUAL_VALUE);
    iTqActual.split.h = *(ec_slave[0].inputs + INPUT_OFFSET_TQ_ACTUAL_VALUE + 1);

    updateStatus(uiStatusWd.hl);
    printStatus(uiDriveStatus);
}


#define VALID_FRAME_FLAG 1
#define INVALID_FRAME_FLAG 0

#define CMD_OPEN_VALVE      'O'
#define CMD_CLOSE_VALVE     'C'
#define CMD_STOP            'P'
#define CMD_SLAVE_STATUS    'S'


/*

'0360' --> Degree of Rotation
'0747' --> Max torque that could be applied
'OK' --> Response if cmd is accepted



CMD_OPEN_VALVE
--------------
Q:
Master Sends: #'O','0360','0747'<CR><LF>
R:
Slave Responds: #'O','OK'<CR><LF> ---> If command can be accepted
Slave Responds: #'O','NK'<CR><LF> ---> If command is Rejected

CMD_CLOSE_VALVE
---------------
Q:
Master Sends: #'C','0360','0747'<CR><LF>
R:
Slave Responds: #'C','OK'<CR><LF> ---> If command can be accepted
Slave Responds: #'C','NK'<CR><LF> ---> If command is Rejected

CMD_STOP
--------
Q:
Master Sends: #'P'<CR><LF>
R:
Slave Responds: #'P','OK'<CR><LF> ---> If command can be accepted

CMD_SLAVE_STATUS
----------------
Q:
Master Sends: #'S'<CR><LF>
R:
Slave Responds: #'S','MV'<CR><LF> ---> If Motor is in rotation
Slave Responds: #'S','PR'<CR><LF> ---> If Motor Stopped with position reached
Slave Responds: #'S','SP'<CR><LF> ---> If Motor Stopped maybe due to torque



Serial Port if # is received then 

*/


struct{
    uint rxByteCntr;
    uint dataReceivedFlag;
    int port;
}serial;
struct termios tty;

//Serial port related activity
int openPort(uint32 uirBaudRate){
    int retVar = 1;
    serial.dataReceivedFlag = 0;
    serial.rxByteCntr = 0;
    serial.port = open("/dev/ttyAMA0", O_RDWR);
    if (serial.port < 0) 
        printf("Error %i from open: %s\n", errno, strerror(errno));
    else{    
        if(tcgetattr(serial.port, &tty) != 0){
            printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        }
        else{
            tty.c_cflag &= ~PARENB; // Clear parity bit
            tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication 
            tty.c_cflag &= ~CSIZE; // Clear all the size bits
            tty.c_cflag |= CS8; // 8 bits per byte
            tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
            tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
            tty.c_lflag &= ~ICANON;
            tty.c_lflag &= ~ECHO; // Disable echo
            tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
            tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
            tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received
            tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
            tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
            tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
            tty.c_cc[VMIN] = 0;
            cfsetispeed(&tty, B9600);
            cfsetospeed(&tty, B9600);
            
        }
    }
    return retVar;
}


void transactData(){
    int retVal; 
    if(serial.dataReceivedFlag == 1){
        retVal = extractData();
        if(retVal == VALID_FRAME_FLAG)
            validFrameAction();
    }
}

int extractData(){
    int retVar = 1;
    
    return retVar;
}



void validFrameAction(){
    
}




void createResponse(){

}

void sendResponse(uint uirCmd){

}

void closePort(){

}