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
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdbool.h>
#include "ethercat.h"

#pragma comment(lib,"ws2_32.lib") //Winsock Library


#ifdef _WIN32
#include <Windows.h>
#endif

#define EC_TIMEOUTMON 500

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "27015"


#define SOCKET_SERVER_APP_START_INIT							1
#define SOCKET_SERVER_CREATE_SOCKET_TO_LISTEN_FOR_CLIENT		2
#define SOCKET_SERVER_CREATE_SOCKET_FAILURE_ACTION				3
#define SOCKET_SERVER_SETUP_LISTENING_SOCKET					4
#define SOCKET_SERVER_LISTENING_SOCKET_SETUP_FAILURE_ACTION		5
#define SOCKET_SERVER_LISTEN_FOR_CLIENT_SOCKET					6
#define SOCKET_SERVER_LISTEN_FOR_CLIENT_SOCKET_FAILURE_ACTION	7
#define SOCKET_SERVER_ACCEPT_CLIENT_SOCKET						8
#define SOCKET_SERVER_GET_DATA_FROM_CLIENT						9
#define SOCKET_SERVER_ANALYZE_QUERY								10
#define SOCKET_SERVER_SEND_STATUS                               11
#define SOCKET_SERVER_CLOSE_SOCKET								12


#define STATE_MACHINE_STAT_UNKNOWN					0
#define STATE_MACHINE_STAT_NOT_RDY_TO_SWITCH_ON		1
#define STATE_MACHINE_STAT_SWITCH_ON_DISABLED		2
#define STATE_MACHINE_STAT_RDY_TO_SWITCH_ON			3
#define STATE_MACHINE_STAT_SWITCHED_ON				4
#define STATE_MACHINE_STAT_OPERATION_ENABLED		5
#define STATE_MACHINE_STAT_FAULT					6
#define STATE_MACHINE_STAT_FAULT_REACTION_ACTIVE	7
#define STATE_MACHINE_STAT_QUICK_STOP_ACTIVE		8


#define CTL_WD_SHUT_DOWN							1
#define CTL_WD_SWITCH_ON							2
#define CTL_WD_DISABLE_VTG							3
#define CTL_WD_QUICK_STOP							4
#define CTL_WD_DISABLE_OPN							5
#define CTL_WD_ENABLE_OPN							6
#define CTL_WD_ENABLE_IP							7
#define CTL_WD_FAULT_RESET							8
#define CTL_WD_STOP									9


#define BIT0		1
#define BIT1		2
#define BIT2		4
#define BIT3		8
#define BIT4		0x10
#define BIT5		0x20
#define BIT6		0x40
#define BIT7		0x80
#define BIT8		0x100
#define BIT9		0x200
#define BIT10		0x400
#define BIT11		0x800
#define BIT12		0x1000
#define BIT13		0x2000
#define BIT14		0x4000
#define BIT15		0x8000




#define	OPN_MODE_PROFILE_VELOCITY					1
#define	OPN_MODE_INTERPOLATED_POSN					2
#define	OPN_MODE_HOMING_MODE						3
#define	OPN_MODE_PROFILE_POSN						4
#define	OPN_MODE_TORQUE								5
#define	OPN_MODE_CYCLIC_SYNCH_POSN					6


#define SERVO_OPN_EXPECTED							1
#define SERVO_OPN_NOT_EXPECTED						0

#define CMD_STAT_UNKNOWN							0
#define CMD_STAT_LOADED								1
#define CMD_STAT_ACCEPTED							2
#define CMD_STAT_COMPLETED_POSN						3
#define CMD_STAT_COMPLETED_TQ						4
#define CMD_STAT_ERR								5
#define CMD_STAT_REJECTED							6


 //Register Definitions

 //Mode setting
#define REG_DRV_OPMODE									0x35B4
#define REG_MODE_OF_OPN									0x6060

//Registers Associated with Profile Position Mode
#define REG_PROF_POSN_MODE_TARGET_POSN					0x607A	
#define REG_PROF_POSN_MODE_SW_POSN_LIMIT				0x607D
#define REG_PROF_POSN_MODE_VELOCITY						0x6081
#define REG_PROF_POSN_MODE_ACCELERATION					0x6083
#define REG_PROF_POSN_MODE_DECELRATION					0x6084


#define DRV_OPMODE_CURRENT_MODE							 0
#define DRV_OPMODE_VELOCITY_MODE						 1
#define DRV_OPMODE_POSITION_MODE						 2

//Profile Position Mode SPecific
#define PPMNEW_SETPOINT_ENABLE								*(ec_slave[0].outputs + 1) |= (BIT4)
#define PPMNEW_SETPOINT_ENABLE_RELEASE						*(ec_slave[0].outputs + 1) &= ~(BIT4)
#define PPMNEW_CHANGE_SET_IMMEDIATELY						*(ec_slave[0].outputs + 1) |= (BIT5)

#define PPMNEW_RELATIVE_MODE								*(ec_slave[0].outputs + 1) |= (BIT6)
#define PPMNEW_ABSOLUTE_MODE								*(ec_slave[0].outputs + 1) &= ~(BIT6) 

#define CW					1	//Full Marks for being right handed
#define ACW					0	//Zero Marks for being left handed.... Indian Preconcieved notions become predefined definitions!!

#define MOTION_TIMEOUT_VAL		60000
#define BLOCK_ROTOR_DECLARE_CNT	100	//This constant will be used for the number of samples for which the rotor consequtively has not achieved its desired position!




char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

//IANET: 261122
volatile int rtcnt;

uint16 uiGFlag = 0;
uint32 uiLoopCntr = 0;
uint32 uiDly = 0;
uint32 uiStepsReqd = 0;

uint32 uiInitStatus = 0;
uint32 uiWriteStat = 0;

uint32 uiRdyToSwitchOn = 0;	//Bit 0
uint32 uiSwitchedOn = 0;//Bit 1
uint32 uiOpnEnabled = 0;//Bit 2
uint32 uiFault = 0;//Bit 3
uint32 uiVtgEnabled = 0;//Bit 4
uint32 uiQuickStop = 0;//Bit 5
uint32 uiSwitchOnDisable = 0;//Bit 6
uint32 uiWarning = 0;//Bit 7
uint32 uiRemote = 0;//Bit 9
uint32 uiTargetReached = 0;//Bit 10
uint32 uiInternalLimitActive = 0;//Bit 11
uint32 uiInterpolationActive = 0;//Bit 12



uint32 u32val;
uint16 slave;

//Operational Variables
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
union {
    uint16 hl;
    struct {
        uint8 l;
        uint8 h;
    }split;
}uiMaxTorque, uiStatusWd, uiTqOffset, uiTqActual;

union {
    uint32 hl;
    struct {
        uint8 ll;
        uint8 lh;
        uint8 hl;
        uint8 hh;
    }split;
}uiSecondFeedbackPosn, uiVelocity, uiVelCmdValue;

union {
    int32 hl;
    struct {
        uint8 ll;
        uint8 lh;
        uint8 hl;
        uint8 hh;
    }split;
}iPosActualValue, iDesiredPositionVal, iFinalDesiredPosnVal;

int32 iBuffPosValue;
uint32 uiBlockedRotorCntr;
uint32 uiBlockedRotorFlag;


int64 i64BuffVal;

uint64 uiEncoderCntsToMove;


//Application Specific
//uint8 ccwEnable = 0;
uint16 u16DirnCntr = 0;
uint16 u16MotionTimeOutCntr = 0;

FILE* fpIn;
FILE* fpOut;
uint32 uiDesiredDegreeOfRtn;
uint32 uiDesiredDirectionOfRtn;
uint32 uiDesiredRPM;
float fDesiredTq;
uint32 uiDesiredStatus;
uint32 uiActualPosn;
uint32 uiActualTq;

uint32 uiRotationOffset;

uint32 uiModifyCmdStatusFlag = 0;
uint32 ui2MsecCntr = 0;
uint32 uiCyclesCntr = 0;
uint64 ui64Cntr = 0;

uint32 ui32StepsExecuted = 0;
uint32 ui32StepsProgramed = 0;


uint32 ui32RtThreadSpeedCntr = 0;
uint32 ui32RtThreadOvFlowCntr = 0;


//Socket Server related variables
uint32 uiServerSocketStatus = SOCKET_SERVER_APP_START_INIT;

WSADATA wsaData;


int iResult;
u_long iMode = 0;
UINT_PTR ListenSocket = INVALID_SOCKET;	//Changed to UINT_PTR from SOCKET because compiler was throwing an error
UINT_PTR ClientSocket = INVALID_SOCKET;

struct addrinfo* result = NULL;
struct addrinfo hints;

int iSendResult;
char recvbuf[DEFAULT_BUFLEN];
int recvbuflen = DEFAULT_BUFLEN;

bool sktConnectedToClient = false;
uint8_t transferArr[10];

int uiDelMeCntr[10];


void fillMotionParams(uint32 uirDesiredPosnVal, uint16 uirTqFeedFwd, uint16 uirMaxTq);
uint16 setLimitingTorqueValue(float frDesiredTorque, float frMotorMaxTorque, uint16 uirGearRatio);
void updateStatus(uint16 uirStatus);
void printStatus(uint16 uirDriveStat);
void modifyControlWord(uint16 uirSlave, uint16 uirDesiredStat, uint8 uiOffset);
void setProfilePositionParameters(uint16 uirSlave, int32 irTargetPosn, int32 irSW_PosnLimit1, int32 irSW_PosnLimit2, uint32 uirProfileVelocity, uint32 uirProfileAcceleration, uint32 uirProfileDeceleration);
void modifyInterpolatedPositionCommandValue(int32 irDesiredPosn, float frMaxTq, int32 irTargetVel);
void modifyLatchControlWordValue(uint16 uirLatchCtlWd, uint8 uirOffset);
void modifyTorqueFeedForwardValue(uint16 uirTqFeedFwd, uint8 uirOffset);
void modifyDigitalOutputValue(uint16 uirDigitalOp, uint8 uirOffset);
void modifyMaxTorqueValue(uint16 uirMaxTorque, uint8 uirOffset);
void modifyMaxTorqueValueRegister(uint16 uirSlave, uint16 uirMaxTorque);
void DriveEnable();
void StopDrive();
void ShutDownDrive();
void setOutputPDO(uint16 slave, uint16 uirOutputPDO);
void setInputPDO(uint16 slave, uint16 uirInputPDO);
void setFlexibleOutputPDO(uint16 uirSlave);
void setFlexibleInputPDO(uint16 uirSlave);
unsigned char checkInterPolationDone();
void drv_SetOperationMode(uint16 slave, int32 irSelectedMode);
void resetDesiredTqAndDegOfRtn();
void GetDesiredTqAndDegOfRtn();
unsigned int ReadInteger(FILE* fp, unsigned int uiOffsetFromBegin);
void StoreInteger(FILE* fp, unsigned int IntegerToStore);
float ReadFloat(FILE* fp, unsigned int uiOffsetFromBegin);
void StoreFloat(FILE* fp, float floatToStore);
UINT32 ValidateInteger(UINT32* uirIntegerToValidate, UINT32 uirIntegerMinVal, UINT32 uirIntegerMaxVal, UINT32 uirFillValIfInvalid);
UINT32 ValidateFloat(float* frFloatToValidate, float frFloatMinVal, float frFloatMaxVal, float frFillValIfInvalid);
void SetActualTqAndPosn();
void SetCommandStatus(UINT32 uirCmdStatus);
void ConvertDegreeOfRotationToCount(uint32 uirRPM, uint32 uirScanIntervalInMsec, uint32 uirDegreesToRotate);
void socketServerAction();
char** str_split(char* a_str, const char a_delim);
void SocketSendResponse(char*);

//Code taken from https://stackoverflow.com/questions/9210528/split-string-with-delimiters-in-c
//by user hmjd
char** str_split(char* a_str, const char a_delim)
{
    char** result = 0;
    size_t count = 0;
    char* tmp = a_str;
    char* last_comma = 0;
    char delim[2];
    delim[0] = a_delim;
    delim[1] = 0;

    /* Count how many elements will be extracted. */
    while (*tmp)
    {
        if (a_delim == *tmp)
        {
            count++;
            last_comma = tmp;
        }
        tmp++;
    }

    /* Add space for trailing token. */
    count += last_comma < (a_str + strlen(a_str) - 1);

    /* Add space for terminating null string so caller
       knows where the list of returned strings ends. */
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


//IANET: 261122
void fillMotionParams(uint32 uirDesiredPosnVal, uint16 uirTqFeedFwd, uint16 uirMaxTq)
{
    union {
        uint32 hl;
        struct {
            uint8 ll;
            uint8 lh;
            uint8 hl;
            uint8 hh;
        }split;
    }uiDesPosnVal;

    union {
        uint16 hl;
        struct {
            uint8 l;
            uint8 h;
        }split;
    }uiMaxTq, uiFeedFwdTq;


    uiDesPosnVal.hl = uirDesiredPosnVal;
    uiMaxTq.hl = uirMaxTq;
    uiFeedFwdTq.hl = uirTqFeedFwd;


    *(ec_slave[0].outputs + 0) = uiDesPosnVal.split.hh;
    *(ec_slave[0].outputs + 1) = uiDesPosnVal.split.hl;
    *(ec_slave[0].outputs + 2) = uiDesPosnVal.split.lh;
    *(ec_slave[0].outputs + 3) = uiDesPosnVal.split.ll;
    *(ec_slave[0].outputs + 6) = uiFeedFwdTq.split.h;
    *(ec_slave[0].outputs + 7) = uiFeedFwdTq.split.l;

    //	*(ec_slave[0].outputs + 10) = uiMaxTq.split.h;			
    //	*(ec_slave[0].outputs + 11) = uiMaxTq.split.l;

}

uint16 setLimitingTorqueValue(float frDesiredTorque, float frMotorMaxTorque, uint16 uirGearRatio)
{
    uint16 uilclValToSet = 0;
    float flclValToSet;
    float flclMaxTorque = frMotorMaxTorque * (float)uirGearRatio;
    //printf("\n flclMaxTorque: %f\n ", flclMaxTorque);
    if (frDesiredTorque <= flclMaxTorque)	//Indicates that a valid torque value has been entered
    {
        flclValToSet = (frDesiredTorque * 1000) / flclMaxTorque;
        //printf("\n flclValToSet: %f\n ", flclValToSet);
        uilclValToSet = (uint16)flclValToSet;

    }
    //printf("\n uilclValToSet: %d\n ", uilclValToSet);
    return uilclValToSet;

}

void updateStatus(uint16 uirStatus)
{
    uint16 lclStat, uiLclLowerNibbleStat;
    lclStat = 0;

    uiLclLowerNibbleStat = (uirStatus & 0x0F);


    switch (uiLclLowerNibbleStat)
    {
    case 0:
        if ((uirStatus & BIT6) == BIT6)
        {
            uiDriveStatus = STATE_MACHINE_STAT_SWITCH_ON_DISABLED;
            uiOpnEnabled = 0;
        }
        else
        {
            uiDriveStatus = STATE_MACHINE_STAT_NOT_RDY_TO_SWITCH_ON;
            uiOpnEnabled = 0;
        }

        break;

    case 1:
        if ((uirStatus & (BIT6 + BIT5)) == BIT5)
        {
            uiDriveStatus = STATE_MACHINE_STAT_RDY_TO_SWITCH_ON;
            uiOpnEnabled = 0;
        }

        break;

    case 3:
        if ((uirStatus & (BIT6 + BIT5)) == BIT5)
        {
            uiDriveStatus = STATE_MACHINE_STAT_SWITCHED_ON;
            uiOpnEnabled = 0;
        }
        break;

    case 7:
        if ((uirStatus & (BIT6 + BIT5)) == BIT5)
        {
            uiDriveStatus = STATE_MACHINE_STAT_OPERATION_ENABLED;
            uiOpnEnabled = 1;
        }
        else
        {

            if ((uirStatus & (BIT6 + BIT5 + BIT3)) == 0)
            {
                uiDriveStatus = STATE_MACHINE_STAT_QUICK_STOP_ACTIVE;
                uiOpnEnabled = 0;
            }

        }
        break;

    case 8:
        if ((uirStatus & BIT6) == 0)
        {
            uiDriveStatus = STATE_MACHINE_STAT_FAULT;
            uiOpnEnabled = 0;
        }
        break;

    case 0x0F:
        if ((uirStatus & BIT6) == 0)
        {
            uiDriveStatus = STATE_MACHINE_STAT_FAULT_REACTION_ACTIVE;
            uiOpnEnabled = 0;
        }
        break;

    default:
        uiOpnEnabled = 0;
        break;
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
    if (uiErrorFlag == 1) {
        printf(" Error Detected  ");
        uiDesiredStat = CTL_WD_FAULT_RESET;
        modifyControlWord(slave, uiDesiredStat, 0);

    }
    printf(" Status: ");

    switch(uirDriveStat)
    {
        case STATE_MACHINE_STAT_UNKNOWN:
            printf("Unknown               \t");
            break;
        case STATE_MACHINE_STAT_NOT_RDY_TO_SWITCH_ON:
            printf("Not Ready to Switch On\t");
            break;
        case STATE_MACHINE_STAT_SWITCH_ON_DISABLED:
            printf("Switch On Disabled    \t");
            break;
        case STATE_MACHINE_STAT_RDY_TO_SWITCH_ON:
            printf("Ready to Switch On   \t");
            break;
        case STATE_MACHINE_STAT_SWITCHED_ON:
            printf("Switched On          \t");
            break;
        case STATE_MACHINE_STAT_OPERATION_ENABLED:
            printf("Operation Enabled    \t");
            break;
        case STATE_MACHINE_STAT_FAULT:
            printf("Fault Detected       \t");
            break;
        case STATE_MACHINE_STAT_FAULT_REACTION_ACTIVE:
            printf("Fault Reaction Active\t");
            break;
        case STATE_MACHINE_STAT_QUICK_STOP_ACTIVE:
            printf("Quick Stop Active    \t");
            break;
        default:
            printf("Unknown              \t");
            break;
    }
    //printf("PAV: %d Pde:%d	 ipa: %d trf: %x tq:%d \r",iPosActualValue.hl,iDesiredPositionVal.hl,uiInterpolationActive,/*uiTargetReachedFlag*/uiStatusWd.hl,uiTqActual.hl); 
}

void modifyControlWord(uint16 uirSlave, uint16 uirDesiredStat, uint8 uiOffset)
{
    union {
        uint16 hl;
        struct {
            uint8 l;
            uint8 h;
        }split;
    }uiCtlWd;


    uint16 uiRetArr[10], uilclRetVar;
    uint32 uilclModifyFlag = 1;
    int l = sizeof(uiRetArr) - 1;
    memset(&uiRetArr, 0, 10);
    ec_SDOread(uirSlave, 0x6040, 0x00, FALSE, &l, &uiRetArr, EC_TIMEOUTRXM);

    uiCtlWd.hl = uiRetArr[0];
    uilclRetVar = 0;


    switch (uirDesiredStat)
    {
    case CTL_WD_SHUT_DOWN:
        uiCtlWd.split.h = 0;
        uiCtlWd.split.l |= (BIT2 + BIT1);
        uiCtlWd.split.l &= ~BIT0;
        break;

    case CTL_WD_SWITCH_ON:
        uiCtlWd.split.h = 0;
        uiCtlWd.split.l |= (BIT2 + BIT1 + BIT0);
        uiCtlWd.split.l &= ~BIT3;
        break;

    case CTL_WD_DISABLE_VTG:
        uiCtlWd.split.h = 0;
        uiCtlWd.split.l &= ~BIT1;
        break;

    case CTL_WD_QUICK_STOP:
        uiCtlWd.split.h = 0;
        uiCtlWd.split.l |= BIT1;
        uiCtlWd.split.l &= ~BIT2;
        break;

    case CTL_WD_DISABLE_OPN:
        uiCtlWd.split.h = 0;
        uiCtlWd.split.l |= (BIT2 + BIT1 + BIT0);
        uiCtlWd.split.l &= ~BIT3;
        break;

    case CTL_WD_ENABLE_OPN:
        uiCtlWd.split.h = 0;
        uiCtlWd.split.l |= (BIT3 + BIT2 + BIT1 + BIT0);
        break;

    case CTL_WD_ENABLE_IP:
        uiCtlWd.split.h = 0;
        uiCtlWd.split.l |= (BIT4 + BIT3 + BIT2 + BIT1 + BIT0);
        break;

    case CTL_WD_FAULT_RESET:
        uiCtlWd.split.h = 0;
        uiCtlWd.split.l |= BIT7;
        break;

    case CTL_WD_STOP:
        uiCtlWd.split.h = 0;
        uiCtlWd.split.l |= BIT8;
        break;

    default:
        uilclModifyFlag = 0;
        break;

    }
    *(ec_slave[0].outputs + 4) = uiCtlWd.split.l;
    *(ec_slave[0].outputs + 5) = uiCtlWd.split.h;
}

void setProfilePositionParameters(uint16 uirSlave, int32 irTargetPosn, int32 irSW_PosnLimit1, int32 irSW_PosnLimit2, uint32 uirProfileVelocity, uint32 uirProfileAcceleration, uint32 uirProfileDeceleration)
{
    uint32 uilclRetval;
    uilclRetval = 0;
    uilclRetval += ec_SDOwrite(uirSlave, REG_PROF_POSN_MODE_TARGET_POSN, 0x00, FALSE, sizeof(irTargetPosn), &irTargetPosn, EC_TIMEOUTRXM);
    uilclRetval += ec_SDOwrite(uirSlave, REG_PROF_POSN_MODE_SW_POSN_LIMIT, 0x01, FALSE, sizeof(irSW_PosnLimit1), &irSW_PosnLimit1, EC_TIMEOUTRXM);
    uilclRetval += ec_SDOwrite(uirSlave, REG_PROF_POSN_MODE_SW_POSN_LIMIT, 0x02, FALSE, sizeof(irSW_PosnLimit2), &irSW_PosnLimit2, EC_TIMEOUTRXM);
    uilclRetval += ec_SDOwrite(uirSlave, REG_PROF_POSN_MODE_VELOCITY, 0x00, FALSE, sizeof(uirProfileVelocity), &uirProfileVelocity, EC_TIMEOUTRXM);
    uilclRetval += ec_SDOwrite(uirSlave, REG_PROF_POSN_MODE_ACCELERATION, 0x00, FALSE, sizeof(uirProfileAcceleration), &uirProfileAcceleration, EC_TIMEOUTRXM);
    uilclRetval += ec_SDOwrite(uirSlave, REG_PROF_POSN_MODE_DECELRATION, 0x00, FALSE, sizeof(uirProfileDeceleration), &uirProfileDeceleration, EC_TIMEOUTRXM);
}

void modifyInterpolatedPositionCommandValue(int32 irDesiredPosn, float frMaxTq, int32 irTargetVel)
{

    int retval = 0;

    union {
        int32 hl;
        struct {
            uint8 ll;
            uint8 lh;
            uint8 hl;
            uint8 hh;
        }split;
    }iSplitVar;


    /*
        uiInterPolTimeInSec = (uint8) uirInterpolationTimeInSec;
        uiInterPolTimeInSec = 12;
        iInterPolTimeIndex = 1;

        retval += ec_SDOwrite(slave, 0x60C2, 0x01, FALSE, sizeof(uiInterPolTimeInSec), &uiInterPolTimeInSec, EC_TIMEOUTRXM);	//Interpolated Time is set
        retval += ec_SDOwrite(slave, 0x60C2, 0x02, FALSE, sizeof(iInterPolTimeIndex), &iInterPolTimeIndex, EC_TIMEOUTRXM);	//Interpolated Time is set
        printf("\n\n\n\n\n\n\n\n\n\n Function CALLLED!!!!!\n\n\n\n\n");
        */
    uiCyclesCntr++;
    //printf("\n\n\n\n**********************************************************************MIPCV CALLED*****************************: %d\n\n\n\n\n", uiCyclesCntr);

    iSplitVar.hl = irDesiredPosn;
    *(ec_slave[0].outputs + 0) = iSplitVar.split.ll;
    *(ec_slave[0].outputs + 1) = iSplitVar.split.lh;
    *(ec_slave[0].outputs + 2) = iSplitVar.split.hl;
    *(ec_slave[0].outputs + 3) = iSplitVar.split.hh;


    /*
        //iSplitVar.hl = 90000;
        irTargetVel = irTargetVel * 10;	//Gear Ratio
        //iSplitVar.hl = irTargetVel * 1000;
        *(ec_slave[0].outputs + 8) = iSplitVar.split.ll;
        *(ec_slave[0].outputs + 9) = iSplitVar.split.lh;
        *(ec_slave[0].outputs + 10) = iSplitVar.split.hl;
        *(ec_slave[0].outputs + 11) = iSplitVar.split.hh;
    */
    iSplitVar.hl = setLimitingTorqueValue(frMaxTq, (float)5.46, 10);
    *(ec_slave[0].outputs + 6) = iSplitVar.split.ll;
    *(ec_slave[0].outputs + 7) = iSplitVar.split.lh;
}

void modifyLatchControlWordValue(uint16 uirLatchCtlWd, uint8 uirOffset)
{
    union {
        uint16 hl;
        struct {
            uint8 l;
            uint8 h;
        }split;
    }uiSplitVar;

    uiSplitVar.hl = uirLatchCtlWd;

    *(ec_slave[0].outputs + uirOffset) = uiSplitVar.split.h;
    *(ec_slave[0].outputs + uirOffset + 1) = uiSplitVar.split.l;
}

void modifyTorqueFeedForwardValue(uint16 uirTqFeedFwd, uint8 uirOffset)
{
    union {
        uint16 hl;
        struct {
            uint8 l;
            uint8 h;
        }split;
    }uiSplitVar;

    uiSplitVar.hl = uirTqFeedFwd;

    *(ec_slave[0].outputs + uirOffset) = uiSplitVar.split.l;
    *(ec_slave[0].outputs + uirOffset + 1) = uiSplitVar.split.h;
}

void modifyDigitalOutputValue(uint16 uirDigitalOp, uint8 uirOffset)
{
    union {
        uint16 hl;
        struct {
            uint8 l;
            uint8 h;
        }split;
    }uiSplitVar;

    uiSplitVar.hl = uirDigitalOp;

    *(ec_slave[0].outputs + uirOffset) = uiSplitVar.split.h;
    *(ec_slave[0].outputs + uirOffset + 1) = uiSplitVar.split.l;
}

void modifyMaxTorqueValue(uint16 uirMaxTorque, uint8 uirOffset)
{
    union {
        uint16 hl;
        struct {
            uint8 l;
            uint8 h;
        }split;
    }uiSplitVar;
    uiSplitVar.hl = uirMaxTorque;

    *(ec_slave[0].outputs + uirOffset) = uiSplitVar.split.h;
    *(ec_slave[0].outputs + uirOffset + 1) = uiSplitVar.split.l;
}

void modifyMaxTorqueValueRegister(uint16 uirSlave, uint16 uirMaxTorque)
{
    uint16 uilclRetval = 0;
    uilclRetval += ec_SDOwrite(uirSlave, 0x6073, 0x00, FALSE, sizeof(uirMaxTorque), &uirMaxTorque, EC_TIMEOUTRXM);
}

void DriveEnable()
{
    uint16 uiDesiredStat = 0;

    switch (uiDriveStatus)
    {

    case STATE_MACHINE_STAT_SWITCH_ON_DISABLED:	//Nothing can be done over Ethercat to change the state!
        uiDesiredStat = CTL_WD_SHUT_DOWN;
        break;
    case STATE_MACHINE_STAT_RDY_TO_SWITCH_ON:
        uiDesiredStat = CTL_WD_SWITCH_ON;
        break;
    case STATE_MACHINE_STAT_SWITCHED_ON:
        uiDesiredStat = CTL_WD_ENABLE_OPN;
        break;
    case STATE_MACHINE_STAT_OPERATION_ENABLED:
        if (uiInterpolationEnableFlag == 1)
        {
            uiDesiredStat = CTL_WD_ENABLE_IP;
        }
        else
        {
            uiDesiredStat = CTL_WD_ENABLE_OPN;
        }
        break;

    case STATE_MACHINE_STAT_QUICK_STOP_ACTIVE:
    case STATE_MACHINE_STAT_NOT_RDY_TO_SWITCH_ON:	//The drive is booting up Wait!
        break;

    case STATE_MACHINE_STAT_FAULT:
        uiDesiredStat = CTL_WD_FAULT_RESET;
        break;

    case STATE_MACHINE_STAT_UNKNOWN:
    case STATE_MACHINE_STAT_FAULT_REACTION_ACTIVE:

    default:
        uiDesiredStat = CTL_WD_SWITCH_ON;
        break;
    }
    modifyControlWord(slave, uiDesiredStat, 0);

    //*(ec_slave[0].outputs + 1) = 0;
}

void StopDrive()
{
    uint16 uiDesiredStat = CTL_WD_STOP;
    modifyControlWord(slave, uiDesiredStat, 0);
}

void ShutDownDrive()
{
    uint16 uiDesiredStat = CTL_WD_SHUT_DOWN;
    modifyControlWord(slave, uiDesiredStat, 0);
}

void setOutputPDO(uint16 slave, uint16 uirOutputPDO)
{

    uint8 u8val;
    uint16 u16val;
    int retval = 0;
    //Refer Pg. 36 of AKD Ethercat Communication Edition K, December 2014
    //Output Bytes in the frame are set by this
    u8val = 0;
    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);	//Mapping Section is cleared
    u16val = uirOutputPDO;	//Desired Value presently is thought to be 1722
    retval += ec_SDOwrite(slave, 0x1c12, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    u8val = 1;
    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
}

void setInputPDO(uint16 slave, uint16 uirInputPDO)
{
    //Refer Pg. 36 of AKD Ethercat Communication Edition K, December 2014
    //Input Bytes in the frame are set by this
    uint8 u8val;
    uint16 u16val;
    int retval = 0;
    u8val = 0;
    retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);	//Mapping Section is cleared
    u16val = uirInputPDO;
    retval += ec_SDOwrite(slave, 0x1c13, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    u8val = 1;
    retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
}

void setFlexibleOutputPDO(uint16 uirSlave)
{

    //Remember: Set Values PDO cannot exceed 22 bytes
    uint8 u8val;
    uint16 u16val;
    uint32 u32val;
    int retval = 0;
    u8val = 0;

    //Mapping Section is cleared
    retval += ec_SDOwrite(uirSlave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    //Clear Output PDO's
    ec_SDOwrite(uirSlave, 0x1600, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    ec_SDOwrite(uirSlave, 0x1601, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    ec_SDOwrite(uirSlave, 0x1602, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    ec_SDOwrite(uirSlave, 0x1603, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    //Map the fields here using the following logic:
    //The meaning of the data (for example 0x60410010 in the mapping of 0x1A00 sub 1) is as follows:
    //0x6041 is the index of the DS402 status word
    //0x00 is the subindex of the DS402 status word
    //0x10 is the number of bits for this entry, i. e. 16 bits or 2 bytes.

    //Interpolated Position Command Value
    u32val = 0x60C10120;
    ec_SDOwrite(uirSlave, 0x1600, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    //Control Word
    u32val = 0x60400010;
    ec_SDOwrite(uirSlave, 0x1600, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    //Max Torque
    u32val = 0x60720010;
    ec_SDOwrite(uirSlave, 0x1600, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    //Target Velocity
    u32val = 0x60810020;
    ec_SDOwrite(uirSlave, 0x1601, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    //set number of PDO entries for 0x1600 
    printf("\n set number of PDO entries for 0x1600 ");

    u8val = 3;//Since 1600 maps only 3 Variable
    ec_SDOwrite(uirSlave, 0x1600, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);	//Since 1600 maps 3 Variables
    u8val = 1;	//Since 1601 maps only 1 Variable
    ec_SDOwrite(uirSlave, 0x1601, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);


    //Map PDO 1600 to RxPDO Assign
    //This makes the 1600 PDO mapping acitve
    //Set the number of PDO's to 1
    //1C12.0 ==> Highest Sub index Supported
    u8val = 0x02;
    ec_SDOwrite(uirSlave, 0x1C12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    //1C12.1 ==> PDO Mapping Index
    u16val = 0x1600;
    ec_SDOwrite(uirSlave, 0x1C12, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);

    u16val = 0x1601;
    ec_SDOwrite(uirSlave, 0x1C12, 0x02, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
}

void setFlexibleInputPDO(uint16 uirSlave)
{

    //Remember: Actual Values PDO cannot exceed 32 bytes
    uint8 u8val;
    uint16 u16val;
    uint32 u32val;
    int retval = 0;
    u8val = 0;

    //Mapping Section is cleared
    retval += ec_SDOwrite(uirSlave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    //Clear Input PDO's
    ec_SDOwrite(uirSlave, 0x1A00, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    ec_SDOwrite(uirSlave, 0x1A01, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    ec_SDOwrite(uirSlave, 0x1A02, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    ec_SDOwrite(uirSlave, 0x1A03, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    //Map the fields here using the following logic:
    //The meaning of the data (for example 0x60410010 in the mapping of 0x1A00 sub 1) is as follows:
    //0x6041 is the index of the DS402 status word
    //0x00 is the subindex of the DS402 status word
    //0x10 is the number of bits for this entry, i. e. 16 bits or 2 bytes.


    //1A00.01 ---> Position Actual Value (4 bytes)
    //1A00.02 ---> Velocity Actual Value (4 bytes)
    //1A01.01 ---> Status Word (2 bytes)
    //1A01.02 ---> Torque Actual Value (2 bytes)
    //1A02.03 ---> Analog Input Value (2 bytes)


    //Position Actual Value
    u32val = 0x60630020;
    ec_SDOwrite(uirSlave, 0x1A00, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    //Velocity Actual Value
    u32val = 0x606C0020;
    ec_SDOwrite(uirSlave, 0x1A00, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    //Status Word
    u32val = 0x60410010;
    ec_SDOwrite(uirSlave, 0x1A01, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    //Torque Actual Value
    u32val = 0x60770010;
    ec_SDOwrite(uirSlave, 0x1A01, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    //Analog Input Value
    u32val = 0x34700410;
    ec_SDOwrite(uirSlave, 0x1A01, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    //Enter the number of PDO's in each variable

    printf("\n Enable PDO 0x1A00");
    u8val = 2;	//Since it maps 2 variables
    ec_SDOwrite(uirSlave, 0x1A00, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    printf("\n Enable PDO 0x1A01");
    u8val = 3;	//Since it maps 3 variables
    ec_SDOwrite(uirSlave, 0x1A01, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    //Map PDO 1600 to TxPDO Assign
    //This makes the TxPDO active

    //Set the number of PDO's to 2
    u8val = 0x02;
    ec_SDOwrite(uirSlave, 0x1C13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u16val = 0x1A00;
    ec_SDOwrite(uirSlave, 0x1C13, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    u16val = 0x1A01;
    ec_SDOwrite(uirSlave, 0x1C13, 0x02, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);

    /*u16val = 0x1A02;
    ec_SDOwrite(uirSlave, 0x1C13, 0x03, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);*/

}

unsigned char checkInterPolationDone()
{
    unsigned char ucRetVar;
    ucRetVar = 0;

    return ucRetVar;
}

void drv_SetOperationMode(uint16 slave, int32 irSelectedMode)
{
    int32 iOperatingMode;
    uint8 u16ErrCntr = 0;
    int8 iModeOfOpnParam = 0;	//This variable value is being determined on the pg. 45 of AKD Ethercat Communication Edition K, December 2014										
    int retval = 0;
    uint8 uiRetArr[10];
    int l = sizeof(uiRetArr) - 1;
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
    case OPN_MODE_TORQUE:
        iModeOfOpnParam = 4;
        iOperatingMode = 1;
        break;
    case OPN_MODE_CYCLIC_SYNCH_POSN:
        iModeOfOpnParam = 8;
        iOperatingMode = 1;
        break;
    default:
        break;
    }
    printf("\n\n\n\n\n\n\n\n\n\n Value of iModeOfOpnParam: %d", iModeOfOpnParam);
    if (iModeOfOpnParam != 0)
    {
        //REG_MODE_OF_OPN = 0x6060
        retval += ec_SDOwrite(slave, REG_MODE_OF_OPN, 0x00, FALSE, sizeof(iModeOfOpnParam), &iModeOfOpnParam, EC_TIMEOUTRXM);
        //REG_DRV_OPMODE = 0x35B4 = DRV.OPMODE
        retval += ec_SDOwrite(slave, REG_DRV_OPMODE, 0x00, FALSE, sizeof(iOperatingMode), &iOperatingMode, EC_TIMEOUTRXM);
    }
    else
    {
        printf("FAILED TO SET THE MODE.... NOT SUPPORTED!!!");
    }

}

unsigned int ReadInteger(FILE* fp, unsigned int uiOffsetFromBegin)
{
    unsigned int readInt;
    fseek(fp, uiOffsetFromBegin, SEEK_SET);
    fread(&readInt, sizeof(readInt), 1, fp);
    return readInt;
}

void StoreInteger(FILE* fp, unsigned int IntegerToStore)
{
    fwrite(&IntegerToStore, sizeof(IntegerToStore), 1, fp);
}

float ReadFloat(FILE* fp, unsigned int uiOffsetFromBegin)
{
    float readFloat;
    fseek(fp, uiOffsetFromBegin, SEEK_SET);
    fread(&readFloat, sizeof(readFloat), 1, fp);
    return readFloat;
}

void StoreFloat(FILE* fp, float floatToStore)
{
    fwrite(&floatToStore, sizeof(floatToStore), 1, fp);
}


//Comment
void resetDesiredTqAndDegOfRtn() {
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
    //To convert string to integer use atoi function
    //To convert string to float use atof function
    //strok to extract the string
    char cmdStr[] = "CTD"; 
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
                    uiDesiredDegreeOfRtn = atoi(*(tokens+lclCntr));
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
        if(uiDesiredStatus != 0)
            uiModifyCmdStatusFlag = 1;
    }
    else {
        printf("Invalid Cmd received");
    }
}

void SetActualTqAndPosn()
{
    //char stringToTx[100];
    printf("\nActPosn: %d, ACT_TQ:%d, DesStat: %d", uiActualPosn, uiActualTq, uiDesiredStatus);
    //sprintf(stringToTx, "STP,%d,%d,%d", uiActualPosn, uiActualTq, uiDesiredStatus);
    //SocketSendResponse(stringToTx);
}

void SetCommandStatus(UINT32 uirCmdStatus)
{
    unsigned int uilclErrNo = 0;
    unsigned int uiAttemptCntr = 0;
    unsigned int uiPrintCntr = 0;
    char txStr[100];
    sprintf(txStr, "SCS,%d", uirCmdStatus);
    while (uiAttemptCntr < 10)
    {
        //uilclErrNo = fopen_s(&fpOut, "C:\\DND_Endurance\\TqDegCmd.txt", "rb+");
        if (uilclErrNo == 0)
        {
            printf("\n StatUpdated: %d", uirCmdStatus);
            //fseek(fpOut, 8, SEEK_SET);	//Go to beginning of file + 8 Bytes
            //StoreInteger(fpOut, uirCmdStatus);
            //fclose(fpOut);
            uiAttemptCntr = 10000;
        }
        else
        {
            uiAttemptCntr++;
            printf("\nSetCommandStatus: File Open Error: %s", strerror(uilclErrNo));
        }
    }
    if (uiAttemptCntr == 10) {
        for (uiPrintCntr = 0; uiPrintCntr < 1000; uiPrintCntr++)
            printf("\n FAILED TO OPEN THE FILE: SetCommandStatus");
    }
}

void ConvertDegreeOfRotationToCount(uint32 uirRPM, uint32 uirScanIntervalInMsec, uint32 uirDegreesToRotate)
{
    float fLclVar;
    //Convert RPM to RPS
    fLclVar = (float)uirRPM / 60;
    //Convert RPS to Degrees in One Second
    fLclVar = fLclVar * 360;
    //Convert Degrees in One Second to Degrees in scan Interval
    fLclVar = (fLclVar * (float)uirScanIntervalInMsec) / 1000;
    //Convert Degrees to Counts
    fLclVar = fLclVar * (float)29127.11111111;		// 2^20 = 1 Rotation = 1048576, Gear Ration 1:10 ==> 1 Rotation = 10485760, 1 Rotation = 360 Deg. thus 1 Deg =  10485760/360 = 29127.11111111
    uiRotationOffset = (uint32)fLclVar;

    //Convert Degrees to rotate to count
    fLclVar = (float)(uirDegreesToRotate) * (float)29127.11111111;

    //Divide by Rotation offset to get the steps to complete
    fLclVar = fLclVar / (float)uiRotationOffset;

    ui32StepsExecuted = 0;
    ui32StepsProgramed = (uint32)fLclVar;


    //!printf("\n********************** Steps Programed: %d", ui32StepsProgramed);
    //!printf("\n**************** Rotation Offset: %d", uiRotationOffset);
}

UINT32 ValidateInteger(UINT32* uirIntegerToValidate, UINT32 uirIntegerMinVal, UINT32 uirIntegerMaxVal, UINT32 uirFillValIfInvalid)
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

    return uiRetVar;
}

UINT32 ValidateFloat(float* frFloatToValidate, float frFloatMinVal, float frFloatMaxVal, float frFillValIfInvalid)
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
    return uiRetVar;
}

int AKDSetup(uint16 slave)
{
    int retval;
    uint8 u8val;
    //	int8 i8val;
    //	uint32 u32val;
    uint32 uiRet32Arr[10];

    int l32 = sizeof(uiRet32Arr) - 1;
    memset(&uiRet32Arr, 0, 10);
    retval = 0;
    u8val = 0;



    //opPDO = 0x1722
    //ipPDO = 0x1b22
    //setOutputPDO(slave,0x1721);
    //setInputPDO(slave,0x1b22);
    setFlexibleOutputPDO(slave);
    setFlexibleInputPDO(slave);

    //Put the drive in Profile Position Mode
    drv_SetOperationMode(slave, OPN_MODE_INTERPOLATED_POSN);



    //Set the interpolation Time
    /*
    u8val = 250;
    retval += ec_SDOwrite(slave, 0x60C2, 0x01, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    i8val = -3;
    retval += ec_SDOwrite(slave, 0x60C2, 0x02, FALSE, sizeof(i8val), &i8val, EC_TIMEOUTRXM);


    printf("\n OK Till here1\n");


    //Disable Synchronization to get rid of F125 message from time to time
    u32val = 0;
    retval += ec_SDOwrite(slave, 0x36E8, 0x00, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);


    printf("\n OK Till here2\n");


    //Following COntrol switched OFF
    u32val = 0;
    retval += ec_SDOwrite(slave, 0x6065, 0x00, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    printf("\n OK Till here3\n");


    //Set the target Velocity
/*	i32val = 1000;
    retval += ec_SDOwrite(slave, 0x60FF, 0x00, FALSE, sizeof(i32val), &i32val, EC_TIMEOUTRXM);	//Interpolated Time is set
*/
/*
    //Set the target Torque Value
    u16val = 10000;	//This sets the torque to 50%
    retval += ec_SDOwrite(slave, 0x6071, 0x00, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);

*/


//Set the maximum Torque Value
//u16val = 30	;	//This sets the Maximum torque = 3% (54.6 * 3/100) = 1.638 N-m

//u16val = 35	;	//This sets the Maximum torque = 3.5% (54.6 * 3.5/100) = 1.911 N-m

//u16val = 40	;	//This sets the Maximum torque = 4% (54.6 * 4/100) = 2.184 N-m
//u16val = 45	;	//This sets the Maximum torque = 4.5% (54.6 * 4.5/100) = 2.457 N-m
//u16val = 65	;	//This sets the Maximum torque = 6.5% (54.6 * 6.5/100) = 3.549 N-m
//u16val = 75	;	//This sets the Maximum torque = 7.5% (54.6 * 7.5/100) = 4.095 N-m
//u16val = 85	;	//This sets the Maximum torque = 8.5% (54.6 * 8.5/100) = 4.641 N-m
//u16val = 90	;	//This sets the Maximum torque = 9% (54.6 * 9.0/100) = 4.914 N-m
//u16val = 100;	//This sets the Maximum torque = 10% (54.6 * 10.0/100) = 5.46 N-m
//u16val = 110;	//This sets the Maximum torque = 11% (54.6 * 11.0/100) = 6.006 N-m.......This tripped the tq. wrench set at 8
//u16val = 220;	//This sets the Maximum torque = 22% (54.6 * 22.0/100) = 12.0012 N-m.......This tripped the tq. wrench set at 8	


//	retval += ec_SDOwrite(slave, 0x6072, 0x00, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);


    printf("\n OK Till here4\n");

    //Set the Velocity Command Value
    /*

    */
    printf("\n OK Till here5\n");

    u8val = 0; //PL.MODPDIR register set to take the shortest direction
    retval += ec_SDOwrite(slave, 0x3430, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    printf("\n OK Till here6\n");

    /*	//Set the Profile Acceleration Command Value
        u32val = 1000;	//This sets the Velocity to 10 (unit is unknown!)
        retval += ec_SDOwrite(slave, 0x6083, 0x00, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

        //Set the Profile Velocity Command Value
        u32val = 5000;	//This sets the Velocity to 10 (unit is unknown!)
        retval += ec_SDOwrite(slave, 0x6081, 0x00, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);


        //FBUS.PARAM 05
        u32val |= (BIT3 + BIT1);
        retval += ec_SDOwrite(slave, 0x36E9, 0x00, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
            printf("\n OK Till here7\n");
        //Target Position
        /*
        u32val = 265403;
        retval += ec_SDOwrite(slave, 0x60C1, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
        */

        //First Point in the position table is selected
        /*
        u32val = 0;
        retval += ec_SDOwrite(slave, 0x365B, 0x00, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
        */
        //Save Data in NV Memory
        /*u32val = 0;	//This sets the Maximum torque to 80%
        retval += ec_SDOwrite(slave, 0x35EB, 0x00, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
        */
    while (EcatError) printf("%s", ec_elist2string());

    printf("KollMorgen AKD slave %d set, retval = %d\n", slave, retval);
    return 1;
}

/* most basic RT thread for process data, just does IO transfer */
void CALLBACK RTthread(UINT uTimerID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2)
{//1
    int32 i32PosnDiff;
    /*	union{
            int32 hl;
            struct{
                uint8 ll;
                uint8 lh;
                uint8 hl;
                uint8 hh;
            }split;
        }iSplitVar;
    */
    int retval;

    retval = 0;
    IOmap[0]++;
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    rtcnt++;

    if (uiWriteStat != 0)
        uiWriteStat = 0;
    ui32RtThreadSpeedCntr++;

    /* do RT control stuff here */
    uiStatusWd.split.l = *(ec_slave[0].inputs + 8);
    uiStatusWd.split.h = *(ec_slave[0].inputs + 9);
    updateStatus(uiStatusWd.hl);

    iPosActualValue.split.ll = *(ec_slave[0].inputs);
    iPosActualValue.split.lh = *(ec_slave[0].inputs + 1);
    iPosActualValue.split.hl = *(ec_slave[0].inputs + 2);
    iPosActualValue.split.hh = *(ec_slave[0].inputs + 3);

    uiTqActual.split.h = *(ec_slave[0].inputs + 10);
    uiTqActual.split.l = *(ec_slave[0].inputs + 11);




    if (inOP == TRUE)	//indicates that servo is in operational state
    {//2
        DriveEnable();

    }//\2
    else//If the drive is not enabled then set the desired value of position as actual value so as not to start the motor on enable!
    {
        iDesiredPositionVal.hl = iPosActualValue.hl;
        modifyInterpolatedPositionCommandValue(iDesiredPositionVal.hl, 0, 5);
    }

    if (uiDriveStatus == STATE_MACHINE_STAT_OPERATION_ENABLED)
    {//3
        if (uiSetMotionFlag == 0)
        {//4						
            if (uiInterpolationActive == 1)
            {//5
                switch (uiDesiredStatus)
                {//6
                case CMD_STAT_LOADED:
                    //printf("\n CMD_STAT_LOADED \n CMD_STAT_LOADED \n CMD_STAT_LOADED");
                    printf("\nRtn:%d,Des.Tq:%f,dirn:%d,rpm:%d", uiDesiredDegreeOfRtn, fDesiredTq, uiDesiredDirectionOfRtn, uiDesiredRPM);
                    if (ValidateInteger(&uiDesiredDegreeOfRtn, 1, 14400, 1) == 1)	//Valid Degree of Rtn Entered !!!3600 should be 2520
                    {//7
                        if (ValidateFloat(&fDesiredTq, 1, 50, 1) == 1)	//Valid Torque Value is entered
                        {//8								
                            if (ValidateInteger(&uiDesiredDirectionOfRtn, 0, 1, 0) == 1)	//Valid Direction of Rotation Has Been Entered
                            {//9	
                                if (ValidateInteger(&uiDesiredRPM, 10, 30, 10) == 1)
                                {//10
                                    //If the function reaches this point then it indicates that all the parameters have been entered correctly!!											
                                    uiDesiredStatus = CMD_STAT_ACCEPTED;
                                    uiModifyCmdStatusFlag = 1;
                                    printf("\n Command Accepted!");
                                    //Fill up the desired Position Value
                                    ConvertDegreeOfRotationToCount(uiDesiredRPM, 2, uiDesiredDegreeOfRtn);
                                    //!printf("\n PAV Used: %d, Rotation Offset: %d",iPosActualValue.hl, uiRotationOffset);
                                    //!printf("\n Setting the Desired Status!!!");
                                    //modifyInterpolatedPositionCommandValue(iDesiredPositionVal.hl,fDesiredTq,5);	
                                    uiSetMotionFlag = 1;
                                    u16DirnCntr = 0;
                                    uiBlockedRotorCntr = 0;
                                    u16MotionTimeOutCntr = 0;
                                    ui2MsecCntr = 0;
                                    ui64Cntr = 0;
                                    //printf("\n And I wanted the motion!!");
                                }//\10
                                else {
                                    uiDesiredStatus = CMD_STAT_REJECTED;
                                    printf("*****REJECTED*****");
                                    uiModifyCmdStatusFlag = 1;
                                }
                            }//\9
                            else {
                                uiDesiredStatus = CMD_STAT_REJECTED;
                                uiModifyCmdStatusFlag = 1;
                            }
                        }//\8
                        else {
                            uiDesiredStatus = CMD_STAT_REJECTED;
                            uiModifyCmdStatusFlag = 1;
                        }
                    }//\7
                    else {
                        uiDesiredStatus = CMD_STAT_REJECTED;
                        uiModifyCmdStatusFlag = 1;
                    }
                    break;

                case CMD_STAT_ACCEPTED:	//If Cmd Stat is accepted then set motion flag has to be 1. If not reset it... this is an error condition!!!
                    uiDesiredStatus = CMD_STAT_UNKNOWN;
                    uiModifyCmdStatusFlag = 1;
                    //SetCommandStatus(CMD_STAT_UNKNOWN);
                    break;

                default:
                    break;
                }//\6
            }//\5				
        }//\4


        //This section of code was implemented to check that the RtThread really runs every 2 msec.
        /*
        if(ui32RtThreadSpeedCntr >= 1000)
        {
            printf("\n\n\n\n TICK TOCK TICK TOCK: %d - %d\n\n\n",ui32RtThreadSpeedCntr,ui32RtThreadOvFlowCntr);
            ui32RtThreadSpeedCntr = 0;
            ui32RtThreadOvFlowCntr++;
            if(ui32RtThreadOvFlowCntr >= 100)
            {
                ui32RtThreadOvFlowCntr = 0;
                printf("\n\n\n\n OVERFLOW DETECTED!!!!\nOVERFLOW DETECTED!!!!\nOVERFLOW DETECTED!!!!\n\n\n");
            }
        }
        */

        if (uiSetMotionFlag == 1)
        {//4				
            u16MotionTimeOutCntr++;
            iBuffPosValue = iDesiredPositionVal.hl;
            if (uiDesiredDirectionOfRtn == CW)
            {//10
                //ccwEnable = 0;
                //printf("\n\n\n\n\n SUBTRACT \n\n\n\n\n");
                iDesiredPositionVal.hl = iPosActualValue.hl - (uiRotationOffset * 21);//21???

            }//\10
            else
            {//10
                //ccwEnable = 1;
                //printf("\n\n\n\n\n ADD \n\n\n\n\n");
                i64BuffVal = (int64)iPosActualValue.hl + ((int64)uiRotationOffset * 21);//21???											
                iDesiredPositionVal.hl = (int32)i64BuffVal;

            }//\10

            i32PosnDiff = iBuffPosValue - iDesiredPositionVal.hl;
            if (i32PosnDiff < 0) i32PosnDiff *= -1;
            //printf("\n Position Diff: %d",i32PosnDiff); 
            if (i32PosnDiff < 1000)
                uiBlockedRotorCntr++;
            else
                uiBlockedRotorCntr = 0;

            if (uiBlockedRotorCntr > BLOCK_ROTOR_DECLARE_CNT) {
                if ((float)ui32StepsExecuted < ((float)ui32StepsProgramed / 2)) {
                    //!printf("\n***TOO FEW STEPS:%d / %d",ui32StepsExecuted,ui32StepsProgramed);
                }
                ui32StepsExecuted = ui32StepsProgramed + 100; //This effectively says that the job is done if rotor is locked!
            }
            ui32StepsExecuted++;
            modifyInterpolatedPositionCommandValue(iDesiredPositionVal.hl, fDesiredTq, 5);

            //inidicates that the motion is complete
            if (ui32StepsExecuted >= ui32StepsProgramed)
            {
                uiDesiredStatus = CMD_STAT_COMPLETED_POSN;
                uiModifyCmdStatusFlag = 1;
                uiSetMotionFlag = 0;
                ui2MsecCntr = 0;
                //while(1){
                //!printf("\n Position Achieved!::Exec:%d : Pgmd:%d",ui32StepsExecuted,ui32StepsProgramed);
                //}
                ui32StepsProgramed = 0;
                ui32StepsExecuted = 0;
            }
            ui64Cntr++;

            if (u16MotionTimeOutCntr > MOTION_TIMEOUT_VAL)	//Indicates that Motion Has Timed Out
            {//5
                iDesiredPositionVal.hl = iPosActualValue.hl;
                modifyInterpolatedPositionCommandValue(iDesiredPositionVal.hl, 0, 5);
                //while(1){
                printf("\n\n\n\n\nTIME OUTTTTTTTTTTTT\n\n\n\n");
                //}
                uiSetMotionFlag = 0;
                u16MotionTimeOutCntr = 0;
                StopDrive();
                ui2MsecCntr = 0;
            }//\5


            //i32PosnDiff = iPosActualValue.hl - iDesiredPositionVal.hl;
            //if(i32PosnDiff < 0) i32PosnDiff *= -1;	//Convert negative value to +ve

        }//\4


    }//\3
    uiLoopCntr++;
}//\1


void simpletest(char* ifname)
{
    int32 i, j, oloop, iloop, wkc_count, chk, slc, chkCntr;
    UINT mmResult;
    chkCntr = 0;
    needlf = FALSE;
    inOP = FALSE;

    printf("Starting simple test\n");
    resetDesiredTqAndDegOfRtn();

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname);
        /* find and auto-config slaves */


        if (ec_config_init(FALSE) > 0)
        {
            printf("%d slaves found and configured.\n", ec_slavecount);

            if ((ec_slavecount >= 1))	//Earlier it was only >1, thus this loop was not getting executed!
            {
                printf("Entered the if loop\n");
                for (slc = 1; slc <= ec_slavecount; slc++)
                {

                    //AKD Ethercat Servo Motor
                    if ((ec_slave[slc].eep_man == 0x0000006A) && (ec_slave[slc].eep_id == 0x00414B44))
                    {
                        printf("It gives us great pleasure to inform you that KollMorgen Ethercat Servo has been found!!!\n");
                        printf("Found %s at position %d\n", ec_slave[slc].name, slc);
                        // link slave specific setup to preop->safeop hook
                        ec_slave[slc].PO2SOconfig = &AKDSetup;
                    }
                }
            }
            else
            {
                printf("Could not enter the if loop\n");
            }
            ec_config_map(&IOmap);
            ec_configdc();

            printf("Slaves mapped, state to SAFE_OP.\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
            //if (oloop > 8) oloop = 8;//Temporarily commented by ASA to find the actual number of bytes in response
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;

            //if (iloop > 8) iloop = 8;//Temporarily commented by ASA to find the actual number of bytes in response

            printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

            printf("Request operational state for all slaves\n");
            printf("oloop: %d, iloop: %d \n", oloop, iloop);
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);

            /* start RT thread as periodic MM timer */
            mmResult = timeSetEvent(2, 0, RTthread, 0, TIME_PERIODIC);

            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 40;
            /* wait for all slaves to reach OP state */
            do
            {
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                printf("Operational state reached for all slaves.\n");
                wkc_count = 0;
                inOP = TRUE;


                /* cyclic loop, reads data from RT thread */
                for (i = 1; i <= 15000; i++)
                {
                    i--;
                    socketServerAction();
                    if (wkc >= expectedWKC)
                    {
                        //printf("\rDS:%d",uiDesiredStatus);
                        //printf("PDIFF: %d", uiBlockedRotorCntr);
                         //printf("PDC: %4d, WKC: %d , I:", rtcnt, wkc);	//Processdata cycle
                         /*
                         for(j = 0 ; j < iloop; j++)
                             printf("%2.2x", *(ec_slave[0].inputs + j));

                         printf("O:");
                         for(j = 0 ; j < oloop; j++)
                             printf("%2.2x", *(ec_slave[0].outputs + j));
                         */

                         //printf(" PAV: %d, SFP: %d, Vel: %d, MaxTq: %dN-m", uiActualPosn,uiSecondFeedbackPosn.hl,uiVelocity.hl,uiMaxTorque);
                         //printf(" T:%lld\r",ec_DCtime);
                         //printf("IIII: %d\r",i);
                        //printStatus(uiDriveStatus);
                        needlf = TRUE;
                        uiActualPosn = iPosActualValue.hl;
                        uiActualTq = uiTqActual.hl;
                          
                        //Read the file to check if the motion is desired
                        
                        chkCntr++;	//This counter is used to limit the speed at which the file is being accessed!
                        if (chkCntr >= 20)
                        {
                            
                            //if (sktConnectedToClient == true) {
                                SetActualTqAndPosn();
                            //}
                   
                        }
                        
                        if (uiModifyCmdStatusFlag != 0)
                        {
                            SetCommandStatus(uiDesiredStatus);
                            uiModifyCmdStatusFlag = 0;
                        }
                    }
                    //Send the information of the status of the current status of the motor to the Socket Client
                    osal_usleep(50000);
                }
                inOP = FALSE;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for (i = 1; i <= ec_slavecount; i++)
                {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }

            j = 0;

            while (j == 0)
            {
                StopDrive();
                uiStatusWd.split.l = *(ec_slave[0].inputs + 24);
                uiStatusWd.split.h = *(ec_slave[0].inputs + 25);
                updateStatus(uiStatusWd.hl);
                if (uiDriveStatus != STATE_MACHINE_STAT_OPERATION_ENABLED)
                {
                    j = 1000;
                }
            }

            /* stop RT thread */
            timeKillEvent(mmResult);

            printf("\nRequest init state for all slaves\n");
            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End simple test, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n", ifname);
    }
}

OSAL_THREAD_FUNC ecatcheck(void* ptr)
{
    int slave;
    (void)ptr;                  /* Not used */
    

    while (1)
    {
        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                printf("\n");
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
        osal_usleep(10000);
    }
}


void socketServerAction() {
    INT8 bLclTryAgain = FALSE;
    
    switch (uiServerSocketStatus) {
    case SOCKET_SERVER_APP_START_INIT:
        sktConnectedToClient = false;
        printf("\nInitialising Winsock...");
        iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
        if (iResult != 0)
        {
            printf("WSAStartup failed with error : %d\n", iResult);
            WSACleanup();
            bLclTryAgain = TRUE;
        }
        printf("Initialised.\n");

        ZeroMemory(&hints, sizeof(hints));
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        hints.ai_protocol = IPPROTO_TCP;
        hints.ai_flags = AI_PASSIVE;

        // Resolve the server address and port
        iResult = getaddrinfo(NULL, DEFAULT_PORT, &hints, &result);
        if (iResult != 0) {
            printf("getaddrinfo failed with error: %d\n", iResult);
            WSACleanup();
            bLclTryAgain = TRUE;
        }
        if (bLclTryAgain == FALSE) {
            uiServerSocketStatus = SOCKET_SERVER_CREATE_SOCKET_TO_LISTEN_FOR_CLIENT;
            printf("\nSOCKET_SERVER_CREATE_SOCKET_TO_LISTEN_FOR_CLIENT");
        }
        break;
    case SOCKET_SERVER_CREATE_SOCKET_TO_LISTEN_FOR_CLIENT:
        printf("\nInitialising Winsock...");
        // Create a SOCKET for the server to listen for client connections.
        ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
        if (ListenSocket == INVALID_SOCKET) {
            printf("socket failed with error: %ld\n", WSAGetLastError());
            freeaddrinfo(result);
            WSACleanup();
            bLclTryAgain = TRUE;
        }

        if (bLclTryAgain == TRUE) {
            uiServerSocketStatus = SOCKET_SERVER_CREATE_SOCKET_FAILURE_ACTION;
            printf("\nSOCKET_SERVER_CREATE_SOCKET_FAILURE_ACTION");

        }
        else {
            uiServerSocketStatus = SOCKET_SERVER_SETUP_LISTENING_SOCKET;
            printf("\nSOCKET_SERVER_CREATE_SOCKET_FAILURE_ACTION");
        }

        break;
    case SOCKET_SERVER_CREATE_SOCKET_FAILURE_ACTION:
        bLclTryAgain = WSAGetLastError();
        switch (bLclTryAgain) {
        case WSANOTINITIALISED:
            uiServerSocketStatus = SOCKET_SERVER_APP_START_INIT;
            printf("\nSOCKET_SERVER_APP_START_INIT");
            break;
        default:
            uiServerSocketStatus = SOCKET_SERVER_CREATE_SOCKET_TO_LISTEN_FOR_CLIENT;
            printf("\nSOCKET_SERVER_CREATE_SOCKET_TO_LISTEN_FOR_CLIENT");
            break;
        }
        break;
    case SOCKET_SERVER_SETUP_LISTENING_SOCKET:
        // Setup the TCP listening socket
        iResult = bind(ListenSocket, result->ai_addr, (int)result->ai_addrlen);
        if (iResult == SOCKET_ERROR) {
            printf("bind failed with error: %d\n", WSAGetLastError());
            freeaddrinfo(result);
            closesocket(ListenSocket);
            WSACleanup();
            uiServerSocketStatus = SOCKET_SERVER_APP_START_INIT;
            printf("\nSOCKET_SERVER_APP_START_INIT");

        }
        else {
            printf("\nbind result: %d\n", iResult);
            freeaddrinfo(result);
            uiServerSocketStatus = SOCKET_SERVER_LISTEN_FOR_CLIENT_SOCKET;
            printf("\nSOCKET_SERVER_LISTEN_FOR_CLIENT_SOCKET");

        }
        break;

    case SOCKET_SERVER_LISTEN_FOR_CLIENT_SOCKET:
        iResult = listen(ListenSocket, SOMAXCONN);
        if (iResult == SOCKET_ERROR) {
            printf("listen failed with error: %d\n", WSAGetLastError());
            closesocket(ListenSocket);
            WSACleanup();
            uiServerSocketStatus = SOCKET_SERVER_APP_START_INIT;
            printf("\nSOCKET_SERVER_APP_START_INIT");

        }
        else {
            uiServerSocketStatus = SOCKET_SERVER_ACCEPT_CLIENT_SOCKET;
            printf("\nSOCKET_SERVER_ACCEPT_CLIENT_SOCKET");
        }
        break;

    case SOCKET_SERVER_ACCEPT_CLIENT_SOCKET:
        // Accept a client socket
        ClientSocket = accept(ListenSocket, NULL, NULL);
        if (ClientSocket == INVALID_SOCKET) {
            printf("accept failed with error: %d\n", WSAGetLastError());
            closesocket(ListenSocket);
            WSACleanup();
            uiServerSocketStatus = SOCKET_SERVER_APP_START_INIT;
            printf("\nSOCKET_SERVER_APP_START_INIT");

        }
        else {
            uiDelMeCntr[2]++;
            sktConnectedToClient = true;
            uiServerSocketStatus = SOCKET_SERVER_GET_DATA_FROM_CLIENT;
            printf("\nSOCKET_SERVER_GET_DATA_FROM_CLIENT");
        }
        // No longer need server socket
        closesocket(ListenSocket);
        break;

    case SOCKET_SERVER_GET_DATA_FROM_CLIENT:
        iResult = recv(ClientSocket, recvbuf, recvbuflen, 0);
        uiDelMeCntr[1]++;
        if (iResult > 0) {
            printf("buf: %s ... Done", recvbuf);
            uiServerSocketStatus = SOCKET_SERVER_ANALYZE_QUERY;
            printf("\nSOCKET_SERVER_ANALYZE_QUERY");
        }
        else {
            if (iResult == 0) {
                uiServerSocketStatus = SOCKET_SERVER_CLOSE_SOCKET;
            }
            else { //Indicates receive failure with error!
                printf("recv failed with error: %d\n", WSAGetLastError());
                closesocket(ClientSocket);
                WSACleanup();
                uiServerSocketStatus = SOCKET_SERVER_APP_START_INIT;
                printf("\nSOCKET_SERVER_APP_START_INIT");

            }
        }
        break;
    case SOCKET_SERVER_ANALYZE_QUERY:
        printf("Query analyze...\n");
        GetDesiredTqAndDegOfRtn();  
        //uiServerSocketStatus = SOCKET_SERVER_GET_DATA_FROM_CLIENT;
        //uiServerSocketStatus = SOCKET_SERVER_SEND_STATUS;
        uiDelMeCntr[0]++;
        

        break;
    case SOCKET_SERVER_SEND_STATUS:
        SetActualTqAndPosn();
        break;
    case SOCKET_SERVER_CLOSE_SOCKET:
        // shutdown the connection since we're done
        iResult = shutdown(ClientSocket, SD_SEND);
        if (iResult == SOCKET_ERROR) {
            printf("shutdown failed with error: %d\n", WSAGetLastError());
            closesocket(ClientSocket);
            WSACleanup();
            uiServerSocketStatus = SOCKET_SERVER_APP_START_INIT;
            printf("\nSOCKET_SERVER_APP_START_INIT:%d,%d,%d", uiDelMeCntr[0], uiDelMeCntr[1], uiDelMeCntr[2]);

        }
        break;
    default:
        WSACleanup();
        uiServerSocketStatus = SOCKET_SERVER_APP_START_INIT;
        printf("\nSOCKET_SERVER_APP_START_INIT");
        break;
    }

}

void SocketSendResponse(char* strDataToSend) {
    iSendResult = send(ClientSocket, strDataToSend, iResult, 0);
    if (iSendResult == SOCKET_ERROR) {
        printf("send failed with error: %d\n", WSAGetLastError());
        closesocket(ClientSocket);
        WSACleanup();
        uiServerSocketStatus = SOCKET_SERVER_APP_START_INIT;
        printf("\nSOCKET_SERVER_APP_START_INIT");

    }
    else {
        printf("Bytes sent: %d\n", iSendResult);
        uiServerSocketStatus = SOCKET_SERVER_GET_DATA_FROM_CLIENT;
        printf("\nSOCKET_SERVER_GET_DATA_FROM_CLIENT");

    }
}


int main(int argc, char* argv[])
{
    printf("Endurance 2\n");
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");
    printf("argc:%d \n", argc);
    
    if (argc > 1)
    {
        // create thread to handle slave error handling in OP 
  //      pthread_create( &thread1, NULL, (void *) &ecatcheck, (void*) &ctime);
        printf("Creating thread...\n");
        osal_thread_create(&thread1, 128000, &ecatcheck, (void*)&ctime);
        printf("Starting cyclic part...\n");
        // start cyclic part 
        simpletest(argv[1]);
    }
    else
    {
        ec_adaptert* adapter = NULL;
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
