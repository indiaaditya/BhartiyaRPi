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
#include <math.h>
#pragma comment(lib,"ws2_32.lib") //Winsock Library


#ifdef _WIN32
#include <Windows.h>
#endif

#define EC_TIMEOUTMON 500

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "27015"

#define ENCODER_RESOLUTION_IN_BITS                              23
#define GEAR_RATIO                                              10
#define MOTOR_TORQUE_RATING                                     5.46

//There is some confusion regarding this value.
//Used in 2 places:
//OSAL Sleep Cycle: Here the value works as it is
//for calculation: the value needs to be 1/10th
#define SCAN_INTERVAL_IN_MSEC                                   20//5  //FBUS.SAMPLEPERIOD //1
#define SOCKET_SCAN_CYCLES                                      20

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
#define CTL_WD_READY_TO_SWITCH_ON                   10


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

#define CTL_WD_BIT_SWITCH_ON    BIT0
#define CTL_WD_BIT_ENABLE_VTG   BIT1
#define CTL_WD_BIT_QUICK_STOP   BIT2
#define CTL_WD_BIT_ENABLE_OPN   BIT3
#define CTL_WD_BIT_FAULT_RESET  BIT7
#define CTL_WD_BIT_HALT         BIT8

#define CTL_WD_BIT_PP_SPECIFIC_NEW_SETPOINT             BIT4
#define CTL_WD_BIT_PP_SPECIFIC_CHANGE_SET_IMMEDIATELY   BIT5
#define CTL_WD_BIT_PP_SPECIFIC_ABS_REL                  BIT6
#define CTL_WD_BIT_PP_SPECIFIC_CHANGE_ON_SETPOINT       BIT9




#define	OPN_MODE_PROFILE_VELOCITY					1   //Available for Panasonic
#define	OPN_MODE_INTERPOLATED_POSN					2   //Not Available for Panasonic
#define	OPN_MODE_HOMING_MODE						3   //Available for Panasonic
#define	OPN_MODE_PROFILE_POSN						4   //
#define	OPN_MODE_PROFILE_TORQUE						5
#define	OPN_MODE_CYCLIC_SYNCH_POSN					6
//Modes added for Panasonic
#define OPN_MODE_VELOCITY                           7
#define	OPN_MODE_CYCLIC_SYNCH_VELOCITY              8
#define	OPN_MODE_CYCLIC_SYNCH_TORQUE                9

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
#define REG_DISP_MODE_OF_OPN                            0x6061

//Registers Associated with Profile Position Mode
#define REG_PROF_POSN_MODE_TARGET_POSN					0x607A	
#define REG_PROF_POSN_MODE_SW_POSN_LIMIT				0x607D
#define REG_PROF_POSN_MODE_VELOCITY						0x6081
#define REG_PROF_POSN_MODE_ACCELERATION					0x6083
#define REG_PROF_POSN_MODE_DECELRATION					0x6084

#define REG_WATCHDOG_MULTIPLIER                         0x400
#define REG_WATCHDOG_TIME_PROCESS_DATA                  0x420


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

#define MOTION_TIMEOUT_VAL		90000
#define BLOCK_ROTOR_DECLARE_CNT	1000	//This constant will be used for the number of samples for which the rotor consequtively has not achieved its desired position!


#define PDO_WRITE_CTL_WD_OFFSET         0
#define PDO_WRITE_POSN_OPCODE_OFFSET    2
#define PDO_WRITE_MAX_TQ                4
#define PDO_WRITE_TARGET_POSN           6
#define PDO_WRITE_MAX_SPEED             8
#define PDO_WRITE_PROFILE_VEL           12

#define PDO_READ_POSN_ACT_VALUE         0
#define PDO_READ_VEL_ACT_VALUE          4
#define PDO_READ_STATUS                 8
#define PDO_READ_TQ_ACT_VALUE           10
#define PDO_READ_ANALOG_IP_VALUE        12



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
uint16 uiStatusResetCntr = 0;
union {
    uint16 hl;
    struct {
        uint8 l;
        uint8 h;
    }split;
}uiMaxTorque, uiStatusWd, uiTqOffset, uiTqActual;

uint8 uiModesOfOperationDisplay;
 
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

int iSocketElapsedCntr = 0;
int iMotionCompletedFlag = 0;
//int uiDelMeCntr[10];


void fillMotionParams(uint32 uirDesiredPosnVal, uint16 uirTqFeedFwd, uint16 uirMaxTq);
uint16 setLimitingTorqueValue(float frDesiredTorque, float frMotorMaxTorque, uint16 uirGearRatio);
void updateStatus(uint16 uirStatus);
void printStatus(uint16 uirDriveStat);
void modifyControlWord(uint16 uirSlave, uint16 uirDesiredStat, uint8 uiOffset);
//void setProfilePositionParameters(uint16 uirSlave, int32 irTargetPosn, int32 irSW_PosnLimit1, int32 irSW_PosnLimit2, uint32 uirProfileVelocity, uint32 uirProfileAcceleration, uint32 uirProfileDeceleration);
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
void drv_SetOperationMode(uint16 slave, int32 irSelectedMode);
void resetDesiredTqAndDegOfRtn();
void GetDesiredTqAndDegOfRtn(void);
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
int32 calculateFinalDesiredPosn(uint32 ui32rDesRtn);
void drv_setPDOTimeout();

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
        if (ui32StepsExecuted > 2500) {
            printf("\tTQ Val Set: %f", flclValToSet);
        }
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

    printf("NNNNStatus: % x", uiLclLowerNibbleStat);
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
            drv_SetOperationMode(slave, OPN_MODE_PROFILE_POSN);//080523
            

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


    //Commented because not applicable to Panasonic Servo
    /*if (uiInterpolationEnableFlag == 1)
    {
        if ((uirStatus & BIT12) == BIT12)
            uiInterpolationActive = 1;
        else
            uiInterpolationActive = 0;
    }*/

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
    printf(" Status:   %d rawStat: %d, inop: %d", uirDriveStat, uiStatusWd.hl, inOP);

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
    printf("PAV: %d Pde:%d status: %x tq:%d MoOpnDisp:%d\n",iPosActualValue.hl,iDesiredPositionVal.hl,uiStatusWd.hl, uiTqActual.hl, uiModesOfOperationDisplay);
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

    uiCtlWd.hl = 0;
    uint16 uiRetArr[10], uilclRetVar;
    uint32 uilclModifyFlag = 1;
    int l = sizeof(uiRetArr) - 1;
    memset(&uiRetArr, 0, 10);
    //ec_SDOread(uirSlave, 0x6040, 0x00, FALSE, &l, &uiRetArr, EC_TIMEOUTRXM);

    //uiCtlWd.hl = uiRetArr[0];
    uilclRetVar = 0;


    switch (uirDesiredStat)
    {
    case CTL_WD_SHUT_DOWN:
        uiCtlWd.hl = 0;
        uiCtlWd.split.l |= (CTL_WD_BIT_QUICK_STOP + CTL_WD_BIT_ENABLE_VTG);
        uiCtlWd.split.l &= ~CTL_WD_BIT_SWITCH_ON;
        break;
    case CTL_WD_SWITCH_ON:
        uiCtlWd.hl = 0;
        uiCtlWd.split.l |= (CTL_WD_BIT_QUICK_STOP + CTL_WD_BIT_ENABLE_VTG + CTL_WD_BIT_SWITCH_ON);
        break;

    case CTL_WD_DISABLE_VTG:
        uiCtlWd.split.h = 0;
        uiCtlWd.split.l &= ~CTL_WD_BIT_ENABLE_VTG;
        break;

    case CTL_WD_QUICK_STOP:
        uiCtlWd.split.h = 0;
        uiCtlWd.split.l |= CTL_WD_BIT_ENABLE_VTG;
        uiCtlWd.split.l &= ~CTL_WD_BIT_QUICK_STOP;
        break;

    case CTL_WD_DISABLE_OPN:
        uiCtlWd.split.h = 0;
        uiCtlWd.split.l |= (CTL_WD_BIT_QUICK_STOP + CTL_WD_BIT_ENABLE_VTG + CTL_WD_BIT_SWITCH_ON);
        uiCtlWd.split.l &= ~CTL_WD_BIT_ENABLE_OPN;
        break;

    case CTL_WD_ENABLE_OPN:
        uiCtlWd.split.h = 0;
        uiCtlWd.split.l |= (CTL_WD_BIT_ENABLE_OPN + CTL_WD_BIT_QUICK_STOP + CTL_WD_BIT_ENABLE_VTG + CTL_WD_BIT_SWITCH_ON);
        break;

  /*    Not Supported in Panasonic Motor
    case CTL_WD_ENABLE_IP:
        uiCtlWd.split.h = 0;
        uiCtlWd.split.l |= (BIT4 + BIT3 + BIT2 + BIT1 + BIT0);
        break;
    */
    case CTL_WD_FAULT_RESET:
        uiCtlWd.split.h = 0;
        uiCtlWd.split.l |= CTL_WD_BIT_FAULT_RESET;
        printf("\nReset....%d", uiCtlWd.hl);
        break;

    case CTL_WD_STOP:
        uiCtlWd.split.h = 0;
        uiCtlWd.split.l |= CTL_WD_BIT_HALT;
        break;

    case CTL_WD_READY_TO_SWITCH_ON:
        uiCtlWd.hl = 0;
        uiCtlWd.split.l |= (CTL_WD_BIT_QUICK_STOP + CTL_WD_BIT_ENABLE_VTG);
        break;
    default:
        uilclModifyFlag = 0;
        break;

    }
    *(ec_slave[0].outputs + PDO_WRITE_CTL_WD_OFFSET) = uiCtlWd.split.l;
    *(ec_slave[0].outputs + (PDO_WRITE_CTL_WD_OFFSET + 1)) = uiCtlWd.split.h;

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
    //printf("\tIP");
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
    iSplitVar.hl = setLimitingTorqueValue(frMaxTq, (float)MOTOR_TORQUE_RATING, GEAR_RATIO);
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
    printf("**DE**");
    switch (uiDriveStatus)
    {

    case STATE_MACHINE_STAT_SWITCH_ON_DISABLED:	//Nothing can be done over Ethercat to change the state!
        //uiDesiredStat = CTL_WD_SHUT_DOWN;//
        //drv_SetOperationMode(slave, OPN_MODE_PROFILE_POSN);
        uiDesiredStat = CTL_WD_READY_TO_SWITCH_ON;
        break;
    case STATE_MACHINE_STAT_RDY_TO_SWITCH_ON:
        uiDesiredStat = CTL_WD_SWITCH_ON;
        break;
    case STATE_MACHINE_STAT_SWITCHED_ON:
        uiDesiredStat = CTL_WD_ENABLE_OPN;
        break;
    case STATE_MACHINE_STAT_OPERATION_ENABLED:
        //If loop commented because IP mode not applicable for Servo.
        /*if (uiInterpolationEnableFlag == 1)
        {
            uiDesiredStat = CTL_WD_ENABLE_IP;
        }
       else
        {*/
            uiDesiredStat = CTL_WD_ENABLE_OPN;
        //}
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

  //The meaning of the data (for example 0x60410010 in the mapping of 0x1A00 sub 1) is as follows:
  //0x6041 is the index of the DS402 status word
  //0x00 is the subindex of the DS402 status word
  //0x10 is the number of bits for this entry, i. e. 16 bits or 2 bytes.


    uint32 u8val;
    uint16 u16val;
    uint32 u32val;
    int retval = 0;
    u8val = 0;
    u32val = 0;

    //Mapping Section is cleared
    retval += ec_SDOwrite(uirSlave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    //Clear Output PDO's
    ec_SDOwrite(uirSlave, 0x1600, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    ec_SDOwrite(uirSlave, 0x1601, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    ec_SDOwrite(uirSlave, 0x1602, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    ec_SDOwrite(uirSlave, 0x1603, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    //Receive PDO is FROM Master to Drive
    //Transmit PDO is FROM Drive to Master

    //Map the fields here using the following logic: (Panasonic)
    //Map ControlWord: (pg. 117) 
    //Index: 0x6040
    //SubIndex: 0x00
    //number of bits: 0x10, 16 bits or 2 bytes
    u32val = 0x60400010;
    ec_SDOwrite(uirSlave, 0x1600, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    /*
    //Map Positioning option code:
    //Index: 0x60F2
    //SubIndex: 0x00
    //number of bits: 0x10, 16 bits or 2 bytes
    u32val = 0x60F20010;
    ec_SDOwrite(uirSlave, 0x1600, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    //Max Torque
    //Index: 0x6072
    //SubIndex: 0x00
    //number of bits: 0x10, 16 bits or 2 bytes
    u32val = 0x60720010;
    ec_SDOwrite(uirSlave, 0x1601, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    //Target Position
    //Index: 0x607A
    //SubIndex: 0x00
    //number of bits: 0x20, 32 bits or 4 bytes
    u32val = 0x607A0020;
    ec_SDOwrite(uirSlave, 0x1601, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    //Max Motor Speed
   //Index: 0x6080
   //SubIndex: 0x00
   //number of bits: 0x20, 32 bits or 4 bytes
    u32val = 0x60800020;
    ec_SDOwrite(uirSlave, 0x1602, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    //Profile Velocity
    //Index: 0x6081
    //SubIndex: 0x00
    //number of bits: 0x20, 32 bits or 4 bytes
    u32val = 0x60810020;
    ec_SDOwrite(uirSlave, 0x1602, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    //6060 ++
    //Index: 0x6060 
    //SubIndex: 0x00
    //number of bits: 0x08, 8 bits
    //u32val = 0x606008;
    ec_SDOwrite(uirSlave, 0x1603, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    */
    u8val = 2;//Since 1600 maps 2 Variables
    u8val = 1;//Since 1600 maps 2 Variables
    ec_SDOwrite(uirSlave, 0x1600, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);	//Since 1600 maps 2 Variables

    /*
    u8val = 2;//Since 1601 maps 2 Variables
    ec_SDOwrite(uirSlave, 0x1601, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);	//Since 1601 maps 2 Variables

    u8val = 2;//Since 1602 maps 2 Variables
    ec_SDOwrite(uirSlave, 0x1602, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);	//Since 1602 maps 2 Variables

    u8val = 1;//Since 1603 maps 1 Variables
    ec_SDOwrite(uirSlave, 0x1603, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);	//Since 1602 maps 2 Variables
    */
    //Map PDO 1600 to RxPDO Assign
    //This makes the 1600 PDO mapping acitve
    //Set the number of PDO's to 1
    //1C12.0 ==> Highest Sub index Supported
    u8val = 0x01;
    ec_SDOwrite(uirSlave, 0x1C12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    //1C12.1 ==> PDO Mapping Index of PDO 1
    u16val = 0x1600;
    ec_SDOwrite(uirSlave, 0x1C12, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    /*
    //1C12.2 ==> PDO Mapping Index of PDO 2
    u16val = 0x1601;
    ec_SDOwrite(uirSlave, 0x1C12, 0x02, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);

    //1C12.3 ==> PDO Mapping Index of PDO 3
    u16val = 0x1602;
    ec_SDOwrite(uirSlave, 0x1C12, 0x03, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);

    u16val = 0x1603;
    ec_SDOwrite(uirSlave, 0x1C12, 0x04, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    */




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
    //u32val = 0x34700410;
         u32val = 0x60610008;
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

   
}

void drv_SetOperationMode(uint16 slave, int32 irSelectedMode)
{
    //int16 tmp1, tmp2;
    int ln = 0;
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
        break;
    case OPN_MODE_INTERPOLATED_POSN:
        iModeOfOpnParam = 7;
        uiInterpolationEnableFlag = 1;
        break;
    case OPN_MODE_HOMING_MODE:
        iModeOfOpnParam = 6;
        break;
    case OPN_MODE_PROFILE_POSN:
        iModeOfOpnParam = 1;
        break;
    case OPN_MODE_PROFILE_TORQUE:
        iModeOfOpnParam = 4;
        break;
    case OPN_MODE_CYCLIC_SYNCH_POSN:
        iModeOfOpnParam = 8;
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
    printf("\n\n\n\n\n\n\n\n\n\n Value of iModeOfOpnParam: %d", iModeOfOpnParam);
    if (iModeOfOpnParam != 0)
    {
        //REG_MODE_OF_OPN = 0x6060
        retval += ec_SDOwrite(slave, REG_MODE_OF_OPN, 0x00, FALSE, sizeof(iModeOfOpnParam), &iModeOfOpnParam, EC_TIMEOUTRXM);
        printf("\n Return val of ec_SDOwrite: %d", retval);

        //retval += ec_SDOwrite(slave, REG_MODE_OF_OPN, 0x00, FALSE, sizeof(iModeOfOpnParam), &iModeOfOpnParam, EC_TIMEOUTRXM);
        //printf("\n Return val of ec_SDOwrite: %d", retval);

        //retval += ec_SDOwrite(slave, REG_MODE_OF_OPN, 0x00, FALSE, sizeof(iModeOfOpnParam), &iModeOfOpnParam, EC_TIMEOUTRXM);
        //printf("\n Return val of ec_SDOwrite: %d", retval);

        //retval += ec_SDOwrite(slave, REG_DISP_MODE_OF_OPN, 0x00, FALSE, sizeof(iModeOfOpnParam), &iModeOfOpnParam, EC_TIMEOUTRXM);
        //printf("\n Return val of ec_SDOwrite: %d", retval);
        //drv_setPDOTimeout();
    }
    else
    {
        printf("FAILED TO SET THE MODE.... NOT SUPPORTED!!!");
    }

}

void drv_setPDOTimeout() {
    uint16_t pdo_timeout = 500; // set PDO timeout to 500 ms
    uint16_t lclIndex = 0x400; // PDO timeout register index
    uint8_t lclSubindex = 0x0; // PDO timeout register subindex
    uint8_t lclData[2]; // data buffer to hold the PDO timeout value

    // convert the PDO timeout value to little-endian byte order
    lclData[0] = pdo_timeout & 0xff;
    lclData[1] = (pdo_timeout >> 8) & 0xff;

    // write the PDO timeout value to the Servo's object dictionary
    if (ec_SDOwrite(slave, lclIndex, lclSubindex, FALSE, 2, lclData, EC_TIMEOUTRXM) == 0) {
        printf("Successfully set PDO timeout to %d ms.\n", pdo_timeout);
    }
    else {
        printf("Failed to set PDO timeout.\n");
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
        if (strncmp(recvbuf, cmdNoActionStr, 3) == 0) {
            printf("...No action desired");
        }
        else {
            printf("Invalid Cmd received");
        }
    }
}

void SetActualTqAndPosn()
{

    if (uiDesiredStatus == CMD_STAT_COMPLETED_POSN) {
        uiStatusResetCntr++;
        if (uiStatusResetCntr > 3) {
            uiStatusResetCntr = 0;
            uiDesiredStatus = CMD_STAT_UNKNOWN;
        }
    }

    char stringToTx[100];
    printf("\nActPosn: %d, ACT_TQ:%d, DesStat: %d", uiActualPosn, uiActualTq, uiDesiredStatus);
    sprintf(stringToTx, "STP,%d,%d,%d", uiActualPosn, uiActualTq, uiDesiredStatus);
    printf("\n Calling fn SocketSendResponse...");
    SocketSendResponse(stringToTx);
}

void SetCommandStatus(UINT32 uirCmdStatus)
{
    unsigned int uilclErrNo = 0;
    unsigned int uiAttemptCntr = 0;
    unsigned int uiPrintCntr = 0;
    char txStr[100];
    sprintf(txStr, "SCS,%d", uirCmdStatus);
}

void ConvertDegreeOfRotationToCount(uint32 uirRPM, uint32 uirScanIntervalInMsec, uint32 uirDegreesToRotate)
{
    float fLclVar;
    float fDegreesPerScanInterval = 0;
    //Refer Sheet 2 of SOEM Calculations Sheet 2 in SOEM-Calculations folder
    //Step 1: Convert RPM to RPS
    fLclVar = (float)uirRPM / 60;
    printf("\n Step 1: %f", fLclVar);
    //Step 2: Convert RPS to Degrees in One Second
    fLclVar = fLclVar * 360;
    printf("\n Step 2: %f", fLclVar);

    //Step 3: Convert Degrees in One Second to Degrees in scan Interval
    fLclVar = fLclVar * ((float)uirScanIntervalInMsec / 1000);
    fDegreesPerScanInterval = fLclVar;
    printf("\n Step 3: %f", fLclVar);

    //Step 4: Count to be traversed in 1 Scan interval
    fLclVar = (float)(((fLclVar * pow(2, ENCODER_RESOLUTION_IN_BITS) * GEAR_RATIO)) / 360);
    uiRotationOffset = (uint32)fLclVar;
    printf("\n Step 4: %f", fLclVar);

    //Step 5: Steps required for completing the motion
    ui32StepsExecuted = 0;
    ui32StepsProgramed = (int32)((float)uirDegreesToRotate/fDegreesPerScanInterval);
    ui32StepsProgramed = ui32StepsProgramed /* + (int)(0 * (double)ui32StepsProgramed)*/;
    //Debug: Reducing for debug purposes
    //ui32StepsProgramed *= 2;
    //uiRotationOffset = uiRotationOffset / 1000;

    
    
    printf("\n********************** Steps Programed: %d", ui32StepsProgramed);
    printf("\n**************** Rotation Offset: %d", uiRotationOffset);
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

int32 calculateFinalDesiredPosn(uint32 ui32rDesRtn) {
    int32 retVal;
    printf("Total Steps:%d", ui32StepsProgramed);
    printf("Rotation Offset:%d", uiRotationOffset);

    uint32 uLclTotalSteps = ui32StepsProgramed * uiRotationOffset;
    if (ui32rDesRtn == CW) {
        retVal = iPosActualValue.hl - uLclTotalSteps;
    }
    else {
        retVal = iPosActualValue.hl + uLclTotalSteps;
    }

    return retVal;
}



int PanasonicSetup(uint16 slave)
{
    int retval;
    uint8 u8val;
    //	int8 i8val;
    //	uint32 u32val;
    uint32 uiRet32Arr[10];
    printf("\n This is the slave parameter: %d", slave);
    int l32 = sizeof(uiRet32Arr) - 1;
    memset(&uiRet32Arr, 0, 10);
    retval = 0;
    u8val = 0;

    drv_SetOperationMode(slave, OPN_MODE_PROFILE_POSN);

    //opPDO = 0x1722
    //ipPDO = 0x1b22
    //setOutputPDO(slave,0x1721);
    //setInputPDO(slave,0x1b22);
    setFlexibleOutputPDO(slave);
    setFlexibleInputPDO(slave);

    //Put the drive in Profile Position Mode
    
    //drv_SetOperationMode(slave, OPN_MODE_PROFILE_VELOCITY);


    printf("\n OK Till here4\n");


    printf("\n OK Till here5\n");

    /*u8val = 0; //PL.MODPDIR register set to take the shortest direction
    retval += ec_SDOwrite(slave, 0x3430, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    */
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

    printf("Panasonic Servo slave %d set, retval = %d\n", slave, retval);
    return 1;
}

/* most basic RT thread for process data, just does IO transfer */
void CALLBACK RTthread(UINT uTimerID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2)
{//1
    int32 i32PosnDiff = 0;
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

    uiModesOfOperationDisplay = *(ec_slave[0].inputs + 12);

    *(ec_slave[0].outputs + 16) = OPN_MODE_PROFILE_POSN;






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
            //if (uiInterpolationActive == 1)
            //{//5
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
                                if (ValidateInteger(&uiDesiredRPM, 5, 30, 10) == 1)
                                {//10
                                    //If the function reaches this point then it indicates that all the parameters have been entered correctly!!											
                                    uiDesiredStatus = CMD_STAT_ACCEPTED;
                                    uiModifyCmdStatusFlag = 1;
                                    printf("\n Command Accepted!");
                                    //Fill up the desired Position Value
                                    ConvertDegreeOfRotationToCount(uiDesiredRPM, SCAN_INTERVAL_IN_MSEC, uiDesiredDegreeOfRtn);
                                    iFinalPosnDesired = calculateFinalDesiredPosn(uiDesiredDirectionOfRtn);
                                    //!printf("\n PAV Used: %d, Rotation Offset: %d",iPosActualValue.hl, uiRotationOffset);
                                    //!printf("\n Setting the Desired Status!!!");
                                    //modifyInterpolatedPositionCommandValue(iDesiredPositionVal.hl,fDesiredTq,5);	
                                    uiSetMotionFlag = 1;
                                    printf("\n\n\n ****SMF: 1");
                                    while (1);  //Debug!
                                    //Set the desired value once here.
                                    /*if (uiDesiredDirectionOfRtn == CW) {
                                        iDesiredPositionVal.hl = iPosActualValue.hl - uiRotationOffset;
                                    }
                                    else {
                                        iDesiredPositionVal.hl = iPosActualValue.hl + uiRotationOffset;
                                    }*/
                                    iDesiredPositionVal.hl = iPosActualValue.hl;

                                    iMotionCompletedFlag = 0;

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
                    printf("\n In CMD_STAT_ACCEPTED....");
                    //SetCommandStatus(CMD_STAT_UNKNOWN);
                    break;

                default:
                    break;
                }//\6
            //}//\5				
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

            if (uiDesiredDirectionOfRtn == CW) {
                iDesiredPositionVal.hl = iDesiredPositionVal.hl - uiRotationOffset;
            }
            else {
                iDesiredPositionVal.hl = iDesiredPositionVal.hl + uiRotationOffset;
            }
            if (ui32StepsExecuted > 2500) {
                printf("\n DP: %d", iDesiredPositionVal.hl);
            }

            //Blocked Rotor Detection Logic Start
            /*
            i32PosnDiff = iBuffPosValue - iPosActualValue.hl;
            if (i32PosnDiff < 0) i32PosnDiff *= -1;
            printf("\nPos Diff: %d", i32PosnDiff);
            if (i32PosnDiff > 100000) { //Indicates very small movement was achieved
                uiBlockedRotorCntr++;
                printf("\n blocked rotor: %d", uiBlockedRotorCntr);
            }
            else
                uiBlockedRotorCntr = 0;

            if (uiBlockedRotorCntr > BLOCK_ROTOR_DECLARE_CNT) {
                ui32StepsExecuted = ui32StepsProgramed + 100; //This effectively says that the job is done if rotor is locked!
            }
            */
            //Blocked Rotor Detection Logic End
            ui32StepsExecuted++;
            modifyInterpolatedPositionCommandValue(iDesiredPositionVal.hl, fDesiredTq, 5);
            if (ui32StepsExecuted > 2500) {
                printf("\tFP: %d", iFinalPosnDesired);
                printf("\tSteps Ex: %d", ui32StepsExecuted);
                printf("\tAV:%ld", iPosActualValue.hl);
            }

            if (uiDesiredDirectionOfRtn == CW) {
                if (iPosActualValue.hl <= iFinalPosnDesired) {
                    iMotionCompletedFlag = 1;
                    printf("\nACSCSCSCSC AV: %d FP: %d", iPosActualValue.hl, iFinalPosnDesired);
                }
            }
            else {
                if (iPosActualValue.hl >= iFinalPosnDesired) {
                    iMotionCompletedFlag = 1;
                    printf("\nBBBBCSCSCSCSC AV: %d FP: %d", iPosActualValue.hl, iFinalPosnDesired);
                }
            }

            //inidicates that the motion is complete
            //if(ui32StepsExecuted >= 2000)
            //if ((ui32StepsExecuted >= (ui32StepsProgramed)))
            if ((ui32StepsExecuted >= (ui32StepsProgramed)))
            {
                printf("\nSE: %d", ui32StepsExecuted);
                iMotionCompletedFlag = 1;
                printf("\nXXXZZZCSCSCSCSC");
            }

            if (iMotionCompletedFlag == 1) {
                uiDesiredStatus = CMD_STAT_COMPLETED_POSN;
                uiStatusResetCntr = 0;
                uiModifyCmdStatusFlag = 1;
                uiSetMotionFlag = 0;
                ui2MsecCntr = 0;
                ui32StepsProgramed = 0;
                ui32StepsExecuted = 0;
                iMotionCompletedFlag = 0;
                modifyInterpolatedPositionCommandValue(iDesiredPositionVal.hl, 0, 5);
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

            if (ec_slavecount > 0)	//Earlier it was only >1, thus this loop was not getting executed!
            {
                printf("Entered the if loop\n");
                for (slc = 1; slc <= ec_slavecount; slc++)
                {

                    //AKD Ethercat Servo Motor
                    //if ((ec_slave[slc].eep_man == 0x0000006A) && (ec_slave[slc].eep_id == 0x00414B44))
                    if ((ec_slave[slc].eep_man == 0x0000066F) && (ec_slave[slc].eep_id == 0x60380008)) //Product ID needs to be found out
                    {
                        printf("It gives us great pleasure to inform you that KollMorgen Ethercat Servo has been found!!!\n");
                        printf("Found %s at position %d\n", ec_slave[slc].name, slc);
                        // link slave specific setup to preop->safeop hook
                        ec_slave[slc].PO2SOconfig = &PanasonicSetup;
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
                    
                    iSocketElapsedCntr++;
                    if ((iSocketElapsedCntr > SOCKET_SCAN_CYCLES) && (uiSetMotionFlag == 0))
                    {
                        iSocketElapsedCntr = 0;
                        //printf("\nA");
                        //socketServerAction();
                    }
                    //printf("PDC: %4d, WKC: %d , I:", rtcnt, wkc);	//Processdata cycle  
                    //printf("\nuiSetMotionFlag: %d uiDesiredStatus: %d, uiModifyCmdStatusFlag: %d, uiDriveStatus: %d", uiSetMotionFlag, uiDesiredStatus, uiModifyCmdStatusFlag, uiDriveStatus);
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
                        printStatus(uiDriveStatus);
                        needlf = TRUE;
                        uiActualPosn = iPosActualValue.hl;
                        uiActualTq = uiTqActual.hl;
                                                  
                        if (uiModifyCmdStatusFlag != 0)
                        {
                            SetCommandStatus(uiDesiredStatus);
                            uiModifyCmdStatusFlag = 0;
                        }
                    }
                    osal_usleep(SCAN_INTERVAL_IN_MSEC * 1); //Originally 1000
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
                //StopDrive();
                uiStatusWd.split.l = *(ec_slave[0].inputs  + PDO_READ_STATUS);
                uiStatusWd.split.h = *(ec_slave[0].inputs + (PDO_READ_STATUS + 1));
                updateStatus(uiStatusWd.hl);
                if (uiDriveStatus != STATE_MACHINE_STAT_OPERATION_ENABLED)
                {
                    //j = 1000;
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
            //uiDelMeCntr[2]++;
            sktConnectedToClient = true;
            uiServerSocketStatus = SOCKET_SERVER_SEND_STATUS;
            printf("\nSOCKET_SERVER_GET_DATA_FROM_CLIENT");
        }
        // No longer need server socket
        closesocket(ListenSocket);
        break;

    case SOCKET_SERVER_SEND_STATUS:
        printf("Sending Status...\n");
        SetActualTqAndPosn();
        uiServerSocketStatus = SOCKET_SERVER_GET_DATA_FROM_CLIENT;
        break;

    case SOCKET_SERVER_GET_DATA_FROM_CLIENT:
        iResult = recv(ClientSocket, recvbuf, recvbuflen, 0);
        //uiDelMeCntr[1]++;
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
        //uiDelMeCntr[0]++;
        uiServerSocketStatus = SOCKET_SERVER_SEND_STATUS;
        break;

    case SOCKET_SERVER_CLOSE_SOCKET:
        // shutdown the connection since we're done
        iResult = shutdown(ClientSocket, SD_SEND);
        if (iResult == SOCKET_ERROR) {
            printf("shutdown failed with error: %d\n", WSAGetLastError());
            closesocket(ClientSocket);
            WSACleanup();
            uiServerSocketStatus = SOCKET_SERVER_APP_START_INIT;
            //printf("\nSOCKET_SERVER_APP_START_INIT:%d,%d,%d", uiDelMeCntr[0], uiDelMeCntr[1], uiDelMeCntr[2]);

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
    printf("Str to send: %s", strDataToSend);
    int len = (int)(strlen(strDataToSend));
    iSendResult = send(ClientSocket, strDataToSend,len , 0);
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
