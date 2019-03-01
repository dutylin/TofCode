/**************************************************************************************************
Filename:       med.c
Revised:        $Date: 2011/08/31 12:10:59 $
Revision:       $Revision: 1.0 $

Description:    This file contains the application that can be use to set a device as End
Device from MAC directly which will communicate with a FFD with full z-stack.

**************************************************************************************************/
/**************************************************************************************************

Work FLow:  poll---process polled data------blast---sleep
|                                               |
|       ---  ploll until no data ---     |
**************************************************************************************************/

/**************************************************************************************************
*                                           Includes
**************************************************************************************************/
/* Hal Driver includes */
#include "hal_types.h"
#include "hal_key.h"
#include "hal_timer.h"
#include "hal_drivers.h"
#include "hal_adc.h"
#include "hal_assert.h"
//#include "FlashUtil.h"
#include "MacUtil.h"
#include "hal_led.h"
/* OS includes */
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "OSAL_PwrMgr.h"

/* App Protocol*/
#include "AppProtocolWrapper.h"

/* Application Includes */
#include "OnBoard.h"

/* MAC Application Interface */
#include "mac_api.h"

/* Application */
#include "med.h"

/*2530 Sleep*/
#include "Hal_sleep.h"

/* watchdog util */
#include "watchdogutil.h"
/* FLASH */
#include "Hal_flash.h"

#include "hal_beeper.h"
/*******************************************************************************
*                                           Constant
*******************************************************************************/
/* Period of wait data when URGENT*/
#define MED_URGENT_POLL_TIMEOUT 50

/* Period of wait ALERTACK */
#define MED_ALERT_TIMEOUT 100

/* sleep length in ms */
#define MED_URGENT_SLEEP_PERIOD 1000

/* Key press time in ms when alert */
#define MED_KEY_PRESS_TIME_ALERT 2500

/* Led flash time when no power*/
#define MED_LED_FLASH_TIME_LOWBATTERY 300

/* Setting beacon order to 15 will disable the beacon */
#define MED_MAC_BEACON_ORDER 15

/* Setting superframe order to 15 will disable the superframe */
#define MED_MAC_SUPERFRAME_ORDER 15

/* (VDD/3)/(VREF/(2e7-1)) (VDD~=2.28V,VREF=1.15V) : VDD = 3.45 * LIMT / 127 */
#define MED_VDD_LIMT            0x54        /* 0x54 = 2.28v, 0x55 = 2.309 */

/* Confirm receive RETREAT signal */
#define MED_RECVRETREAT_MAX     5

/* Alert period  in second */
#define MED_ALERT_TIME          10

/* Confirm time: 10min = 120 x 5s */
#define MED_MAX_CONFIRM_CNT     120

/* MED_REPORT_PERIOD * MED_REPORT_TICK equals 10 Minutes */
#define MED_REPORT_PERIOD       120

/* Not received reply maximum wait times */
#define MED_NOSIGNAL            3

/* stop POLL, Wait 1 minute = 12 * 5s*/
#define MED_SLEEP_POLL          12

/* two OUT + a POLL , (OUT__5s__OUT__5s__POLL) */
#define MED_GOTO_POLL           2

/* mac payload length by datasheet. */
#define MED_MAC_PAYLOAD_LEN          127

/* Short Address of PAN Coordinator, will always be 0x0000 */
#define MED_SHORTADDR_PANCOORD       0x0000

/* Short Address of all Devs*/
#define MED_BROADCAST_SHORTADDR_DEVALL      0xFFFF

/* Application Work State */
/* The priority is: Alert > Urgent > Normal */
#define MED_STATE_NORMAL        0
#define MED_STATE_URGENT        1
#define MED_STATE_ALERT         2
#define MED_STATE_HALL          3

#define MED_UNICAST             0
#define MED_BROADCAST           1

//#define MED_VER               0
//#define MED_SHORTADDR         1
#define MED_EXITADDR            2

//0x1E00-0x1D00 are used to store paramters
#define MED_PARAM_ADDR          0x1E00
#define MED_DEVINFO_ADDR        0x780C

#define MED_LED_BLUE            HAL_LED_1
#define MED_LED_RED             HAL_LED_2

#define MED_VERSION_LEN         15

/*********************************************************************
* TYPEDEFS
*/
typedef uint8 report_buff[30];
typedef uint8 P_Addr[8];

typedef struct
{
    bool bState;
    uint32 u32Delay;
} stTimeControl;

typedef enum
{
    POLL_WAIT,
    BLAST_WAIT,
    DELAYSTATELEN
} enDelayState;

typedef struct
{
    sAddrExt_t ExitAddr;
} Dev_Info_t;

typedef struct
{
    uint8 med_WorkState;
    uint8 med_WorkState_bk;

    bool med_IsBatteryLow;

    uint8 med_blast_cnt ;

    uint16 med_seqnum;

    bool med_AlertSuccess;

    uint8 med_AlertCnt;
    uint8 med_urgent_cnt;

    bool med_Confirm;

    uint8 med_ConfirmCnt;
    uint8 med_HallConfirmCnt;
} Dev_Param_t;

/*******************************************************************************
*                                Local Variables
*******************************************************************************/

/* Size table for MAC structures */
const CODE uint8 med_cbackSizeTable[] =
{
    0,                                   /* unused */
    sizeof(macMlmeAssociateInd_t),       /* MAC_MLME_ASSOCIATE_IND */
    sizeof(macMlmeAssociateCnf_t),       /* MAC_MLME_ASSOCIATE_CNF */
    sizeof(macMlmeDisassociateInd_t),    /* MAC_MLME_DISASSOCIATE_IND */
    sizeof(macMlmeDisassociateCnf_t),    /* MAC_MLME_DISASSOCIATE_CNF */
    sizeof(macMlmeBeaconNotifyInd_t),    /* MAC_MLME_BEACON_NOTIFY_IND */
    sizeof(macMlmeOrphanInd_t),          /* MAC_MLME_ORPHAN_IND */
    sizeof(macMlmeScanCnf_t),            /* MAC_MLME_SCAN_CNF */
    sizeof(macMlmeStartCnf_t),           /* MAC_MLME_START_CNF */
    sizeof(macMlmeSyncLossInd_t),        /* MAC_MLME_SYNC_LOSS_IND */
    sizeof(macMlmePollCnf_t),            /* MAC_MLME_POLL_CNF */
    sizeof(macMlmeCommStatusInd_t),      /* MAC_MLME_COMM_STATUS_IND */
    sizeof(macMcpsDataCnf_t),            /* MAC_MCPS_DATA_CNF */
    sizeof(macMcpsDataInd_t),            /* MAC_MCPS_DATA_IND */
    sizeof(macMcpsPurgeCnf_t),           /* MAC_MCPS_PURGE_CNF */
    sizeof(macEventHdr_t)                /* MAC_PWR_ON_CNF */
};

/*******************************************************************************
*                           Local Variables
*******************************************************************************/
/* Task ID */
uint8 MED_TaskId;

/* Dev number and Extended address of the device. */
#ifdef DEBUG
static uint8 med_version;
#endif

/* Coordinator and Device information */
static uint16 med_PanId = CARD_NWK_ADDR;
static uint16 med_PANCoordShortAddr = MED_SHORTADDR_PANCOORD;
static uint16 med_DevShortAddr = MED_BROADCAST_SHORTADDR_DEVALL;

/* TRUE and FALSE value */
static bool med_MACTrue = TRUE;
static bool med_MACFalse = FALSE;

/* flags used in the application */
#ifdef RTR_NWK
/* True if the device is started as a Pan Coordinate */
static bool med_IsCoordinator = FALSE;
#endif

/*Should be UNINIT, Normal   Urgent Alert */
static uint8 med_WorkState = MED_STATE_NORMAL;
static uint8 med_WorkState_bk = MED_STATE_NORMAL;
static bool med_IsBatteryLow = FALSE;

/* Beacon order */
static uint8 med_SuperFrameOrder = MED_MAC_SUPERFRAME_ORDER;
static uint8 med_BeaconOrder = MED_MAC_BEACON_ORDER;

/* counter */
static uint8 med_blast_cnt = 0;

/* Device Info from flash */
static Dev_Info_t med_DevInfo;

/* med_seqnum */
static uint16 med_seqnum = 0;

/* med state counter */
static bool med_AlertSuccess = false;
static uint8 med_AlertCnt = 0;
static uint8 med_urgent_cnt = 0;
static uint8 med_KeyConfirmRetransCnt = 0;
static uint8 s_med_u8HallConfirmCnt = 0;

/* med confirm */
static bool med_Confirm = false;
static uint8 med_ConfirmCnt = 0;

/* report vdd value period 10 minutes */
static uint8 g_u8VddReportTick = 0;
static bool g_bVddReportEnable = false;

report_buff g_Appdata;
app_card_status_t* g_pstCard = (app_card_status_t*) g_Appdata;

static uint8 s_u8SignNum;

static stTimeControl g_stEventDelay[DELAYSTATELEN];

/*******************************************************************************
*                     Local Function Prototypes
*******************************************************************************/
/* Setup routines */
static void MED_DeviceStartup(uint8* pData);

/* Support */
static void MED_blast(uint8 reqtype);
static bool MED_TestLongPress(uint16 TimeOut);
static void MED_Delay(uint16 timeout);

#if (!defined BLINK_LEDS)
static void MED_LedBlink(uint8 leds,
                         uint8 numBlinks,
                         uint8 percent,
                         uint16 period);
#endif

static void MED_ReadDevInfo(void);
static void MED_BatteryCheck(void);

#ifdef DEBUG
static void MED_SetDevInfo(uint8 type, void* pData);
#endif

static void MED_IntervalCheck(void);
static void MED_PeriodReset(void);
static void MED_Restart(void);
static void MED_SaveParam2RAM(void);
static void MED_ReadParmFromRAM(void);
static void MED_ReadPrimaryAddr(P_Addr);
static void MED_StateMachine(void);
static void MED_SleepDelay(enDelayState state, uint16 delay);
static void MsgDelayTickProc(uint8 events);

/*******************************************************************************
* Every 10 minutes to report a system information
*******************************************************************************/
static void MED_ReportInfoProc(void);

/*******************************************************************************
* SYS_EVENT_MSG Event processing function
*******************************************************************************/
static void MsgSysEventProc(void);

/*******************************************************************************
* MED_HALL_EVENT Event processing function
*******************************************************************************/
static void MsgHallProc(void);

/*******************************************************************************
* MED_SEND_KEYCONFIRM_EVT Event processing function
*******************************************************************************/
static void MsgKeyConfirmProc(void);

/*******************************************************************************
* MED_SEND_RECVRETREAT_EVT Event processing function
*******************************************************************************/
static void MsgRecvRetreatProc(void);

/*******************************************************************************
* MED_ALERT_EVENT Event processing function
*******************************************************************************/
static void MsgAlertProc(void);

/*******************************************************************************
* MED_BLAST_EVENT Event processing function
*******************************************************************************/
static void MsgBlastProc(void);

/*******************************************************************************
*                                   Macro function
*******************************************************************************/
#define TURN_ON_LED_BLUE()          HAL_TURN_ON_LED1()
#define TURN_OFF_LED_BLUE()         HAL_TURN_OFF_LED1()
#define TURN_ON_LED_RED()           HAL_TURN_ON_LED2()
#define TURN_OFF_LED_RED()          HAL_TURN_OFF_LED2()

#define STATE_LED_BLUE()            HAL_STATE_LED1()
#define STATE_LED_RED()             HAL_STATE_LED2()

#define BACK_AND_OFF_LED(blue,red)  \
    st(                             \
        blue = STATE_LED_BLUE();    \
        red  =  STATE_LED_RED();    \
        TURN_OFF_LED_BLUE();        \
        TURN_OFF_LED_RED();         \
    )

#define SET_LED_BLUE(blue)          \
    st(                             \
        if (blue) {                 \
            TURN_ON_LED_BLUE();     \
        }                           \
        else {                      \
            TURN_OFF_LED_BLUE();    \
        }                           \
    )

#define SET_LED_RED(red)            \
    st(                             \
        if (red) {                  \
            TURN_ON_LED_RED();      \
        }                           \
        else {                      \
            TURN_OFF_LED_RED();     \
        }                           \
    )

/*******************************************************************************
*                                   function Defined
*******************************************************************************/

/*******************************************************************************
*
* @fn          MED_ReadPrimaryAddr
*
* @brief       read a unique 64 bit IEEE address
*
* @param       pAddr - read buffer
*
* @return      none
*
*******************************************************************************/
static void MED_ReadPrimaryAddr(P_Addr pAddr)
{
    if (pAddr != NULL)
    {
        uint8* u8Offset = (uint8*) (HAL_INFOP_BASE + HAL_INFOP_IEEE_OSET);
        osal_memcpy(pAddr, u8Offset, Z_EXTADDR_LEN);
    }
}

/*******************************************************************************
*
* @fn          MED_Init
*
* @brief       Initialize the application
*
* @param       taskId - taskId of the task after it was added in the OSAL task
*              queue
*
* @return      none
*
*******************************************************************************/
void MED_Init(uint8 taskId)
{
    /* Initialize the task id */
    MED_TaskId = taskId;

    /* initialize MAC features */
    // MAC_Init(); write in main
    MAC_InitDevice();

    /* Reset the MAC */
    MAC_MlmeResetReq(TRUE);

    /* initial MacUtil*/
    MacUtil_t Macutil;

    Macutil.panID = 0xFFFF;                      // Card broadcast to all PANs
    Macutil.dst_endpoint = APS_DST_ENDPOINT;
    Macutil.src_endpoint = APS_SRC_ENDPOINT;
    Macutil.cluster_id = APS_CLUSTER_ID;
    Macutil.profile_id = APS_PROFILE_ID;
    Macutil.NodeType = NODETYPE_DEVICE;
    MAC_UTIL_INIT(&Macutil);

    med_blast_cnt = 0;
    g_pstCard = (app_card_status_t *) g_Appdata;

    MED_ReadDevInfo();
    MED_DeviceStartup(NULL);

    /* save param to Ram, if watchdog reset by accident,this value is default */
    MED_SaveParam2RAM();

    TURN_OFF_LED_RED();
    TURN_OFF_LED_BLUE();

#ifdef WATCHDOG_TEST
    MED_LedBlink(MED_LED_RED, 10, 30, 100);
#endif

    /*Start Watch Dog*/
#ifdef WATCHDOG
    StartWatchDog(DOGTIMER_INTERVAL_1S);
#endif

    /* goto BLAST state */
    med_WorkState = MED_STATE_NORMAL;
    osal_set_event(MED_TaskId, MED_SEND_RECVRETREAT_EVT | MED_BLAST_EVENT);
}

/*******************************************************************************
*
* @fn          MED_ProcessEvent
*
* @brief       This routine handles events
*
* @param       taskId - ID of the application task when it registered with the OSAL
*              events - Events for this task
*
* @return      16bit - Unprocessed events
*
*******************************************************************************/
uint16 MED_ProcessEvent(uint8 taskId, uint16 events)
{
    if (events & MED_TIME_VERIFY_EVENT)
    {
        MsgDelayTickProc(events);
        return events ^ MED_TIME_VERIFY_EVENT;
    }

    if (events & SYS_EVENT_MSG)
    {
        MsgSysEventProc();
        return events ^ SYS_EVENT_MSG;
    }

    if (events & MED_HALL_EVENT)
    {
        MsgHallProc();
        return events ^ MED_HALL_EVENT;
    }

    if (events & MED_SEND_KEYCONFIRM_EVT)
    {
        MsgKeyConfirmProc();
        return events ^ MED_SEND_KEYCONFIRM_EVT;
    }

    if (events & MED_ALERT_EVENT)
    {
        MsgAlertProc();
        return events ^ MED_ALERT_EVENT;
    }

    if (events & MED_SEND_RECVRETREAT_EVT)
    {
        MED_StateMachine();
        MsgRecvRetreatProc();
        return events ^ MED_SEND_RECVRETREAT_EVT;
    }

    if (events & MED_BLAST_EVENT)
    {
        MsgBlastProc();
        return events ^ MED_BLAST_EVENT;
    }

    return 0;
}

/*******************************************************************************
*
* @fn          MED_SleepDelay
*
* @brief       Specific event state will wait for a period of time before sleep
*
* @param    state - wait state
*                 delay - wait some time, milliseconds
*
* @return      void
*
*******************************************************************************/
static void MED_SleepDelay(enDelayState state, uint16 delay)
{
    if (state < DELAYSTATELEN)
    {
        g_stEventDelay[state].bState = TRUE;
        g_stEventDelay[state].u32Delay = delay + osal_GetSystemClock();
        osal_set_event(MED_TaskId, MED_TIME_VERIFY_EVENT);
    }
}

/*******************************************************************************
*
* @fn          MsgDelayTickProc
*
* @brief       A millisecond update a state delay events
*
* @param    events - current os all events state
*
*
* @return      void
*
*******************************************************************************/
static void MsgDelayTickProc(uint8 events)
{
    bool bSleep = TRUE;
    uint8 u8Index;
    uint32 u32Tick;
    uint16 u16Space;
    uint16 u16Min;

    u32Tick = osal_GetSystemClock();

#ifdef WATCHDOG
    FeedWatchDog();
#endif
    u16Space = ~((uint16) 0);

    for (u8Index = POLL_WAIT; u8Index < DELAYSTATELEN; u8Index++)
    {
        if (g_stEventDelay[u8Index].bState)
        {
            if (g_stEventDelay[u8Index].u32Delay > u32Tick)
            {
                u16Min = (uint16) (g_stEventDelay[u8Index].u32Delay - u32Tick);
                u16Space = u16Min < u16Space ? u16Min : u16Space;
                bSleep = FALSE;
            }
            else
            {
                g_stEventDelay[u8Index].bState = FALSE;
            }
        }
    }
    // os task is idle
    if (bSleep && !(events ^ MED_TIME_VERIFY_EVENT))
    {
        TURN_OFF_LED_BLUE();
        TURN_OFF_LED_RED();
        osal_pwrmgr_task_state(MED_TaskId, PWRMGR_CONSERVE);
        MAC_MlmeSetReq(MAC_RX_ON_WHEN_IDLE, &med_MACFalse);
    }
    else
    {   // if u16Space == -1
        if (!(~u16Space))
        {// 1ms
            u16Space = 1;
        }
        osal_pwrmgr_task_state(MED_TaskId, PWRMGR_HOLD);
        MAC_MlmeSetReq(MAC_RX_ON_WHEN_IDLE, &med_MACTrue);
        osal_start_timerEx(MED_TaskId, MED_TIME_VERIFY_EVENT, u16Space);
    }
}

/*******************************************************************************
* SYS_EVENT_MSG Event Data Parse function
*******************************************************************************/
static void MsgSysEventParse(sData_t* pstAppData)
{
    if (!pstAppData)
        return;

    switch (*((MSGType *) (pstAppData->p)))
    {
      case URGENT:
          switch (((app_Urgent_t *) (pstAppData->p))->urgenttype)
          {
            case RETREAT:
                if (!med_Confirm && med_WorkState == MED_STATE_NORMAL)
                {
                    med_urgent_cnt = 0;
                    med_WorkState = MED_STATE_URGENT;
                }
                break;
            case CANCELRETREAT:
                if (med_WorkState == MED_STATE_URGENT)
                {
                    med_WorkState = MED_STATE_NORMAL;
                }
                break;
            case ALERTACK:
                med_AlertSuccess = TRUE;
                break;
          }
          break;
      default:
          break;
    }
}

/*******************************************************************************
* MED_HALL_EVENT Event processing function
*******************************************************************************/
static void MsgSysEventProc(void)
{
    uint8* pMsg;
    macCbackEvent_t* pData;

    while ((pMsg = osal_msg_receive(MED_TaskId)) != NULL)
    {
        switch (*pMsg)
        {
          case MAC_MCPS_DATA_CNF:
              pData = (macCbackEvent_t *) pMsg;
              osal_msg_deallocate((uint8 *) pData->dataCnf.pDataReq);
              break;
          case MAC_MCPS_DATA_IND:
              /* Proess Command */
              pData = (macCbackEvent_t *) pMsg;

              if (MAC_UTIL_GetClusterID(pData->dataInd.msdu) == CARD_CLUSTERID)
              {
                  sData_t AppData = MAC_UTIL_RemoveHeader(pData->dataInd.msdu);

                  s_u8SignNum = 0;

                  MsgSysEventParse(&AppData);
              }
              break;
        }
        /* Deallocate */
        osal_msg_deallocate((uint8 *) pMsg);
    }
}

/*******************************************************************************
* MED_HALL_EVENT Event processing function
*******************************************************************************/
static void MsgHallProc(void)
{
    static bool bFlag = true;

    if (bFlag)
    {
        HalStartBeeper(MED_STATE_HALL, 20);
        osal_start_timerEx(MED_TaskId, MED_HALL_EVENT, 1000);
    }
    else
    {
        HalStopBeeper(MED_STATE_HALL, true);
    }
    bFlag = !bFlag;
}

/*******************************************************************************
* MED_SEND_KEYCONFIRM_EVT Event processing function
*******************************************************************************/
static void MsgKeyConfirmProc(void)
{
    if (MED_STATE_URGENT == HalBeeperType())
    {
        HalStopBeeper(MED_STATE_URGENT, true);
    }
    if (--med_KeyConfirmRetransCnt > 0)
    {
        app_Urgent_t app_Urgent;

        app_Urgent.msgtype = URGENT;
        app_Urgent.urgenttype = NOPWR;
        app_Urgent.value = URGENT_NOPWR_KEYCONFIRM;

        MAC_UTIL_BuildandSendData((uint8 *) &app_Urgent,
                                  sizeof(app_Urgent),
                                  MED_UNICAST,
                                  0,
                                  NULL);
        osal_start_timerEx(MED_TaskId, MED_SEND_KEYCONFIRM_EVT, 2000);
    }
}

/*******************************************************************************
* MED_ALERT_EVENT Event processing function
*******************************************************************************/
static void MsgAlertProc(void)
{
#ifdef WATCHDOG
    FeedWatchDog();
#endif
    // if sucess, last 3 seconds flash blue
    if (med_AlertSuccess && med_AlertCnt + 3 > MED_ALERT_TIME)
    {
        // MED_LedBlink(MED_LED_BLUE, 2, 30, 500);
        HalLedBlink(MED_LED_BLUE, 2, 30, 300);
    }
    else
    {
        // MED_LedBlink(MED_LED_RED, 2, 30, 500);
        HalLedBlink(MED_LED_RED, 2, 30, 300);
    }

    if (med_AlertCnt++ < MED_ALERT_TIME)   /* alerting */
    {
        if (!med_AlertSuccess)
        {
            app_Urgent_t app_Urgent;

            app_Urgent.msgtype = URGENT;
            app_Urgent.urgenttype = ALERT;
            app_Urgent.value = 0;

            MAC_MlmeSetReq(MAC_RX_ON_WHEN_IDLE, &med_MACTrue);
            MAC_UTIL_BuildandSendData((uint8 *) &app_Urgent,
                                      sizeof(app_Urgent_t),
                                      MED_UNICAST,
                                      0,
                                      NULL);
            MED_SleepDelay(BLAST_WAIT, MED_ALERT_TIMEOUT);
        }
        if (MED_ALERT_EVENT != HalBeeperType())
        {
            HalStartBeeper(MED_STATE_ALERT, 10);
        }
        osal_start_timerEx(MED_TaskId, MED_ALERT_EVENT, MED_URGENT_SLEEP_PERIOD);
    }
    else                            /* alert end */
    {
        TURN_OFF_LED_BLUE();

        if (med_WorkState_bk == MED_STATE_NORMAL
         || med_WorkState_bk == MED_STATE_URGENT)
        {
            med_WorkState = med_WorkState_bk;
        }
        else
        {
            med_WorkState = MED_STATE_NORMAL;
        }
        if (MED_STATE_ALERT == HalBeeperType())
        {
            HalStopBeeper(MED_STATE_ALERT, true);
        }
    }
}

/*******************************************************************************
* MED_SEND_RECVRETREAT_EVT Event processing function
*******************************************************************************/
static void MsgRecvRetreatProc(void)
{
    static uint16 u16SleepTotal = 0;
    uint16 u16SleepPeriod = MED_SLEEP_PERIOD;

    if (MED_STATE_URGENT == med_WorkState)
    {
        u16SleepPeriod = u16SleepPeriod >> 1;
        u16SleepTotal += u16SleepPeriod;
        HalLedBlink(MED_LED_RED, 4, 30, 150);    // blink, unblock

        if (!HalBeeperBusy())
        {
            HalStartBeeper(MED_STATE_URGENT, 20);
        }

        if (MED_SLEEP_PERIOD <= u16SleepTotal
         && MED_RECVRETREAT_MAX >= med_urgent_cnt)
        {
            app_Urgent_t app_Urgent;

            med_urgent_cnt++;
            u16SleepTotal = 0;
            app_Urgent.msgtype = URGENT;
            app_Urgent.urgenttype = NOPWR;
            app_Urgent.value = URGENT_NOPWR_RECVRETREAT;

            MAC_UTIL_BuildandSendData((uint8 *) &app_Urgent,
                                      sizeof(app_Urgent),
                                      MED_UNICAST,
                                      0,
                                      NULL);
            MED_SleepDelay(BLAST_WAIT, 20);
        }
    }
    /* in urgent state, send out recvretreat */
    osal_start_timerEx(MED_TaskId, MED_SEND_RECVRETREAT_EVT, u16SleepPeriod);
}

/*******************************************************************************
* MED_BLAST_EVENT Event processing function
*******************************************************************************/
static void MsgBlastProc(void)
{
    /* Send out  message */
    med_blast_cnt++;
    if (s_u8SignNum < MED_NOSIGNAL && !(med_blast_cnt % MED_POLL_INTERVAL))
    {
        uint16 u16TimeOut = (MED_STATE_URGENT != med_WorkState)
                          ? MED_POLL_TIMEOUT
                          : MED_URGENT_POLL_TIMEOUT;
        s_u8SignNum++;
        MAC_MlmeSetReq(MAC_RX_ON_WHEN_IDLE, &med_MACTrue);
        MED_SleepDelay(POLL_WAIT, u16TimeOut);
        MED_blast(SSREQ_POLL);
    }
    else
    {
        // If not received signal then into a minute a POLL
        if (s_u8SignNum >= MED_NOSIGNAL && ++s_u8SignNum == MED_SLEEP_POLL)
        {
            s_u8SignNum = MED_GOTO_POLL;
        }
        MED_blast(SSREQ_OUT);
    }

    TURN_OFF_LED_BLUE();
    MED_PeriodReset();
    osal_start_timerEx(MED_TaskId, MED_BLAST_EVENT, MED_SLEEP_PERIOD);
}

static void MED_StateMachine(void)
{
#ifdef WATCHDOG
    FeedWatchDog();
#endif
    MED_BatteryCheck();

    if (med_ConfirmCnt && !(--med_ConfirmCnt))
    {
        med_Confirm = false;
    }

    if (MED_STATE_NORMAL == med_WorkState)
    {
        if (HalBeeperBusy())
        {
            HalStopBeeper(NULL, true);
        }
        /* set blast LED */
        if (!med_IsBatteryLow)
        {
            TURN_ON_LED_BLUE();
        }
    }
    else if (MED_STATE_URGENT == med_WorkState)
    {
        if (med_Confirm)
        {
            med_WorkState = MED_STATE_NORMAL;
        }
    }
}


/*******************************************************************************
*
* @fn          MED_ReportInfoProc
*
* @brief       Every 10 minutes to report a system information
*
* @param       None.
*
* @return      None.
*
*******************************************************************************/
static void MED_ReportInfoProc(void)
{
    g_u8VddReportTick++;

    /* MED_REPORT_PERIOD * MED_REPORT_TICK equals 10 Minutes */
    if (!((med_blast_cnt + 1) % MED_POLL_INTERVAL)
     && (MED_REPORT_PERIOD <= g_u8VddReportTick || !g_bVddReportEnable))
    {
        uint8 u8VddValue;
        /* add report info */
        g_bVddReportEnable = true;
        /* vdd value       */
        HalAdcCheckVdd2(MED_VDD_LIMT, &u8VddValue);
        g_pstCard->msgtype = CARD_STATUS;
        g_pstCard->cardtype = APP_CARD_TYPE_BATTERY_2530;
        g_pstCard->batteryvalue = u8VddValue;
        MED_ReadPrimaryAddr(g_pstCard->primaryIEEEaddr);
        g_pstCard->seqnum++;

        if (sizeof(CARD_VERSION) > MED_VERSION_LEN)
        {
            g_pstCard->len = sizeof(app_card_status_t) + MED_VERSION_LEN;

            /* system version */
            osal_memcpy(g_pstCard + 1, CARD_VERSION, MED_VERSION_LEN);
        }
        else
        {
            g_pstCard->len = sizeof(app_card_status_t) + sizeof(CARD_VERSION);
            osal_memcpy(g_pstCard + 1, CARD_VERSION, sizeof(CARD_VERSION));
        }

        MAC_UTIL_BuildandSendData(g_Appdata, g_pstCard->len, MED_UNICAST, 0, 0);
        g_u8VddReportTick = 0;
    }
}

/*******************************************************************************
*
* @fn          MAC_CbackEvent
*
* @brief       This callback function sends MAC events to the application.
*              The application must implement this function.  A typical
*              implementation of this function would allocate an OSAL message,
*              copy the event parameters to the message, and send the message
*              to the application's OSAL event handler.  This function may be
*              executed from task or interrupt context and therefore must
*              be reentrant.
*
* @param       pData - Pointer to parameters structure.
*
* @return      None.
*
*******************************************************************************/
void MAC_CbackEvent(macCbackEvent_t* pData)
{
    macCbackEvent_t* pMsg = NULL;

    uint8 len = med_cbackSizeTable[pData->hdr.event];

    switch (pData->hdr.event)
    {
      case MAC_MLME_BEACON_NOTIFY_IND:
          len += sizeof(macPanDesc_t)
               + pData->beaconNotifyInd.sduLength
               + MAC_PEND_FIELDS_LEN(pData->beaconNotifyInd.pendAddrSpec);
          if ((pMsg = (macCbackEvent_t *) osal_msg_allocate(len)) != 0)
          {
              /* Copy data over and pass them up */
              osal_memcpy(pMsg, pData, sizeof(macMlmeBeaconNotifyInd_t));
              pMsg->beaconNotifyInd.pPanDesc = (macPanDesc_t *)
                                               ((uint8 *) pMsg
                                              + sizeof(macMlmeBeaconNotifyInd_t));
              osal_memcpy(pMsg->beaconNotifyInd.pPanDesc,
                          pData->beaconNotifyInd.pPanDesc,
                          sizeof(macPanDesc_t));
              pMsg->beaconNotifyInd.pSdu = (uint8 *)
                                           (pMsg->beaconNotifyInd.pPanDesc + 1);
              osal_memcpy(pMsg->beaconNotifyInd.pSdu,
                          pData->beaconNotifyInd.pSdu,
                          pData->beaconNotifyInd.sduLength);
          }
          break;

      case MAC_MCPS_DATA_IND:
          pMsg = pData;
          break;
      default:
          if ((pMsg = (macCbackEvent_t *) osal_msg_allocate(len)) != NULL)
          {
              osal_memcpy(pMsg, pData, len);
          }
          break;
    }

    if (pMsg != NULL)
    {
        osal_msg_send(MED_TaskId, (uint8 *) pMsg);
    }
}

/*******************************************************************************
*
* @fn      MAC_CbackCheckPending
*
* @brief   Returns the number of indirect messages pending in the application
*
* @param   None
*
* @return  Number of indirect messages in the application
*
*******************************************************************************/
uint8 MAC_CbackCheckPending(void)
{
    return (0);
}

/*******************************************************************************
*
* @fn      MED_DeviceStartup(uint8* pData)
*
* @brief   Update the timer per tick
*
* @param   beaconEnable: TRUE/FALSE
*
* @return  None
*
*******************************************************************************/
void MED_DeviceStartup(uint8* pData)
{
    MAC_MlmeSetReq(MAC_EXTENDED_ADDRESS, &med_DevInfo.ExitAddr);

    /* Setup MAC_BEACON_PAYLOAD_LENGTH */
    //MAC_MlmeSetReq(MAC_BEACON_PAYLOAD_LENGTH, &med_BeaconPayloadLen);

    /* Setup MAC_BEACON_PAYLOAD */
    //MAC_MlmeSetReq(MAC_BEACON_PAYLOAD, &med_BeaconPayload);

    /* Setup PAN ID */
    MAC_MlmeSetReq(MAC_PAN_ID, &med_PanId);

    /* This device is setup for Direct Message */
    MAC_MlmeSetReq(MAC_RX_ON_WHEN_IDLE, &med_MACFalse);

    /* Setup Coordinator short address */
    MAC_MlmeSetReq(MAC_COORD_SHORT_ADDRESS, &med_PANCoordShortAddr);

    /* Setup Beacon Order */
    MAC_MlmeSetReq(MAC_BEACON_ORDER, &med_BeaconOrder);

    /* Setup Super Frame Order */
    MAC_MlmeSetReq(MAC_SUPERFRAME_ORDER, &med_SuperFrameOrder);


    //uint8 tmp8 = MED_MAC_CHANNEL;
    uint8 tmp8 = med_DevInfo.ExitAddr[EXT_MACADDR_CHANNEL];
    MAC_MlmeSetReq(MAC_LOGICAL_CHANNEL, &tmp8);

    MAC_MlmeSetReq(MAC_SHORT_ADDRESS, &med_DevShortAddr);

    MAC_MlmeSetReq(MAC_ASSOCIATED_PAN_COORD, &med_MACTrue);
}

/*******************************************************************************
*
* @fn      MED_HandleKeys
*
* @brief   Callback service for keys
*
* @param   keys  - keys that were pressed
*          state - shifted
*
* @return  void
*
*******************************************************************************/
void MED_HandleKeys(uint16 keys, uint8 shift)
{
#ifdef USE_STATE_UNINIT
    if (med_WorkState != MED_STATE_UNINIT)
                                #endif
    {
#ifdef DEBUG
        TURN_ON_LED_RED();
        MED_Delay(10);
        TURN_OFF_LED_RED();
#endif
        uint8 red_state;
        uint8 blue_state;

        BACK_AND_OFF_LED(blue_state, red_state);
        uint16 timeout;
        /* If The ISR fun is run from sleep, the clock is 16MHz,
         * else the clock is 32MHz
         */
        if (CLKCONSTA & 0x01 == 0)
        {
            timeout = MED_KEY_PRESS_TIME_ALERT;
        }
        else
        {
            timeout = MED_KEY_PRESS_TIME_ALERT / 2;
        }

        if (MED_TestLongPress(timeout))   /* Alert */
        {
            if (med_WorkState != MED_STATE_ALERT)
            {
                med_WorkState_bk = med_WorkState;
            }
            med_WorkState = MED_STATE_ALERT;
            med_AlertSuccess = FALSE;
            med_AlertCnt = 0;

            osal_set_event(MED_TaskId, MED_ALERT_EVENT);
        }// if state is urgent, send keyconfirm
        else if (MED_STATE_URGENT == med_WorkState)
        {
            med_KeyConfirmRetransCnt = 20;
            med_Confirm = true;
            med_ConfirmCnt = MED_MAX_CONFIRM_CNT;
            med_WorkState = MED_STATE_NORMAL;

            osal_set_event(MED_TaskId, MED_SEND_KEYCONFIRM_EVT);
        }

        SET_LED_BLUE(blue_state);
        SET_LED_RED(red_state);
    }
}

/*******************************************************************************
*
* @fn      MED_HandleHall
*
* @brief   Callback service for Hall element
*
* @return  void
*
*******************************************************************************/
#if (defined HAL_HALL) && (HAL_HALL == TRUE)
void MED_HandleHall(void)
{
    // if (!s_med_u8HallConfirmCnt)
    {
        // s_med_u8HallConfirmCnt = 10;
        osal_start_timerEx(MED_TaskId, MED_HALL_EVENT, 1);
    }
}
#endif

/*******************************************************************************
*
* @fn      MED_blast(void)
*
* @brief   Blast once to all Coords and routers without ACK or retrans.
*
* @param
*
* @return
*
*******************************************************************************/
void MED_blast(uint8 reqtype)
{
    MED_IntervalCheck();

    APPWrapper_t Appdata;
    //Appdata = MED_buildApp_SSREQ(reqtype);
    Appdata.app_ssReq.msgtype = SSREQ;
    Appdata.app_ssReq.reqtype = reqtype;
    Appdata.app_ssReq.NWK_ADDR = CARD_NWK_ADDR;
    Appdata.app_ssReq.seqnum = med_seqnum++;

    MAC_UTIL_BuildandSendData((uint8 *) &Appdata,
                              sizeof(app_ssReq_t),
                              MED_UNICAST,
                              0,
                              0);
}

/* Test If a key press is a long press */
bool MED_TestLongPress(uint16 TimeOut)
{
    uint16 testInterval = 300;   // test once each 300 ms
    uint16 testnum = TimeOut / testInterval;

    for (uint16 i = 0; i < testnum; i++)
    {
        MED_Delay(testInterval);

        if (MED_KEY_SW_7_PORT & MED_KEY_SW_7_BIT)   // low voltage when key press
        {
            return false;
        }
    }
    return true;
}

/* timeout is  in ms */
void MED_Delay(uint16 timeout)
{
    uint16 i;
    uint16 j;
    uint16 k;
    uint16 timeBig = timeout >> 9;
    uint16 timeSmall = timeout - timeBig * 512;

    for (i = 0; i < timeBig; i++)
    {
#ifdef WATCHDOG
        FeedWatchDog();
#endif
        //MED_FeedWatchDog(); // feed dog every 512 ms
        for (j = 0; j < 512; j++)
        {
            /* One Nop counts 12/32M, So 889  cyc is a ms*/
            k = 880;//k = 889;
            while (k--)
            {
                asm("NOP");
                asm("NOP");
                asm("NOP");
            }
        }
    }
#ifdef WATCHDOG
    FeedWatchDog();
#endif
    for (i = 0; i < timeSmall; i++)
    {
        k = 880;

        while (k--)
        {
            asm("NOP");
            asm("NOP");
            asm("NOP");
        }
    }
}

#if (!defined BLINK_LEDS)
/* Set Led Blink, program will blocked here*/
void MED_LedBlink(uint8 leds, uint8 numBlinks, uint8 percent, uint16 period)
{
    uint8 i;
    uint16 ontime;
    uint16 offtime;

    if (percent >= 100)
    {
        ontime = period;
        offtime = 0;
    }
    else if (percent <= 0)
    {
        ontime = 0;
        offtime = period;
    }
    else
    {
        ontime = period * percent / 100;
        offtime = period - ontime;
    }
    if (leds == MED_LED_BLUE)
    {
        for (i = 0; i < numBlinks; i++)
        {
            TURN_ON_LED_BLUE();
            MED_Delay(ontime);
            TURN_OFF_LED_BLUE();
            MED_Delay(offtime);
        }
    }
    else if (leds == MED_LED_RED)
    {
        for (i = 0; i < numBlinks; i++)
        {
            TURN_ON_LED_RED();
            MED_Delay(ontime);
            TURN_OFF_LED_RED();
            MED_Delay(offtime);
        }
    }
}
#endif

void MED_ReadDevInfo()
{
#if 0
    /* Make a fake DEV Info*/
    med_DevInfo.ExitAddr[0] = 0x00;
    med_DevInfo.ExitAddr[1] = 0xFF;
    med_DevInfo.ExitAddr[2] = 0xFF;
    med_DevInfo.ExitAddr[3] = 0xFF;
    med_DevInfo.ExitAddr[4] = 0x00;
    med_DevInfo.ExitAddr[5] = 0x00;
    med_DevInfo.ExitAddr[6] = 0xFF;
    med_DevInfo.ExitAddr[7] = 0x00;
#else
    HalFlashRead(HAL_FLASH_IEEE_PAGE, HAL_FLASH_IEEE_OSET,
                 (uint8 *)&med_DevInfo.ExitAddr, HAL_FLASH_IEEE_SIZE);

    if (ResetReason() == RESET_FLAG_WATCHDOG)
    {
        MED_ReadParmFromRAM();
    }

    /*For Card, the lowest Byte of Exit Addr should be 0x01 */
    HAL_ASSERT(med_DevInfo.ExitAddr[EXT_MACADDR_TYPE] == EXT_MACADDR_TYPE_CARD);
    med_DevShortAddr = BUILD_UINT16(med_DevInfo.ExitAddr[EXT_MACADDR_DEVID_LBYTE],
                                    med_DevInfo.ExitAddr[EXT_MACADDR_DEVID_HBYTE]);

#endif
}

#ifdef DEBUG
void MED_SetDevInfo(uint8 type, void* pData)
{
    MED_ReadDevInfo();
    switch (type)
    {
      case   MED_EXITADDR:
          {
              uint8* p = (uint8*) pData;
              for (uint8 i = 0; i < 8; i++)
              {
                  med_DevInfo.ExitAddr[i] = *p++;
              }
              break;
          }
    }
    FlashWrite(MED_DEVINFO_ADDR, (uint8 *) &med_DevInfo, sizeof(Dev_Info_t));

    return;
}
#endif

void MED_IntervalCheck(void)
{
#define     MED_MAX_INTERVAL    20000   // 20s

    static uint32 last_ticks = 0;

    /* read current ticks */
    uint32 ticks;
    ((uint8 *) &ticks)[0] = ST0;
    ((uint8 *) &ticks)[1] = ST1;
    ((uint8 *) &ticks)[2] = ST2;
    ((uint8 *) &ticks)[3] = 0;

    if (last_ticks != 0)
    {
        uint32 diff_ticks;

        if (ticks > last_ticks)
        {
            diff_ticks = ticks - last_ticks;
        }
        else
        {
            diff_ticks = 0x1000000 + ticks - last_ticks;
        }

        diff_ticks >>= 5; // convert 1/32k to  ms,  diff_ticks = diff_ticks/32;

        if (diff_ticks > MED_MAX_INTERVAL)   //  if interval > 20s, reset
        {
            MED_Restart();
        }
    }
    last_ticks = ticks;
}

void MED_PeriodReset()
{
#define MED_PERIODRESET_NUM     (1*60*60/5)   // restart per hour

    static uint16 MED_periodReset_cnt = 0;

    /* only restart in normal */
    if (med_WorkState == MED_STATE_NORMAL
     && MED_periodReset_cnt++ > MED_PERIODRESET_NUM)
    {
        MED_Restart();
    }
}

void MED_Restart()
{
    EA = 0; // HAL_DISABLE_INTERRUPTS();
    MED_SaveParam2RAM();
    STARTWATCHDOG(DOGTIMER_INTERVAL_2MS);
    while (1);
}

void MED_SaveParam2RAM(void)
{
    Dev_Param_t param;
    param.med_WorkState = med_WorkState;
    param.med_WorkState_bk = med_WorkState_bk;
    param.med_IsBatteryLow = med_IsBatteryLow;
    param.med_blast_cnt = med_blast_cnt;
    param.med_seqnum = med_seqnum;
    param.med_AlertSuccess = med_AlertSuccess;
    param.med_AlertCnt = med_AlertCnt;
    param.med_urgent_cnt = med_urgent_cnt;
    param.med_Confirm = med_Confirm;
    param.med_ConfirmCnt = med_ConfirmCnt;
    param.med_HallConfirmCnt = s_med_u8HallConfirmCnt;

    *((Dev_Param_t *) (MED_PARAM_ADDR)) = param; // save parameters to idata ram
}

void MED_ReadParmFromRAM(void)
{
    Dev_Param_t DevParam = *((Dev_Param_t*) (MED_PARAM_ADDR));
    med_WorkState = DevParam.med_WorkState;
    med_WorkState_bk = DevParam.med_WorkState_bk;
    med_IsBatteryLow = DevParam.med_IsBatteryLow;
    med_blast_cnt = DevParam.med_blast_cnt;
    med_AlertSuccess = DevParam.med_AlertSuccess;
    med_AlertCnt = DevParam.med_AlertCnt;
    med_urgent_cnt = DevParam.med_urgent_cnt;
    med_seqnum = DevParam.med_seqnum;
    med_Confirm = DevParam.med_Confirm;
    med_ConfirmCnt = DevParam.med_ConfirmCnt;
    s_med_u8HallConfirmCnt = DevParam.med_HallConfirmCnt;
    /* incorrect value */
    if (med_ConfirmCnt > MED_MAX_CONFIRM_CNT)
    {
        med_Confirm = false;
        med_ConfirmCnt = 0;
    }
}

static void MED_BatteryCheck(void)
{
#define LOWBATTERY_CHECK_CNT 3

    static uint8 med_lowbatt_cnt = 0;

    MED_ReportInfoProc();

    if (MED_STATE_NORMAL == med_WorkState)
    {
        // check every 20 second
        if (!(med_blast_cnt & LOWBATTERY_CHECK_CNT) || med_lowbatt_cnt > 0)
        {
            uint8 u8VddValue = 0;

            if (!HalAdcCheckVdd2(MED_VDD_LIMT, &u8VddValue))
            {
                /* only send when polling, there are more waiting time */
                if (med_lowbatt_cnt >= LOWBATTERY_CHECK_CNT
                 && !((med_blast_cnt + 1) % MED_POLL_INTERVAL))
                {
                    med_IsBatteryLow = TRUE;

                    /* send NOPWR */
                    app_Urgent_t app_Urgent;
                    app_Urgent.msgtype = URGENT;
                    app_Urgent.urgenttype = NOPWR;
                    app_Urgent.value = u8VddValue;

                    MAC_UTIL_BuildandSendData((uint8 *) &app_Urgent,
                                              sizeof(app_Urgent_t),
                                              MED_UNICAST,
                                              0,
                                              NULL);
                }
                else
                {
                    med_lowbatt_cnt++;
                }
            }
            else
            {
                med_lowbatt_cnt = 0;
                med_IsBatteryLow = FALSE;
            }
        }

        if (med_IsBatteryLow)
        {
#ifdef WATCHDOG
            FeedWatchDog();
#endif
            TURN_ON_LED_RED();
            MED_Delay(MED_LED_FLASH_TIME_LOWBATTERY);
            TURN_OFF_LED_RED();
        }
    }
}
/**************************************************************************************************
**************************************************************************************************/
