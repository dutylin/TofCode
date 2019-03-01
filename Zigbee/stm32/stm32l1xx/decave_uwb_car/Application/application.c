#include "stm32l1xx.h"
#include <stdlib.h>
#include <string.h>

#include "version.h"
#include "tls_route.h"
#include "NT_protocol.h"

#include "board.h"
#include "timer_event.h"
#include "mac_msg.h"
#include "Mem.h"

#include "app_card_cfg.h"
#include  "instance.h"
#include "port.h"
#include "printf_util.h"
#include "sleep.h"
#include "bsmac_header.h"
#include "crc.h"
#include "nwk_protocol.h"
#include "app_protocol.h"
#include "config.h"



#define MAX_TIMER_TICK  (0xFFFFFFFF/2)

/*****************************************************************************
* CONSTANTS AND DEFINES
*/


// on last sector tail 6byte
#define DEV_ADDRESS (0x0801FF00)
#define DEVTYPE_ADDR  (0x08080FF0)
#define CARD_1S_MOTION_CNT           1500        //30minutes 1.2s card
#define CARD_5S_MOTION_CNT            300       //30minutes   6s card

#define D_VALUE  4
#define DEFF(x,y)   ((x)>=(y)?((x)-(y)):((y)-(x)))

#define TOF_BUZZER_TIMEOUT			200	
#define TOF_MAX_DISTANCE_INLINE     (400)
#define TOF_MIN_RSSI_INLINE         (-100)
#define CARD_RESET_NOTOF_COUNT       10

/*****************************************************************************
* TYPEDEFS
*/

typedef enum
{
    Ranging_Report_TOA          = 0,
    Ranging_Report_Distance     = 1,
    Ranging_Report_Misc_Dist    = 2,
}Ranging_Report_Mode_e;
typedef struct
{
	uint_8 u8Sysmod;
	uint_8 u8IntSrc;
	uint_8 u8FFMtSrc;
	uint_8 u8TransientSrc;
	uint_8 u8XbyteH;
	uint_8 u8YbyteH;
	uint_8 u8ZbyteH;
}teMMA845x_Data;


teMMA845x_Data mma845xData;

/*******************************************************/


//状态优先级: 报警>撤离>低电>其他(无事件)
typedef enum
{
    STATE_URGENT_IDLE = 0,
    STATE_URGENT_RETREAT,
    STATE_URGENT_CNF,    //按键确认后
}Urgent_State_e;

typedef enum
{
    CARD_STATE_NOMAL = 0,
    CARD_STATE_URGENT,      //撤离闪灯时处于改状态
    CARD_STATE_HELP,
}Card_Work_Status_e;

typedef enum
{
    CARD_MOTION_REST = 0,
    CARD_MOTION_MOVE
}Card_Motion_Status_e;


/*
typedef struct
{
    uint16 u16CardNum[MAX_SET_CARD_CNT];
    uint8 u8CardCnt;
} tsCardTypeSet;
*/
extern  slot_msg_t my_cardslotmsg; //
extern  Sub_alarm_msg_t sub_alarmmsg;
uint8 set_helpover_sleep=0;      //when press help button and beep over ,count the sleep tick ,it's diffrent from other times

uint16 u16CardMotionTimeout = CARD_1S_MOTION_CNT;
uint16 u16CardMotionCnt = 0;

app_UWBVersionReport_t tsVersionReport[APP_TOF_VERSION_MAX_NUM];
uint8					 tsVersionReportLen;


extern ts_Car_cardlist Car_revcardlist;
/*****************************************************************************
*   DECLARATION
*/

static uint_32 eventHold;           //事件保持，以免休眠

//static Card_Work_Status_e cardWorkState = CARD_STATE_NOMAL;
//static Card_Motion_Status_e cardMotionState = CARD_MOTION_MOVE;

// motion detection counter
Bool isMoving = True;

//static uint_32 cardMotionCheckTime;
//static uint_32 cardAdcCheckTime;
//static uint_32 reportVersionTime = 0;



uint8 u8HelpCnt = 0;

//uint16 u16StationPanId;
uint16 u16ArmId = 0xFFFF;
uint8 u8ReportStatusNum = 0;
uint16 u16Hdl_tx_frame_cnt = 0;
uint16 u16Hdl_rx_frame_cnt;
static uint8 u8cardtype=1;
static uint8 helpask=0;
uint8 cur_slot= 0;
uint8 tof_count=0;

uint8 next_retreat =1 ;  //if is the next retreat beep 
uint16 count_tof=0;      //the number of the tof 
uint32 pretick=0;
uint8 quiet_count= 0;    //not moving and wake up time(s), if <10 short sleep ,if >10 long sleep when the card stay qiuet
uint16 u16ShortAddr;
uint8 newstarttype=0;
uint8 avg_tof_tick=0;
uint8 ever_rev_dis=0;   //wether the card have rev distance this time
uint8 bool_check_poll=0;
uint8 tStationStatus=0;
uint8 U8iCount = 0,u8ExciteTure=FALSE ;
uint16 U16ExciterID = 0 ,u16LastExciterID=0;
static uint16 U16TailMs;
uint8 new_inblink=0;
uint8 is_sendlink=0;

tsCardTypeSet CardType_5s;
tsCardTypeSet CardType_1s;
uint32 last_time=0;
extern uint8 rev_retreat_ack;
static uint16 pre_seqnum=0;
uint16 idle_list_count=0;   //the card Continuous idle count

uint16 car_cardwork_time=0;
uint16 car_rssiwork_time=0;
uint16 last_tof_seq=0;

void vCheckBattery(void);
void Appsleepinit(void);
/*****************************************************************************
* FUNCTION
*/
void Write_Devtype(uint16 type)
{
	DATA_EEPROM_Unlock();		 //解锁FLASH 
	DATA_EEPROM_EraseWord(DEVTYPE_ADDR);
	//DATA_EEPROM_ProgramHalfWord(DEVTYPE_ADDR,type);//*RamAdr
	*(uint16*)(DEVTYPE_ADDR)= type;
	DATA_EEPROM_Lock();	
}
uint16 instance_get_cardtype(void)
{
	uint16 cardtype=1,temp=0;
	temp= *(uint16*)(DEVTYPE_ADDR);
	if(temp!=1 && temp!=5)
	{
		temp = *(uint16*)(DEV_ADDRESS+2);
		if(temp==1 || temp ==5)
		{
			cardtype = temp;
			//Write_Devtype(temp);
		}
	}
	else
		cardtype = temp;
	return cardtype;
}


void AppSleepOff(uint_32 u32Event)
{
    eventHold |= u32Event;
}

void AppSleepOn(uint_32 u32Event)
{
    eventHold &= ~u32Event;
}

void AppGreenLedFlash()
{
    LED_Green_On();
    AppSleepOff(EVENT_GREEN_LED_FLASH);
    event_timer_add(EVENT_GREEN_LED_FLASH, LED_FLASH_TIME);
}




void AppRedLedFlash()
{
    LED_Red_On();
    AppSleepOff(EVENT_RED_LED_FLASH);
    event_timer_add(EVENT_RED_LED_FLASH, LED_FLASH_TIME);
}




void process_deca_irq(void)
{
    do{

    	instance_process_irq(0);
    }while(port_CheckIRQ() == 1); //while IRS line active (ARM can only do edge sensitive interrupts)

}

/*
* 电量检测
*/

uint16 card_clc_count =0;
uint8 motor_count=0;

void vCheckBattery()
{
	uint16 battery=0 ,battery1=0;

	if((card_clc_count-2)%(500/u8cardtype) ==0)
	{
		battery1 = ADC_Get_ADCValue();
		battery = (battery1 +60)/2.9+1;
		if(battery>300)
		{
			if(battery < 380)
				my_cardslotmsg.status |=UWB_CARD_STATUS_NOPWD;
			else
				my_cardslotmsg.status &=~UWB_CARD_STATUS_NOPWD;
			instance_set_vbat(battery);
			PrintfUtil_vPrintf(" battery =%d \n",battery);
		}
		else
			card_clc_count--;
	}
	
}


void Stop_ledvbeep(uint8 type)
{
	LED_Red_Off();
	LED_Green_Off();
	BEEP_Stop();
	if(type==0)
	{
		MOTOR_Stop();
		motor_count=0;
	}
	else
	{
		if(motor_count %2==0&&motor_count %4!=0)
			MOTOR_Stop();
		motor_count++;
	}
}

static void Check_card_lostnum(void)
{
	if(ever_rev_dis ==0 &&my_cardslotmsg.b1Used == USED_TOF)  //if rev poll but not tof seccuss, 4 times continuous than set idle
	{
		my_cardslotmsg.u8LostNum++;
		if(my_cardslotmsg.u8LostNum >=LOST_TOF_MAX_NUM){
			instance_set_idle();
			my_cardslotmsg.sleeptick = CARD_1S_SEC_TIME/4;  //when set idle form tof ,the this slot sleep time is short
			DBG(PrintfUtil_vPrintf("set idle\n");)
		}
	}
	else
	{
	my_cardslotmsg.u8LostNum =0;

	}
}

int8 card_reset_dw1000()
{
	uint32 devID ;
	instanceConfig_t instConfig;
    int  result;
	
	NVIC_DisableIRQ(EXTI0_IRQn);

	reset_DW1000();
    SPI_ConfigFastRate(SPI_BaudRatePrescaler_8);  //max SPI before PLLs configured is ~4M
	
	//this is called here to wake up the device (i.e. if it was in sleep mode before the restart)
    devID = instancereaddeviceid() ;
    if(DWT_DEVICE_ID != devID) //if the read of devide ID fails, the DW1000 could be asleep
    {
    	port_SPIx_clear_chip_select();	//CS low
    	mSleep(1);	//200 us to wake up then waits 5ms for DW1000 XTAL to stabilise
    	port_SPIx_set_chip_select();  //CS high
    	mSleep(7);

		//add Sleep(50) to stabilise the XTAL
        mSleep(50);

    	devID = instancereaddeviceid() ;
        // SPI not working or Unsupported Device ID
    	if(DWT_DEVICE_ID != devID)
    		return(-1) ;
    	//clear the sleep bit - so that after the hard reset below the DW does not go into sleep
    	dwt_softreset();
    }
	
	reset_DW1000();
	reinit_dw1000();
	//instance_init();
	SPI_ConfigFastRate(SPI_BaudRatePrescaler_4); //increase SPI to max
    devID = instancereaddeviceid() ;

    if (DWT_DEVICE_ID != devID)   // Means it is NOT MP device
    {
        // SPI not working or Unsupported Device ID
		return(-1) ;
    }

    instConfig.channelNumber = chConfig[0].channel ;
    instConfig.preambleCode = chConfig[0].preambleCode ;
    instConfig.pulseRepFreq = chConfig[0].prf ;
    instConfig.pacSize = chConfig[0].pacSize ;
    instConfig.nsSFD = chConfig[0].nsSFD ;

    instConfig.dataRate = chConfig[0].datarate ;
    instConfig.preambleLen = chConfig[0].preambleLength ;

    instance_config(&instConfig) ;                  // Set operating channel etc
    
#if (DR_DISCOVERY == 0)
    addressconfigure() ;                            // set up initial payload configuration
#endif
   // instancesetreplydelay(FIXED_REPLY_DELAY);

	Appsleepinit();
	NVIC_EnableIRQ(EXTI0_IRQn);
	return devID;
}

#ifdef DEC_UWB_ANCHOR

static void Beep_and_redflsh(void)
{
	AppRedLedFlash();
	BEEP_Start();
	event_timer_add(EVENT_LED_HUNGER,TOF_BUZZER_TIMEOUT);
}
static void AppHelpBegin(void)
{
//	cardWorkState = UWB_CARD_STATUS_HELP;
	u8HelpCnt = 0;
	AppSleepOff(EVENT_HELP_MSG);
}



static void AppHelpProc(void)
{
	uint8 status=0;
	status = my_cardslotmsg.status;
	status = instance_get_helpstatus();      //reset when rev help resp
	
	event_timer_unset(EVENT_SLEEP_EVENT);
	helpask =1;
	if(status ==0)
	{
		if(u8HelpCnt++ <= 10)
		{
			
			event_timer_add(EVENT_HELP_MSG,500);
			Beep_and_redflsh();
			DBG(PrintfUtil_vPrintf("Help cnt:%d\n",u8HelpCnt);)
		}
		else
		{
			AppGreenLedFlash();
			BEEP_Stop();
			MOTOR_Stop();
			DBG(PrintfUtil_vPrintf("Help stop:%d\n",u8HelpCnt);)
			event_timer_del(EVENT_HELP_MSG);	
			event_timer_add(EVENT_SLEEP_EVENT,1000); 
		//	instance_set_idle();
			instance_reset_helpstatus();
			u8HelpCnt = 0;
			helpask =0;
			set_helpover_sleep=1;
		}
	}
	else   //1or 2      //if rev help ack ,but not rev counter resp ,beep 5 times at lest
	{
		if(u8HelpCnt++ <6)   
		{
			event_timer_add(EVENT_HELP_MSG,500);
			Beep_and_redflsh();

			DBG(PrintfUtil_vPrintf("*Help cnt:%d\n",u8HelpCnt);)
		}
		else
		{
			AppGreenLedFlash();
			BEEP_Stop();
			MOTOR_Stop();
			DBG(PrintfUtil_vPrintf("Help rev stop:%d\n",u8HelpCnt);)
		//	instance_set_idle();
			instance_reset_helpstatus();
			
			event_timer_del(EVENT_HELP_MSG);			
			event_timer_add(EVENT_SLEEP_EVENT,1000);
			u8HelpCnt = 0;
			helpask =0;
			set_helpover_sleep=1;
		}
	}
	
}



static void AppAlarmProc(void)
{
//	event_timer_del(EVENT_SLEEP_EVENT);
	if(my_cardslotmsg.status &UWB_CARD_STATUS_RETREAT)
	{
		Beep_and_redflsh();
		if(motor_count %4==0)
			MOTOR_Start();
		event_timer_add(EVENT_URGENT_RETREAT, 500); 
		
		helpask =1;
	//	PrintfUtil_vPrintf("<AppAlarmProc>  %d \n",my_cardslotmsg.status);
	}
	else
	{
		helpask =0;
		Stop_ledvbeep(0);
	}
}



/*******************************************************************************
* 处理求救按键的函数
*/
//uint8 alarm_beep_stop=1;

static uint8 AppButtonProc(void)
{
	uint8 key_code=0;

	if ((key_code = BUTTON_KeyCode_Read()) == BTN_IDLE)
	{
		helpask = 0;
		event_timer_del(EVENT_SLEEP_EVENT);
		event_timer_add(EVENT_SLEEP_EVENT,1000);
	    return 0;
	}
//	PrintfUtil_vPrintf("key_code= <%d> <%d>\n",key_code,next_retreat);
	if ((key_code & BTN_HELP) )
	{
		helpask =1;
		event_timer_del(EVENT_SLEEP_EVENT);
		event_timer_add(EVENT_SLEEP_EVENT,1000);
		if(u8HelpCnt<10)
		{
			u8HelpCnt ++;
			set_helpover_sleep=0;
			event_timer_add(EVENT_BUTTON_MSG,200);
			DBG(PrintfUtil_vPrintf("H<%d> \n",u8HelpCnt);)
			return 2;
		}
		else
		{
			u8HelpCnt =0;
			helpask =0;
			my_cardslotmsg.status |= UWB_CARD_STATUS_HELP ;
		//	event_timer_del(EVENT_SLEEP_EVENT);
			event_timer_set(EVENT_HELP_MSG);
			instance_set_helpexcit(1);
		//	MOTOR_Start();
			return 1;
		}
		
	}
	else if ((key_code & BTN_CFRM)&& next_retreat ==1)
	{
		
		if(my_cardslotmsg.status & UWB_CARD_STATUS_RETREAT)
		{
			helpask =1;
			event_timer_del(EVENT_SLEEP_EVENT);
			event_timer_add(EVENT_SLEEP_EVENT,1000);
			if(u8HelpCnt<5)
			{
				u8HelpCnt ++;
			//	alarm_beep_stop =0;   
				set_helpover_sleep=0;
				event_timer_add(EVENT_BUTTON_MSG,200);
				DBG(PrintfUtil_vPrintf("<%d> \n",u8HelpCnt);)
				return 2;
			}
			else
			{
				BEEP_Stop();
				DBG(PrintfUtil_vPrintf("|* %d|",u8HelpCnt);)
				PrintfUtil_vPrintf("BEEP_Stop |* %d %d|",u8HelpCnt, my_cardslotmsg.status);
				
				u8HelpCnt =0; 
				next_retreat =0 ;   //when this time press stop ,then never beep again before the new retreat come
			//	instance_set_idle();
				if(my_cardslotmsg.status & UWB_CARD_STATUS_RETREAT)
				{
					my_cardslotmsg.status |= UWB_CARD_STATUS_RETREAT_ACK;
					txretreat_ack_send();  //send buttun ack
				
					event_timer_add(EVENT_URGENT_RESET,600000);
					event_timer_del(EVENT_URGENT_RETREAT);
				}
				event_timer_del(EVENT_SLEEP_EVENT);
				event_timer_add(EVENT_SLEEP_EVENT,1000);
				helpask =0;
				set_helpover_sleep =1;
				return 0;
			}
		}
		
	}
}

void CardMotionDetect()
{
    static teMMA845x_Data vmma845xData;

    static uint8 mma845x_invalid = FALSE;
    uint8 value;
    uint8 PlStauts;

    if (mma845x_invalid)
    {
    	PrintfUtil_vPrintf("invalid\n");
        return;
    }

    value = mma845xData.u8IntSrc = MMA845x_ReadReg(0x0C);     //INT_SOURCE

 //   if(0xFF == value)
    {
        if(MMA845x_ChipCheck())
        {
            mma845x_invalid = TRUE;
            u16CardMotionCnt = 0;
            my_cardslotmsg.status &= ~UWB_CARD_STATUS_IS_WITH_ACCEL;
            return;
        }
        else
        {
            mma845xData.u8IntSrc = MMA845x_ReadReg(0x0C);     //INT_SOURCE
            mma845x_invalid = FALSE;
            my_cardslotmsg.status |= UWB_CARD_STATUS_IS_WITH_ACCEL;
        }
    }

	mma845xData.u8FFMtSrc = MMA845x_ReadReg(0x16);    //FF_MT_SRC
	//mma845xData.u8Sysmod = MMA845x_ReadReg(0x0B);     //SYSMOD
	mma845xData.u8TransientSrc = MMA845x_ReadReg(0x1E);
	mma845xData.u8XbyteH = MMA845x_ReadReg(0x01);
	mma845xData.u8YbyteH = MMA845x_ReadReg(0x03);
	//mma845xData.u8ZbyteH = MMA845x_ReadReg(0x05);

    //DBG(PrintfUtil_vPrintf("FFM %d,Trans %d\n",mma845xData.u8FFMtSrc,mma845xData.u8TransientSrc);)

	PlStauts = MMA845x_ReadReg(0x10);   //PL Status Register

	if((DEFF(mma845xData.u8XbyteH,vmma845xData.u8XbyteH) > D_VALUE)
		|| (DEFF(mma845xData.u8YbyteH,vmma845xData.u8YbyteH) > D_VALUE))
		//|| (DEFF(mma845xData.u8ZbyteH,vmma845xData.u8ZbyteH) > D_VALUE))
	{
		my_cardslotmsg.status |= UWB_CARD_STATUS_ACTIVE;
	}
	//else if((mma845xData.u8TransientSrc & 0x40) || (mma845xData.u8FFMtSrc & 0x80))
    else if((mma845xData.u8TransientSrc & 0x40) || (mma845xData.u8FFMtSrc & 0x80))
	{
		my_cardslotmsg.status|= UWB_CARD_STATUS_ACTIVE;
	}
	else
	{
		my_cardslotmsg.status &= ~UWB_CARD_STATUS_ACTIVE;
	}

	vmma845xData = mma845xData;
	if((PlStauts & 0x04) && (!(PlStauts & 0x40)))
	{
		my_cardslotmsg.status |= UWB_CARD_STATUS_ORIENTATION;
	}
	else
	{
		my_cardslotmsg.status &= ~UWB_CARD_STATUS_ORIENTATION;
	}

    if(UWB_CARD_STATUS_ACTIVE & my_cardslotmsg.status)
    {
        u16CardMotionCnt = 0;
		quiet_count =0 ;
		isMoving = True;
    }
    else
    {
        if(u16CardMotionCnt <u16CardMotionTimeout)
            u16CardMotionCnt++;
		else
		{
			isMoving = False;
		//	cardMotionState = CARD_MOTION_REST;
			motion_detect_int_open();
		}
	//	PrintfUtil_vPrintf("MotionCnt = %d\n" ,u16CardMotionCnt);

    }

//	PrintfUtil_vPrintf("card status %x\n",my_cardslotmsg.status);
}

void vCheck_Devtype_change(void)
{

	if(instance_getchange_devtype())   //change the card's cycle
	{
		if(my_cardslotmsg.u8DeviceType == CARD_1S)
		{
			instance_init_cardslot(CARD_5S); //1:1s card  5:5s card
			u8cardtype = CARD_5S;
		}
		else if(my_cardslotmsg.u8DeviceType == CARD_5S)
		{
			instance_init_cardslot(CARD_1S); //1:1s card  5:5s card
			u8cardtype = CARD_1S;
		}
		instance_set_idle();
		PrintfUtil_vPrintf("--change devtype ok!-- !\n");
		Write_Devtype((uint16)u8cardtype);
	}
}

void Check_used_status(void)
{
	uint8 revpolltype=0;
	revpolltype =instance_get_revpolltype();
	if(my_cardslotmsg.b1Used != IDLE)
	{
		if(revpolltype ==0 && helpask==0)  //nothing rev then idle at once 
		{
			instance_set_idle();
			my_cardslotmsg.sleeptick = CARD_1S_SEC_TIME/4;  //when set idle form tof ,the this slot sleep time is short
		}
		else if(revpolltype !=0)   //rev some blink msg but not rev tof msg not idle at once 
			instance_set_AnchorPanid((uint8)ANCHOR_TOF_PRO);
	}
	else
	{
		new_inblink = instance_get_inblinkmsg();
		if(new_inblink)     //new in blink
		{
			instance_set_AnchorPanid((uint8)ANCHOR_TOF_PRO);
			my_cardslotmsg.b1Used = USED_TOF;
			ever_rev_dis =1; //is not a lost
		}
		//tdoa_send();
	}

}

uint8 calcrc_1byte(uint8 abyte)
{
    uint8 i,crc_1byte;
    crc_1byte=0;                //设定crc_1byte初值为0
    for(i = 0; i < 8; i++)
     {
       if(((crc_1byte^abyte)&0x01))
          {
            crc_1byte^=0x18;
            crc_1byte>>=1;
            crc_1byte|=0x80;
           }
        else
           crc_1byte>>=1;
        abyte>>=1;
      }
      return crc_1byte;
 }

// EVENT_EXCIT_EVENT
void ProcessExcite(void)
{
	static uint16 U16HeadMs;
	//static UINT16 U16ExciterID = 0;
	uint16 crcCode,crc;
	uint16 CycleLenth;
	uint8 u8PacketNum=0;
	U16HeadMs = (uint16)GetSysClock();
	if(U8iCount == 0)
	{
		U8iCount++;
		U16TailMs = U16HeadMs;
		U16ExciterID = 0;
		CycleLenth = 0;
	}
	else
	{
		CycleLenth = U16HeadMs - U16TailMs;
		U16TailMs = U16HeadMs;
	}

	if(CycleLenth > 1)  //两次中断之间间隔超过1ms则认为无效了
	{
		U8iCount = 1;
		U16ExciterID = 0;
	}

	uint8 U32SDAdio = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10);//PA_10;
	if(U32SDAdio & 0x1)
	{
		U16ExciterID |= 1<<(16-U8iCount);
	}
	U8iCount++;
	AppRedLedFlash();
	BEEP_Start();
	event_timer_add(EVENT_LED_HUNGER,20);
	if(U8iCount == 17)
	{
		uint8 u8HighExciterID=0,u8LowExciterID=0;
		U8iCount = 0;
		u8HighExciterID = (uint8)((U16ExciterID>>8)&0xff);
		u8LowExciterID = (uint8)(U16ExciterID&0xff);

		crcCode =calcrc_1byte(u8HighExciterID);
		crcCode = crcCode & 0x00FF;
		if((u8LowExciterID == crcCode) && (U16ExciterID !=0) && (u8HighExciterID == u16LastExciterID))
		{
			U16ExciterID = (uint16)u8HighExciterID;
			u8ExciteTure = TRUE;
			my_cardslotmsg.status |= UWB_CARD_STATUS_IMPEL;
			event_timer_set(EVENT_EXCIT_EVENT);   //go send excit status to sub
			instance_set_helpexcit(2);
			PrintfUtil_vPrintf("--excit secuss  ! %d -- !\n",U16ExciterID);
		}
		event_timer_add(EVENT_SLEEP_EVENT, 1000);
		u16LastExciterID =(uint16) u8HighExciterID;
	}
}


void card_newslot_init(void)
{
	uint32 time=0;
	clear_inblinkmsg();      //every new slot ,the blink status start from 0	
	newstarttype =1;
	time = pretick - last_time;
	if(time>u8cardtype*CARD_1S_SEC_TIME-200)
	{
		my_cardslotmsg.u16SeqNum++;
		last_time = pretick;
		card_clc_count ++;
	}
  	my_cardslotmsg.sleeptick =0;
	my_cardslotmsg.m_distance = 0;
	CardMotionDetect();          //检查卡的运动情况
	new_inblink=0;
	//dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG , 1); 
}


#endif

uint8 clear_car_cardlist(int count)
{
	int i=0,sum=0;
	if(count ==0)
		return 0;
	sum = Car_revcardlist.u8CardCnt;
	if(count>=sum )
	{
		Car_revcardlist.m_distance = 0;
		Car_revcardlist.s_distance = 0;
		Car_revcardlist.u8CardCnt = 0;
		count = MAX_CAR_CARD_CNT;
	}
	for(i=0;i<count;i++)
	{
		memset(&Car_revcardlist.cardmsg[i],0,sizeof(Car_cardsmsg_t));
	}
	if(count<sum)
	{
		Car_revcardlist.u8CardCnt = sum - count ;
		for(i=0;count<sum;count++,i++)
			memcpy(&Car_revcardlist.cardmsg[i],&Car_revcardlist.cardmsg[count],sizeof(Car_cardsmsg_t));
		return 1;
	}
	return 0;
}
/*******************************************************************************/
void vProcessCardVersion(uint_16 u16DevId, uint_16 u16OadVersion, uint_16 u16Battery)
{
    uint_8 i;
    uint8 bfind = FALSE;

    /* if already exist */
    for(i=0; i<APP_TOF_VERSION_MAX_NUM; i++)
    {
        if(tsVersionReport[i].devid!=0 && tsVersionReport[i].devid == u16DevId)
        {
            tsVersionReport[i].devid = u16DevId;
            tsVersionReport[i].oad_ver = u16OadVersion;
            tsVersionReport[i].battery = u16Battery;
            bfind = TRUE;
            break;
        }
    }

    /* else fill a new position */
    if(!bfind)
    {
        for(i=0; i<APP_TOF_VERSION_MAX_NUM; i++)
        {
            if(tsVersionReport[i].devid ==0)
            {
                tsVersionReport[i].devid = u16DevId;
                tsVersionReport[i].oad_ver = u16OadVersion;
                tsVersionReport[i].battery = u16Battery;
                tsVersionReportLen++;
                bfind = TRUE;
                break;
            }
        }
    }

    if(tsVersionReportLen >= APP_TOF_VERSION_MAX_NUM)
    {
        //立即发送
        event_timer_unset(EVENT_REPORT_CARDVERSION);
		event_timer_set(EVENT_REPORT_CARDVERSION);
    }
}


void clear_car_cardmsg(uint8 start)
{
	int i=0;
	for(i=0;i<MAX_CAR_CARD_CNT;i++)
	{
		Car_revcardlist.cardmsg[i].devtype=0;
		Car_revcardlist.cardmsg[i].status =0;
		Car_revcardlist.cardmsg[i].u8cardaddr[0] =0;
		Car_revcardlist.cardmsg[i].u8cardaddr[1] =0;
	}
	if(start ==0)
	{
		Car_revcardlist.u8CardCnt =0;
		Car_revcardlist.m_distance=0;
		Car_revcardlist.s_distance=0;
	}
	else
	{
		Car_revcardlist.u8CardCnt =start;
	}	
}


//return 1 :cardtype have change ;return 0:no change
uint8 check_cardtype(uint16 cardid, uint8 cardtype)
{
	uint8 i=0;
	if(CardType_5s.u8CardCnt == 0&& CardType_1s.u8CardCnt==0)
		return 0;
	if(cardtype == CARD_1S)
	{
		for(i=0;i<CardType_5s.u8CardCnt;i++)
		{
			if(CardType_5s.u16CardNum[i] == cardid)
				return 1;
		}
	}
	else if(cardtype == CARD_5S)
	{
		for(i=0;i<CardType_1s.u8CardCnt;i++)
		{
			if(CardType_1s.u16CardNum[i] == cardid)
				return 1;
		}
	}
	return 0;
}


void reset_cardtype_list()
{
	memset(&CardType_5s,0,sizeof(tsCardTypeSet));
	memset(&CardType_1s,0,sizeof(tsCardTypeSet));
	CardType_5s.u8CardCnt =0;
	CardType_1s.u8CardCnt =0;
}
/*******************************************************************************/
void vInitParameter()
{
	memset((uint8*)tsVersionReport, 0, APP_TOF_VERSION_MAX_NUM*sizeof(app_UWBVersionReport_t));
}

void car_cardwork_start()
{
	chip_car_lowpower_finish();
	if(my_cardslotmsg.status &UWB_CARD_STATUS_NOPWD)
		LED_Red_On();
	else
		LED_Green_On();
	event_timer_add(EVENT_RED_LED_FLASH,10);
	pretick = GetSysClock();
	instance_set_slot_starttick(pretick);
//new12	PrintfUtil_vPrintf("--------card ----- -!\n");
	
	event_timer_set(EVENT_RAGING_REPORT);
	if((idle_list_count)>=CARD_RESET_NOTOF_COUNT && !new_inblink)//*u8cardtype
	{
		if(card_reset_dw1000() ==-1)  //reset the dw1000
			NVIC_SystemReset();
		idle_list_count=0;
		//instance_set_idle();
		my_cardslotmsg.b1Used = IDLE;
//new12		PrintfUtil_vPrintf("--------reset----- -!\n");
	}
	
	idle_list_count++;
	
	
	if(my_cardslotmsg.b1Used != IDLE && helpask ==0)
	{
		event_timer_add(EVENT_CHECK_IDLE, car_cardwork_time-20); 
		instance_set_AnchorPanid((uint8)ANCHOR_TOF_PRO);
		last_tof_seq = my_cardslotmsg.u16SeqNum;
	}
	else if(my_cardslotmsg.b1Used == IDLE )
	{
		event_timer_add(EVENT_CHECK_IDLE, car_cardwork_time-5); 
		instance_set_AnchorPanid((uint8)ANCHOR_BLINK_PRO);
		//PrintfUtil_vPrintf("--------blink pro----- -!\n");
	}
	reset_appstate();

}
static void App_SleepProc(void)
{
	int sleeptime=0;
	uint16 uptick=0,wakeuptime=CARD_WAKE_UP_MAX;
    uint32 systicksPritnf = portGetTickCnt();
	uint32 temp=0,temp1=0;

	uptick = instance_get_uptick();
	if(uptick!=0)
		last_tof_seq = my_cardslotmsg.u16SeqNum;  //rev any station's sigal ,then belive there have a tof station in near the car's card
	dwt_setleds(0);
	if(helpask==0)     //if the button have down, not sleep
	{
	   // PrintfUtil_vPrintf(" help need to sleep \n");
		Stop_ledvbeep(0);
	//	dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG   | DWT_INT_RFTO, 0); 
		if((quiet_count <=5 && isMoving==False) || isMoving == True)
		{
			quiet_count ++;
			if(uptick ==0 && my_cardslotmsg.b1Used == IDLE)  //not rev anything and in idle status
			{
				if(u8cardtype ==1)
					sleeptime = CARD_1S_SEC_TIME;
				else if(u8cardtype ==5)
					sleeptime = CARD_5S_SEC_TIME;
				if(my_cardslotmsg.sleeptick == CARD_1S_SEC_TIME/4)
				{
					sleeptime = my_cardslotmsg.sleeptick;
				}
			}
			else    //have been rev blink or poll ,or in tof status
			{
				Check_card_lostnum();
				if(uptick <2)
					uptick=5;    //uptick is the tick should be wake up before of the card

				temp = (uint32)(GetSysClock()- instance_get_slot_starttick());
				if(set_helpover_sleep ==0)
				{
					temp1 =(uint32)( temp-uptick) ;
					if(temp1>0xfffff000)
						temp1 = 0xffffffff- temp1;

					if(my_cardslotmsg.sleeptick >(temp1+11))
						sleeptime  = my_cardslotmsg.sleeptick - temp1-11;
					else
						sleeptime =0;
				}
				else   //just help beep over then enter sleep ,but it's have a different count
				{
					temp1 =(uint32)( temp-uptick) ;
					while(my_cardslotmsg.sleeptick <(temp1+11)){
						if(u8cardtype ==1)
							my_cardslotmsg.sleeptick += CARD_1S_SEC_TIME;
						else if(u8cardtype ==5)
							my_cardslotmsg.sleeptick += CARD_5S_SEC_TIME;
					}
					sleeptime  = my_cardslotmsg.sleeptick - temp1-11;
					set_helpover_sleep =0;
				}
				if(u8cardtype ==CARD_1S&& sleeptime >=2000)
					sleeptime = CARD_1S_SEC_TIME-CARD_WAKE_UP_MAX;
				else if(u8cardtype ==CARD_5S&& sleeptime >=7000)
					sleeptime = CARD_5S_SEC_TIME-CARD_WAKE_UP_MAX*2;
			}
			wakeuptime = CARD_WAKE_UP_MAX;	
		}
		else
		{
			DBG(PrintfUtil_vPrintf(" ***********  long sleep :: isMoving =%d >> quiet_count =%d\n",isMoving,quiet_count);)
			quiet_count =0 ;
			sleeptime = 120*1000;  //2minute
			instance_set_idle();
			wakeuptime = CARD_WAKE_UP_MAX+30;  //long sleep need more time to rev blink 
		}
		ever_rev_dis =0;
		
		if(u8cardtype ==CARD_5S)
		{
		//	sleeptime += 30;  //20ms more ,becuse the time is not synchronization ,not *0.88 anymore 22ms less    6S card is 30ms
			wakeuptime = CARD_WAKE_UP_MAX+20;
		}
/*
		if(sleeptime >15)
		{
			system_sleep((uint32)sleeptime);
			dec_sleep_wakeup();
		}
*/		
		//mSleep((uint32)sleeptime-50);
		dec_sleep_wakeup();
		vCheckBattery();
		
	
	}
	else
	{
		DBG(PrintfUtil_vPrintf(" help not sleep \n");)
		wakeuptime = CARD_1S_SEC_TIME+100;
		
		Check_card_lostnum();
		set_helpover_sleep =1;
		//system_sleep(25);
		//dec_sleep_wakeup();
		if(my_cardslotmsg.u16SeqNum - pre_seqnum>=1)
			ever_rev_dis =0;
		pre_seqnum = my_cardslotmsg.u16SeqNum;
	}
	// if idle  to much ,then reset the dw100

	car_cardwork_time = wakeuptime;
	car_rssiwork_time = sleeptime-7;
	//mSleep(car_rssiwork_time);
	//event_timer_add(EVENT_SLEEP_EVENT,wakeuptime+100 );
	if(my_cardslotmsg.b1Used == IDLE)
		car_rssiwork_time +=35;
	event_timer_add(EVENT_CAR_CARD_WORK,car_rssiwork_time );
	event_timer_add(EVENT_CAR_RSSI_SEND,car_rssiwork_time-7 );
	if(sleeptime>15)
		event_timer_set(EVENT_CAR_RSSI_WORK);
//new12	PrintfUtil_vPrintf(" out sleep ,time=%d | %d|%d |%d |%d |%i\n\n",sleeptime,temp,uptick,my_cardslotmsg.sleeptick,u16CardMotionCnt,instance_get_powerlever());//

}

void AppEventsProcess(void)
{
  	uint32 events = event_timer_take_all();
    uint32 systicksPritnf = portGetTickCnt();
#ifdef DEC_UWB_ANCHOR	
	if (events & EVENT_MMA8452Q_EVENT)
	{
		motion_detect_int_close();
		isMoving = True;
		u16CardMotionCnt =0 ;
	    CardMotionDetect();          //检查卡的运动情况    
	//    AppUpdateCardMotionState(); //更新卡是否运动，改变卡的工作周期
	//	 DBG(PrintfUtil_vPrintf("----------------------------\n");)
	}
#endif
	if(events &(EVENT_CAR_RSSI_WORK))
	{
	    //位置  休眠时间 systick car_cardwork_time car_rssiwork_time
		// PrintfUtil_vPrintf("rssi [%d] [%d][%d] %d \n", my_cardslotmsg.b1Used, car_cardwork_time, car_rssiwork_time, systicksPritnf);
		// car card do rssi station work
		chip_car_lowpower_ready();
		init_car_forrssi();
		event_timer_set(EVENT_RAGING_REPORT);
	}
	
	if (events & (EVENT_RAGING_REPORT))
	{	
			
	    instance_run(newstarttype);
		if((instancegetrole() == ANCHOR || instancegetrole() == SUB_STA )&&newstarttype ==1)
			newstarttype=0;

	}

	if (events & (EVENT_CAR_CARD_WORK))
	{
		dwt_forcetrxoff();  //must be do this first,or the card will Frequently fall net 
		init_car_forcard();
		dec_sleep_wakeup();
		car_cardwork_start();
		card_newslot_init();   //should be after car_cardwork_start
		WatchdogReset();
		
		event_timer_add(EVENT_SLEEP_EVENT,car_cardwork_time );
	}
#ifdef DEC_UWB_ANCHOR
	if (events & EVENT_BUTTON_MSG)
	{
		uint8 type;
		type = AppButtonProc();
	    if(type==1)       //help ask
		{
			dwt_forcetrxoff();
			instance_set_AnchorPanid((uint8)ANCHOR_TOF_PRO); //tof pandi ask help
			helpask =1;
		}

	
	}
	if (events & EVENT_EXCIT_EVENT)
	{
		if(u8ExciteTure != TRUE)      //excit interrupt
			ProcessExcite();
		
		if(u8ExciteTure == TRUE) 
		{
			txexcit_wait_send(U16ExciterID);    //send to sub station
			u8ExciteTure = FALSE;
		}
	}
	if (events & EVENT_LED_HUNGER)
	{
		Stop_ledvbeep(1);
	}
	
	if (events & EVENT_HELP_MSG)
	{

	    AppHelpProc();
		if(instance_get_helpstatus()== 0 &&helpask ==1)
		{
			txhelp_call_send();
			PrintfUtil_vPrintf("begain send help !\n");
		}	
	}

	if (events & EVENT_URGENT_RETREAT)
	{
		uint8 temp = instance_get_retreatstatus() ;
		if(temp && (next_retreat == 1) )
		{
	    	//PrintfUtil_vPrintf("card set reteat beep !\n");      // 卡测试撤离过程
			AppAlarmProc();
		}
		else if(!temp)
		{
			PrintfUtil_vPrintf("1-retreat : %d | %d!\n",instance_get_retreatstatus(),next_retreat); 
			next_retreat = 1;
			helpask =0;
			Stop_ledvbeep(0);
		}
		else
			PrintfUtil_vPrintf("2-retreat : %d | %d!\n",instance_get_retreatstatus(),next_retreat); 
	}
#endif

	if (events & EVENT_URGENT_RESET)
	{

#ifndef DEC_UWB_ANCHOR	
		instance_set_alarmlist(0,0xFF,0);
#else
		my_cardslotmsg.status &= ~UWB_CARD_STATUS_RETREAT_ACK;
		next_retreat = 1;    //be ready for the next one to beep

#endif
	}

	if (events & EVENT_DEVTYPE_RESET)
	{
		reset_cardtype_list();
	}

	if (events & (EVENT_GREEN_LED_FLASH))
	{
	    LED_Green_Off();
	}

	if (events & (EVENT_RED_LED_FLASH))
	{
	    LED_Red_Off();
		LED_Green_Off();
		
	}


	if (events & (EVENT_CAR_RSSI_SEND))
	{
		uint16 temp;
		if(Car_revcardlist.u8CardCnt>0)
		{
			
			temp = my_cardslotmsg.u16SeqNum - last_tof_seq ;//if bigger than 4, there have not a tof station
			if(my_cardslotmsg.b1Used != IDLE && helpask ==0 && temp<=4)
			{
				instance_set_AnchorPanid((uint8)ANCHOR_TOF_PRO);
			}
			else if(my_cardslotmsg.b1Used == IDLE )
			{
				instance_set_AnchorPanid((uint8)ANCHOR_BLINK_PRO);
			}
			else
			{
				instance_set_AnchorPanid((uint8)ANCHOR_BLINK_PRO);
			}
			
			if(Car_revcardlist.u8CardCnt < MAX_CAR_CARD_SEND_COUNT)
			{
				txCar_list_wait_send(Car_revcardlist.u8CardCnt);
				clear_car_cardlist(Car_revcardlist.u8CardCnt);
			}
			else
			{
				txCar_list_wait_send(MAX_CAR_CARD_SEND_COUNT);
				clear_car_cardlist(MAX_CAR_CARD_SEND_COUNT);
				//event_timer_add(EVENT_CAR_RSSI_SEND,4);
			}
				
		}
	}

	if (events & (EVENT_CHECK_IDLE))
	{
		if(my_cardslotmsg.b1Used == USED_TOF)
		{
			if(!instance_get_revpolltype())
			{
				if(helpask==0)
					instance_set_AnchorPanid((uint8)ANCHOR_BLINK_PRO);
			}
			else
				instance_set_revpolltype(1);
			bool_check_poll=1;
		}
		else if(my_cardslotmsg.b1Used == IDLE)
		{
			tdoa_send();
		}
	}
	if (events & (EVENT_SLEEP_EVENT))
	{
		if(U8iCount >0) //excit
		{
			uint16 temp = (uint16)GetSysClock();
			if(temp -U16TailMs >100)
				U8iCount =0;
			else
			{
				event_timer_add(EVENT_SLEEP_EVENT, 50); 
				return;
			}
		}
		bool_check_poll=0;
		Check_used_status();
		//dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG , 0); 	//after send the tdoa msg
		//dwt_forcetrxoff();
		vCheck_Devtype_change();
		App_SleepProc();

		//dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG , 1); 	
	}


}


void led_tofresult(void)
{


	
	if(instancenewrange())
	{
		uwb_tof_distance_ts *distance;
		int seqnum = 0 ,precount=0;
		ever_rev_dis =1;
		if(tof_count >0)
			precount = tof_count-1;
		else
			precount =0;
		seqnum = instance_get_seqnum();
#ifdef DEC_UWB_ANCHOR
		idle_list_count=0;
		if(instancegetrole() == ANCHOR)
		{
			
			event_timer_del(EVENT_SLEEP_EVENT);			
			event_timer_set(EVENT_SLEEP_EVENT);
			PrintfUtil_vPrintf("\n------------dist= %d cm  ; seq = %d \n",my_cardslotmsg.m_distance,seqnum);
		}
#else
		led_station_off();
		
		if (!instance_get_distancelist(&distlist[tof_count],&distlist[precount],0)&& instancegetrole() == SUB_STA)
			tof_count= precount;
		distance = (uwb_tof_distance_ts *)&distlist[tof_count];
		//else
			tof_count++;
		
		DBG(PrintfUtil_vPrintf("\n------------M_dist= %d cm  ; seq = %d  |%x\n",distance->u32StationDistance,seqnum,distance->u8Status);)

		count_tof++;
		if(instancegetrole() == SUB_STA)
		{
			
			DBG(PrintfUtil_vPrintf("------------S_dist= %d cm  cardid = %d \n\n",distance->u32LocDistance,distance->u16ShortAddr);)
			instance_clear_substa();
		}
		
		//tof_count++;
		if(tof_count >=18)
		{	
			event_timer_set(EVENT_UART_SEND);
		}
		
#endif

	}

}


/*
* 系统入口
*/

void Appsleepinit(void)
{

	dwt_setdblrxbuffmode(0); //disable double RX buffer
	dwt_enableautoack(ACK_RESPONSE_TIME); //wait for 5 symbols before replying with the ACK

#if (DEEP_SLEEP == 1)
#if (DEEP_SLEEP_AUTOWAKEUP == 1)
	dwt_configuresleep(DWT_LOADUCODE|DWT_PRESRV_SLEEP|DWT_CONFIG|DWT_TANDV, DWT_WAKE_SLPCNT|DWT_WAKE_CS|DWT_SLP_EN); //configure the on wake parameters (upload the IC config settings)
#else
	//NOTE: on the EVK1000 the DEEPSLEEP is not actually putting the DW1000 into full DEEPSLEEP mode as XTAL is kept on
#if (DEEP_SLEEP_XTAL_ON == 1)
	dwt_configuresleep(DWT_LOADUCODE|DWT_PRESRV_SLEEP|DWT_CONFIG|DWT_TANDV, DWT_WAKE_CS|DWT_SLP_EN|DWT_XTAL_EN); //configure the on wake parameters (upload the IC config settings)
#else
	dwt_configuresleep(DWT_LOADUCODE|DWT_PRESRV_SLEEP|DWT_CONFIG|DWT_TANDV, DWT_WAKE_CS|DWT_SLP_EN); //configure the on wake parameters (upload the IC config settings)
#endif
#endif
#endif
}


void Application(void)
{
    PrintfUtil_vPrintf("\n ************** main start************* u16ShortAddr = %d\n", u16ShortAddr);

    Appsleepinit();
    LED_Red_Off();
    LED_Green_Off();
	vInitParameter();
	reset_cardtype_list();
	/*if(instancegetrole() == TAG)//(mode==1)    //M_station
	{
		event_timer_set(EVENT_NEWSLOT_EVENT);
		event_timer_add(EVENT_REPORT_CARDVERSION, 10); 
		
		instance_init_slotlist();
		event_timer_add_reload(EVENT_NEWSLOT_EVENT, EVERY_SLOT_TIME);   
	}
	else*/
	{
		newstarttype =1;  //first power on ,the  anchor wake up enter the init status		
		event_timer_set(EVENT_RAGING_REPORT);
		if(instancegetrole() == ANCHOR)
			event_timer_add(EVENT_SLEEP_EVENT, CARD_WAKE_UP_MAX+20); 
        
		if(instancegetrole() == ANCHOR)
		{
			u8cardtype = 1;//instance_get_cardtype();
			instance_init_cardslot(u8cardtype); //1:1s card  5:5s card
			instance_set_idle();
		}
		else if(instancegetrole() == SUB_STA)
		{
			instance_clear_substa();
			event_timer_add(EVENT_REPORT_CARDVERSION, 10); 
			instance_change_channel(ANCHOR_TOF_CHANNEL);
		}
	}
    while (1)
    {

		if(instance_get_status()==1)
		{
			event_timer_unset(EVENT_RAGING_REPORT);
			event_timer_set(EVENT_RAGING_REPORT);
		}
		event_timer_update();
		AppEventsProcess();
		led_tofresult();
	//	AppSavePower();

	}
}

