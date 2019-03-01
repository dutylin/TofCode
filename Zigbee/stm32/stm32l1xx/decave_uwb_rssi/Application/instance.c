// -------------------------------------------------------------------------------------------------------------------
//
//  File: instance.c - application level message exchange for ranging demo
//
//  Copyright 2008 (c) DecaWave Ltd, Dublin, Ireland.
//
//  All rights reserved.
//
//  Author: Billy Verso, December 2008
//
// -------------------------------------------------------------------------------------------------------------------

#include "compiler.h"
#include "port.h"
#include "deca_device_api.h"
#include "deca_spi.h"
#include "printf_util.h"
#include "instance.h"
#include "timer_event.h"
#include "config.h"

// -------------------------------------------------------------------------------------------------------------------

#define INST_DONE_WAIT_FOR_NEXT_EVENT   1   //this signifies that the current event has been processed and instance is ready for next one
#define INST_DONE_WAIT_FOR_NEXT_EVENT_TO    2   //this signifies that the current event has been processed and that instance is waiting for next one with a timeout
                                        //which will trigger if no event coming in specified time
#define INST_NOT_DONE_YET               0   //this signifies that the instance is still processing the current event

//function codes
#define RTLS_DEMO_MSG_RNG_INIT				(0x20)			// Ranging initiation message
#define RTLS_DEMO_MSG_TAG_POLL              (0x21)          // Tag poll message
#define RTLS_DEMO_MSG_ANCH_RESP             (0x10)          // Anchor response to poll
#define RTLS_DEMO_MSG_TAG_FINAL             (0x29)          // Tag final massage back to Anchor (0x29 because of 5 byte timestamps needed for PC app)
#define RTLS_DEMO_MSG_ANCH_TOFR             (0x2A)          // Anchor TOF Report message
#define RTLS_TOF_MSG_TAG_ACK                (0x22)          //ACK
#define RTLS_MSG_HELP_CALL                  (0x23)          //card ask help 
#define RTLS_MSG_HELP_RESP                  (0x24)          //station rev help and resp
#define RTLS_EVACUATE_ASK_SEND              (0x25)          //evacute alarm 
#define RTLS_EXCIT_ASK_SEND                 (0x26)          //exicit 
#define RTLS_EXCIT_ACK_SEND                 (0x27)          //exicit 
#define RTLS_TDOA_BLINK_SEND                (0x28)          //TDOA  same as the resp
#define RTLS_RETREAT_ACK_SEND               (0x2B)          //press the buttun to stop retreat
#define RTLS_CAR_REVTOFCARD_MSG             (0x2C)          //the car's card send the human card msg



#define BLINK_BYTE   0xC5
#define DK1000_WAKEUP_TIME   (35*CLOCKS_PER_MILLI)

//application data message byte offsets
#define FCODE                               0               // Function code is 1st byte of messageData
#define PTXT                                1
#define RRXT                                6
#define FTXT                                11
#define TOFR                                1
#define RES_R1                              1               // Response option octet 0x02 (1),
#define RES_R2                              2               // Response option paramter 0x00 (1) - used to notify Tag that the report is coming
#define RES_R3                              3               // Response option paramter 0x00 (1),
#define RES_T1                              3               // Ranging request response delay low byte
#define RES_T2                              4               // Ranging request response delay high byte
#define RES_SLOT                            5
#define RES_TICK1                           6
#define RES_TICK2                           7
#define RES_TICK3                           8
#define RES_TICK4                           9

#define POLL_TEMP                           1               // Poll message TEMP octet
#define POLL_VOLT                           2               // Poll message Voltage octet

#define MAX_NUMBER_OF_POLL_RETRYS 			2//20

#define ACK_REQUESTED                       (1)             // Request an ACK frame
#define BLINK_ALL_RESP                     (0)              //all idle card nedd response
#define BLINK_5S_RESP                       (2)             //only 5s card need response
#define BLINK_NO_RESP                       (3)             //rev blink but not need send ranging init

#define CARD_STATUS                          1              //card current status
#define SLOT_MSG                             2              //1s card or 5s card

#define allot_slot_1                         3              //blink the first allot addr's place
enum inst_states
{
    TA_INIT, //0

    TA_TXE_WAIT,                //1/
    TA_TXPOLL_WAIT_SEND,        //2/
    TA_TXFINAL_WAIT_SEND,       //3/
    TA_TXRESPONSE_WAIT_SEND,    //4/
    TA_TXREPORT_WAIT_SEND,      //5
    TA_TX_WAIT_CONF,            //6

    TA_RXE_WAIT,                //7
    TA_RX_WAIT_DATA,            //8

    TA_SLEEP,					//9
    TA_SLEEP_DONE,				//10
	TA_TXBLINK_WAIT_SEND,		//11
    TA_TXRANGINGINIT_WAIT_SEND,  //12
    TA_TX_RANGING_ACK_SEND ,     //13
    TA_HELP_CALL_SEND ,           //14
    TA_HELP_RESP_SEND ,           //15
    TA_EVACUATE_ASK_SEND,         //16
    TA_EXCIT_WAIT_SEND,           //17
    TA_EXCIT_ACK_SEND,            //18
    TA_TDOA_WAIT_SEND,             //19
    TA_RETREAT_ACK_SEND           //20
} ;

typedef struct {
                uint8 PGdelay;

                //TX POWER
                //31:24     BOOST_0.125ms_PWR
                //23:16     BOOST_0.25ms_PWR-TX_SHR_PWR
                //15:8      BOOST_0.5ms_PWR-TX_PHR_PWR
                //7:0       DEFAULT_PWR-TX_DATA_PWR
                uint32 txPwr[2]; //
}tx_struct;

//The table below specifies the default TX spectrum configuration parameters... this has been tuned for DW EVK hardware units
const tx_struct txSpectrumConfig[8] =
{
    //Channel 0 ----- this is just a place holder so the next array element is channel 1
    {
            0x0,   //0
            {
                    0x0, //0
                    0x0 //0
            }
    },
    //Channel 1
    {
            0xc9,   //PG_DELAY
            {
                    0x75757575, //16M prf power
                    0x67676767 //64M prf power
            }

    },
    //Channel 2
    {
            0xc2,   //PG_DELAY
            {
                    0x75757575, //16M prf power
                    0x67676767 //64M prf power
            }
    },
    //Channel 3
    {
            0xc5,   //PG_DELAY
            {
                    0x6f6f6f6f, //16M prf power
                    0x8b8b8b8b //64M prf power
            }
    },
    //Channel 4
    {
            0x95,   //PG_DELAY
            {
                    0x5f5f5f5f, //16M prf power
                    0x9a9a9a9a //64M prf power
            }
    },
    //Channel 5
    {
            0xc0,   //PG_DELAY
            {
                    0x48484848, //16M prf power
                    0x85858585 //64M prf power
            }
    },
    //Channel 6 ----- this is just a place holder so the next array element is channel 7
    {
            0x0,   //0
            {
                    0x0, //0
                    0x0 //0
            }
    },
    //Channel 7
    {
            0x93,   //PG_DELAY
            {
                    0x92929292, //16M prf power
                    0xd1d1d1d1 //64M prf power
            }
    }
};

//these are default antenna delays for EVB1000, these can be used if there is no calibration data in the DW1000,
//or instead of the calibration data
const uint16 rfDelays[2] = {
        (uint16) ((DWT_PRF_16M_RFDLY/ 2.0) * 1e-9 / DWT_TIME_UNITS),//PRF 16
        (uint16) ((DWT_PRF_64M_RFDLY/ 2.0) * 1e-9 / DWT_TIME_UNITS)
};


// -------------------------------------------------------------------------------------------------------------------
//      Data Definitions
// -------------------------------------------------------------------------------------------------------------------
#define MAX_NUMBER_OF_REPORT_RETRYS (3)         // max number of times to send/re-send the report message

#define FIXED_REPORT_DELAY                  2 //15 //ms             //tx delay when sending the (ToF) report
#define RX_ON_TIME                          2 //ms                  //the time RX is turned on before the reply from the other end
#define RX_FWTO_TIME                        (RX_ON_TIME + 5) //ms //total "RX on" time
#define BLINK_SLEEP_DELAY					1000 //ms

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// NOTE: the maximum RX timeout is ~ 65ms
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// -------------------------------------------------------------------------------------------------------------------
/*
int stateCount = 0;
double inst_idist = 0;
double inst_adist = 0;
double inst_ldist = 0;
double inst_m_idist =0;
uint32 u32inst_m_idist =0;
*/
instance_data_t instance_data[NUM_INST] ;


uint32 TAGRngInitTick = 0;
uint32 TAGReportTick = 0;
uint16 TAGPollCnt = 0;

uint32 AnchorPollTick = 0;
uint32 RangeTick = 0;
uint32 LastRangeTick = 0;
uint32 DiffRangeTick = 0;
uint16 tx_antennaDelay =0;



uint16 test_tof_an_addr=0;        //tof poll start process dest addr
//uint8 card_rev_poll_continue=0;   //if station power on again ,card didn't revpoll anytime then  "is_in_blink=0"  ;slot_msg_t->u8LostNum
int rx_time=0;         //how many time this slot rev
uint8 helpreportoff=0;   //if the service counter report the help ask ,1:yes
uint8 new_2blink_tick=0;



alloc_Slot_t my_slotlist[10] ;    //TOF_SLOT_LOC_PERIOD  = 1000/EVERY_SLOT_TIME
alarm_addrlist_t my_alarmlist;
slot_msg_t my_cardslotmsg ,my_staslotmsg;
Sub_alarm_msg_t sub_alarmmsg;
ts_Car_cardlist Car_revcardlist;

extern uint16 u16ShortAddr;

//uint16 test_final_dea=0;

uint64 tag_txTimeTamp= 0;
uint64 anchor_rxTimeTamp = 0;
uint16 squem_count=0;
uint8 tag_addr=0;
uint8 quickORslow;
uint16 recv_cardid =0;
uint8 rev_retreat_ack=0;
extern app_uwb_rssi_ts app_uwb_rssi_data;
extern uint_8 u8rssiNum;
int8 rev_rssi=0;
// -------------------------------------------------------------------------------------------------------------------
// Functions
// -------------------------------------------------------------------------------------------------------------------

void led_station_flash(void)
{
	uint8 temp=0;
	temp = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12);//PA_10;
	if(temp)
		led_station_on();
	else
		led_station_off();
}


uint16 instance_get_cardid(void)
{
	uint16 cardid=0;

	cardid = *(uint16*)(DEV_ADDRESS);
	return cardid;
}

void instance_set_insleep(void)
{
	//event_timer_unset(EVENT_SLEEP_EVENT);			
	//event_timer_add(EVENT_SLEEP_EVENT, 10); 

}

void instance_set_sta_status(uint8 status)  //set main station's status
{
	instance_data[0].station_status |= status;
}

void instance_reset_sta_status(uint8 status) //reset main station's status
{
	instance_data[0].station_status &= ~status;

}

uint8 instance_get_sta_status(uint8 status)   //sub station check the help status,if return 1 send uart ask help
{
	uint8 x = instance_data[0].station_status & status;
	instance_data[0].station_status &= ~status;
	return x;
}

// -------------------------------------------------------------------------------------------------------------------
// convert microseconds to device time
uint64 convertmicrosectodevicetimeu (double microsecu)
{
    uint64 dt;
    long double dtime;

    dtime = (microsecu / (double) DWT_TIME_UNITS) / 1e6 ;

    dt =  (uint64) (dtime) ;

    return dt;
}



// -------------------------------------------------------------------------------------------------------------------
//
// function to construct the message/frame header bytes
//
// -------------------------------------------------------------------------------------------------------------------
//
void instanceconfigframeheader(instance_data_t *inst, int ackrequest)
{
	
	inst->msg.panID[0] = 0xdd;
	inst->msg.panID[1] = 0xdd;


    //set frame type (0-2), SEC (3), Pending (4), ACK (5), PanIDcomp(6)
    inst->msg.frameCtrl[0] = 0x1 /*frame type 0x1 == data*/ | 0x40 /*PID comp*/;
    inst->msg.frameCtrl[0] |= (ackrequest ? 0x20 : 0x00);
#if (USING_64BIT_ADDR==1)
    //source/dest addressing modes and frame version
    inst->msg.frameCtrl[1] = 0xC /*dest extended address (64bits)*/ | 0xC0 /*src extended address (64bits)*/;
#else
    inst->msg.frameCtrl[1] =0x8 /*dest short address (16bits)*/ | 0x80 /*src short address (16bits)*/;
#endif

    inst->msg.seqNum = inst->cur_slot;	
	if(inst->testAppState == TA_TXBLINK_WAIT_SEND)
	{
		inst->msg.destAddr[0] = 0xFF;
		inst->msg.destAddr[1] = 0xFF;
	}
	else
	{
		memcpy(&inst->msg.destAddr,&test_tof_an_addr,ADDR_BYTE_SIZE);
	}
	memcpy(&inst->msg.sourceAddr,&u16ShortAddr,ADDR_BYTE_SIZE);
}



// -------------------------------------------------------------------------------------------------------------------
//
// function to configure the mac frame data, prior to issuing the PD_DATA_REQUEST
//
// -------------------------------------------------------------------------------------------------------------------
//
void setupmacframedata(instance_data_t *inst, int len, int fcode, int ack)
{
    inst->macdata_msdu[FCODE] = fcode; //message function code (specifies if message is a poll, response or other...)

    if(len)
        memcpy(inst->msg.messageData, inst->macdata_msdu, len); //copy application data

    inst->psduLength = len + FRAME_CRTL_AND_ADDRESS + FRAME_CRC;
//	 inst->psduLength = len + 9 + FRAME_CRC;

    //inst->psduLength = adduserpayload(inst, inst->psduLength, len); //add any user data to the message payload

    instanceconfigframeheader(inst, ack); //set up frame header (with/without ack request)

    if(ack == ACK_REQUESTED)
        inst->wait4ack = DWT_RESPONSE_EXPECTED;

    inst->ackexpected = ack ; //used to ignore unexpected ACK frames
}

// -------------------------------------------------------------------------------------------------------------------
//
// Turn on the receiver with/without delay
//
void instancerxon(int delayed, uint64 delayedReceiveTime)
{
    if (delayed)
    {
        uint32 dtime;
        dtime =  (uint32) (delayedReceiveTime>>8);
        dwt_setdelayedtrxtime(dtime) ;
    }

#if (SNIFF_MODE == 1)
    dwt_setrxmode(RX_SNIFF, 0, 0x02, 0xFF); //Off time 0xFF, on time 0x2
#endif

    dwt_rxenable(delayed) ;               // turn receiver on, immediate/delayed

} // end instancerxon()


int instancesendpacket(instance_data_t *inst, int delayedTx)
{
    int result = 0;

    dwt_writetxdata(inst->psduLength, (uint8 *)  &inst->msg, 0) ;   // write the frame data
    dwt_writetxfctrl(inst->psduLength, 0);
    if(delayedTx)
    {
        uint32 dtime;
        dtime = (uint32) (inst->delayedReplyTime>>8);
        dwt_setdelayedtrxtime(dtime) ;
    }

    if(inst->wait4ack)
    {
        //if the ACK is requested there is a 5ms timeout to stop RX if no ACK coming
        dwt_setrxtimeout(5000);  //units are us - wait for 5ms after RX on
    }

    //begin delayed TX of frame
    if (dwt_starttx(delayedTx | inst->wait4ack))  // delayed start was too late
    {
        result = 1; //late/error
    }


    return result;                                              // state changes
    // after sending we should return to TX ON STATE ?
}



void xtalcalibration(void)
{
    int i;
    uint8 chan = 2 ;
    uint8 prf = DWT_PRF_16M ;
    dwt_txconfig_t  configTx ;

    SPI_ConfigFastRate(SPI_BaudRatePrescaler_16); //reduce the SPI speed before putting device into low power mode
    //
    //  reset device
    //
    dwt_softreset();

    //
    //  configure TX channel parameters
    //

    configTx.PGdly = txSpectrumConfig[chan].PGdelay ;
    configTx.power = txSpectrumConfig[chan].txPwr[prf - DWT_PRF_16M];

    dwt_configuretxrf(&configTx);

    dwt_configcwmode(chan);

    for(i=0; i<=0x1F; i++)
    {
        dwt_xtaltrim(i);
        //measure the frequency
        //Spectrum Analyser set:
        //FREQ to be channel default e.g. 3.9936 GHz for channel 2
        //SPAN to 10MHz
        //PEAK SEARCH
    }

    return;
}

void instance_set_alarmlist(uint16 addr,uint8 type,uint8 status)
{
	if(status ==1)  //set alarm
	{
		if(type==0xff)  //all
		{
			instance_set_sta_status(STATION_TYPE_ALARM);   //³·Àë
			instance_reset_sta_status(STATION_TYPE_ALARM_RESET | STATION_TYPE_ALARM_ANY);
		}
		return;
	}
	else if(status ==0)          //reset alarm £¬stop retreat
	{
		if(type==0xff)   //all
		{
			instance_set_sta_status(STATION_TYPE_ALARM_RESET);
			instance_reset_sta_status(STATION_TYPE_ALARM |STATION_TYPE_ALARM_ANY);
		}
	}
}


uint8 instance_inset_alarmmsg(instance_data_t *inst,uint8 start)
{	
	inst->macdata_msdu[start]  = inst->station_status;
	return start;
}


void reset_sub_alarmmsg()
{
	sub_alarmmsg.alarmaddr = 0;
	sub_alarmmsg.alarmstatus =0;
	sub_alarmmsg.excitid =0;
}

// -------------------------------------------------------------------------------------------------------------------
//
// the main instance state machine (all the instance modes Tag, Anchor or Listener use the same statemachine....)
//
// -------------------------------------------------------------------------------------------------------------------
//
/*
TA_TXPOLL_WAIT_SEND,		//2
TA_TXFINAL_WAIT_SEND,		//3
TA_TXRESPONSE_WAIT_SEND,	//4
TA_TXREPORT_WAIT_SEND,		//5
TA_TX_WAIT_CONF,			//6

TA_RXE_WAIT,				//7
TA_RX_WAIT_DATA,			//8

TA_SLEEP,					//9
TA_SLEEP_DONE,				//10
TA_TXBLINK_WAIT_SEND,		//11
TA_TXRANGINGINIT_WAIT_SEND,  //12
TA_TX_RANGING_ACK_SEND		//13
TA_HELP_CALL_SEND , 		  //14
 TA_HELP_RESP_SEND ,		   //15
 TA_EVACUATE_ASK_SEND		  //16

*/



void ta_rxe_wait(instance_data_t *inst)
{

    if(inst->wait4ack == 0) //if this is set the RX will turn on automatically after TX
    {
        uint64 delayedReceiveTime = 0;

       if (inst->shouldDoDelayedRx) //we don't need to turn receiver on immediately, as we know the response will come in a while
        {
            delayedReceiveTime = (inst->txu.txTimeStamp + inst->rxOnDelay) & MASK_40BIT;

            if(inst->previousState == TA_TXBLINK_WAIT_SEND) //check if we need to use long response delay for the blink response
            {
            	if(inst->fixedReplyDelay_ms > FIXED_LONG_REPLY_DELAY)
            	{
            		delayedReceiveTime = (delayedReceiveTime + (DELAY_MULTIPLE*inst->fixedReplyDelay)) & MASK_40BIT;
            	}
            }
        }

        //turn RX on
       // instancerxon(inst->shouldDoDelayedRx, delayedReceiveTime) ;   // turn RX on, with/without delay
        instancerxon(0,0);
    }
    else
    {
        inst->wait4ack = 0 ; //clear the flag, the next time we want to turn the RX on it might not be auto
    }

    inst->shouldDoDelayedRx = FALSE ; //clear the flag

    if (inst->mode != LISTENER)
    {
        if (inst->previousState != TA_TXREPORT_WAIT_SEND) //we are going to use anchor timeout and re-send the report
            inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT; //using RX FWTO
    }

    inst->testAppState = TA_RX_WAIT_DATA;   // let this state handle it

    // end case TA_RXE_WAIT, don't break, but fall through into the TA_RX_WAIT_DATA state to process it immediately.
//    if(message == 0) break;
}

void txretreat_ack_send()//uint8 *help_payload,int len)
{
	uint8 newHelpAskSent=0; 		  //if help fail ,send again
	instance_data_t *inst  =(instance_data_t*) (&instance_data[0]);
	dwt_forcetrxoff() ;
	
	inst->testAppState =TA_RETREAT_ACK_SEND ;
	inst->macdata_msdu[1]=  my_cardslotmsg.status;
	setupmacframedata(inst, 2, RTLS_RETREAT_ACK_SEND, !ACK_REQUESTED);

	inst->previousState = TA_RETREAT_ACK_SEND ;
	if(instancesendpacket(inst, DWT_START_TX_IMMEDIATE))
	{
		EDBG(PrintfUtil_vPrintf("retreat ack fail! \n");)
		newHelpAskSent++;
		if(newHelpAskSent <3)
		{
			mSleep(3);
		}
	}
	else
	{
		DBG(PrintfUtil_vPrintf("retreat ack seccuss! \n");)
		inst->testAppState = TA_TX_WAIT_CONF ;
	//	dwt_setrxtimeout((RX_FWTO_TIME+2) * 1000);  //units are us - wait for 5ms after RX on
     	dwt_setrxtimeout(0); 

	}

}


void txexcit_ack_send(void)
{
	instance_data_t *inst  =(instance_data_t*) (&instance_data[0]);
	dwt_forcetrxoff() ;
	inst->macdata_msdu[1] = my_cardslotmsg.status;
	setupmacframedata(inst, 1, RTLS_EXCIT_ACK_SEND, !ACK_REQUESTED);
	inst->instToSleep=0;
	inst->previousState = TA_EXCIT_ACK_SEND ;
	if(instancesendpacket(inst, DWT_START_TX_IMMEDIATE))
	{
		EDBG(PrintfUtil_vPrintf("txexcit_ack_send fail ");)
		ta_rxe_wait(inst);
	}
	else
	{
		inst->testAppState = TA_TX_WAIT_CONF;	// wait confirmation
		dwt_setrxtimeout((RX_FWTO_TIME) * 1000);  //units are us - wait for 5ms after RX on
   //  dwt_setrxtimeout(0); 

	}
}


void txhelp_resp_send(instance_data_t *inst)
{
	uint8 newHelprespSent=0;
HELPRESPAGAIN:
	dwt_forcetrxoff() ;
	inst->macdata_msdu[1]= sub_alarmmsg.alarmstatus;  ///1:rev but the service counter haven't report ;  2:the service counter have report
	inst->testAppState =TA_HELP_RESP_SEND ;
	setupmacframedata(inst, 2, RTLS_MSG_HELP_RESP, !ACK_REQUESTED);

	inst->instToSleep=0;
	inst->previousState = TA_HELP_RESP_SEND ;
	mSleep(1);
	if(instancesendpacket(inst, DWT_START_TX_IMMEDIATE))
	{
		EDBG(PrintfUtil_vPrintf("help resp fail! \n");)
		newHelprespSent++;
		if(newHelprespSent <3)
		{
			mSleep(3);
			goto HELPRESPAGAIN;
		}
	}
	else
	{
		
		newHelprespSent=0;
     	dwt_setrxtimeout(0); 

	}

}

void txblink_wait_send(instance_data_t *inst)
{
	uint8 temp,len=0;
	dwt_forcetrxoff() ;

	temp =0;  //+2 send process
	new_2blink_tick = temp;
	

	inst->macdata_msdu[1]= BLINK_ALL_RESP; 

	inst->macdata_msdu[2]= 0;     ///2 Slot_have_run_tick

	inst->macdata_msdu[3]= TOF_SLOT_LOC_PERIOD; // =40  for 1s cards 
	inst->macdata_msdu[4]= 200;//MAX_CARD_CYCLE_SEC *TOF_SLOT_LOC_PERIOD  =200    fro 5s cards

	len = instance_inset_alarmmsg(inst,5);  //inset alarm massage
	test_tof_an_addr = 0xffff;

	setupmacframedata(inst, len+1, SIG_RX_BLINK, !ACK_REQUESTED);

	inst->instToSleep=0;
	inst->testAppState = TA_TXBLINK_WAIT_SEND ;  // wait to receive a new poll
	if(instancesendpacket(inst, DWT_START_TX_IMMEDIATE))
	{
		 EDBG(PrintfUtil_vPrintf("sN ");)
		 inst->previousState = TA_TXBLINK_WAIT_SEND ;  // wait to receive a new poll
		 inst->done = INST_NOT_DONE_YET;

	}
	else
	{
		inst->testAppState = TA_TX_WAIT_CONF;//TA_RXE_WAIT;//TA_TX_WAIT_CONF ;	
		inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set below)

		//dwt_setrxtimeout((RX_FWTO_TIME+3) * 1000);  //units are us - wait for 5ms after RX on
	}

}


void blink_to_retreat()
{
//	instance_change_channel(ANCHOR_BLINK_CHANNEL);
	txblink_wait_send(&instance_data[0]);
}

/*
void txpoll_wait_send(instance_data_t *inst)
{
	static uint32 adjustTick = 0 ;
	uint32 temp;
	uint8 tags_txTimeStamp[5] = {0, 0, 0, 0, 0};
	LastPollTick = portGetTickCount();
	TAGSlotCnt = inst->TagSlot; 
	

	if(inst->TxSpeed_Mode == SLOW_Speed)
		inst->macdata_msdu[1] = 0x0;
	else if(inst->TxSpeed_Mode == QUICK_Speed)
		inst->macdata_msdu[1] = 0x1;
	inst->macdata_msdu[2] = (u16ShortAddr>> 8)&0xff;
	inst->macdata_msdu[3] = (u16ShortAddr)&0xff;


	//seqnum
	inst->seqnum++;
	inst->macdata_msdu[4] = (inst->seqnum >> 8)&0xff;
	inst->macdata_msdu[5] = (inst->seqnum)&0xff;


	setupmacframedata(inst, 6, RTLS_DEMO_MSG_TAG_POLL, !ACK_REQUESTED);


	inst->previousState = TA_TXPOLL_WAIT_SEND ;
	if(instancesendpacket(inst, DWT_START_TX_IMMEDIATE))
	{

		 inst->testAppState = TA_TXPOLL_WAIT_SEND ;  // wait to receive a new poll
		 inst->done = INST_NOT_DONE_YET;
		 inst->seqnum--;

	}
	else
	{				
		stateCount = 0;
		inst->testAppState = TA_TXPOLL_WAIT_SEND ;	
		inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set below)

	}
//	dwt_forcetrxoff() ;
	dwt_setleds(2);
}


void send_tdoa_poll()
{
	txpoll_wait_send(&instance_data[0]);
}



void init_tag(instance_data_t *inst)
{
	PrintfUtil_vPrintf("********** TAG *************** \n");
	isnewblinkon=1;

	my_alarmlist.count = 0;
	
	dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN); //allow data, ack frames;
	dwt_setpanid(inst->panid);
//	dwt_seteui(inst->eui64);
#if (USING_64BIT_ADDR==0)
	dwt_setaddress16(u16ShortAddr);
	PrintfUtil_vPrintf("	 ADDR = %d \n",u16ShortAddr);
#endif
	inst->shortaddr16 = u16ShortAddr;
	//set source address into the message structure
	memcpy(&inst->msg.sourceAddr, &u16ShortAddr, ADDR_BYTE_SIZE);
	//change to next state - send a Poll message to 1st anchor in the list
#if (DR_DISCOVERY == 1)
	//inst->mode = TAG_TDOA ;
	inst->testAppState = TA_TXBLINK_WAIT_SEND;
//	memcpy(inst->blinkmsg.tagID, inst->eui64, ADDR_BYTE_SIZE);
#else
	inst->testAppState = TA_TXPOLL_WAIT_SEND;
#endif

	dwt_setautorxreenable(inst->rxautoreenable); //not necessary to auto RX re-enable as the receiver is on for a short time (Tag knows when the response is coming)
#if (DOUBLE_RX_BUFFER == 1)
	dwt_setdblrxbuffmode(0); //disable double RX buffer
#endif
#if (ENABLE_AUTO_ACK == 1)
	dwt_enableautoack(ACK_RESPONSE_TIME); //wait for 5 symbols before replying with the ACK
#endif



}
*/

void init_anchor(instance_data_t *inst)
{
//	if(inst->mode == ANCHOR)

	{
		inst->panid = 0xFFFF ;//;0xdeca  0xdddd
		DBG(PrintfUtil_vPrintf("SUB STA	 ADDR = %d \n",u16ShortAddr);)
	//	instance_clear_substa();
		sub_alarmmsg.alarmstatus = 0x0;
		sub_alarmmsg.alarmaddr = 0xff;
	}

	inst->shortaddr16 = u16ShortAddr;
#if (DR_DISCOVERY == 0)
	uint8 eui64[8] ;
	memcpy(eui64, &inst->payload.anchorAddress, sizeof(uint64));

	dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN); //allow data, ack frames;
	dwt_seteui(eui64);
#else
//	dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN ); //allow data, ack frames;
//	dwt_enableframefilter(DWT_FF_NOTYPE_EN); //allow data, ack frames;
dwt_setpanid(inst->panid);

	//dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN|DWT_FF_COORD_EN|DWT_FF_MAC_EN); //allow data, ack frames; DWT_FF_MAC_EN
	

#endif
	
	


#if (USING_64BIT_ADDR==0)
	{
		dwt_setaddress16(u16ShortAddr);
	}
#endif


#if (DR_DISCOVERY == 0)
	//set source address into the message structure
	memcpy(&inst->msg.sourceAddr, &inst->payload.anchorAddress, ADDR_BYTE_SIZE);
#else
	//set source address into the message structure
	memcpy(&inst->msg.sourceAddr, &u16ShortAddr, ADDR_BYTE_SIZE);
#endif
	// First time anchor listens we don't do a delayed RX
	inst->shouldDoDelayedRx = FALSE ;
	//change to next state - wait to receive a message
	//inst->testAppState = TA_RXE_WAIT ;
	
#if (ENABLE_AUTO_ACK == 1)
	dwt_setrxaftertxdelay(WAIT_FOR_RESPONSE_DLY); //set the RX after TX delay time
#endif

#if (DECA_BADF_ACCUMULATOR == 0) //can use RX auto re-enable when not logging/plotting errored frames
	inst->rxautoreenable = 1;
#endif
	dwt_setautorxreenable(inst->rxautoreenable);
#if (DOUBLE_RX_BUFFER == 1)
	dwt_setdblrxbuffmode(0); //enable double RX buffer
#endif
	dwt_setrxtimeout(0);

	ta_rxe_wait(inst);

}

//TA_TX_WAIT_CONF
void ta_tx_wait_conf(instance_data_t *inst,int message)
{
//  instancerxon(0, 0);
	if(message == DWT_SIG_RX_TIMEOUT) //got RX timeout - i.e. did not get the response (e.g. ACK)
	{
		//printf("RX timeout in TA_TX_WAIT_CONF (%d)\n", inst->previousState);
		EDBG(PrintfUtil_vPrintf("-WOT ");)
		//if we got no ACKs after sending MAX_NUMBER_OF_REPORT_RETRYS reports go to wait for RX state
		if(inst->newReportSent && (inst->newReportSent >= MAX_NUMBER_OF_REPORT_RETRYS))
		{
			//don't change state - next event will the TX confirm from the sent report, so will just fall through to RX
			inst->shouldDoDelayedRx = FALSE ;
			inst->wait4ack = 0 ; //clear the flag as the ACK has been received
			dwt_setrxtimeout(0);
			if(inst->mode == ANCHOR&& inst->previousState == TA_TXREPORT_WAIT_SEND){
				//inst->testAppState = TA_RXE_WAIT;
				ta_rxe_wait(inst);
			}
		}
		else
		{
			//got timeout before TX confirm
			inst->testAppState = TA_RX_WAIT_DATA ;//TA_TXE_WAIT;
			DBG(PrintfUtil_vPrintf("(1)");)
		//	inst->nextState = inst->previousState ; // send poll / response / final / report (with ACK request)
		}
		return;
	}


	if(inst->previousState==TA_TXFINAL_WAIT_SEND)
	{
		uint32 temp =portGetTickCount();//LastPollTick = portGetTickCount();
//		PrintfUtil_vPrintf("poll_final_Y=%d ",temp-LastPollTick);
		if(inst->mode == ANCHOR)
			inst->newrange = 1;
	}
	else if(inst->previousState==TA_HELP_RESP_SEND)
	{
		DBG(PrintfUtil_vPrintf("help resp seccuss! \n");)
	}
	else if(inst->previousState==TA_TXPOLL_WAIT_SEND&&inst->mode == ANCHOR)
	{
		uint32 temp =portGetTickCount();//LastPollTick = portGetTickCount();
//		PrintfUtil_vPrintf("poll_Y=%d  addr =%x ",temp-LastPollTick,inst->rev_shortaddr16);
		
	}	
	ta_rxe_wait(inst);

}

void rx_power_lever(instance_data_t *inst)
{
	double rx_lever=0;
	dwt_readdignostics(&inst->devicelogdata.diag);
	double CIR = (double)inst->devicelogdata.diag.maxGrowthCIR;
	double NPC = (double)inst->devicelogdata.diag.rxPreamCount;
	
	rx_lever = 10 * log10((CIR*131072)/(NPC*NPC))- 115.72  ;//121.74 -----64MHz  pow(2, 17)

	inst->i8rssi = (int8)rx_lever;
	//DBG(PrintfUtil_vPrintf("rssi= %i ",inst->i8rssi);)
}

int16 rx_power_lever1(instance_data_t *inst)
{
 	double PathAmp1=0,PathAmp2=0,PathAmp3=0,rx_lever=0;
 	int16 result=0;
	dwt_readdignostics(&inst->devicelogdata.diag);
	PathAmp1 = (double)inst->devicelogdata.diag.firstPathAmp1;
	PathAmp2 = (double)inst->devicelogdata.diag.firstPathAmp2;
	PathAmp3 = (double)inst->devicelogdata.diag.firstPathAmp3;
	double NPC = (double)inst->devicelogdata.diag.rxPreamCount;
	rx_lever = 10 * log10((PathAmp1*PathAmp1+PathAmp2*PathAmp2+PathAmp3*PathAmp3 )/(NPC*NPC))- 115.72  ;//121.74 -----64MHz

	result = (int16)rx_lever;
	//DBG(PrintfUtil_vPrintf("rssi= -%i  -%i  \n",result,result+115);)
	return result;
}
void inset_rev_cardmsg(uint16 cardid,uint16 seq, uint8 status, uint8 devtype,int8 rssi)
{
	app_uwb_rssi_data.rssi_data[u8rssiNum].u16ShortAddr = cardid;
	app_uwb_rssi_data.rssi_data[u8rssiNum].u16SeqNum = seq;
	app_uwb_rssi_data.rssi_data[u8rssiNum].u8Status = status ;//rxmsg->messageData[1];
	app_uwb_rssi_data.rssi_data[u8rssiNum].u8DevType = devtype ;//rxmsg->messageData[2];
	app_uwb_rssi_data.rssi_data[u8rssiNum].i8Rssi = rssi;
	u8rssiNum++;
	if(u8rssiNum >= APP_UWB_MAX_CARD_NUM)
	{
		event_timer_unset(EVENT_REPORT_RSSI);
		event_timer_set(EVENT_REPORT_RSSI);
	}

}

void inset_rev_cardmsg_ext(void)
{
	uint8 i=0,sum=0;
	sum = Car_revcardlist.u8CardCnt;
	for(i=0;i<sum;i++)
	{
		memcpy(&app_uwb_rssi_data.rssi_data[u8rssiNum].u16ShortAddr, &Car_revcardlist.cardmsg[i].u8cardaddr[0],2);
		app_uwb_rssi_data.rssi_data[u8rssiNum].u16SeqNum = 0;
		app_uwb_rssi_data.rssi_data[u8rssiNum].u8Status = Car_revcardlist.cardmsg[i].status;//rxmsg->messageData[1];
		app_uwb_rssi_data.rssi_data[u8rssiNum].u8DevType = Car_revcardlist.cardmsg[i].devtype;//rxmsg->messageData[2];
		app_uwb_rssi_data.rssi_data[u8rssiNum].i8Rssi = rev_rssi;
		u8rssiNum++;
	}
}

int testapprun(instance_data_t *inst, int message)//new_slot_msg_t *slotmsg)
{
//	printf_event(inst,message);

    switch (inst->testAppState)
    {

        case TA_RX_WAIT_DATA :                 // Wait RX data

            switch (message)
            {

                case DWT_SIG_RX_OKAY :
                {
                    srd_msg *rxmsg = &inst->rxmsg;
                    uint16  srcAddr,destAddr;
					uint8 rev_curslot;
					int fcode = 0;

                    // 16 or 64 bit addresses
                    memcpy(&srcAddr, &(rxmsg->sourceAddr), ADDR_BYTE_SIZE);
					memcpy(&destAddr, &(rxmsg->destAddr), ADDR_BYTE_SIZE);
					fcode = rxmsg->messageData[FCODE];
					//rev_curslot = rxmsg->seqNum;
					rev_curslot = (rxmsg->messageData[4]<<8)+rxmsg->messageData[5];
					inst->relpyAddress = srcAddr; //remember who to send the reply to
					test_tof_an_addr = srcAddr;
					rx_power_lever(inst);
					DBG(PrintfUtil_vPrintf("---------recv = %x  |%d \n",fcode,srcAddr );)

					led_station_flash();
					switch(fcode)
					{
						//case 0x10:
						case RTLS_DEMO_MSG_RNG_INIT://RTLS_DEMO_MSG_ANCH_RESP:
						{
							//memcpy(&seq,&rxmsg->messageData[RES_R1+5],sizeof(uint16));
							//PrintfUtil_vPrintf("---------recv RNG_INIT = %d  \n",srcAddr );
							
							inset_rev_cardmsg(srcAddr,0, rxmsg->messageData[1], rxmsg->messageData[2],inst->i8rssi);
						}
						ta_rxe_wait(inst);
						break;
						case RTLS_TDOA_BLINK_SEND:
						{
							inst->seqnum = rxmsg->messageData[4] << 8;        //get the seqnum
							inst->seqnum += rxmsg->messageData[5];
                            memcpy(&inst->rev_shortaddr16,&srcAddr,ADDR_BYTE_SIZE);
                            DBG(PrintfUtil_vPrintf("recv rssi %d\n",srcAddr);)
                            inset_rev_cardmsg(srcAddr,inst->seqnum, rxmsg->messageData[2], rxmsg->messageData[3],inst->i8rssi);
							inst->testAppState = TA_RXE_WAIT;	// let this state handle it
							if(rxmsg->messageData[6]==0xFF && rxmsg->messageData[7]==0xFF)
							{
								ta_rxe_wait(inst);
							}
							else
							{
								uint16 battery,oad_ver;
								memcpy(&battery,&rxmsg->messageData[6],sizeof(battery));
								memcpy(&oad_ver,&rxmsg->messageData[8],sizeof(oad_ver));

								vProcessCardVersion(inst->rev_shortaddr16,oad_ver,battery);
								ta_rxe_wait(inst);
							}
						}
						break;
						case RTLS_MSG_HELP_CALL:
							memcpy(&inst->rev_shortaddr16,&srcAddr,ADDR_BYTE_SIZE);
							if(sub_alarmmsg.alarmaddr ==inst->rev_shortaddr16 )
							{
								if(!(sub_alarmmsg.alarmstatus & UWB_CARD_STATUS_HELP) )  //reset ,0xff is  the first time 
								{
									reset_sub_alarmmsg();
								}
								else
								{
									//uart send to Stm32
									event_timer_set(EVENT_UART_SEND); 
									instance_set_sta_status(STATION_TYPE_ALARM);
									sub_alarmmsg.alarmstatus = rxmsg->messageData[1] ;
									ta_rxe_wait(inst);
								}
							}
							else
							{
								event_timer_set(EVENT_UART_SEND); 
								instance_set_sta_status(STATION_TYPE_ALARM);
								sub_alarmmsg.alarmaddr = inst->rev_shortaddr16;
								sub_alarmmsg.alarmstatus = rxmsg->messageData[1] ;
							}
							txhelp_resp_send(inst);
						break;
						case RTLS_RETREAT_ACK_SEND:
							event_timer_set(EVENT_UART_SEND);
							rev_retreat_ack =1;
							sub_alarmmsg.alarmaddr = inst->rev_shortaddr16;
							sub_alarmmsg.alarmstatus = rxmsg->messageData[1] ;
							break;
                  		case RTLS_EXCIT_ASK_SEND:
							DBG(PrintfUtil_vPrintf("******RTLS_EXCIT_ASK_SEND  = %d  \n ",sub_alarmmsg.alarmstatus);)
							inst->rev_shortaddr16 = srcAddr;
							if(sub_alarmmsg.alarmaddr == srcAddr )  //reset ,0xff is  the first time 
							{
								if(!(sub_alarmmsg.alarmstatus & UWB_CARD_STATUS_IMPEL))
								{
									reset_sub_alarmmsg();
								}
								else
								{
									event_timer_set(EVENT_UART_SEND); 
									instance_set_sta_status(STATION_TYPE_EXCIT);
									sub_alarmmsg.alarmaddr = srcAddr ;
									sub_alarmmsg.alarmstatus = rxmsg->messageData[1] ;
									sub_alarmmsg.excitid = rxmsg->messageData[2]; 
								}
							}
							else
							{
								//uart send to Stm32
								event_timer_set(EVENT_UART_SEND); 
								instance_set_sta_status(STATION_TYPE_EXCIT);
								sub_alarmmsg.alarmaddr = srcAddr ;
								sub_alarmmsg.alarmstatus = rxmsg->messageData[1] ;
								sub_alarmmsg.excitid = rxmsg->messageData[2]; 
							}
							txexcit_ack_send();
						break;
						case RTLS_CAR_REVTOFCARD_MSG:
						{
							uint8 sum=0,i=0,j=0,status=0,devtype=0;
							uint16 cardid;
							sum = rxmsg->messageData[1];
							//count , m_distance(2)/ s_distance(2) / <cardid1, status, dev> (4)/ ......<cardidn, status, dev>(4)/ 
							if(sum)
							{
								//memcpy(&Car_revcardlist.m_distance,&rxmsg->messageData[2],2);
								//memcpy(&Car_revcardlist.s_distance,&rxmsg->messageData[4],2);
								//inset_rev_cardmsg(srcAddr,0, status,devtype,inst->i8rssi);
								j=Car_revcardlist.u8CardCnt;
								for(i=0;i<sum;i++)
								{
									if(u8rssiNum < APP_UWB_MAX_CARD_NUM)
									{
										memcpy(&cardid,&rxmsg->messageData[6+4*i],2);
										status = rxmsg->messageData[8+4*i];
										devtype = rxmsg->messageData[9+4*i];
										inset_rev_cardmsg(cardid,0, status,devtype,inst->i8rssi);
										//PrintfUtil_vPrintf("-id= <%d |%d> ",cardid,devtype);
									}
									else
									{
										if(j<=MAX_CAR_CARD_CNT)
										{
											memcpy(&Car_revcardlist.cardmsg[j].u8cardaddr[0],&rxmsg->messageData[6+4*i],ADDR_BYTE_SIZE);
											status = rxmsg->messageData[8+4*i];
											devtype = rxmsg->messageData[9+4*i];
											Car_revcardlist.cardmsg[j].status = status;
											Car_revcardlist.cardmsg[j].devtype = devtype;
											//PrintfUtil_vPrintf("-id= <%d |%d> ",cardid ,devtype);
											j++;
										}
										instance_set_car_rev();
									}
								}
								//PrintfUtil_vPrintf("\n ");
								Car_revcardlist.u8CardCnt =j;
								rev_rssi = inst->i8rssi;
							}
							
							ta_rxe_wait(inst);
						}	
						break;


                        default:
                        {

                            inst->shouldDoDelayedRx = FALSE ;               // no delay turning on RX
                            
							ta_rxe_wait(inst);

                        }
                        break;
                    } //end switch (rxmsg->functionCode)


                    if((inst->instToSleep == 0) && (inst->mode == LISTENER) /*|| (inst->mode == ANCHOR)*/)//update received data, and go back to receiving frames
                    {

                        inst->shouldDoDelayedRx = FALSE ;               // no delay turning on RX
                        
						ta_rxe_wait(inst);
                    }

                }
                break ;

                case DWT_SIG_RX_TIMEOUT :
                {
				//	PrintfUtil_vPrintf("OT ");
                    inst->rxTimeouts ++ ;
                    inst->done = INST_NOT_DONE_YET;

                    if(inst->mode == ANCHOR || inst->mode == SUB_STA) //we did not receive the final - wait for next poll
                    {
                     	if(inst->mode == ANCHOR && inst->previousState == TA_TXPOLL_WAIT_SEND)//&&inst->rxTimeouts <=MAX_NUMBER_OF_POLL_RETRYS)
                 		{
                 			inst->testAppState = TA_TXPOLL_WAIT_SEND ;
								
                 		}
						else
                        {
                        	inst->rxTimeouts=0;
							//  dwt_forcetrxoff() ;
                            //inst->testAppState = TA_RXE_WAIT ;
                            dwt_setrxtimeout(0);
                            inst->ackexpected = 0 ; //timeout... so no acks expected anymore
                          
							ta_rxe_wait(inst);
							break;
                        }
                    }
                    message = 0; //clear the message as we have processed the event

                    //timeout - disable the radio (if using SW timeout the rx will not be off)
           //         dwt_forcetrxoff() ;
                }
                break ;

                default :
                {
                    //printf("\nERROR - Unexpected message %d ??\n", message) ;
                    //assert(0) ;                                             // Unexpected Primitive what is going on ?
                }
                break ;

            }
            break ; // end case TA_RX_WAIT_DATA

            default:
                //printf("\nERROR - invalid state %d - what is going on??\n", inst->testAppState) ;
            break;
    } // end switch on testAppState

    return inst->done;
} // end testapprun()

// -------------------------------------------------------------------------------------------------------------------
#if NUM_INST != 1
#error These functions assume one instance only
#else

// -------------------------------------------------------------------------------------------------------------------
// Set this instance role as the Tag, Anchor or Listener
void instancesetrole(int inst_mode)
{
    // assume instance 0, for this
    instance_data[0].mode =  (uint8)inst_mode;                   // set the role
}

uint8 instancegetrole(void)
{
    return instance_data[0].mode;
}

int instancenewrange(void)
{
    if(instance_data[0].newrange)
    {
        instance_data[0].newrange = 0;
        return 1;
    }

    return 0;
}
int instancenewCar(void)
{
    if(instance_data[0].car_revcard)
    {
        instance_data[0].car_revcard = 0;
        return 1;
    }

    return 0;
}
// have been rev the car's cards msg list
void instance_set_car_rev(void)
{
	instance_data[0].car_revcard = 1;
}


void instancesettxtype(int speedtype)
{
	instance_data[0].TxSpeed_Mode = speedtype;
}

// -------------------------------------------------------------------------------------------------------------------
// function to clear counts/averages/range values
//
void instanceclearcounts(void)
{
    int instance = 0 ;

    instance_data[instance].rxTimeouts = 0 ;

    instance_data[instance].frame_sn = 0;
    instance_data[instance].longTermRangeSum  = 0;
    instance_data[instance].longTermRangeCount  = 0;

 //   instance_data[instance].idistmax = 0;
 //   instance_data[instance].idistmin = 1000;

    dwt_configeventcounters(1); //enable and clear

    instance_data[instance].frame_sn = 0;
 //   instance_data[instance].lastReportSN = 0xff;

    instance_data[instance].tofcount = 0 ;
    instance_data[instance].tofindex = 0 ;

#if (DEEP_SLEEP == 1)
    instance_data[instance].txmsgcount = 0;
    instance_data[instance].rxmsgcount = 0;
#endif
} // end instanceclearcounts()


// -------------------------------------------------------------------------------------------------------------------
// function to initialise instance structures
//
// Returns 0 on success and -1 on error
#if DECA_SUPPORT_SOUNDING==1
int instance_init(accBuff_t *buf)
#else
int instance_init(void)
#endif
{
    int instance = 0 ;
    int result;
    //uint16 temp = 0;

    instance_data[instance].shouldDoDelayedRx = FALSE ;

//    instance_data[instance].mode = LISTENER ;                                // assume listener,
    instance_data[instance].testAppState = TA_INIT ;

    instance_data[instance].anchorListIndex = 0 ;
    instance_data[instance].instToSleep = 0;

//    instance_data[instance].sentSN = 0;
//    instance_data[instance].ackdSN = 0;

    instance_data[instance].tofindex = 0;
    instance_data[instance].tofcount = 0;
//    instance_data[instance].last_update = -1 ;           // detect changes to status report

    // Reset the IC (might be needed if not getting here from POWER ON)
    // ARM code: Remove soft reset here as using hard reset in the inittestapplication() in the main.c file
    dwt_softreset();

#if (DEEP_SLEEP_AUTOWAKEUP == 1)
#if 1
    {
        double t;
        instance_data[instance].lp_osc_cal = dwt_calibratesleepcnt(); //calibrate low power oscillator
        //the lp_osc_cal value is number of XTAL/2 cycles in one cycle of LP OSC
        //to convert into seconds (38.4MHz/2 = 19.2MHz (XTAL/2) => 1/19.2MHz ns)
        //so to get a sleep time of 5s we need to program 5 / period and then >> 12 as the register holds upper 16-bits of 28-bit counter
        t = ((double)5/((double)instance_data[instance].lp_osc_cal/19.2e6));
        instance_data[instance].blinktime = (int) t;
        instance_data[instance].blinktime >>= 12;

        dwt_configuresleepcnt(instance_data[instance].blinktime);//configure sleep time

    }
#else
    instance_data[instance].blinktime = 0xf;
#endif
#endif

    //we can enable any configuration loding from NVM/ROM on initialisation
    result = dwt_initialise(DWT_LOADUCODE | DWT_LOADTXCONFIG | DWT_LOADANTDLY| DWT_LOADXTALTRIM) ;

    //temp = dwt_readtempvbat();
    //printf("Vbat = %d (0x%02x) \tVtemp = %d  (0x%02x)\n",temp&0xFF,temp&0xFF,(temp>>8)&0xFF,(temp>>8)&0xFF);

    // if using auto CRC check (DWT_INT_RFCG and DWT_INT_RFCE) are used instead of DWT_INT_RDFR flag
    // other errors which need to be checked (as they disable receiver) are
//    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);
	 dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | ( DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);
    //this is platform dependant - only program if DW EVK/EVB
    dwt_setleds(2) ; //configure the GPIOs which control the leds on EVBs

    //this is set though the instance_data[instance].configData.smartPowerEn in the instance_config function
    //dwt_setsmarttxpower(0); //disable smart TX power


    if (DWT_SUCCESS != result)
    {
        return (-1) ;   // device initialise has failed
    }

    dwt_setcallbacks(instance_txcallback, instance_rxcallback);

    instanceclearcounts() ;

    instance_data[instance].panid = 0xdeca ;

#if (DR_DISCOVERY == 1)
    instance_data[instance].sendTOFR2Tag = 1;
#else
    instance_data[instance].sendTOFR2Tag = 0;
#endif

    instance_data[instance].tag2rxReport = 0; //Tag is not expecting the report

    instance_data[instance].fixedReportDelay = (convertmicrosectodevicetimeu (FIXED_REPORT_DELAY * 1000.0) & MASK_TXDTS);
    instance_data[instance].fixedReplyDelay = (convertmicrosectodevicetimeu (FIXED_REPLY_DELAY * 1000.0) & MASK_TXDTS);
    instance_data[instance].fixedReplyDelay_ms = FIXED_REPLY_DELAY ;
    instance_data[instance].rxOnDelay = convertmicrosectodevicetimeu ((FIXED_REPLY_DELAY - RX_ON_TIME) * 1000.0);
	instance_data[instance].tagBlinkSleepTime_us = BLINK_SLEEP_DELAY*1000 ;

    instance_data[instance].newReportSent = 0; //clear the flag
    instance_data[instance].wait4ack = 0;
    instance_data[instance].ackexpected = 0;
    instance_data[instance].stoptimer = 0;
    instance_data[instance].instancetimer_en = 0;

	instance_data[instance].slottimer = portGetTickCount();

    instance_data[instance].dwevent[0] = 0;
    instance_data[instance].dwevent[1] = 0;
    instance_data[instance].dweventCnt = 0;

    instance_data[instance].anchReportTimeout_us = 10*1000 ; //10 ms

    instance_data[instance].rxautoreenable = 0;

    //sample test calibration functions
    //xtalcalibration();
    //powertest();
    instance_data[instance].seqnum = 0;
	instance_data[instance].is_intoflist = 0;
	instance_data[instance].slotlist_full = FALSE;
	instance_data[instance].up_revrpoll_time =0;
	instance_data[instance].have_rev_pollOblink =0;
	instance_data[instance].change_devtype =0 ;
	
	dwt_geteui(instance_data[instance].eui64);

	u16ShortAddr = instance_get_cardid();
	
	init_anchor(&instance_data[instance]);


    return 0 ;
}




// -------------------------------------------------------------------------------------------------------------------
//
// Return the Device ID register value, enables higher level validation of physical device presence
//

uint32 instancereaddeviceid(void)
{
    return dwt_readdevid() ;
}


// -------------------------------------------------------------------------------------------------------------------
//
// function to allow application configuration be passed into instance and affect underlying device opetation
//
void instance_config(instanceConfig_t *config)
{
    int instance = 0 ;
    int use_nvmdata = DWT_LOADANTDLY | DWT_LOADXTALTRIM;
    uint32 power = 0;

    instance_data[instance].configData.chan = ANCHOR_BLINK_CHANNEL;//config->channelNumber ;
    instance_data[instance].configData.rxCode =  config->preambleCode ;
    instance_data[instance].configData.txCode = config->preambleCode ;
    instance_data[instance].configData.prf = config->pulseRepFreq ;
    instance_data[instance].configData.dataRate = config->dataRate ;
    instance_data[instance].configData.txPreambLength = config->preambleLen ;
    instance_data[instance].configData.rxPAC = config->pacSize ;
    instance_data[instance].configData.nsSFD = config->nsSFD ;
    instance_data[instance].configData.phrMode = DWT_PHRMODE_STD ;
    instance_data[instance].configData.sfdTO = DWT_SFDTOC_DEF; //default value

    instance_data[instance].configData.smartPowerEn = 0;

    //configure the channel parameters
    dwt_configure(&instance_data[instance].configData, use_nvmdata) ;

    instance_data[instance].configTX.PGdly = txSpectrumConfig[config->channelNumber].PGdelay ;

	//firstly check if there are calibrated TX power value in the DW1000 OTP
	power = dwt_getotptxpower(config->pulseRepFreq, instance_data[instance].configData.chan);

	if((power == 0x0) || (power == 0xFFFFFFFF)) //if there are no calibrated values... need to use defaults
    {
        power = txSpectrumConfig[config->channelNumber].txPwr[config->pulseRepFreq- DWT_PRF_16M];
    }

    //Configure TX power
	//if smart power is used then the value as read from NVM is used directly
	//if smart power is used the user needs to make sure to transmit only one frame per 1ms or TX spectrum power will be violated
    if(instance_data[instance].configData.smartPowerEn == 1)
    {
        instance_data[instance].configTX.power = power;
    }
	else //if the smart power if not used, then the low byte value (repeated) is used for the whole TX power register
    {
        uint8 pow = power & 0xFF ;
        instance_data[instance].configTX.power = (pow | (pow << 8) | (pow << 16) | (pow << 24));
    }
	instance_data[instance].configTX.power = 0x751F1F75;
	dwt_setsmarttxpower(instance_data[instance].configData.smartPowerEn);

	//configure the tx spectrum parameters (power and PG delay)
    dwt_configuretxrf(&instance_data[instance].configTX);

	//check if to use the antenna delay calibration values as read from the NVM
    if((use_nvmdata & DWT_LOADANTDLY) == 0)
    {
        instance_data[instance].txantennaDelay = rfDelays[config->pulseRepFreq - DWT_PRF_16M];
        // -------------------------------------------------------------------------------------------------------------------
        // set the antenna delay, we assume that the RX is the same as TX.
        dwt_setrxantennadelay(instance_data[instance].txantennaDelay);
        dwt_settxantennadelay(instance_data[instance].txantennaDelay);
    }
    else
    {
        //get the antenna delay that was read from the OTP calibration area
        instance_data[instance].txantennaDelay = dwt_readantennadelay(config->pulseRepFreq) >> 1;

        // if nothing was actually programmed then set a reasonable value anyway
		if (instance_data[instance].txantennaDelay == 0)
		{
			instance_data[instance].txantennaDelay = rfDelays[config->pulseRepFreq - DWT_PRF_16M];
			// -------------------------------------------------------------------------------------------------------------------
			// set the antenna delay, we assume that the RX is the same as TX.
			dwt_setrxantennadelay(instance_data[instance].txantennaDelay);
			dwt_settxantennadelay(instance_data[instance].txantennaDelay);
		}


    }

	tx_antennaDelay = instance_data[instance].txantennaDelay;

}


void instance_change_channel(uint8 channel)
{
	 int instance = 0 ;
	 int use_nvmdata = DWT_LOADANTDLY | DWT_LOADXTALTRIM;
    uint32 power = 0;
	
    instance_data[instance].configData.chan = channel ;

    //configure the channel parameters
    dwt_configure(&instance_data[instance].configData, use_nvmdata) ;

    instance_data[instance].configTX.PGdly = txSpectrumConfig[channel].PGdelay ;

	//firstly check if there are calibrated TX power value in the DW1000 OTP
	power = dwt_getotptxpower(DWT_PRF_16M, instance_data[instance].configData.chan);

	if((power == 0x0) || (power == 0xFFFFFFFF)) //if there are no calibrated values... need to use defaults
    {
        power = txSpectrumConfig[channel].txPwr[DWT_PRF_16M- DWT_PRF_16M];
    }

    //Configure TX power
	//if smart power is used then the value as read from NVM is used directly
	//if smart power is used the user needs to make sure to transmit only one frame per 1ms or TX spectrum power will be violated
    if(instance_data[instance].configData.smartPowerEn == 1)
    {
        instance_data[instance].configTX.power = power;
    }
	else //if the smart power if not used, then the low byte value (repeated) is used for the whole TX power register
    {
        uint8 pow = power & 0xFF ;
        instance_data[instance].configTX.power = (pow | (pow << 8) | (pow << 16) | (pow << 24));
    }
	dwt_setsmarttxpower(instance_data[instance].configData.smartPowerEn);
}

// -------------------------------------------------------------------------------------------------------------------
// function to set the tag sleep time (in ms)
//
void instancesettagsleepdelay(uint32 sleepdelay) //sleep in ms
{
    int instance = 0 ;
    instance_data[instance].tagSleepTime_us = sleepdelay*((uint32)1000) ;
}

// -------------------------------------------------------------------------------------------------------------------
// function to set the fixed blink reply delay time (in ms)
//
void instancesetblinkreplydelay(double delayms) //delay in ms
{
	int instance = 0 ;
	instance_data[instance].fixedReplyDelay_ms = delayms ;
}
// -------------------------------------------------------------------------------------------------------------------
// function to set the fixed reply delay time (in ms)
//
void instancesetreplydelay(double delayms) //delay in ms
{
    int instance = 0 ;
    instance_data[instance].fixedReplyDelay = convertmicrosectodevicetimeu (delayms * 1e3) ;
    instance_data[instance].fixedReplyDelay_ms = delayms ;
    instance_data[instance].rxOnDelay = convertmicrosectodevicetimeu ((delayms - RX_ON_TIME) * 1e3);
    //printf("Set response delay time to %d ms.\n", (int) delayms);
}

// -------------------------------------------------------------------------------------------------------------------
// function to configure anchor instance whether to send TOF reports to Tag
//
void instancesetreporting(int anchorSendsTofReports)
{
    int instance = 0 ;
    instance_data[instance].sendTOFR2Tag = anchorSendsTofReports ;        // Set whether TOF reports are sent
    //Listener will only listen for reports when this is set (all other frames are filtered out)
    //instance_data[instance].listen4Reports = anchorSendsTofReports ;
}

#if (DR_DISCOVERY == 0)
// -------------------------------------------------------------------------------------------------------------------
//
// Set Payload parameters for the instance
//
// -------------------------------------------------------------------------------------------------------------------
void instancesetaddresses(instanceAddressConfig_t *plconfig)
{
    int instance = 0 ;

    instance_data[instance].payload = *plconfig ;       // copy configurations

    if(instance_data[instance].payload.sendReport == 1)
        instance_data[instance].sendTOFR2Tag = 1;
    else
        instance_data[instance].sendTOFR2Tag = 0;
}
#endif

// -------------------------------------------------------------------------------------------------------------------
//
// Access for low level debug
//
// -------------------------------------------------------------------------------------------------------------------


void instance_close(void)
{
#ifdef _MSC_VER
    uint8 buffer[1500];
    // close/tidy up any device functionality - i.e. wake it up if in sleep mode
    if(dwt_spicswakeup(buffer, 1500) == DWT_ERROR)
    {
        //printf("FAILED to WAKE UP\n");
    }
#else
    //wake up device from low power mode
    //NOTE - in the ARM  code just drop chip select for 200us
    port_SPIx_clear_chip_select();  //CS low
    mSleep(1);   //200 us to wake up then waits 5ms for DW1000 XTAL to stabilise
    port_SPIx_set_chip_select();  //CS high
    mSleep(5);
#endif

    dwt_entersleepaftertx(0); // clear the "enter deep sleep after tx" bit

    dwt_setinterrupt(0xFFFFFFFF, 0); //don't allow any interrupts

}


void instance_txcallback(const dwt_callback_data_t *txd)
{
    int instance = 0;
    uint8 txTimeStamp[5] = {0, 0, 0, 0, 0};
    uint32 temp = 0;
    uint8 txevent = txd->event;
	instance_set_status(1);
    if(instance_data[instance].ackreq) //the ACK has been requested in the last RX frame - we got TX event, this means the ACK has been sent
    {
        txevent = DWT_SIG_TX_AA_DONE;
        instance_data[instance].ackreq = 0;
    }

    if(txevent == DWT_SIG_TX_DONE)
    {
        //uint64 txtimestamp = 0;

        if(instance_data[instance].dweventCnt < 2) //if no outstanding event to process
        {
            //NOTE - we can only get TX good (done) while here
            //dwt_readtxtimestamp((uint8*) &instance_data[instance].txu.txTimeStamp);

            dwt_readtxtimestamp(txTimeStamp) ;
            temp = txTimeStamp[0] + (txTimeStamp[1] << 8) + (txTimeStamp[2] << 16) + (txTimeStamp[3] << 24);
            instance_data[instance].txu.txTimeStamp = txTimeStamp[4];
            instance_data[instance].txu.txTimeStamp <<= 32;
            instance_data[instance].txu.txTimeStamp += temp;

            instance_data[instance].stoptimer = 0;

      //      instance_data[instance].dwevent[instance_data[instance].dweventCnt++] = DWT_SIG_TX_DONE ;
        }


#if (DEEP_SLEEP == 1)
        instance_data[instance].txmsgcount++;
#endif


		ta_tx_wait_conf(&instance_data[instance],DWT_SIG_TX_DONE);
    }
    else if(txevent == DWT_SIG_TX_AA_DONE)
    {
        if(instance_data[instance].dweventCnt < 2) //if no outstanding event to process
		{
			//auto ACK confirmation
			instance_data[instance].dwevent[instance_data[instance].dweventCnt++] = DWT_SIG_TX_AA_DONE ;
		}
	}
}


void instance_rxcallback(const dwt_callback_data_t *rxd)
{
    int instance = 0;
    uint8 rxTimeStamp[5]  = {0, 0, 0, 0, 0};
    uint32 temp = 0;
    uint8 rxd_event = 0;
    int bufferfull = 0;

	instance_set_status(1);
    if(rxd->event == DWT_SIG_RX_OKAY)
    {
        uint8 buffer[2];

        instance_data[instance].ackreq = rxd->aatset;

        dwt_readrxdata(buffer, 1, 0);  // Read Data Frame
        //at the moment using length and 1st byte to distinguish between different fame types and "blinks".
        switch(rxd->datalength)
        {
            case SIG_RX_ACK:
                rxd_event = SIG_RX_ACK;
                break;
 
            default:
                rxd_event = DWT_SIG_RX_OKAY;
                break;
        }

		
    	if(rxd_event == DWT_SIG_RX_OKAY)
	    {
	        if(instance_data[instance].dweventCnt < 2) //if no outstanding events to process
	        {
	            //dwt_readrxtimestamp((uint8*) &instance_data[instance].rxu.rxTimeStamp) ;

				dwt_readrxtimestamp(rxTimeStamp) ;
				temp =  rxTimeStamp[0] + (rxTimeStamp[1] << 8) + (rxTimeStamp[2] << 16) + (rxTimeStamp[3] << 24);
				instance_data[instance].rxu.rxTimeStamp = rxTimeStamp[4];
				instance_data[instance].rxu.rxTimeStamp <<= 32;
				instance_data[instance].rxu.rxTimeStamp += temp;
				anchor_rxTimeTamp =  instance_data[instance].rxu.rxTimeStamp;
				instance_data[instance].rxLength = rxd->datalength;

				dwt_readrxdata((uint8 *)&instance_data[instance].rxmsg, rxd->datalength, 0);  // Read Data Frame

				//instance_readaccumulatordata();     // for diagnostic display in DecaRanging PC window
				
				if(instance_data[instance].rxmsg.messageData[FCODE] == RTLS_DEMO_MSG_ANCH_TOFR)
				{
					instance_data[instance].stoptimer = 0;
				}

				//dwt_readdignostics(&instance_data[instance].devicelogdata.diag);

				instance_data[instance].stoptimer = 1;

				instance_data[instance].dwevent[instance_data[instance].dweventCnt++] = DWT_SIG_RX_OKAY;
				instance_data[instance].testAppState = TA_RX_WAIT_DATA;   // let this state handle it
				#if DECA_LOG_ENABLE==1
				#if DECA_KEEP_ACCUMULATOR==1
				{
						instance_data[instance].newAccumData = 1 ;
						instance_data[instance].erroredFrame = DWT_SIG_RX_NOERR ;   //no error
						processSoundingData();
				}
				#endif
						logSoundingData(DWT_SIG_RX_NOERR);
				#endif
				
			//	PrintfUtil_vPrintf("RX OK \n");
				//printf("RX time %f\n",convertdevicetimetosecu(instance_data[instance].rxu.rxTimeStamp));
	        }
	        else
	        {
	        	bufferfull = 1;
	        }
			rx_time++;

#if (DEEP_SLEEP == 1)
	        instance_data[instance].rxmsgcount++;
#endif
	    }


	}
    else if (rxd->event == DWT_SIG_RX_TIMEOUT)
    {
        if(instance_data[instance].dweventCnt < 2) //if no outstanding events to process
        {
            instance_data[instance].dwevent[instance_data[instance].dweventCnt++] = DWT_SIG_RX_TIMEOUT;
            //printf("RX timeout while in %d count %d\n", instance_data[instance].testAppState, instance_data[instance].eventCnt);
        }
        else
        {
        	bufferfull = 8;
        }
		if(instance_data[instance].testAppState == TA_TX_WAIT_CONF )
			ta_tx_wait_conf(&instance_data[instance],DWT_SIG_RX_TIMEOUT);
			//printf("RX timeout ignored !!! %d (count %d) \n", instance_data[instance].testAppState, instance_data[instance].eventCnt);
    }
    else //assume other events are errors
    {
        //printf("RX error %d \n", instance_data[instance].testAppState);
        if(instance_data[instance].rxautoreenable == 0)
        {
            //re-enable the receiver
            instancerxon(0, 0); //immediate enable

        }

    }

    if(bufferfull > 0) //buffer full re-enable receiver
    {
    	bufferfull = 10;

#if (DOUBLE_RX_BUFFER == 0)
    	dwt_forcetrxoff() ;
    	dwt_rxenable(0) ;
#endif

    }
//	PrintfUtil_vPrintf("RX OK< %x >",instance_data[instance].rxmsg.messageData[0]);
}


// -------------------------------------------------------------------------------------------------------------------


int instance_get_anchoraddr(void)
{
	uint64 x = instance_data[0].get_anchoraddr ;
	int temp = x%16;
	return (temp);
}

int instance_get_status(void)
{
	int x = instance_data[0].is_newstatus;
	return (x);
}

void instance_set_status(int eventstatus)
{
	instance_data[0].is_newstatus = eventstatus;
}



uint16 instance_get_seqnum(void) //get seqnum
{
//    return instance_data[0].seqnum;
	int x = squem_count;
	squem_count =0;
	return (x);
}

uint64 get_AnchroTimeStamp(void)
{
	uint64 x =anchor_rxTimeTamp  & MASK_40BIT;
	anchor_rxTimeTamp =0;
	return (x);
}
uint64 get_TagTimeStamp(void)
{
	uint64 x = tag_txTimeTamp & MASK_40BIT;
	tag_txTimeTamp =0 ;
	return (x);
}
uint16 get_rcvCardID(void)
{
	uint16 x = recv_cardid;
	recv_cardid = 0;
	return (x);
	
}

uint8 get_TagspeedType(void)
{
	uint8 x = instance_data[0].TxSpeed_Mode;
	return (x);
}

uint8 get_TxspeedType(void)
{
	uint8 x = quickORslow;
	return (x);
}

int instance_readaccumulatordata(void)
{
#if DECA_SUPPORT_SOUNDING==1
    int instance = 0;
    uint16 len = 992 ; //default (16M prf)

    if (instance_data[instance].configData.prf == DWT_PRF_64M)  // Figure out length to read
        len = 1016 ;

    instance_data[instance].buff.accumLength = len ;                                       // remember Length, then read the accumulator data

    len = len*4+1 ;   // extra 1 as first byte is dummy due to internal memory access delay

    dwt_readaccdata((uint8*)&(instance_data[instance].buff.accumData->dummy), len, 0);
#endif  // support_sounding
    return 0;
}

// -------------------------------------------------------------------------------------------------------------------


int instance_run(uint8 type)
{
    int instance = 0 ;
    //int update ;
    //int cnt ;
    int message = instance_data[instance].dwevent[0];

    if(type ==0)
    {
		instance_data[instance].testAppState;
		testapprun(&instance_data[instance], message);//,slotmsg); 
	}
	else
		ta_rxe_wait(&instance_data[instance]);    //anchor wake up enter the rx status


	if(message) // there was an event in the buffer
	{
		instance_data[instance].dwevent[0] = 0; //clear the buffer
		instance_data[instance].dweventCnt--;
		//printf("process event %d in (%d) ecount %d\n", message, state, instance_data[instance].dweventCnt);

		if(instance_data[instance].dwevent[1]) // there is another event in the buffer move it to front
		{
			instance_data[instance].dwevent[0] = instance_data[instance].dwevent[1];
			instance_data[instance].dwevent[1] = 0; //clear the buffer
			//instance_data[instance].dweventCnt = 1;
		}
	}
	instance_data[instance].dwevent[1] = 0; //clear the buffer

	//we've processed message
	message = 0;


    return 0 ;
}

#endif


/* ==========================================================

Notes:

Previously code handled multiple instances in a single console application

Now have changed it to do a single instance only. With minimal code changes...(i.e. kept [instance] index but it is always 0.

Windows application should call instance_init() once and then in the "main loop" call instance_run().

*/
