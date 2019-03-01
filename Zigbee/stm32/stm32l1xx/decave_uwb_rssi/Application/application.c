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
#include "dotie.h"


#define MAX_TIMER_TICK  (0xFFFFFFFF/2)

/*****************************************************************************
* CONSTANTS AND DEFINES
*/

// on last sector tail 6byte
#define DEV_ADDRESS (0x0801FF00)
#define CARD_1S_MOTION_CNT           1800        //30minutes
#define CARD_5S_MOTION_CNT            360       //30minutes

#define D_VALUE  4
#define DEFF(x,y)   ((x)>=(y)?((x)-(y)):((y)-(x)))

#define MAX_SET_CARD_CNT    30
#define TOF_BUZZER_TIMEOUT			200	
#define sendintime         1
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
/*typedef struct         //网络发送结构体
{
	uint16 beTest_cardID;            //待测标签ID
	uint16 standard_cardID;          //基准标签ID
	uint16 stationID;                //基站ID           sys_option.u32BsId;
	uint16 Card_seqnum;              //待测标签序列号
	uint8  S_QANH_Tie[8];               //慢发与最近一次快发的时间戳差
	uint8  Q_QANH_Tie[8];               //慢发后一次快发减去上一次快发的时间戳差

//	char is_origin_sta;                    //是否为原点标签 0为非原点基站，1表示原点基站
	
}TimeStamp_mgr1;


typedef struct
{
	uint16 n;
	TimeStamp_mgr1 send_msg[send_max];
}Send_Pack1;
*/
/* card's distance */
/*  typedef struct
  {
	  app_header_t app_tof_head;
	  TimeStamp_mgrold my_send_Pack[5];	  // the length(app_tof_head.len) of this array is alterable, max length is APP_MAX_CARD_NUM
  } app_uwb_tdoa_distance_ts;
*/

typedef struct
{
    uint16 u16CardNum[MAX_SET_CARD_CNT];
    uint8 u8CardCnt;
} tsCardTypeSet;

uint8 set_helpover_sleep=0;      //when press help button and beep over ,count the sleep tick ,it's diffrent from other times

uint16 u16CardMotionTimeout = CARD_1S_MOTION_CNT;
uint16 u16CardMotionCnt = 0;

//uint8					 tsVersionReportLen;

app_UWBVersionReport_t tsVersionReport[APP_TOF_VERSION_MAX_NUM];
uint8					 tsVersionReportLen;

extern  Sub_alarm_msg_t sub_alarmmsg;



/*****************************************************************************
*   DECLARATION
*/

//static uint_16 cardSeqNum = 0;
//static uint_32 eventHold;           //事件保持，以免休眠


// motion detection counter
//static uint_8 motion_detect_cnt;
Bool isMoving = True;

//static uint_32 cardMotionCheckTime;
//static uint_32 cardAdcCheckTime;
//static uint_32 reportVersionTime = 0;

#ifdef TOA_RESULT
static Toa_Result_t toaFrm;
#endif
//static Distance_Frm_t distanceFrm;
//static Misc_Frm_t miscFrm;
//static basic_data_t basicData;


uint8 u8HelpCnt = 0;

uint32 u32CommonBuf[128];
//uint16 u16StationPanId;
uint16 u16ArmId = 0xFFFF;
uint8 u8ReportStatusNum = 0;
uint16 u16Hdl_tx_frame_cnt = 0;
uint16 u16Hdl_rx_frame_cnt;
//static uint8 u8cardtype=1;
//static uint8 helpask=0;
uint8 cur_slot= 0;
uint8 tof_count=0;

uint8 u8UartSendBuf[512];
app_uwb_rssi_ts app_uwb_rssi_data;
uint_8 u8rssiNum = 0;

uint8 quiet_count= 0;    //not moving and wake up time(s), if <10 short sleep ,if >10 long sleep when the card stay qiuet
uint16 u16ShortAddr;


uint8 ispower_newon=1 ,newstarttype=0;
uint8 sta_uart_dist=0;    //the uart evet should send the distance
uint8 avg_tof_tick=0;
uint16 own_cardid=0;
extern uint8 rev_retreat_ack;

void vWriteData2Stm32(uint8* pbuf,uint16 len);
uint8 bsmac_build_packet( unsigned char * pbuf,
                              const unsigned char * pdata, unsigned short len,
                              const unsigned char frame_type);

//void vReportCardDistance(void);
void uart_rx_callback(unsigned char *pbuf, unsigned char len);

void vCheckBattery();
/*****************************************************************************
* FUNCTION
*/
uint16 instance_get_cardtype(void)
{
	uint16 cardtype=1;

	cardtype = *(uint16*)(DEV_ADDRESS+2);
	return cardtype;
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

void vCheckBattery()
{

}

uint8 bsmac_build_packet( unsigned char * pbuf,
                              const unsigned char * pdata, unsigned short len,
                              const unsigned char frame_type)
{
    unsigned short tx_len;
    unsigned short crc;
    bsmac_header_t *ph;
	
    struct nwkhdr *pnwkhdr;

    // add mac header
    if (pbuf == NULL || len > 512 )
    {
        EDBG(PrintfUtil_vPrintf("Build Failed pbuf %X len%d\n", pbuf, len);)
        return 0;
    }


    ph = (bsmac_header_t *) pbuf;

    pnwkhdr = (struct nwkhdr *)(ph+1); 

    pnwkhdr->type = NWK_DATA;
    pnwkhdr->ttl = 1;
    pnwkhdr->src = u16ShortAddr;
    pnwkhdr->dst = u16ArmId;
	pnwkhdr->len = len;

	//PrintfUtil_vPrintf("")
	

    ph->preamble_H = BSMAC_PREAMBLE_H;
    ph->preamble_L = BSMAC_PREAMBLE_L;

    BSMAC_SET_DEVICETYPE(ph->frame_control, BSMAC_DEVICE_TYPE_LOC);    // I am location module
    BSMAC_SET_RDY(ph->frame_control, 1);           							// always ready
    BSMAC_SET_FRAMETYPE(ph->frame_control, frame_type);
    BSMAC_SET_PRIORITY(ph->frame_control, 1);

	

    if (frame_type == BSMAC_FRAME_TYPE_ACK) // for ack, use recieved frame_cnt
    {
        ph->frame_count_H = (u16Hdl_rx_frame_cnt & 0xff00) >> 8;
        ph->frame_count_L = u16Hdl_rx_frame_cnt  & 0xff;
    }
    else
    {
        ph->frame_count_H = ( u16Hdl_tx_frame_cnt & 0xff00) >> 8; // framecnt_h
        ph->frame_count_L =  u16Hdl_tx_frame_cnt& 0xff; // framecnt_l
        u16Hdl_tx_frame_cnt++;
    }

    ph->src_addr_H = (u16ShortAddr >> 8) & 0xff;
    ph->src_addr_L = (u16ShortAddr) & 0xff;            // source mac address
    ph->dst_addr_H = 0;                                                     // dst address is useless
    ph->dst_addr_L = 0;
    //ph->reserverd = ;

    /* ack do not need payload, Live may have payload */
    if (len != 0 && pdata && frame_type != BSMAC_FRAME_TYPE_ACK)
    {
        memcpy((void*) (pbuf + BSMAC_HEADER_LEN+sizeof(struct nwkhdr)), pdata, len);
    }

    //LIVE packet needs to be a long frame
    if (frame_type == BSMAC_FRAME_TYPE_LIVE)
    {
        len = BSMAC_MAX_TX_PAYLOAD_LEN;
    }
    else if(frame_type == BSMAC_FRAME_TYPE_ACK)
    {
        len = 0;
    }

    tx_len = len + BSMAC_FOOTER_LEN+sizeof(struct nwkhdr); // length = payload+footer
    ph->data_len_H = (tx_len >> 8) & 0xff; //
    ph->data_len_L = tx_len & 0xff; //

    crc = CRC16((unsigned char *)(pbuf+2), len+BSMAC_HEADER_LEN+sizeof(struct nwkhdr)-2, 0xffff);   // caculate header and payload
    // padding footer
    pbuf[len+BSMAC_HEADER_LEN+sizeof(struct nwkhdr)] = (crc >> 8) & 0xff;
    pbuf[len+BSMAC_HEADER_LEN+sizeof(struct nwkhdr)+1] = crc & 0xff;
	
    vWriteData2Stm32(u8UartSendBuf,sizeof(bsmac_header_t) + tx_len);
    return sizeof(bsmac_header_t) + tx_len;
}

void vReportCardRssi(void)
{
    if(0 < u8rssiNum)
    {
        //PrintfUtil_vPrintf("report rssi:%d\n",u8rssiNum);
        app_uwb_rssi_data.app_tof_head.len = u8rssiNum*sizeof(uwb_rssi_ts);        
        u8rssiNum = 0;
        bsmac_build_packet(u8UartSendBuf,(uint8 *)(&app_uwb_rssi_data), sizeof(app_header_t)+app_uwb_rssi_data.app_tof_head.len,BSMAC_FRAME_TYPE_DATA);
    }
}

void vReportStatus(void)
{
    //struct nwkhdr *pNwkHdr = (struct nwkhdr *)u32CommonBuf;
    app_header_t *pHeader = (app_header_t *)u32CommonBuf;
    app_LSrfReport_t *pStationReport = (app_LSrfReport_t *)(pHeader + 1);

    /* app header */
    pHeader->len = sizeof(app_LSrfReport_t)+strlen(VERSION);

    pHeader->msgtype = APP_UWB_MSG_STATION_VER_LINK;
    pHeader->protocoltype = APP_PROTOCOL_TYPE_UWB_CARD;

    /* rf report */
    pStationReport->hdr.dstaddr = u16ArmId;

    pStationReport->hdr.srcaddr = u16ShortAddr;

    pStationReport->len = strlen(VERSION);

    pStationReport->reporttype = APP_LS_REPORT_STATUS_LOC;
    pStationReport->devtype = BSMAC_DEVICE_TYPE_LOC;
    pStationReport->seqnum = u8ReportStatusNum++;

    memcpy((uint8*)(pStationReport+1),VERSION,strlen(VERSION));
    bsmac_build_packet(u8UartSendBuf,(uint8 *)u32CommonBuf,pHeader->len+sizeof(app_header_t),BSMAC_FRAME_TYPE_DATA);

}


void vWriteData2Stm32(uint8* pbuf,uint16 len)
{
	uint8 i;
	if(pbuf == NULL || len == 0)
	{
		return;
	}
	for(i=0;i<len;i++)
	{
		USART_SendData(USART1,(unsigned char)( *(pbuf+i))); /* Loop until the end of transmission */

		//PrintfUtil_vPrintf(" write \n");
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);
	}
}

void uart_rx_callback(unsigned char *pbuf, unsigned char len)
{
	uint8 *revdata;
//	PrintfUtil_vPrintf("uart rev data!\n");
	RfTofWrapper_tu* psAppPkt = (RfTofWrapper_tu*)(pbuf+sizeof(bsmac_header_t) + sizeof(struct nwkhdr));

	if(len <sizeof(bsmac_header_t)+ sizeof(struct nwkhdr) + sizeof(app_header_t))
	{
		EDBG(PrintfUtil_vPrintf("err0 len =%d pt=%x  ;mt=%d\n",len,psAppPkt->tof_head.protocoltype,psAppPkt->tof_head.msgtype);)
			
		return;
	}

	//not for me
	if(APP_PROTOCOL_TYPE_UWB_CARD != psAppPkt->tof_head.protocoltype)
		return;
	revdata = (uint8*)(psAppPkt+1);
	switch(psAppPkt->tof_head.msgtype)
	{
	case  APP_UWB_MSG_RETREAT://撤离 5
		instance_set_alarmlist(0,0xFF,1);
		event_timer_set(EVENT_URGENT_RETREAT);
		event_timer_unset(EVENT_URGENT_RESET);
		//event_timer_add(EVENT_URGENT_RESET,60000);
		break;

	case APP_UWB_MSG_CANCEL_RETREAT:    //取消撤离6
		instance_set_alarmlist(0,0xFF,0);
		event_timer_set(EVENT_URGENT_RETREAT);
		event_timer_unset(EVENT_URGENT_RESET);
		event_timer_add(EVENT_URGENT_RESET,120000);
		break;

	case APP_UWB_MSG_ALARM_ACK:     //求救返回ack
		int i=0;
		uint16 addr=0;
		for(i=0;i<psAppPkt->tof_head.len/2;i++)
		{
			memcpy(&addr,&revdata[2*i],2);
			//instance_set_helpstatus(addr);
		}
		break;

	case APP_UWB_MSG_REQ_LOC_DISTANCE:
		//event_timer_set(EVENT_UART_SEND);
		break;

	case APP_UWB_MSG_SET:
		break;
	case APP_UWB_MSG_LINK:
		vReportStatus();
		break;
			
	}
}


//#endif

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
        event_timer_unset(EVENT_CARD_VER_BATTERY_EVENT);
		event_timer_set(EVENT_CARD_VER_BATTERY_EVENT);
    }
}

void vReportCardVersion(void)
{
    if(tsVersionReportLen>0 && tsVersionReportLen<=APP_TOF_VERSION_MAX_NUM)
    {
        app_header_t *pHeader = (app_header_t *)u32CommonBuf;
        //app_LSrfReport_t *pStationReport = (app_LSrfReport_t *)(pHeader + 1);
        app_UWBVersionReport_t *pVersionReport = (app_UWBVersionReport_t *)(pHeader + 1);

        /*app header */
        pHeader->msgtype = APP_UWB_MSG_CARD_VER_BATTRY;
        pHeader->protocoltype = APP_PROTOCOL_TYPE_UWB_CARD;
        pHeader->len = tsVersionReportLen*sizeof(app_UWBVersionReport_t);

		for(int i=0;i<tsVersionReportLen;i++)
		{
			memcpy(pVersionReport + i,&tsVersionReport[i],sizeof(app_UWBVersionReport_t));
		}
        //memcpy((uint8*)pVersionReport, (uint8*)tsVersionReport, tsVersionReportLen*sizeof(app_UWBVersionReport_t));
		bsmac_build_packet(u8UartSendBuf,(uint8 *)(u32CommonBuf), sizeof(app_header_t)+pHeader->len,BSMAC_FRAME_TYPE_DATA);
		
        /* clear the buffer after send */
        memset((uint8*)tsVersionReport, 0, APP_TOF_VERSION_MAX_NUM*sizeof(app_UWBVersionReport_t));
        tsVersionReportLen = 0;
    }
    else
    {
        tsVersionReportLen = 0;
    }
}


void vReportCardAlarm(uint16 addr,uint16 exciterid,uint8 status)
{
	app_uwb_alarm_ts app_uwb_alarm_data;
	app_uwb_alarm_data.app_tof_head.protocoltype = APP_PROTOCOL_TYPE_UWB_CARD;
	app_uwb_alarm_data.app_tof_head.msgtype = APP_UWB_MSG_ALARM ;
	app_uwb_alarm_data.u16ShortAddr = addr;
	app_uwb_alarm_data.u8Status = status;
	app_uwb_alarm_data.u8ExciterID = exciterid;

	app_uwb_alarm_data.app_tof_head.len = 4;
	bsmac_build_packet(u8UartSendBuf,(uint8 *)(&app_uwb_alarm_data), sizeof(app_header_t)+app_uwb_alarm_data.app_tof_head.len,BSMAC_FRAME_TYPE_DATA);

}

void card_retreat(uint8 *revdata,uint8 revlen)
{
	uint8 count,i;
	uint16 addr;
	if(revlen ==0)
		return;
	count = revlen/2;
	instance_set_alarmlist(addr,0xFF,0); //clear all pre alarm buff
	for(i=0;i<count;i++)
	{
		memcpy(&addr,&revdata[2*i],2);
		instance_set_alarmlist(addr,0,1);
	}
	event_timer_del(EVENT_URGENT_RESET);
	event_timer_add(EVENT_URGENT_RESET, 60000); 
}


/***********************************************************************
作用:send_pack 将数据发送出去，若传入的数据够大，直接发送出去，若比较小
	则放入到pack中待包一定大时发送出去
参数:
	pack :待发送的包空间
	msg_array :待放入到pack的数据
	time :放入pack包的次数
返回: 空
***********************************************************************/

void AppEventsProcess(void)

{
  	uint32 events = event_timer_take_all();

	if(events &(EVENT_NEWSLOT_EVENT))
	{
		//uint16 cardid;
		//uint8 type,cardtype;

		dwt_forcetrxoff();
		//DBG(PrintfUtil_vPrintf("----------------------------\n");)

		if(ispower_newon==1)
			newstarttype =1;  //first power on ,the  anchor wake up enter the init status
		else
			newstarttype =0; //new slot anchor wake up enter the rx status
		event_timer_unset(EVENT_RAGING_REPORT);			
		event_timer_set(EVENT_RAGING_REPORT);
	
		ispower_newon=0;
		
	}

	
	if (events & (EVENT_RAGING_REPORT))
	{	
			
	    instance_run(newstarttype);
		if((instancegetrole() == ANCHOR || instancegetrole() == SUB_STA )&&newstarttype ==1)
			newstarttype=0;
	 //   event_timer_unset(EVENT_RAGING_REPORT);

	}

	if(events & (EVENT_CARD_VER_BATTERY_EVENT))
	{
        vReportCardVersion();
		event_timer_add(EVENT_CARD_VER_BATTERY_EVENT,60000);
	}
	
	if(events & EVENT_URGENT_RETREAT)
	{
		blink_to_retreat();
		event_timer_add(EVENT_URGENT_RETREAT,25);
	}
	
	if(events & EVENT_URGENT_RESET)
	{
		event_timer_del(EVENT_URGENT_RETREAT);
	}
	
    if(events & EVENT_REPORT_RSSI)
	{
        vReportCardRssi();
        event_timer_add(EVENT_REPORT_RSSI,500);
		if(instancenewCar())
		{
			inset_rev_cardmsg_ext();
			vReportCardRssi();
		}
    }


	if (events & (EVENT_UART_SEND))
	{
		led_station_off();
		newstarttype =1;
		event_timer_set(EVENT_RAGING_REPORT);
			if((sub_alarmmsg.alarmstatus & UWB_CARD_STATUS_HELP && instance_get_sta_status(STATION_TYPE_ALARM))
			|| (sub_alarmmsg.alarmstatus & UWB_CARD_STATUS_RETREAT_ACK &&rev_retreat_ack ==1))	//help msg send
		{
			rev_retreat_ack =0;
			vReportCardAlarm(sub_alarmmsg.alarmaddr,0xff,sub_alarmmsg.alarmstatus);
		}
		else if(sub_alarmmsg.alarmstatus & UWB_CARD_STATUS_IMPEL && instance_get_sta_status(STATION_TYPE_EXCIT))
		{
			vReportCardAlarm(sub_alarmmsg.alarmaddr,sub_alarmmsg.excitid,sub_alarmmsg.alarmstatus);
		}
		dwt_setrxtimeout(0);
		instancerxon(0,0);

	}

	if(events & (EVENT_REPORT_LINK))
	{
		static uint16 i;
		if((i++ %24) == 0)
		{
			vReportStatus();
			PrintfUtil_vPrintf("report link\n");
		}
		event_timer_add(EVENT_REPORT_LINK,5000);
		//event_timer_add(EVENT_REPORT_LINK,5000);
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
	LED_Red_Off();
	LED_Green_Off();
	msg_analyser_register(uart_rx_callback);
	
	memset((uint8*)tsVersionReport, 0, APP_TOF_VERSION_MAX_NUM*sizeof(app_UWBVersionReport_t));

    app_uwb_rssi_data.app_tof_head.protocoltype = APP_PROTOCOL_TYPE_UWB_CARD;
    app_uwb_rssi_data.app_tof_head.msgtype = APP_UWB_MSG_RSSI;
    u8rssiNum = 0;

    event_timer_add(EVENT_REPORT_RSSI,1000);

	event_timer_add(EVENT_REPORT_LINK,200);

	event_timer_add(EVENT_CARD_VER_BATTERY_EVENT,1000);

	event_timer_set(EVENT_NEWSLOT_EVENT);
		
	vReportStatus();
    while (1)
    {
		if(instance_get_status()==1)
		{
			event_timer_unset(EVENT_RAGING_REPORT);
			event_timer_set(EVENT_RAGING_REPORT);
		}
		WatchdogReset();
		event_timer_update();
		AppEventsProcess();

	}
}

