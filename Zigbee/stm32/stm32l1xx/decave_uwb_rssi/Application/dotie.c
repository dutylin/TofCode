#include"dotie.h"
#include "printf_util.h"


AVG_TIE avgTie[standard_num_max];

uint64 max_tie;
unsigned int standardID[standard_num_max]={0} ;
extern uint16 own_cardid;
/*
*******************************************************************************/
/***********************************************************************
作用:avg_ANH_Tie 求平均的TIE
参数:
	rx_msg :一组接收到的基准标签信息
	msg_array:以基准标签为组的单元
返回: 
	NULL
***********************************************************************/	

uint64 gettie(AVG_TIE avgtie,uint64 tie)
{
	uint64 temp;
	if(tie <= avgtie.Max_tie && tie >= avgtie.Min_tie)
		temp = tie;
	else if(tie > avgtie.Max_tie)
	{
		temp = avgtie.Max_tie;
		avgtie.Max_tie = tie;
	}
	else if(tie < avgtie.Min_tie)
	{
		temp = avgtie.Min_tie;
		avgtie.Min_tie = tie;
	}
	return temp;
}


void avg_ANH_Tie(Rx_msg rx_msg,Msg_array *msg_array ,int n)
{
	int i;
	uint64 Tie,temp;

	if(rx_msg.speedtype!=1)
		return ;

	if(rx_msg.seqnum - msg_array->pre_quick.seqnum == 1)
	{
		Tie = (rx_msg.own_timestamp - msg_array->pre_quick.own_timestamp + Tick_max) % Tick_max;
		if(avgTie[n].count ==0){
			avgTie[n].Max_tie = Tie;
			avgTie[n].avg_TIE =0;
		}
		else if(avgTie[n].count ==1)
		{
			if(Tie >avgTie[n].Max_tie)
			{
				avgTie[n].Min_tie = avgTie[n].Max_tie;
				avgTie[n].Max_tie = Tie;
			}
			else
				avgTie[n].Min_tie = Tie;
			avgTie[n].avg_TIE =0;
		}
		else if(avgTie[n].count >=2 && avgTie[n].count<102)
		{
			temp = gettie(avgTie[n], Tie);
			avgTie[n].avg_TIE = (uint64)(((avgTie[n].count-2) *avgTie[n].avg_TIE +temp)/(avgTie[n].count-1)) ;
		}
		else if(avgTie[n].count==102)
		{
			if(Tie<avgTie[n].Max_tie && Tie>avgTie[n].Min_tie)
			{
				avgTie[n].avg_TIE =(uint64)(((avgTie[n].count-2) *avgTie[n].avg_TIE +Tie)/(avgTie[n].count-1)) ;
				avgTie[n].count--;
			}
		}
		avgTie[n].count++;		//大于10的时候才能使用
	}	
}


/***********************************************************************
作用:build_unity 将接受的的每一组数据，放到相对应的以基准标签分类的单元中
参数:
	rx_msg :一组接收到的基准标签信息
	msg_array:以基准标签为组的单元
返回: 
	NULL
***********************************************************************/	
void clear_Array(Msg_array *msg_array)
{
	int i=0;
	int size =sizeof(Rx_msg);
	for(i=0;i<array_max;i++)
		memset(&msg_array->send_msg[i],0,size);
	memset(&msg_array->last_quick,0,size);
	msg_array->n = 0;
	msg_array->type = 0;
}


void build_unity(Rx_msg rx_msg,Msg_array *msg_array,int k)
{
	int i,n=0;
	__int64 Q_QANH_Tie;
	n = msg_array->n;
//	PrintfUtil_vPrintf("build_unity : n =%d  ",n);
	avg_ANH_Tie(rx_msg,msg_array ,k);
	if(n==0 && rx_msg.speedtype == 1)
	{
//		PrintfUtil_vPrintf("-<1>-");
		if(max_tie==0&&rx_msg.seqnum - msg_array->pre_quick.seqnum ==1)
			max_tie = (rx_msg.own_timestamp - msg_array->pre_quick.own_timestamp+Tick_max)%Tick_max;
		msg_array->pre_quick.cardid = rx_msg.cardid;
		msg_array->pre_quick.own_timestamp = rx_msg.own_timestamp;
		msg_array->pre_quick.seqnum = rx_msg.seqnum;
		msg_array->pre_quick.speedtype = rx_msg.speedtype;
	//	PRINT_SHOW("-0 pre_slow- :cardid==%d ;seqnum=%d ; speedtype=%d \n",msg_array->pre_quick.cardid,msg_array->pre_quick.seqnum,msg_array->pre_quick.speedtype);
		
	}
	else if(msg_array->pre_quick.own_timestamp!= 0 && rx_msg.speedtype == 0 &&n<40)			  //慢速，待测标签来到 
	{
	/*********************************************************************
		第一个与pre_slow 时间戳相差太大，则说明快发中间漏掉，不可取，
		一般相差为2，可用平均时间戳估计上一个的时间戳
	*********************************************************************/
//		PrintfUtil_vPrintf("-<2>-");
		if(n==0)
		{
			if(max_tie<1000000)
					max_tie = avgTie[k].avg_TIE;  ;         //about 120ms
//			PRINT_SHOW("-1- : max_tie= %x \n",max_tie);
			msg_array->send_msg[n].S_QANH_Tie = (rx_msg.own_timestamp - msg_array->pre_quick.own_timestamp+Tick_max)%Tick_max;
			if(msg_array->send_msg[n].S_QANH_Tie > 1.5*max_tie)          //避免快发中间漏发
			{
				memset(&msg_array,0,sizeof(Rx_msg));
				return;
			}
		}
		msg_array->n = msg_array->n+1;
		msg_array->send_msg[n].beTest_cardID = rx_msg.cardid;
		msg_array->send_msg[n].standard_cardID = msg_array->pre_quick.cardid;
		msg_array->send_msg[n].stationID = own_cardid;
		msg_array->send_msg[n].S_QANH_Tie = (rx_msg.own_timestamp - msg_array->pre_quick.own_timestamp+Tick_max)%Tick_max;
		msg_array->send_msg[n].Card_seqnum = rx_msg.seqnum;
		
	}
	else if(n>=1 && rx_msg.speedtype == 1)
	{
//		PrintfUtil_vPrintf("-3 last_quick- : rx_msg.timestamp=%x ; -pre_slow.timestamp = %x  \n",rx_msg.own_timestamp, msg_array->pre_quick.own_timestamp);
		
		msg_array->last_quick = rx_msg;
	//	PrintfUtil_vPrintf("-3 last_quick- : seqnum=%d ; -pre_quick.seqnum = %d  \n",rx_msg.seqnum, msg_array->pre_quick.seqnum );

		if(rx_msg.seqnum - msg_array->pre_quick.seqnum ==1)   
		{
			Q_QANH_Tie = (rx_msg.own_timestamp - msg_array->pre_quick.own_timestamp+Tick_max)%Tick_max;
//			PRINT_SHOW("Q_QANH_Tie = %x \n",Q_QANH_Tie);
			for(i=0;i<n;i++)
			{
				if(avgTie[k].count <20)
					msg_array->send_msg[i].Q_QANH_Tie = Q_QANH_Tie;
				else
				{
					msg_array->send_msg[i].Q_QANH_Tie = avgTie[k].avg_TIE;
					msg_array->send_msg[i].S_QANH_Tie = (uint64)((double)(avgTie[k].avg_TIE)/(double)(Q_QANH_Tie) * (double)(msg_array->send_msg[i].S_QANH_Tie)) ;
				}
			}
			msg_array->type = 1;			   //可以发送出去 
		}
		else{                                //lost one seqnum
//			PRINT_SHOW("-4- : clear_Array \n",n);
			clear_Array(msg_array);
		}
		msg_array->pre_quick = rx_msg; 
	}
}


/***********************************************************************
作用:check_standardID 检测某一个快发(基准标签)是否有记录
参数:
	rx_msg :一组接收到的基准标签信息
返回: 
	是否有该标签ID
	返回0说明新出现的“快发”，应新申请Msg_array空间，返回1则为已有
Bug:当记录表中满了后出现新的，则会有bug发生
***********************************************************************/

int check_standardID(Rx_msg rx_msg)    
{
	int i,n=-1,single=0;
	for(i=0;i<standard_num_max;i++)
	{
		if(standardID[i]==0&&n==-1){       //find the first empty place
			n=i;
		//	break;
		}
		if(standardID[i] == rx_msg.cardid)
		{
			n=i;
			single = 1;
			break;
		}
			
	}
	if(n>=0&&n<standard_num_max)
		standardID[n] = rx_msg.cardid;
	else 
		return -1;
	return single;
}





