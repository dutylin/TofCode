#include"type_def.h"


#define array_max 20
#define Tick_max 1099511627775             //0xffffffffff 即最大tick数
#define standard_num_max    (5)  //同一个基准能够接收到的最多的基准基站的数量
#define send_max   (1)          //发送最大 TimeStamp_mgr 的个数 

typedef struct         //网络发送结构体
{
	uint16 beTest_cardID;            //待测标签ID
	uint16 standard_cardID;          //基准标签ID
	uint16 stationID;                //基站ID           sys_option.u32BsId;
	uint16 Card_seqnum;              //待测标签序列号
	uint64 S_QANH_Tie;               //慢发与最近一次快发的时间戳差
	uint64 Q_QANH_Tie;               //慢发后一次快发减去上一次快发的时间戳差

//	char is_origin_sta;                    //是否为原点标签 0为非原点基站，1表示原点基站
	
}TimeStamp_mgr;
typedef struct 
{
//	uint8 rx_timestamp[8];
	uint8 own_timestamp[8];
	uint16 seqnum;
	uint16 cardid;
	uint8 speedtype;
	
}TimeStamp_mgrold;


typedef struct rxmsg 
{
	uint64 own_timestamp;
	uint16 seqnum;      
	uint16 cardid;     //待测卡ID 
	uint8 speedtype;
}Rx_msg;               //标签以及基准标签发过来的信息

typedef struct msgarray          //以基准标签放 
{
	Rx_msg pre_quick;
	TimeStamp_mgr send_msg[array_max];
	Rx_msg last_quick;
	uint16 n;
	uint8 type;               //0为未满，1为满可以组包发出 
}Msg_array;

typedef struct sendpack
{
	uint16 n;
	TimeStamp_mgr send_msg[send_max];
}Send_Pack;

typedef struct           //基站时间戳
{
	uint64 avg_TIE;
	uint64 Max_tie;
	uint64 Min_tie;
	int count;              //计入平均的数目
}AVG_TIE;



void build_unity(Rx_msg rx_msg,Msg_array *msg_array,int k);

int check_standardID(Rx_msg rx_msg);

/***********************************************************************
作用:send_pack 将数据发送出去，若传入的数据够大，直接发送出去，若比较小
	则放入到pack中待包一定大时发送出去
参数:
	pack :待发送的包空间
	msg_array :待放入到pack的数据
	time :放入pack包的次数
返回: 空
***********************************************************************/

//void send_pack(Send_Pack *pack,Msg_array *msg_array,int *con_times);


