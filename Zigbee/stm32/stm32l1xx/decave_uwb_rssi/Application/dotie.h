#include"type_def.h"


#define array_max 20
#define Tick_max 1099511627775             //0xffffffffff �����tick��
#define standard_num_max    (5)  //ͬһ����׼�ܹ����յ������Ļ�׼��վ������
#define send_max   (1)          //������� TimeStamp_mgr �ĸ��� 

typedef struct         //���緢�ͽṹ��
{
	uint16 beTest_cardID;            //�����ǩID
	uint16 standard_cardID;          //��׼��ǩID
	uint16 stationID;                //��վID           sys_option.u32BsId;
	uint16 Card_seqnum;              //�����ǩ���к�
	uint64 S_QANH_Tie;               //���������һ�ο췢��ʱ�����
	uint64 Q_QANH_Tie;               //������һ�ο췢��ȥ��һ�ο췢��ʱ�����

//	char is_origin_sta;                    //�Ƿ�Ϊԭ���ǩ 0Ϊ��ԭ���վ��1��ʾԭ���վ
	
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
	uint16 cardid;     //���⿨ID 
	uint8 speedtype;
}Rx_msg;               //��ǩ�Լ���׼��ǩ����������Ϣ

typedef struct msgarray          //�Ի�׼��ǩ�� 
{
	Rx_msg pre_quick;
	TimeStamp_mgr send_msg[array_max];
	Rx_msg last_quick;
	uint16 n;
	uint8 type;               //0Ϊδ����1Ϊ������������� 
}Msg_array;

typedef struct sendpack
{
	uint16 n;
	TimeStamp_mgr send_msg[send_max];
}Send_Pack;

typedef struct           //��վʱ���
{
	uint64 avg_TIE;
	uint64 Max_tie;
	uint64 Min_tie;
	int count;              //����ƽ������Ŀ
}AVG_TIE;



void build_unity(Rx_msg rx_msg,Msg_array *msg_array,int k);

int check_standardID(Rx_msg rx_msg);

/***********************************************************************
����:send_pack �����ݷ��ͳ�ȥ������������ݹ���ֱ�ӷ��ͳ�ȥ�����Ƚ�С
	����뵽pack�д���һ����ʱ���ͳ�ȥ
����:
	pack :�����͵İ��ռ�
	msg_array :�����뵽pack������
	time :����pack���Ĵ���
����: ��
***********************************************************************/

//void send_pack(Send_Pack *pack,Msg_array *msg_array,int *con_times);


