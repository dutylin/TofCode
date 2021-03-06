#ifndef GASMENULIB_GLOBAL_H
#define GASMENULIB_GLOBAL_H
#include "hal_types.h"
#include "GasMonitor_MenuLib.h"
#include "AppProtocol.h"
#include "AppProtocolWrapper.h"
#include "ch4.h"
#include "temperature.h"

#define SCREEN_LINES        3
#define TIME_LEN                 8
#define DATE_LEN                11
#define MAX_DATA_BUF      32
#define SCREEN_LINES         3
#define STACK_DEPTH         4
#define TOP_POS_TIME        5

#define MENU_RESET_NODEINFO()    st(node_info.high_line =1; \
	node_info.sel_item = 0; \
	node_info.show_item = 0;)

#define FIRSTTIME_INTO_NODE()    (CurrentNodeID!= NearLastNodeID)

/*-----------------Typedefs-----------------*/
enum node_type
{
    NODE_TYPE_ORPHAN,
    NODE_TYPE_TREE,
};

typedef struct
{
    void(*display)(void);
    void(*on_key)(uint8 keys, uint8 status);
} MenuOper_t;

typedef struct
{
    uint8 len;
    uint8* p;
} buf_t;

typedef struct
{
    uint8   high_line;
    uint8   sel_item;
    uint8   show_item;
} node_info_t;

typedef struct
{
    uint8 id;
    node_info_t node_info;
} pipeline_t;

typedef struct
{
    uint8 id;
    node_info_t node_info;
} stack_p_t;

typedef struct
{
    uint8           stack_i;
    uint8           stack_depth;
    stack_p_t*   stack_p;
} stack_t;

typedef struct
{
    uint8 backlight_ctl;
    uint16 over_density;
    TYRCH4  ch4_ctl;
    tyrTemp_t tyrTemper;
    uint8 timeformat_t;
} set_info_t;

enum MenuID
{
    MENU_ID_ROOT = 0,

    /*independent ID in the root, parrelled with main menu*/
    MENU_ID_POWERON_ANIMATION,
    MENU_ID_POWEROFF_ANIMATION,

    MENU_ID_SHOWMESSAGE,
    MENU_ID_SHOWQUESTION,
    MENU_ID_SHOWALERT,

    MENU_ID_SOS,
    MENU_ID_SOS_RESULT,

    MENU_ID_SOS_ALARM,
    MENU_ID_SOS_ALARM_RESULT,

    MENU_ID_ADJUSTTIME,
    MENU_ID_ADJUSTDATE,

    MENU_ID_RETREAT,

    MENU_ID_MAIN,//27

    /*--------separator  of orphan nodes and tree nodes------------------*/
    MENU_ID_SEPARATOR,

    /*the tree menu begins from functionlist*/
    MENU_ID_FUNCTIONLIST,

    /*Pages in FUNCTIONLIST */
#ifdef CFG_GAS_SHORTMESSAGE
    MENU_ID_SHORTMESSAGE_NUMJUDGE,
#endif

    MENU_ID_ZEROADJUST,
    MENU_ID_CALIBRATION,
    //MENU_ID_ACCIDENTALARM,
#ifdef TURN_ON_LOCATE_DISPLAY    
    MENU_ID_LOCATE,
#endif
    MENU_ID_TEMPERATURE,
    MENU_ID_SOSALARM,
#ifdef CFG_GAS_CARDSEARCH    
    MENU_ID_CARDSEARCH,
#endif    
#ifdef CFG_GAS_CARDCHECK
    MENU_ID_CARDCHECK,
#endif
    MENU_ID_SETTINGS,
#ifdef CFG_GAS_INFORMATION
    MENU_ID_TOOLS,
#endif

#ifdef CFG_GAS_SHORTMESSAGE
    MENU_ID_SHORTMESSAGE,
#endif
    /*Pages in ZeroAdjust */
    MENU_ID_ZEROADJUST_RESULT,

    /*Pages in Calibration */
    MENU_ID_CALIBRATION_SETDENSITY,
    MENU_ID_CALIBRATION_RESULT,

#ifdef CFG_GAS_SHORTMESSAGE
    /*Pages in ShortMessage */
    MENU_ID_SHORTMESSAGE_READ,
    MENU_ID_SHORTMESSAGE_HANDLE,
    MENU_ID_SHORTMESSAGE_DELETE_RESULT,
#endif

    /*Pages in settings */
    MENU_ID_SETTINGS_OVERDENSITY,
    MENU_ID_SETTINGS_BACKLIGHT,
    MENU_ID_SETTINGS_TEMPERATURE,
    MENU_ID_SETTINGS_TIME,

    /*Pages in SOSalarm */
    MENU_ID_SOSALARM_ALERT,

    /*Pages in CardSearch */
#ifdef CFG_GAS_CARDSEARCH            
    MENU_ID_CARDSEARCH_RESULT,
#endif

    /*Pages in TimeSetting */
    MENU_ID_SETTINGS_TIME_TIMEFORMAT,
};

/*-----------------global variable declares-----------------*/
extern  uint8             CurrentNodeID;
extern  uint8             NearLastNodeID;

extern node_info_t     node_info;
extern stack_t           global_stack;

/* global buffer to send variables or strings when state jump, no detail defination, you can define its usage by spectial use*/
extern uint8             g_jump_buf[MAX_DATA_BUF];

/*-----------------Function declares-----------------*/
uint8 GetTypeFromID(uint8 ID);
uint8 GetIDFromIdx(uint8 node_type, uint8 idx);
uint8 GetIdxFromID(uint8 id);
void* Buffer_Init(buf_t* const, uint8);
void Buffer_Free(buf_t* const);
void Buffer_Copy(buf_t*  dstbuf, const buf_t * srcbuf);
void Buffer_Clear(buf_t *buf);
void Stack_Push(stack_t* const, uint8, node_info_t const*);
bool  Stack_Pop(stack_t* const, uint8*, node_info_t*);
void Stack_Clear(stack_t* const);
void   menu_display(void);
void  menu_JumptoMenu(uint8 ID);
void  menu_Mark(void);
void  menu_JumpandMark(uint8 ID);
void  menu_JumpBackWithMark(void);
void menu_JumpBackMarkParent(void);
uint8 menu_GetJumpMark(void);
void  menu_JumpandPush(uint8 ID);
void  menu_JumpandPop(void);
void menu_JumptoItself(void);
char *  menu_itoa (uint16 value, char * buffer);
#endif
