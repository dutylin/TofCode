#include "lcd_serial.h"
#include "hal_types.h"
#include "hal_key.h"
#include "key.h"
#include "51PY.h"
#include "osal.h"
#include "MenuLib_global.h"
#include "MobilePhone_MenuLibChinese.h"
#include "MenuChineseInputUtil.h"
#include "MobilePhone_Function.h"
#include "OnBoard.h"
#include "ZComdef.h"
#include "string.h"

#define                  LETTER_LEN                  7
#define                  CHINESE_BACK_LEN      14
#define                  CHINESE_OUT_LEN    (3+2)

//letters input
static uint8             inputstr[LETTER_LEN + 2];   //add another 2 byte to avoid memory voerflow, I hate bad style codes!
static uint8             letter_id = 0;
//chinese output from the input letters
static uint8             chinese_output[CHINESE_BACK_LEN];
static uint8             chinese_select = 0;
static uint8             chinese_len = 0;
//the last output of the function
static uint8             output_last[CHINESE_OUT_LEN];
static uint8             output_last_len = 0;

//static uint8             letter_save = '0';
//static uint8             key_save = 'a';
//backup the last input status
static uint8             last_input_status = CHINESE_IN;
static uint8             input_status = OUTPUT_STATUS;

/* flags to indicate  a continues press */
static uint8        isContinuePress;
static uint8        ContinuePressKey;

static uint8             const  *result = NULL;
static uint8             const  punctuation[] = {0x2C, 0x2E, 0x21, 0x22, 0x28, 0x29, 0x3B, 0x3A, 0x25, 0x2B, 0X2D, 0X2F, 0x23, 0x0};

static void menu_ChineseProcessKeyLeft(void);
static void menu_ChineseProcessKeyRight(void);
static void menu_ChineseContinuesPressTestStart(uint8 keys);

uint8*  menu_ChineseOutput(void)
{
    return output_last;
}


uint8  menu_ChineseOutput_Length(void)
{
    return output_last_len;
}
void    menu_ChineseOutputClear(void)
{
    osal_memcpy(output_last, '\0', CHINESE_OUT_LEN);
    output_last_len = 0;
}

static void    menu_InputClear(void)
{
    osal_memcpy(inputstr, 0, LETTER_LEN);
    letter_id = 0;
}

void    menu_inputchinese_display(void)
{
    letter_id = 0;
    input_status = OUTPUT_STATUS;
    if(data_buf.len < (MAX_NAME_LEN-1))
        last_input_status = CHINESE_IN;
    menu_ChineseOutputClear();
}

uint8    menu_inputchinese_onkey(uint8 keys, uint8 status)
{
    uint8 num_ascii = '0';

    switch(keys)
    {
    case HAL_KEY_STAR:
        return input_status;
    case HAL_KEY_0:
        input_status = last_input_status;
        if(input_status == NUMBER_IN)
        {
            letter_id = 1;
            num_ascii = MP_Key2ASCII(keys);
        }
        break;
    case HAL_KEY_2:
    case HAL_KEY_3:
    case HAL_KEY_4:
    case HAL_KEY_5:
    case HAL_KEY_6:
    case HAL_KEY_7:
        input_status = last_input_status;
        if(input_status == NUMBER_IN)
        {
            letter_id = 1;
            num_ascii = MP_Key2ASCII(keys);
        }
        else if((input_status == LETTER_IN) || (input_status == CHINESE_IN))
        {

            if(isContinuePress && ContinuePressKey == keys)
            {
                menu_ChineseProcessKeyRight();
            }
            else
            {
                if(input_status == LETTER_IN)
                {
                    letter_id = 0;
                }
                if(letter_id < LETTER_LEN)
                {
                    inputstr[letter_id] = 3 * MP_Key2ASCII(keys) - 2 * '2' + 47;
                    //key_save = MP_Key2ASCII(keys);
                    //letter_save = inputstr[letter_id];
                    inputstr[++letter_id] = '\0';
                }
            }
        }
        break;
    case HAL_KEY_8:
        input_status = last_input_status;
        if(input_status == NUMBER_IN)
        {
            letter_id = 1;
            num_ascii = MP_Key2ASCII(keys);
        }
        else if((input_status == LETTER_IN) || (input_status == CHINESE_IN))
        {
            if(isContinuePress && ContinuePressKey == keys)
            {
                menu_ChineseProcessKeyRight();
            }
            else
            {
                if(input_status == LETTER_IN)
                {
                    letter_id = 0;
                }
                if(letter_id < LETTER_LEN)
                {
                    inputstr[letter_id] = MP_Key2ASCII(keys) + 60;
                    //key_save = MP_Key2ASCII(keys);
                    //letter_save = inputstr[letter_id];
                    inputstr[++letter_id] = '\0';
                }
            }
        }
        break;
    case HAL_KEY_9:
        input_status = last_input_status;
        if(input_status == NUMBER_IN)
        {
            letter_id = 1;
            num_ascii = MP_Key2ASCII(keys);
        }
        else if((input_status == LETTER_IN) || (input_status == CHINESE_IN))
        {
            if(isContinuePress && ContinuePressKey == keys)
            {
                menu_ChineseProcessKeyRight();
            }
            else
            {
                if(input_status == LETTER_IN)
                {
                    letter_id = 0;
                }
                if(letter_id < LETTER_LEN)
                {
                    inputstr[letter_id] = MP_Key2ASCII(keys) + 62;
                    //key_save = MP_Key2ASCII(keys);
                    //letter_save = inputstr[letter_id];
                    inputstr[++letter_id] = '\0';
                }
            }
        }
        break;
    case HAL_KEY_1:
        input_status = last_input_status;
        if(input_status == NUMBER_IN)
        {
            letter_id = 1;
            num_ascii = MP_Key2ASCII(keys);
        }
        else if(input_status == PUNCTUATION_IN)
        {
            letter_id = 0;
            last_input_status = PUNCTUATION_IN;
        }
        else if(input_status == CHINESE_IN)
        {
            result=NULL;
        }
        break;
    case HAL_KEY_POUND:
    {
        if(last_input_status == CHINESE_IN)
        {
            input_status = OUTPUT_STATUS;//LETTER_IN;
            last_input_status  = LETTER_IN;
            letter_id = 0;
        }
        else if(last_input_status == LETTER_IN)
        {
            input_status = OUTPUT_STATUS;//NUMBER_IN;
            last_input_status  = NUMBER_IN;
            letter_id = 0;
        }
        else if(last_input_status == NUMBER_IN)
        {
            input_status = OUTPUT_STATUS;//PUNCTUATION_IN;
            last_input_status  = PUNCTUATION_IN;
            letter_id = 0;
        }
        else if(last_input_status == PUNCTUATION_IN)
        {
            input_status = OUTPUT_STATUS;//CHINESE_IN;
            last_input_status  = CHINESE_IN;
            letter_id = 0;
        }
        break;
    }
    case HAL_KEY_RIGHT:
        menu_ChineseProcessKeyRight();
        break;
    case HAL_KEY_LEFT:
        menu_ChineseProcessKeyLeft();
        break;
    case HAL_KEY_UP:
        if(input_status == CHINESE_BACK)
        {
            if(chinese_select <= (CHINESE_BACK_LEN - 2))
            {
                input_status = CHINESE_IN;
            }
        }
        break;
    case HAL_KEY_DOWN:
        if((input_status == CHINESE_IN) && (result != NULL))
        {
            chinese_select = 0;
            input_status = CHINESE_BACK;
        }
        break;
    case HAL_KEY_SELECT:
        if(input_status == CHINESE_BACK)
        {
            if(output_last_len < CHINESE_OUT_LEN)
            {
                osal_memcpy(output_last, &chinese_output[chinese_select % 14], 2);
                output_last[2] = '\0';
                chinese_select = 0;
                last_input_status = CHINESE_IN;
                input_status = OUTPUT_STATUS;
                output_last_len += 2;
                menu_InputClear();

            }
        }

        else if(input_status == LETTER_IN)
        {
            if(letter_id > 0 && output_last_len < CHINESE_OUT_LEN)
            {
                output_last[output_last_len] = inputstr[--letter_id];
                output_last_len++;
                last_input_status = input_status;
                input_status = OUTPUT_STATUS;
            }
        }
        else if(input_status == PUNCTUATION_IN)
        {
            if(output_last_len < CHINESE_OUT_LEN)
            {
                output_last[output_last_len] = punctuation[letter_id];
                output_last_len++;
                last_input_status = input_status;
                input_status = OUTPUT_STATUS;
            }
        }
        break;
    case HAL_KEY_BACKSPACE:
        if((input_status == CHINESE_IN) || (input_status == CHINESE_BACK))
        {
            if(input_status == CHINESE_BACK)
                input_status = CHINESE_IN;

            if(letter_id == 1)
            {
                letter_id = 0;
                input_status = OUTPUT_STATUS;
            }
        }
        break;
    default:
        break;
    }

    // start a continus press test for the next press
    menu_ChineseContinuesPressTestStart(keys);

    if(input_status == CHINESE_IN)
    {
        if(letter_id > 0)
        {
            if((result =  py_ime(inputstr)) != 0)
            {
                chinese_len = osal_strlen((char*)result);
                memcpy((void*)chinese_output, (void*)result, CHINESE_BACK_LEN);
            }
        }
    }
    else if((input_status == NUMBER_IN) && (keys != HAL_KEY_POUND))
    {
        if(letter_id > 0 && output_last_len < CHINESE_OUT_LEN)
        {
            output_last[output_last_len] = num_ascii;
            output_last_len++;
            last_input_status = input_status;
            input_status = OUTPUT_STATUS;
        }
    }

    return input_status;
}


static void menu_ChineseProcessKeyLeft(void)
{
    if(input_status == CHINESE_BACK)
    {
        if(chinese_select > 0)
        {
            chinese_select -= 2;
        }
        else if(chinese_select == 0)
        {
            chinese_select = chinese_len - 2;
        }
    }
    else if(((input_status == LETTER_IN) || (input_status == CHINESE_IN)) && (letter_id > 0) && (letter_id < LETTER_LEN))
    {
        if((inputstr[letter_id - 1] <= 'o') && (inputstr[letter_id - 1] >= 'a'))
        {
            if((inputstr[letter_id - 1] - 'a') % 3 == 0)
            {
                inputstr[letter_id - 1] += 2;
            }
            else
            {
                --inputstr[letter_id - 1];
            }
        }
        else if((inputstr[letter_id - 1] <= 'v') && (inputstr[letter_id - 1] >= 't'))
        {
            if((inputstr[letter_id - 1] - 't') % 3 == 0)
            {
                inputstr[letter_id - 1] += 2;
            }
            else
            {
                --inputstr[letter_id - 1];
            }
        }
        else if((inputstr[letter_id - 1] <= 's') && (inputstr[letter_id - 1] >= 'p'))
        {
            if((inputstr[letter_id - 1] - 'p') % 4 == 0)
            {
                inputstr[letter_id - 1] += 3;
            }
            else
            {
                --inputstr[letter_id - 1];
            }
        }
        else if((inputstr[letter_id - 1] <= 'z') && (inputstr[letter_id - 1] >= 'w'))
        {
            if((inputstr[letter_id - 1] - 'w') % 4 == 0)
            {
                inputstr[letter_id - 1] += 3;
            }
            else
            {
                --inputstr[letter_id - 1];
            }
        }
    }
    else if(input_status == PUNCTUATION_IN)
    {
        if(letter_id > 0)
        {
            letter_id--;
        }
        else if(letter_id == 0)
        {
            letter_id = osal_strlen((char*)punctuation) - 1;
        }
    }

}

static void menu_ChineseProcessKeyRight(void)
{
    if(((input_status == LETTER_IN) || (input_status == CHINESE_IN)) && (letter_id > 0) && (letter_id < LETTER_LEN))
    {
        if((inputstr[letter_id - 1] <= 'o') && (inputstr[letter_id - 1] >= 'a'))
        {
            if((inputstr[letter_id - 1] - 'a') % 3 == 2)
            {
                inputstr[letter_id - 1] -= 2;
            }
            else
            {
                ++inputstr[letter_id - 1];
            }
        }
        else if((inputstr[letter_id - 1] <= 'v') && (inputstr[letter_id - 1] >= 't'))
        {
            if((inputstr[letter_id - 1] - 't') % 3 == 2)
            {
                inputstr[letter_id - 1] -= 2;
            }
            else
            {
                ++inputstr[letter_id - 1];
            }
        }
        else if((inputstr[letter_id - 1] <= 's') && (inputstr[letter_id - 1] >= 'p'))
        {
            if((inputstr[letter_id - 1] - 'p') % 4 == 3)
            {
                inputstr[letter_id - 1] -= 3;
            }
            else
            {
                ++inputstr[letter_id - 1];
            }
        }
        else if((inputstr[letter_id - 1] <= 'z') && (inputstr[letter_id - 1] >= 'w'))
        {
            if((inputstr[letter_id - 1] - 'w') % 4 == 3)
            {
                inputstr[letter_id - 1] -= 3;
            }
            else
            {
                ++inputstr[letter_id - 1];
            }
        }
    }
    else if(input_status == CHINESE_BACK)
    {
        if(chinese_select < chinese_len - 2)
        {
            chinese_select += 2;
            if((chinese_select % 14) == 0)
            {
                if((result =  py_ime(inputstr)) != 0)
                {
                    uint8 len;

                    len = (chinese_len > (chinese_select + CHINESE_BACK_LEN)) ? CHINESE_BACK_LEN : (chinese_len - chinese_select);
                }
            }
        }
    }
    else if(input_status == PUNCTUATION_IN)
    {
        if(letter_id < osal_strlen((char*)punctuation) - 1)
        {
            letter_id++;
        }
    }

}

void menu_ChineseContinuesPressTestStart(uint8 keys)
{
    if((input_status == LETTER_IN) || (input_status == CHINESE_IN))
    {
        switch(keys)
        {
        case HAL_KEY_2:
        case HAL_KEY_3:
        case HAL_KEY_4:
        case HAL_KEY_5:
        case HAL_KEY_6:
        case HAL_KEY_7:
        case HAL_KEY_8:
        case HAL_KEY_9:
        {
            isContinuePress = TRUE;
            ContinuePressKey = keys;
            if(ZSuccess != osal_start_timerEx(MP_Function_TaskID, MP_FUNC_CONTINUESPRESS_TEST_EVENT, 1000))
            {
                SystemReset();
            }
            break;
        }
        default:
        {
            isContinuePress = FALSE;
            ContinuePressKey = 0;
            osal_stop_timerEx(MP_Function_TaskID, MP_FUNC_CONTINUESPRESS_TEST_EVENT);
            break;
        }
        }

    }
    else
    {
        isContinuePress = FALSE;
        ContinuePressKey = 0;
        osal_stop_timerEx(MP_Function_TaskID, MP_FUNC_CONTINUESPRESS_TEST_EVENT);
    }

}

void menu_ChineseContinuesPressTestEnd(void)
{
    isContinuePress = FALSE;
    ContinuePressKey = 0;
}

