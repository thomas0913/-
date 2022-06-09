/*********************************************************************************************************/ /**
                                                                                                             * @file    I2C/10_bit_mode/main.c
                                                                                                             * @version $Rev:: 242          $
                                                                                                             * @date    $Date:: 2016-02-20 #$
                                                                                                             * @brief   Main program.
                                                                                                             *************************************************************************************************************
                                                                                                             * @attention
                                                                                                             *
                                                                                                             * Firmware Disclaimer Information
                                                                                                             *
                                                                                                             * 1. The customer hereby acknowledges and agrees that the program technical documentation, including the
                                                                                                             *    code, which is supplied by Holtek Semiconductor Inc., (hereinafter referred to as "HOLTEK") is the
                                                                                                             *    proprietary and confidential intellectual property of HOLTEK, and is protected by copyright law and
                                                                                                             *    other intellectual property laws.
                                                                                                             *
                                                                                                             * 2. The customer hereby acknowledges and agrees that the program technical documentation, including the
                                                                                                             *    code, is confidential information belonging to HOLTEK, and must not be disclosed to any third parties
                                                                                                             *    other than HOLTEK and the customer.
                                                                                                             *
                                                                                                             * 3. The program technical documentation, including the code, is provided "as is" and for customer reference
                                                                                                             *    only. After delivery by HOLTEK, the customer shall use the program technical documentation, including
                                                                                                             *    the code, at their own risk. HOLTEK disclaims any expressed, implied or statutory warranties, including
                                                                                                             *    the warranties of merchantability, satisfactory quality and fitness for a particular purpose.
                                                                                                             *
                                                                                                             * <h2><center>Copyright (C) Holtek Semiconductor Inc. All rights reserved</center></h2>
                                                                                                             ************************************************************************************************************/

/* Includes ------------------------------------------------------------------------------------------------*/
#include "ht32.h"
#include "ht32_board.h"
#include "ht32_board_config.h"
#include <inttypes.h>
#include <time.h>

/** @addtogroup HT32_Series_Peripheral_Examples HT32 Peripheral Examples
 * @{
 */

/** @addtogroup I2C_Examples I2C
 * @{
 */

/** @addtogroup I2C_10_bit_mode
 * @{
 */
// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define I2C_1062_E 0x04  // Enable bit
#define I2C_1062_RW 0x02 // Read/Write bit
#define I2C_1062_RS 0x01 // Register select bit

#define LCD_I2C_CH HT_I2C1
#define LCD_MAX_ROWS 2

u8 _backlightval;
u8 _displaycontrol;
u8 _displayfunction;

#define I2C_MASTER_ADDRESS 0x60
#define I2C_SLAVE_ADDRESS 0x3F // PCF8574A
//#define I2C_SLAVE_ADDRESS      0x27 //PCF8574
#define ClockSpeed 400000

/*MCTM*/
TM_TimeBaseInitTypeDef MCTM_TimeBaseInitStructure;
TM_OutputInitTypeDef MCTM_OutputInitStructure;
MCTM_CHBRKCTRInitTypeDef MCTM_CHBRKCTRInitStructure;
TM_OutputInitTypeDef GPTM_OutputInitStructure;
TM_TimeBaseInitTypeDef GPTM_TimeBaseInitStructure;

#define HTCFG_MCTM_RELOAD (48000000 / 2000)
#define HTCFG_MCTM_DEAD_TIME (72)

/*延時函數*/
void Delay_us(int cnt)
{
    int i;
    for (i = 0; i < cnt; i++)
        ;
}

/*藉由通訊協定把資料寫入從端之DDRAM*/
void I2C_Write(HT_I2C_TypeDef *I2Cx, u16 slave_address, u8 *buffer, u8 BufferSize)
{
    u8 Tx_Index = 0;

    /* Send I2C START & I2C slave address for write                                                           */
    I2C_TargetAddressConfig(I2Cx, slave_address, I2C_MASTER_WRITE);

    /* Check on Master Transmitter STA condition and clear it                                                 */
    while (!I2C_CheckStatus(I2Cx, I2C_MASTER_SEND_START))
        ;

    /* Check on Master Transmitter ADRS condition and clear it                                                */
    while (!I2C_CheckStatus(I2Cx, I2C_MASTER_TRANSMITTER_MODE))
        ;

    /* Send data                                                                                              */
    while (Tx_Index < BufferSize)
    {
        /* Check on Master Transmitter TXDE condition                                                           */
        while (!I2C_CheckStatus(I2Cx, I2C_MASTER_TX_EMPTY))
            ;
        /* Master Send I2C data                                                                                 */
        I2C_SendData(I2Cx, buffer[Tx_Index++]);
    }
    /* Send I2C STOP condition                                                                                */
    I2C_GenerateSTOP(I2Cx);
    /*wait for BUSBUSY become idle                                                                            */
    while (I2C_ReadRegister(I2Cx, I2C_REGISTER_SR) & 0x80000)
        ;
}

void I2C_Read(HT_I2C_TypeDef *I2Cx, u16 slave_address, u8 *buffer, u8 BufferSize)
{
    u8 Rx_Index = 0;

    /* Send I2C START & I2C slave address for read                                                            */
    I2C_TargetAddressConfig(I2Cx, slave_address, I2C_MASTER_READ);

    /* Check on Master Transmitter STA condition and clear it                                                 */
    while (!I2C_CheckStatus(I2Cx, I2C_MASTER_SEND_START))
        ;

    /* Check on Master Transmitter ADRS condition and clear it                                                */
    while (!I2C_CheckStatus(I2Cx, I2C_MASTER_RECEIVER_MODE))
        ;

    I2C_AckCmd(I2Cx, ENABLE);
    /* Send data                                                                                              */
    while (Rx_Index < BufferSize)
    {

        /* Check on Slave Receiver RXDNE condition                                                              */
        while (!I2C_CheckStatus(I2Cx, I2C_MASTER_RX_NOT_EMPTY))
            ;
        /* Store received data on I2C1                                                                          */
        buffer[Rx_Index++] = I2C_ReceiveData(I2Cx);
        if (Rx_Index == (BufferSize - 1))
        {
            I2C_AckCmd(I2Cx, DISABLE);
        }
    }
    /* Send I2C STOP condition                                                                                */
    I2C_GenerateSTOP(I2Cx);
    /*wait for BUSBUSY become idle                                                                            */
    while (I2C_ReadRegister(I2Cx, I2C_REGISTER_SR) & 0x80000)
        ;
}

/*印出字符的影像處理*/
void LCD_I2C_1602_4bit_Write(u8 data)
{
    data = data | _backlightval;

    I2C_Write(LCD_I2C_CH, I2C_SLAVE_ADDRESS, &data, 1);

    data = data | I2C_1062_E;
    I2C_Write(LCD_I2C_CH, I2C_SLAVE_ADDRESS, &data, 1);
    Delay_us(1000);

    data = data & ~I2C_1062_E; //~I2C_1062_E 為把I2C_1062_E位置取反
    I2C_Write(LCD_I2C_CH, I2C_SLAVE_ADDRESS, &data, 1);
    Delay_us(50000);
}

/*命令融合器*/
void LCD_command(u8 command)
{
    u8 high_4b = command & 0xF0;
    u8 low_4b = (command << 4) & 0xF0;

    LCD_I2C_1602_4bit_Write(high_4b);
    LCD_I2C_1602_4bit_Write(low_4b);
}

/*LCD 測試並初始化*/
void LCD_ini(void)
{
    _backlightval = LCD_BACKLIGHT;

    _displayfunction = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
    Delay_us(200000);

    LCD_I2C_1602_4bit_Write(0x30);
    Delay_us(500000);
    LCD_I2C_1602_4bit_Write(0x30);
    Delay_us(200000);
    LCD_I2C_1602_4bit_Write(0x30);
    Delay_us(200000);
    // printf("33");
    LCD_I2C_1602_4bit_Write(LCD_FUNCTIONSET | LCD_4BITMODE);

    LCD_command(LCD_FUNCTIONSET | _displayfunction);

    _displaycontrol = LCD_DISPLAYOFF | LCD_CURSOROFF | LCD_BLINKOFF;
    LCD_command(LCD_DISPLAYCONTROL | _displaycontrol);

    LCD_command(LCD_CLEARDISPLAY);
    Delay_us(200000);

    LCD_command(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);

    LCD_command(LCD_RETURNHOME);
    Delay_us(200000);

    _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    LCD_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void LCD_Backlight(u8 enable)
{
    u8 data = 0;
    if (enable)
        _backlightval = LCD_BACKLIGHT;
    else
        _backlightval = LCD_NOBACKLIGHT;
    data = _backlightval;
    I2C_Write(LCD_I2C_CH, I2C_SLAVE_ADDRESS, &data, 1);
}

/*設定通訊協定*/ //也需設定CKCU、AFIO
void I2C_Configuration(/*HT_I2C_TypeDef* I2Cx*/)
{
    /* I2C Master configuration                                                                               */

    /*宣告USART_InitTypeDef型態變數*/
    I2C_InitTypeDef I2C_InitStructure;

    /*設定*/
    I2C_InitStructure.I2C_GeneralCall = DISABLE;
    I2C_InitStructure.I2C_Acknowledge = DISABLE;

    /*設定字長*/
    I2C_InitStructure.I2C_AddressingMode = I2C_ADDRESSING_7BIT; // I2C_ADDRESSING_7BIT
    /*設定校驗位*/
    I2C_InitStructure.I2C_OwnAddress = I2C_MASTER_ADDRESS; //自訂
                                                           /*設定模式*/
    I2C_InitStructure.I2C_Speed = ClockSpeed;              //自訂
                                                           /*依上述設定初始化*/
    I2C_Init(HT_I2C1, &I2C_InitStructure);                 //初始化SDA通道

    I2C_InitStructure.I2C_Acknowledge = DISABLE;
    I2C_InitStructure.I2C_OwnAddress = I2C_SLAVE_ADDRESS;
    // I2C_InitStructure.I2C_Speed = ClockSpeed;
    I2C_Init(HT_I2C0, &I2C_InitStructure); //初始化SCL通道

    /* Enable I2C                                                                                             */
    I2C_Cmd(HT_I2C0, ENABLE);
    I2C_Cmd(HT_I2C1, ENABLE);
}

/* Global functions ----------------------------------------------------------------------------------------*/
/*********************************************************************************************************/ /**
                                                                                                             * @brief  Main program.
                                                                                                             * @retval None
                                                                                                             ***********************************************************************************************************/
/*游標座標算法*/
void LCD_setCursor(uint8_t col, uint8_t row)
{
    int row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    if (row > LCD_MAX_ROWS)
    {                           // ROW為奇數倍顯示於第一行，偶數倍顯示於第二行
        row = LCD_MAX_ROWS - 1; // we count rows starting w/0
    }
    LCD_command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

/*LCD printer*/
void LCD_Write(u8 Data)
{ //收到資料並顯示於LCD上
    u8 high_4b = (Data & 0xF0) | I2C_1062_RS;
    u8 low_4b = ((Data << 4) & 0xF0) | I2C_1062_RS;

    LCD_I2C_1602_4bit_Write(high_4b);
    LCD_I2C_1602_4bit_Write(low_4b);
}

//更新並反白
void LCD_ResetPrint(void)
{
    LCD_setCursor(0, 0);
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    Delay_us(1000000 / 2);

    LCD_setCursor(0, 1);
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    LCD_Write(' ');
    Delay_us(1000000 / 2);
}

void CKCU_Configuration(void)
{
    //宣告時脈閘設定用型態變數，變數名CKCUClock，且其內預設值為0
    CKCU_PeripClockConfig_TypeDef CKCUClock = {{0}};
    CKCUClock.Bit.PDMA = 0;
    CKCUClock.Bit.USBD = 0;
    CKCUClock.Bit.CKREF = 0;
    CKCUClock.Bit.EBI = 0;
    CKCUClock.Bit.CRC = 0;
    CKCUClock.Bit.PA = 1; // AHBCCR register (PxEN (x = A,B,C,D))
    CKCUClock.Bit.PB = 1;
    CKCUClock.Bit.PC = 1;
    CKCUClock.Bit.PD = 1;
    CKCUClock.Bit.I2C0 = 1; // SCL時脈
    CKCUClock.Bit.I2C1 = 1; // SDA時脈
    CKCUClock.Bit.SPI0 = 0;
    CKCUClock.Bit.SPI1 = 0;
    CKCUClock.Bit.USART0 = 0;
    CKCUClock.Bit.USART1 = 1;
    CKCUClock.Bit.UART0 = 0;
    CKCUClock.Bit.UART1 = 0;
    CKCUClock.Bit.AFIO = 1; //複用功能輸入/輸出控制單元(Alternate Function Input/Output Control Unit)
    CKCUClock.Bit.EXTI = 0;
    CKCUClock.Bit.SCI0 = 0;
    CKCUClock.Bit.SCI1 = 0;
    CKCUClock.Bit.I2S = 0;
    CKCUClock.Bit.MCTM0 = 0;
    CKCUClock.Bit.WDT = 0;
    CKCUClock.Bit.BKP = 0;
    CKCUClock.Bit.GPTM0 = 1;
    CKCUClock.Bit.GPTM1 = 0;
    CKCUClock.Bit.BFTM0 = 1; //計數器
    CKCUClock.Bit.BFTM1 = 0;
    CKCUClock.Bit.CMP = 0;
    CKCUClock.Bit.ADC = 1;
    CKCUClock.Bit.SCTM0 = 0;
    CKCUClock.Bit.SCTM1 = 0;
    CKCUClock.Bit.SCTM2 = 0;
    CKCUClock.Bit.SCTM3 = 0;
    //對設立旗標的功能之時脈閘進行更動
    CKCU_PeripClockConfig(CKCUClock, ENABLE);
}

void GPIO_Configuration()
{
    /*LCD螢幕*/
    AFIO_GPxConfig(GPIO_PA, AFIO_PIN_0 | AFIO_PIN_1, AFIO_FUN_I2C);
    // PA GPIO_PIN_0:I2C1_SCL =>提供時脈
    // PA GPIO_PIN_1:I2C1_SDA =>控制從端使其接收/回傳資料

    /*三色LED燈*/
    AFIO_GPxConfig(GPIO_PC, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8, AFIO_FUN_MCTM_GPTM);

    /*LED燈for錯誤警告*/
    AFIO_GPxConfig(GPIO_PC, AFIO_PIN_1, AFIO_FUN_GPIO); // led                                         */
    GPIO_DirectionConfig(HT_GPIOC, GPIO_PIN_1, GPIO_DIR_OUT);

    /*BUTTON*/
    /* 設定映射到腳位上之功能                                                                      */
    AFIO_GPxConfig(GPIO_PB, AFIO_PIN_12, AFIO_FUN_GPIO);
    AFIO_GPxConfig(GPIO_PB, AFIO_PIN_7, AFIO_FUN_GPIO);
    AFIO_GPxConfig(GPIO_PB, AFIO_PIN_5, AFIO_FUN_GPIO);
    AFIO_GPxConfig(GPIO_PB, AFIO_PIN_3, AFIO_FUN_GPIO);
    /* 設定腳位I/O方向                                                          */
    GPIO_DirectionConfig(HT_GPIOB, GPIO_PIN_12, GPIO_DIR_IN);
    GPIO_DirectionConfig(HT_GPIOB, GPIO_PIN_7, GPIO_DIR_IN);
    GPIO_DirectionConfig(HT_GPIOB, GPIO_PIN_5, GPIO_DIR_IN);
    GPIO_DirectionConfig(HT_GPIOB, GPIO_PIN_3, GPIO_DIR_IN);
    /* 設定腳位是否開啟Input功能                                                           */
    GPIO_InputConfig(HT_GPIOB, GPIO_PIN_12, ENABLE);
    GPIO_InputConfig(HT_GPIOB, GPIO_PIN_7, ENABLE);
    GPIO_InputConfig(HT_GPIOB, GPIO_PIN_5, ENABLE);
    GPIO_InputConfig(HT_GPIOB, GPIO_PIN_3, ENABLE);

    /*光敏電阻*/
    /* Configure MCTM Channel 0 output pin                                                                    */
    // AFIO_GPxConfig(GPIO_PC, AFIO_PIN_1, AFIO_FUN_MCTM_GPTM);//LED接腳
    /* Configure MCTM Break pin                                                                               */
    // AFIO_GPxConfig(GPIO_PB, AFIO_PIN_4, AFIO_FUN_MCTM_GPTM);
    /* Config AFIO mode as ADC function                                                                       */
    AFIO_GPxConfig(GPIO_PA, AFIO_PIN_6, AFIO_FUN_ADC);

    /*紅外線模組*/
    /*設定映射到腳位上之功能*/
    AFIO_GPxConfig(GPIO_PA, AFIO_PIN_2, AFIO_FUN_GPIO);
    /*GPIO設定*/
    GPIO_DirectionConfig(HT_GPIOA, GPIO_PIN_2, GPIO_DIR_IN);        //設定腳位I/O方向
    GPIO_PullResistorConfig(HT_GPIOA, GPIO_PIN_2, GPIO_PR_DISABLE); //設定腳位是否加上提升/接地電阻
    GPIO_InputConfig(HT_GPIOA, GPIO_PIN_2, ENABLE);                 //設定腳位是否開啟Input功能
}

u16 CRR = 255;

// GPTM
void GPTM_Time_Configuration(void)
{ //設定計數器基本設定
    //宣告GPTM_InitTypeDef型態變數
    TM_TimeBaseInitTypeDef GPTM_TimeBaseInitStructure;
    //設定重載值(計數上限/週期)
    GPTM_TimeBaseInitStructure.CounterReload = CRR - 1;
    //設定除頻器大小
    GPTM_TimeBaseInitStructure.Prescaler = 1 - 1;
    //設定計數模式
    GPTM_TimeBaseInitStructure.CounterMode = TM_CNT_MODE_UP;
    //設定是否初始化計數器
    GPTM_TimeBaseInitStructure.PSCReloadTime = TM_PSC_RLD_IMMEDIATE;
    //依上述設定初始化
    TM_TimeBaseInit(HT_GPTM0, &GPTM_TimeBaseInitStructure);
}

// GPTM Outpute Configuration
void GPTM_Output_Configuration(void)
{
    //宣告GPTM_InitTypeDef型態變數
    TM_OutputInitTypeDef GPTM_OutputInitStructure;

    //  u16 CH0CCR = CRR * 0.75;
    //  u16 CH1CCR = CRR * 0.5;
    //  u16 CH2CCR = CRR * 0.25;

    u16 CH0CCR = CRR;
    u16 CH1CCR = CRR;
    u16 CH2CCR = CRR;

    //選擇設定的Channel
    GPTM_OutputInitStructure.Channel = TM_CH_0;
    //設定輸出模式
    GPTM_OutputInitStructure.OutputMode = TM_OM_PWM1;
    //設定通道Enable
    GPTM_OutputInitStructure.Control = TM_CHCTL_ENABLE;
    //設定極性(電位反向)
    GPTM_OutputInitStructure.Polarity = TM_CHP_NONINVERTED;
    //設定比較值 [比較值(正常模式)] / [上數比較值(非對稱模式)]
    GPTM_OutputInitStructure.Compare = CH0CCR - 1;
    //設定下數比較值(非對稱模式)
    GPTM_OutputInitStructure.AsymmetricCompare = 0;
    //依上述設定初始化
    TM_OutputInit(HT_GPTM0, &GPTM_OutputInitStructure);

    GPTM_OutputInitStructure.Channel = TM_CH_1;
    GPTM_OutputInitStructure.Compare = CH1CCR - 1;
    GPTM_OutputInitStructure.AsymmetricCompare = 0;
    TM_OutputInit(HT_GPTM0, &GPTM_OutputInitStructure);

    GPTM_OutputInitStructure.Channel = TM_CH_2;
    GPTM_OutputInitStructure.Compare = CH2CCR - 1;
    GPTM_OutputInitStructure.AsymmetricCompare = 0;
    TM_OutputInit(HT_GPTM0, &GPTM_OutputInitStructure);

    TM_Cmd(HT_GPTM0, ENABLE);
}

void GPTM_PWM1_Output(HT_TM_TypeDef *HT_GPTMx, TM_CH_Enum TM_CH_x, u16 Compare)
{
    TM_OutputInitTypeDef GPTM_OutputInitStructure;
    GPTM_OutputInitStructure.Channel = TM_CH_x;
    GPTM_OutputInitStructure.OutputMode = TM_OM_PWM1;
    GPTM_OutputInitStructure.Control = TM_CHCTL_ENABLE;
    GPTM_OutputInitStructure.Polarity = TM_CHP_NONINVERTED;
    GPTM_OutputInitStructure.Compare = Compare;
    TM_OutputInit(HT_GPTMx, &GPTM_OutputInitStructure);
}

static void _delay(u32 nCount)
{
    vu32 i;
    for (i = 0; i < 10000 * nCount; i++)
        ;
}

void BFTM_Configuration_0(float Time_s, ControlStatus able)
{ /*32-bit 向上計數型計數設定*/
    /*將BFTM設為每(Time_s)秒為一週期
          SystemCoreClock為SDK內建參數,AHB時鐘跑1秒的計數量*/

    /*宣告Compare值*/
    u32 Compare = SystemCoreClock * Time_s;

    /*設定比較值*/
    BFTM_SetCompare(HT_BFTM0, Compare);
    /*設定計數次數*/
    BFTM_SetCounter(HT_BFTM0, 0);           //歸零
                                            /*設定是否啟用單次模式*/
    BFTM_OneShotModeCmd(HT_BFTM0, DISABLE); //重複模式：計數器在比較匹配發生時重啟
                                            /*設定BFTM中斷源開啟*/
    BFTM_IntConfig(HT_BFTM0, ENABLE);
    /*BFTM開啟*/
    BFTM_EnaCmd(HT_BFTM0, able);
}

void USART_configuration(void)
{
    USART_InitTypeDef USART_InitStructure;
    /* USART0 configuration ----------------------------------------------------------------------------------*/
    /* USART0 configured as follow:
          - BaudRate = 115200 baud
          - Word Length = 8 Bits
          - One Stop Bit
          - None parity bit
    */
    USART_InitStructure.USART_BaudRate = 115200;                //設定鮑值為  115200
    USART_InitStructure.USART_WordLength = USART_WORDLENGTH_8B; //設定字長    8bits
    USART_InitStructure.USART_StopBits = USART_STOPBITS_1;      //設定停止位  1bits
    USART_InitStructure.USART_Parity = USART_PARITY_NO;         //設定校驗位  no
    USART_InitStructure.USART_Mode = USART_MODE_NORMAL;         //設定模式    正常

    USART_Init(COM1_PORT, &USART_InitStructure);
    USART_TxCmd(COM1_PORT, ENABLE);
    USART_RxCmd(COM1_PORT, ENABLE);
}
void ADC_configuration(void)
{
    //設定轉換模式及Cycle、 Subgroup大小(1~8)
    ADC_RegularGroupConfig(HT_ADC, CONTINUOUS_MODE, 1, 1);
    /* Continuous Mode, Length 1, SubLength 1                                                                 */

    //設定轉換順序(不支援多組同時設定)
    ADC_RegularChannelConfig(HT_ADC, ADC_CH_6 /*ADC的輸入數位訊號源*/, 0 /*為其排序(0~7)*/);
    /* ADC Channel n, Rank 0,
    Sampling clock is (1.5 + 0) ADCLK
  Conversion time = (sampling clock + 12.5) / ADCLK = 12.4 uS */

    //設定觸發事件源
    ADC_RegularTrigConfig(HT_ADC, ADC_TRIG_SOFTWARE);
    /* Use Software Trigger as ADC trigger source                                                             */

    //設定ADC Enable
    ADC_Cmd(HT_ADC, ENABLE);

    //手動設定給予觸發訊號(Software)
    ADC_SoftwareStartConvCmd(HT_ADC, ENABLE);
    /* Software trigger to start continuous mode                                                              */
}

void MCTM_Configuration(void)
{
    /* MCTM Time Base configuration                                                                            */

    //設定重載值(計數上限/週期)
    MCTM_TimeBaseInitStructure.CounterReload = HTCFG_MCTM_RELOAD - 1 /*16bits暫存器*/; //
                                                                                       /* HTCFG_MCTM_RELOAD為48000000/2000，
                                                                                         48000000(48MHz)為系統工作頻率，
                                                                                         即為每24000個clock(48000000/2000)重載計數器，
                                                                                         因為從0開始計數，因此需要減1 */

    //設定除頻器大小
    MCTM_TimeBaseInitStructure.Prescaler = 0 /*16bits暫存器*/;
    /*Prescaler為除頻器，
    可以延長MCTM計數週期，
    若設定為2000，
    搭配CounterReload設定為48000000/2000可讓週期為1秒 */

    //設定更新速率
    MCTM_TimeBaseInitStructure.RepetitionCounter = 0 /*8bits暫存器*/;
    //設定計數模式
    MCTM_TimeBaseInitStructure.CounterMode = TM_CNT_MODE_UP;
    /*CounterMode設定TM_CNT_MODE_UP，
    為從0計數至上限值；
    反之TM_CNT_MODE_DOWN為從上限值計數至0 */

    //設定是否初始化計數器
    MCTM_TimeBaseInitStructure.PSCReloadTime = TM_PSC_RLD_IMMEDIATE /*每次or偶爾*/;
    //最後依上述設定初始化
    TM_TimeBaseInit(HT_MCTM0, &MCTM_TimeBaseInitStructure);

    /* MCTM Channel 0 output configuration                                                                    */
    //選擇設定的Channel
    MCTM_OutputInitStructure.Channel = TM_CH_0;
    //設定比較值
    MCTM_OutputInitStructure.Compare = HTCFG_MCTM_RELOAD * 1 / 2 /*16bits暫存器*/;
    /*當計數至12000(HTCFG_MCTM_RELOAD*1/2)後啟動，
      打開LED，此為暗0.5秒，亮0.5秒的閃爍LED燈，
      在主函式中會覆蓋設定改為呼吸燈 */

    //設定輸出模式
    MCTM_OutputInitStructure.OutputMode = TM_OM_PWM1;
    //設定通道Enable
    MCTM_OutputInitStructure.Control = TM_CHCTL_ENABLE;
    MCTM_OutputInitStructure.ControlN = TM_CHCTL_ENABLE;
    //設定極性(電位反向)
    MCTM_OutputInitStructure.Polarity = TM_CHP_NONINVERTED;
    MCTM_OutputInitStructure.PolarityN = TM_CHP_NONINVERTED;
    //設定暫停時空閒狀態下之輸出
    MCTM_OutputInitStructure.IdleState = MCTM_OIS_LOW;
    MCTM_OutputInitStructure.IdleStateN = MCTM_OIS_HIGH;
    //最後依上述設定初始化
    TM_OutputInit(HT_MCTM0, &MCTM_OutputInitStructure);

    /* MCTM Off State, lock, Break, Automatic Output enable, dead time configuration                          */
    //正常運行狀態下通道關閉狀態選擇
    MCTM_CHBRKCTRInitStructure.OSSRState = MCTM_OSSR_STATE_ENABLE;
    //空閒模式下通道關閉狀態選擇
    MCTM_CHBRKCTRInitStructure.OSSIState = MCTM_OSSI_STATE_ENABLE;
    //鎖定級別設置
    MCTM_CHBRKCTRInitStructure.LockLevel = MCTM_LOCK_LEVEL_2;
    //暫停信號使能位
    MCTM_CHBRKCTRInitStructure.Break0 = MCTM_BREAK_ENABLE;
    //暫停信號輸入極性位
    MCTM_CHBRKCTRInitStructure.Break0Polarity = MCTM_BREAK_POLARITY_LOW;
    //通道自動輸出使能位
    MCTM_CHBRKCTRInitStructure.AutomaticOutput = MCTM_CHAOE_ENABLE;
    //通道死區時間週期定義
    MCTM_CHBRKCTRInitStructure.DeadTime = HTCFG_MCTM_DEAD_TIME /*8bits暫存器*/;
    //暫停信號輸入濾波器設置
    MCTM_CHBRKCTRInitStructure.BreakFilter = 0 /*4bits暫存器*/;
    //依上述設定初始化
    MCTM_CHBRKCTRConfig(HT_MCTM0, &MCTM_CHBRKCTRInitStructure);

    /* MCTM counter enable                                                                                    */
    TM_Cmd(HT_MCTM0, ENABLE);

    /* MCTM通道輸出enable                                                                        */
    MCTM_CHMOECmd(HT_MCTM0, ENABLE);
}

void NVIC_Configuration(void)
{ // set of interrupt
    /*啟用BFTM中斷功能*/
    NVIC_EnableIRQ(BFTM0_IRQn);
}

uint8_t count = 0;
uint32_t data;
vu32 flag_state;

//紅外線接收解碼器
uint32_t receive_data(void)
{
    uint32_t code = 0;
    int i = 0;

    while (!GPIO_ReadInBit(HT_GPIOA, GPIO_PIN_2))
        ;
    while (GPIO_ReadInBit(HT_GPIOA, GPIO_PIN_2))
        ;

    for (i = 0; i < 32; i++)
    {
        flag_state = FALSE;
        count = 0;
        while (!GPIO_ReadInBit(HT_GPIOA, GPIO_PIN_2))
            ;
        while (GPIO_ReadInBit(HT_GPIOA, GPIO_PIN_2))
        {
            count++;
            BFTM_Configuration_0(0.0001, ENABLE);
            while (!flag_state)
                ;

            BFTM_Configuration_0(1, DISABLE);
            flag_state = FALSE;
        }

        if (count > 12) // if the space is more than 1.2 ms
        {
            code |= (1UL << (31 - i)); // write 1
        }
        else
        {
            code &= ~(1UL << (31 - i)); // write 0
        }
    }
    return code;
}

//紅外線選擇器
void convert_code(uint32_t code)
{
    switch (code)
    {
    case (0xFFA25D): // 0000 0000 1000
        printf("CH-\r\n");
        break;

    case (0xFF629D):
        printf("CH\r\n");
        break;

    case (0xFFE21D):
        printf("CH+\r\n");
        break;

    case (0xFF22DD):
        printf("PREV\r\n");
        break;

    case (0xFF02FD):
        printf("NEXT\r\n");
        break;

    case (0xFFC23D):
        printf("PLAY/PAUSE\r\n");
        break;

    case (0xFFE01F):
        printf("-\r\n");
        break;

    case (0xFFA857):
        printf("+\r\n");
        break;

    case (0xFF906F):
        printf("EQ\r\n");
        break;

    case (0XFF6897):
        printf("0\r\n");
        break;

    case (0xFF9867):
        printf("100+\r\n");
        break;

    case (0xFFB04F):
        printf("200+\r\n");
        break;

    case (0xFF30CF): // 0xFF30CF
        printf("1\r\n");
        break;

    case (0xFF18E7):
        printf("2\r\n");
        break;

    case (0xFF7A85):
        printf("3\r\n");
        break;

    case (0xFF10EF):
        printf("4\r\n");
        break;

    case (0xFF38C7):
        printf("5\r\n");
        break;

    case (0xFF5AA5):
        printf("6\r\n");
        break;

    case (0xFF42BD):
        printf("7\r\n");
        break;

    case (0xFF4AB5):
        printf("8\r\n");
        break;

    case (0xFF52AD):
        printf("9\r\n");
        break;

    default:
        break;
    }
}

#include <ctype.h>
void itoa(int n, char s[])//整數轉字元陣列
{
    int i, j, sign;
    if ((sign = n) < 0)
    {           //記錄符號
        n = -n; //使n成為正數
    }
    i = 0;
    do
    {
        s[i++] = n % 10 + '0'; //取下一個數字
    } while ((n /= 10) > 0);   //刪除該數字
    if (sign < 0)
    {
        s[i++] = '-';
    }
    s[i] = '\0';
    //生成的數字是逆序的，所以要逆序輸出
    char reg[5];
    int u, v;
    for (u = 0; u <= (i - 1); u++)
    {
        reg[u] = s[(i - 1) - u];
    }
    for (v = 0; v <= (i - 1); v++)
    {
        s[v] = reg[v];
    }
    /*
    for(j=i;j>=0;j--){
        printf("%c",s[j]);
    }
    */
}

int main(void)
{
    // u8 i,n;//u8型態等同char型態
    // u8 rlt;
    u64 Data = 0;

    int R = 13;
    int G = 191;
    int B = 140;

    // NVIC_Configuration();
    // BFTM_Configuration_0(1, DISABLE);

    CKCU_Configuration();      //時脈致能開啟
    GPIO_Configuration();      //腳位功能開啟
    ADC_configuration();       //電壓偵測開啟
    GPTM_Time_Configuration(); //三色LED開啟
    GPTM_Output_Configuration();
    I2C_Configuration(); // LCD螢幕時脈開啟
    RETARGET_Configuration();

    LCD_ini();

    while (1)
    {
        char mode;                 //模式狀態
        int colorcnt = 1;          //顏色選擇器
        bool buttonDown_1 = false; //按鈕放開狀態
        bool buttonDown_2 = false;
        bool buttonDown_3 = false;
        bool buttonDown_4 = false;
        int input_1;                                     //按鈕一
        int input_2;                                     //按鈕二
        int input_3;                                     //按鈕三
        int input_4;                                     //按鈕四
        input_1 = GPIO_ReadInBit(HT_GPIOB, GPIO_PIN_12); //模式一: breath mode呼吸燈
        input_2 = GPIO_ReadInBit(HT_GPIOB, GPIO_PIN_7);  //模式二: flash mode閃爍燈
        input_3 = GPIO_ReadInBit(HT_GPIOB, GPIO_PIN_5);  //模式三: blin mode閃一下燈
        input_4 = GPIO_ReadInBit(HT_GPIOB, GPIO_PIN_3);  // Preset mode預設模式

        while (input_1 == 1 && buttonDown_1 == false)
        { //模式一:RGB呼吸燈
            LCD_ResetPrint();
            LCD_setCursor(0, 0); //設定游標座標
            LCD_Write('<');
            LCD_Write('<');
            LCD_Write('B'); //設定顯示字符
            LCD_Write('R');
            LCD_Write('E');
            LCD_Write('A');
            LCD_Write('T');
            LCD_Write('H');
            LCD_Write('-');
            LCD_Write('-');
            LCD_Write('M');
            LCD_Write('O');
            LCD_Write('D');
            LCD_Write('E');
            LCD_Write('>');
            LCD_Write('>');
            Delay_us(100000);

            // ADC
            float volt = 0.0;
            float V_min = (4.42 / 4095); //單位:V伏特
            float resister = 0.0;
            float R_max = (830000.0 / 4095); //單位:omega歐姆
            Data = ADC_GetConversionData(HT_ADC, 0);
            volt = Data * V_min;
            resister = Data * R_max;
            // printf("MODE_1: ADC_Data = %llu\r\n",Data);
            // printf("MODE_1: ADC_Data = %llu ; Volt = %f (Volt)\r\n",Data,volt);
            // printf("MODE_1: ADC_Data = %llu ; resister = %f (omega)\r\n",Data,resister);
            // printf("=================================================\r\n");

            //碳足跡(P = V^2 / R)
            float power = 0.0;
            power = ((volt * volt) / resister) * 1000; //單位:kW*hour千瓦小時(度)
            printf("MODE_1: Power = %f (Wh)\r\n", power);
            float Carbon_Footprint = 0.0;
            Carbon_Footprint = power * 0.502; //碳足跡(kgCO2e)=活動強度數據(kWh)*民國109年排放係數(kgCO2e/kWh)
            printf("MODE_1: Carbon Footprint = %f (kgCO2e)\r\n", Carbon_Footprint);
            printf("=================================================\r\n");

            LCD_setCursor(0, 1); //設定游標座標
            LCD_Write(' ');
            LCD_Write('R');
            LCD_Write('G');
            LCD_Write('B');
            LCD_Write(' ');
            LCD_Write(' ');
            LCD_Write('B');
            LCD_Write('r');
            LCD_Write('e');
            LCD_Write('a');
            LCD_Write('t');
            LCD_Write('h');
            LCD_Write('i');
            LCD_Write('n');
            LCD_Write('g');
            LCD_Write(' ');
            Delay_us(100000);

            /*
            char carbon_data[8];
            sprintf(carbon_data, "%f",Carbon_Footprint);
            LCD_setCursor(0, 1); //設定游標座標
            LCD_Write(carbon_data[0]);      //設定顯示字符
            LCD_Write(carbon_data[1]);      //上限16*char
            LCD_Write(carbon_data[2]);
            LCD_Write(carbon_data[3]);
            LCD_Write(carbon_data[4]);
            LCD_Write(carbon_data[5]);
            LCD_Write(carbon_data[6]);
            LCD_Write(carbon_data[7]);
            LCD_Write('(');
            LCD_Write('k');
            LCD_Write('g');
            LCD_Write('C');
            LCD_Write('O');
            LCD_Write('2');
            LCD_Write('e');
            LCD_Write(')');
            Delay_us(100000);
            */

            // LED
            CRR = 100;
            TM_TimeBaseInitTypeDef GPTM_TimeBaseInitStructure;
            GPTM_TimeBaseInitStructure.CounterReload = CRR - 1;
            GPTM_TimeBaseInitStructure.Prescaler = 1 - 1;
            GPTM_TimeBaseInitStructure.CounterMode = TM_CNT_MODE_UP;
            GPTM_TimeBaseInitStructure.PSCReloadTime = TM_PSC_RLD_IMMEDIATE;
            TM_TimeBaseInit(HT_GPTM0, &GPTM_TimeBaseInitStructure);
            u16 CH0CCR = CRR * 0.75;
            u16 CH1CCR = CRR * 0.5;
            u16 CH2CCR = CRR * 0.25;
            GPTM_OutputInitStructure.Channel = TM_CH_0;
            GPTM_OutputInitStructure.OutputMode = TM_OM_PWM1;
            GPTM_OutputInitStructure.Control = TM_CHCTL_ENABLE;
            GPTM_OutputInitStructure.Polarity = TM_CHP_NONINVERTED;
            GPTM_OutputInitStructure.Compare = CH0CCR - 1;
            GPTM_OutputInitStructure.AsymmetricCompare = 0;
            TM_OutputInit(HT_GPTM0, &GPTM_OutputInitStructure);
            GPTM_OutputInitStructure.Channel = TM_CH_1;
            GPTM_OutputInitStructure.Compare = CH1CCR - 1;
            GPTM_OutputInitStructure.AsymmetricCompare = 0;
            TM_OutputInit(HT_GPTM0, &GPTM_OutputInitStructure);
            GPTM_OutputInitStructure.Channel = TM_CH_2;
            GPTM_OutputInitStructure.Compare = CH2CCR - 1;
            GPTM_OutputInitStructure.AsymmetricCompare = 0;
            TM_OutputInit(HT_GPTM0, &GPTM_OutputInitStructure);
            TM_Cmd(HT_GPTM0, ENABLE);

            if (colorcnt > 3)
            {
                colorcnt = 1;
            }
            if (colorcnt == 1)
            {
                for (int i = 100; i >= 10; i--)
                {
                    GPTM_PWM1_Output(HT_GPTM0, TM_CH_0, (i - 5) % 101);
                    GPTM_PWM1_Output(HT_GPTM0, TM_CH_1, 100);
                    GPTM_PWM1_Output(HT_GPTM0, TM_CH_2, 100);
                    _delay(15 / 2);
                }
                for (int i = 0; i <= 95; i++)
                {
                    GPTM_PWM1_Output(HT_GPTM0, TM_CH_0, (i + 5) % 101);
                    GPTM_PWM1_Output(HT_GPTM0, TM_CH_1, 100);
                    GPTM_PWM1_Output(HT_GPTM0, TM_CH_2, 100);
                    _delay(15 / 2);
                }
            }
            else if (colorcnt == 2)
            {
                for (int i = 100; i >= 10; i--)
                {
                    GPTM_PWM1_Output(HT_GPTM0, TM_CH_0, 100);
                    GPTM_PWM1_Output(HT_GPTM0, TM_CH_1, (i - 5) % 101);
                    GPTM_PWM1_Output(HT_GPTM0, TM_CH_2, 100);
                    _delay(15 / 2);
                }
                for (int i = 0; i <= 95; i++)
                {
                    GPTM_PWM1_Output(HT_GPTM0, TM_CH_0, 100);
                    GPTM_PWM1_Output(HT_GPTM0, TM_CH_1, (i + 5) % 101);
                    GPTM_PWM1_Output(HT_GPTM0, TM_CH_2, 100);
                    _delay(15 / 2);
                }
            }
            else if (colorcnt == 3)
            {
                for (int i = 100; i >= 10; i--)
                {
                    GPTM_PWM1_Output(HT_GPTM0, TM_CH_0, 100);
                    GPTM_PWM1_Output(HT_GPTM0, TM_CH_1, 100);
                    GPTM_PWM1_Output(HT_GPTM0, TM_CH_2, (i - 5) % 101);
                    _delay(15 / 2);
                }
                for (int i = 0; i <= 95; i++)
                {
                    GPTM_PWM1_Output(HT_GPTM0, TM_CH_0, 100);
                    GPTM_PWM1_Output(HT_GPTM0, TM_CH_1, 100);
                    GPTM_PWM1_Output(HT_GPTM0, TM_CH_2, (i + 5) % 101);
                    _delay(15 / 2);
                }
            }
            colorcnt++;

            // Button for 單觸式
            // input_1 = GPIO_ReadInBit(HT_GPIOB, GPIO_PIN_12);
            input_2 = GPIO_ReadInBit(HT_GPIOB, GPIO_PIN_7);
            input_3 = GPIO_ReadInBit(HT_GPIOB, GPIO_PIN_5);
            input_4 = GPIO_ReadInBit(HT_GPIOB, GPIO_PIN_3);
            if (input_2 == 1 || input_3 == 1 || input_4 == 1)
            {
                buttonDown_1 = true;
            }
            input_1 = 1;
        }

        while (input_2 == 1 && buttonDown_2 == false)
        { //模式二:急促快閃燈
            LCD_ResetPrint();
            LCD_setCursor(0, 0); //設定游標座標
            LCD_Write('<');
            LCD_Write('<');
            LCD_Write('<');
            LCD_Write('F'); //設定顯示字符
            LCD_Write('L');
            LCD_Write('A');
            LCD_Write('S');
            LCD_Write('H');
            LCD_Write('-');
            LCD_Write('M');
            LCD_Write('O');
            LCD_Write('D');
            LCD_Write('E');
            LCD_Write('>');
            LCD_Write('>');
            LCD_Write('>');
            Delay_us(1000000 * 2);

            // ADC
            float volt = 0.0;
            float V_min = (4.42 / 4095); //單位:V伏特
            float resister = 0.0;
            float R_max = (830000.0 / 4095); //單位:omega歐姆
            Data = ADC_GetConversionData(HT_ADC, 0);
            volt = Data * V_min;
            resister = Data * R_max;
            // printf("MODE_2: ADC_Data = %llu\r\n",Data);
            // printf("MODE_2: ADC_Data = %llu ; Volt = %f (Volt)\r\n",Data,volt);
            // printf("MODE_2: ADC_Data = %llu ; resister = %f (omega)\r\n",Data,resister);
            // printf("=================================================\r\n");

            //碳足跡(P = V^2 / R)
            float power = 0.0;
            power = ((volt * volt) / resister) * 1000; //單位:kW*hour千瓦小時(度)
            printf("MODE_2: Power = %f (Wh)\r\n", power);
            float Carbon_Footprint = 0.0;
            Carbon_Footprint = power * 0.502; //碳足跡(kgCO2e)=活動強度數據(kWh)*民國109年排放係數(kgCO2e/kWh)
            printf("MODE_2: Carbon Footprint = %f (kgCO2e)\r\n", Carbon_Footprint);
            printf("=================================================\r\n");

            LCD_setCursor(0, 1); //設定游標座標
            LCD_Write('F');
            LCD_Write('l');
            LCD_Write('a');
            LCD_Write('s');
            LCD_Write('h');
            LCD_Write('i');
            LCD_Write('n');
            LCD_Write('g');
            LCD_Write(' ');
            LCD_Write('R');
            LCD_Write('a');
            LCD_Write('p');
            LCD_Write('i');
            LCD_Write('d');
            LCD_Write('l');
            LCD_Write('y');
            Delay_us(1000000 * 2);

            // Carbon Footprint
            /*
            char carbon_data[8];
            sprintf(carbon_data, "%f",Carbon_Footprint);
            LCD_setCursor(0, 1); //設定游標座標
            LCD_Write(carbon_data[0]);      //設定顯示字符
            LCD_Write(carbon_data[1]);      //上限16*char
            LCD_Write(carbon_data[2]);
            LCD_Write(carbon_data[3]);
            LCD_Write(carbon_data[4]);
            LCD_Write(carbon_data[5]);
            LCD_Write(carbon_data[6]);
            LCD_Write(carbon_data[7]);
            LCD_Write('(');
            LCD_Write('k');
            LCD_Write('g');
            LCD_Write('C');
            LCD_Write('O');
            LCD_Write('2');
            LCD_Write('e');
            LCD_Write(')');
            */
            Delay_us(1000000 * 2);

            // LED
            CRR = 100;
            TM_TimeBaseInitTypeDef GPTM_TimeBaseInitStructure;
            GPTM_TimeBaseInitStructure.CounterReload = CRR - 1;
            GPTM_TimeBaseInitStructure.Prescaler = 1 - 1;
            GPTM_TimeBaseInitStructure.CounterMode = TM_CNT_MODE_UP;
            GPTM_TimeBaseInitStructure.PSCReloadTime = TM_PSC_RLD_IMMEDIATE;
            TM_TimeBaseInit(HT_GPTM0, &GPTM_TimeBaseInitStructure);
            u16 CH0CCR = CRR * 0.75;
            u16 CH1CCR = CRR * 0.5;
            u16 CH2CCR = CRR * 0.25;
            GPTM_OutputInitStructure.Channel = TM_CH_0;
            GPTM_OutputInitStructure.OutputMode = TM_OM_PWM1;
            GPTM_OutputInitStructure.Control = TM_CHCTL_ENABLE;
            GPTM_OutputInitStructure.Polarity = TM_CHP_NONINVERTED;
            GPTM_OutputInitStructure.Compare = CH0CCR - 1;
            GPTM_OutputInitStructure.AsymmetricCompare = 0;
            TM_OutputInit(HT_GPTM0, &GPTM_OutputInitStructure);
            GPTM_OutputInitStructure.Channel = TM_CH_1;
            GPTM_OutputInitStructure.Compare = CH1CCR - 1;
            GPTM_OutputInitStructure.AsymmetricCompare = 0;
            TM_OutputInit(HT_GPTM0, &GPTM_OutputInitStructure);
            GPTM_OutputInitStructure.Channel = TM_CH_2;
            GPTM_OutputInitStructure.Compare = CH2CCR - 1;
            GPTM_OutputInitStructure.AsymmetricCompare = 0;
            TM_OutputInit(HT_GPTM0, &GPTM_OutputInitStructure);
            TM_Cmd(HT_GPTM0, ENABLE);
            for (int i = 100; i >= 10; i--)
            {
                GPTM_PWM1_Output(HT_GPTM0, TM_CH_0, (i - 5) % 101);
                GPTM_PWM1_Output(HT_GPTM0, TM_CH_1, (i - 5) % 101);
                GPTM_PWM1_Output(HT_GPTM0, TM_CH_2, (i - 5) % 101);
                _delay((15 / 4));
            }
            for (int i = 0; i <= 10; i++)
            {
                GPTM_PWM1_Output(HT_GPTM0, TM_CH_0, 101);
                _delay(15);
                GPTM_PWM1_Output(HT_GPTM0, TM_CH_1, 101);
                _delay(15);
                GPTM_PWM1_Output(HT_GPTM0, TM_CH_2, 101);
                _delay(15);
                GPTM_PWM1_Output(HT_GPTM0, TM_CH_0, 0);
                _delay(15);
                GPTM_PWM1_Output(HT_GPTM0, TM_CH_1, 0);
                _delay(15);
                GPTM_PWM1_Output(HT_GPTM0, TM_CH_2, 10);
                _delay(15);
            }
            for (int i = 0; i <= 95; i++)
            {
                GPTM_PWM1_Output(HT_GPTM0, TM_CH_0, (i + 5) % 101);
                GPTM_PWM1_Output(HT_GPTM0, TM_CH_1, (i + 5) % 101);
                GPTM_PWM1_Output(HT_GPTM0, TM_CH_2, (i + 5) % 101);
                _delay((15 / 4));
            }

            // Button for 單觸式
            input_1 = GPIO_ReadInBit(HT_GPIOB, GPIO_PIN_12);
            // input_2 = GPIO_ReadInBit(HT_GPIOB, GPIO_PIN_7);
            input_3 = GPIO_ReadInBit(HT_GPIOB, GPIO_PIN_5);
            input_4 = GPIO_ReadInBit(HT_GPIOB, GPIO_PIN_3);
            if (input_1 == 1 || input_3 == 1 || input_4 == 1)
            {
                buttonDown_2 = true;
            }
            input_2 = 1;
        }

        while (input_3 == 1 && buttonDown_3 == false)
        { //模式三:氣氛BLINBLIN燈
            LCD_ResetPrint();
            LCD_setCursor(0, 0); //設定游標座標
            LCD_Write('<');
            LCD_Write('<');
            LCD_Write('<');
            LCD_Write('B'); //設定顯示字符
            LCD_Write('L');
            LCD_Write('I');
            LCD_Write('N');
            LCD_Write('-');
            LCD_Write('-');
            LCD_Write('M');
            LCD_Write('O');
            LCD_Write('D');
            LCD_Write('E');
            LCD_Write('>');
            LCD_Write('>');
            LCD_Write('>');
            Delay_us(1000);

            // ADC
            float volt = 0.0;
            float V_min = (4.42 / 4095); //單位:V伏特
            float resister = 0.0;
            float R_max = (830000.0 / 4095); //單位:omega歐姆
            Data = ADC_GetConversionData(HT_ADC, 0);
            volt = Data * V_min;
            resister = Data * R_max;
            // printf("MODE_3: ADC_Data = %llu\r\n",Data);
            // printf("MODE_3: ADC_Data = %llu ; Volt = %f (Volt)\r\n",Data,volt);
            // printf("MODE_3: ADC_Data = %llu ; resister = %f (omega)\r\n",Data,resister);
            // printf("=================================================\r\n");

            //碳足跡(P = V^2 / R)
            float power = 0.0;
            power = ((volt * volt) / resister) * 1000; //單位:kW*hour千瓦小時(度)
            printf("MODE_3: Power = %f (Wh)\r\n", power);
            float Carbon_Footprint = 0.0;
            Carbon_Footprint = power * 0.502; //碳足跡(kgCO2e)=活動強度數據(kWh)*民國109年排放係數(kgCO2e/kWh)
            printf("MODE_3: Carbon Footprint = %f (kgCO2e)\r\n", Carbon_Footprint);
            printf("=================================================\r\n");

            LCD_setCursor(0, 1); //設定游標座標
            LCD_Write(' ');
            LCD_Write(' ');
            LCD_Write('R');
            LCD_Write('a');
            LCD_Write('n');
            LCD_Write('d');
            LCD_Write('o');
            LCD_Write('m');
            LCD_Write(' ');
            LCD_Write('C');
            LCD_Write('o');
            LCD_Write('l');
            LCD_Write('o');
            LCD_Write('r');
            LCD_Write(' ');
            LCD_Write(' ');
            Delay_us(1000);

            // Carbon Footprint
            /*
            u8 carbon_data[8];
            //sprintf(carbon_data, "%f",Carbon_Footprint);
            LCD_setCursor(0, 1); //設定游標座標
            LCD_Write(carbon_data[0]);      //設定顯示字符
            LCD_Write(carbon_data[1]);      //上限16*char
            LCD_Write(carbon_data[2]);
            LCD_Write(carbon_data[3]);
            LCD_Write(carbon_data[4]);
            LCD_Write(carbon_data[5]);
            LCD_Write(carbon_data[6]);
            LCD_Write(carbon_data[7]);
            LCD_Write('(');
            LCD_Write('k');
            LCD_Write('g');
            LCD_Write('C');
            LCD_Write('O');
            LCD_Write('2');
            LCD_Write('e');
            LCD_Write(')');
            Delay_us(1000);
            */

            // LED
            CRR = 100;
            TM_TimeBaseInitTypeDef GPTM_TimeBaseInitStructure;
            GPTM_TimeBaseInitStructure.CounterReload = CRR - 1;
            GPTM_TimeBaseInitStructure.Prescaler = 1 - 1;
            GPTM_TimeBaseInitStructure.CounterMode = TM_CNT_MODE_UP;
            GPTM_TimeBaseInitStructure.PSCReloadTime = TM_PSC_RLD_IMMEDIATE;
            TM_TimeBaseInit(HT_GPTM0, &GPTM_TimeBaseInitStructure);
            u16 CH0CCR = CRR * 0.75;
            u16 CH1CCR = CRR * 0.5;
            u16 CH2CCR = CRR * 0.25;
            GPTM_OutputInitStructure.Channel = TM_CH_0;
            GPTM_OutputInitStructure.OutputMode = TM_OM_PWM1;
            GPTM_OutputInitStructure.Control = TM_CHCTL_ENABLE;
            GPTM_OutputInitStructure.Polarity = TM_CHP_NONINVERTED;
            GPTM_OutputInitStructure.Compare = CH0CCR - 1;
            GPTM_OutputInitStructure.AsymmetricCompare = 0;
            TM_OutputInit(HT_GPTM0, &GPTM_OutputInitStructure);
            GPTM_OutputInitStructure.Channel = TM_CH_1;
            GPTM_OutputInitStructure.Compare = CH1CCR - 1;
            GPTM_OutputInitStructure.AsymmetricCompare = 0;
            TM_OutputInit(HT_GPTM0, &GPTM_OutputInitStructure);
            GPTM_OutputInitStructure.Channel = TM_CH_2;
            GPTM_OutputInitStructure.Compare = CH2CCR - 1;
            GPTM_OutputInitStructure.AsymmetricCompare = 0;
            TM_OutputInit(HT_GPTM0, &GPTM_OutputInitStructure);
            TM_Cmd(HT_GPTM0, ENABLE);
            for (int i = 100; i >= 10; i--)
            {
                GPTM_PWM1_Output(HT_GPTM0, TM_CH_0, (i - 5) % 101);
                GPTM_PWM1_Output(HT_GPTM0, TM_CH_1, (i - 5) % 101);
                GPTM_PWM1_Output(HT_GPTM0, TM_CH_2, (i - 5) % 101);
                _delay((15 / 4));
            }
            for (int i = 0; i <= 100; i++)
            {
                GPTM_PWM1_Output(HT_GPTM0, TM_CH_0, (i + 0) % 101);  // 0~100~0
                GPTM_PWM1_Output(HT_GPTM0, TM_CH_1, (i + 33) % 101); // 33~83~33
                GPTM_PWM1_Output(HT_GPTM0, TM_CH_2, (i + 66) % 101); // 66~16~66
                _delay(10);
            }
            for (int i = 0; i <= 95; i++)
            {
                GPTM_PWM1_Output(HT_GPTM0, TM_CH_0, (i + 5) % 101);
                GPTM_PWM1_Output(HT_GPTM0, TM_CH_1, (i + 5) % 101);
                GPTM_PWM1_Output(HT_GPTM0, TM_CH_2, (i + 5) % 101);
                _delay((15 / 2));
            }

            // Button for 單觸式
            input_1 = GPIO_ReadInBit(HT_GPIOB, GPIO_PIN_12);
            input_2 = GPIO_ReadInBit(HT_GPIOB, GPIO_PIN_7);
            // input_3 = GPIO_ReadInBit(HT_GPIOB, GPIO_PIN_5);
            input_4 = GPIO_ReadInBit(HT_GPIOB, GPIO_PIN_3);
            if (input_1 == 1 || input_2 == 1 || input_4 == 1)
            {
                buttonDown_3 = true;
            }
            input_3 = 1;
        }

        while (input_4 == 1 && buttonDown_4 == false)
        { //模式四:返回預設模式
            LCD_ResetPrint();
            LCD_setCursor(0, 0); //設定游標座標
            LCD_Write('<');      //設定顯示字符
            LCD_Write('<');
            LCD_Write('E');
            LCD_Write('N');
            LCD_Write('E');
            LCD_Write('R');
            LCD_Write('G');
            LCD_Write('Y');
            LCD_Write('-');
            LCD_Write('-');
            LCD_Write('M');
            LCD_Write('O');
            LCD_Write('D');
            LCD_Write('E');
            LCD_Write('>');
            LCD_Write('>');
            Delay_us(1000000 * 2);

            // ADC
            float volt = 0.0;
            float V_min = (4.42 / 4095); //單位:V伏特
            float resister = 0.0;
            float R_max = (830000.0 / 4095); //單位:omega歐姆
            Data = ADC_GetConversionData(HT_ADC, 0);
            volt = Data * V_min;
            resister = Data * R_max;
            // printf("MODE_4: ADC_Data = %llu\r\n",Data);
            // printf("MODE_4: ADC_Data = %llu ; Volt = %f (Volt)\r\n",Data,volt);
            // printf("MODE_4: ADC_Data = %llu ; resister = %f (omega)\r\n",Data,resister);
            // printf("=================================================\r\n");

            //碳足跡(P = V^2 / R)
            float power = 0.0;
            power = ((volt * volt) / resister) * 1000; //單位:kW*hour千瓦小時(度)
            printf("MODE_4: Power = %f (Wh)\r\n", power);
            float Carbon_Footprint = 0.0;
            Carbon_Footprint = power * 0.502; //碳足跡(kgCO2e)=活動強度數據(kWh)*民國109年排放係數(kgCO2e/kWh)
            Carbon_Footprint *= 1000000;
            printf("MODE_4: Carbon Footprint = %f (kgCO2e)\r\n", Carbon_Footprint);
            printf("=================================================\r\n");

            // Carbon Footprint
            char carbon_data[5];
            itoa(Carbon_Footprint, carbon_data);
            // sprintf(carbon_data, "%f",Carbon_Footprint);
            LCD_setCursor(0, 1); //設定游標座標
            LCD_Write('-');      //設定顯示字符
            LCD_Write('-');      //上限16*char
            LCD_Write('>');
            LCD_Write(carbon_data[0]);
            LCD_Write(carbon_data[1]);
            LCD_Write(carbon_data[2]);
            LCD_Write(carbon_data[3]);
            LCD_Write(carbon_data[4]);
            LCD_Write('(');
            LCD_Write('m');
            LCD_Write('g');
            LCD_Write('C');
            LCD_Write('O');
            LCD_Write('2');
            LCD_Write('e');
            LCD_Write(')');
            Delay_us(1000000 * 2);

            // LED
            CRR = 255;
            TM_TimeBaseInitTypeDef GPTM_TimeBaseInitStructure;
            GPTM_TimeBaseInitStructure.CounterReload = CRR - 1;
            GPTM_TimeBaseInitStructure.Prescaler = 1 - 1;
            GPTM_TimeBaseInitStructure.CounterMode = TM_CNT_MODE_UP;
            GPTM_TimeBaseInitStructure.PSCReloadTime = TM_PSC_RLD_IMMEDIATE;
            TM_TimeBaseInit(HT_GPTM0, &GPTM_TimeBaseInitStructure);
            u16 CH0CCR = CRR;
            u16 CH1CCR = CRR;
            u16 CH2CCR = CRR;
            GPTM_OutputInitStructure.Channel = TM_CH_0;
            GPTM_OutputInitStructure.OutputMode = TM_OM_PWM1;
            GPTM_OutputInitStructure.Control = TM_CHCTL_ENABLE;
            GPTM_OutputInitStructure.Polarity = TM_CHP_NONINVERTED;
            GPTM_OutputInitStructure.Compare = CH0CCR - 1;
            GPTM_OutputInitStructure.AsymmetricCompare = 0;
            TM_OutputInit(HT_GPTM0, &GPTM_OutputInitStructure);
            GPTM_OutputInitStructure.Channel = TM_CH_1;
            GPTM_OutputInitStructure.Compare = CH1CCR - 1;
            GPTM_OutputInitStructure.AsymmetricCompare = 0;
            TM_OutputInit(HT_GPTM0, &GPTM_OutputInitStructure);
            GPTM_OutputInitStructure.Channel = TM_CH_2;
            GPTM_OutputInitStructure.Compare = CH2CCR - 1;
            GPTM_OutputInitStructure.AsymmetricCompare = 0;
            TM_OutputInit(HT_GPTM0, &GPTM_OutputInitStructure);
            TM_Cmd(HT_GPTM0, ENABLE);
            GPTM_PWM1_Output(HT_GPTM0, TM_CH_0, 255 - R); //(i+0)%101)
            GPTM_PWM1_Output(HT_GPTM0, TM_CH_1, 255 - G); //(i+33)%101)
            GPTM_PWM1_Output(HT_GPTM0, TM_CH_2, 255 - B); //(i+66)%101)
            _delay(10);

            // Button for 單觸式
            input_1 = GPIO_ReadInBit(HT_GPIOB, GPIO_PIN_12);
            input_2 = GPIO_ReadInBit(HT_GPIOB, GPIO_PIN_7);
            input_3 = GPIO_ReadInBit(HT_GPIOB, GPIO_PIN_5);
            // input_4 = GPIO_ReadInBit(HT_GPIOB, GPIO_PIN_3);
            if (input_1 == 1 || input_2 == 1 || input_3 == 1)
            {
                buttonDown_4 = true;
            }
            input_4 = 1;
        }

        if (input_1 == 0 && input_2 == 0 && input_3 == 0 && input_4 == 0 && buttonDown_1 == false && buttonDown_2 == false && buttonDown_3 == false && buttonDown_4 == false)
        { //預設模式:光感小夜燈
            LCD_ResetPrint();
            LCD_setCursor(0, 0); //設定游標座標
            LCD_Write('<');      //設定顯示字符
            LCD_Write('<');
            LCD_Write('E');
            LCD_Write('N');
            LCD_Write('E');
            LCD_Write('R');
            LCD_Write('G');
            LCD_Write('Y');
            LCD_Write('-');
            LCD_Write('-');
            LCD_Write('M');
            LCD_Write('O');
            LCD_Write('D');
            LCD_Write('E');
            LCD_Write('>');
            LCD_Write('>');
            Delay_us(1000000 * 2);

            // ADC
            float volt = 0.0;
            float V_min = (4.42 / 4095); //單位:V伏特
            float resister = 0.0;
            float R_max = (830000.0 / 4095); //單位:omega歐姆
            Data = ADC_GetConversionData(HT_ADC, 0);
            volt = Data * V_min;
            resister = Data * R_max;
            // printf("ADC_Data = %llu\r\n",Data);
            // printf("ADC_Data = %llu ; Volt = %f (Volt)\r\n",Data,volt);
            // printf("ADC_Data = %llu ; resister = %f (omega)\r\n",Data,resister);
            // printf("=================================================\r\n");

            //碳足跡(P = V^2 / R)
            float power = 0.0;
            power = ((volt * volt) / resister) * 1000; //單位:kW*hour千瓦小時(度)
            printf("Power = %f (Wh)\r\n", power);
            float Carbon_Footprint = 0.0;
            Carbon_Footprint = power * 0.502; //碳足跡(kgCO2e)=活動強度數據(kWh)*民國109年排放係數(kgCO2e/kWh)
            Carbon_Footprint *= 1000000;
            printf("Carbon Footprint = %f (kgCO2e)\r\n", Carbon_Footprint);
            printf("=================================================\r\n");

            char carbon_data[8];
            itoa(Carbon_Footprint, carbon_data);
            // sprintf(carbon_data, "%f",Carbon_Footprint);
            LCD_setCursor(0, 1); //設定游標座標
            LCD_Write('-');      //設定顯示字符
            LCD_Write('-');      //上限16*char
            LCD_Write('>');
            LCD_Write(carbon_data[0]);
            LCD_Write(carbon_data[1]);
            LCD_Write(carbon_data[2]);
            LCD_Write(carbon_data[3]);
            LCD_Write(carbon_data[4]);
            LCD_Write('(');
            LCD_Write('m');
            LCD_Write('g');
            LCD_Write('C');
            LCD_Write('O');
            LCD_Write('2');
            LCD_Write('e');
            LCD_Write(')');
            Delay_us(1000000 * 2);

            // LED
            GPTM_PWM1_Output(HT_GPTM0, TM_CH_0, 255 - R);
            GPTM_PWM1_Output(HT_GPTM0, TM_CH_1, 255 - G);
            GPTM_PWM1_Output(HT_GPTM0, TM_CH_2, 255 - B);
            _delay(10);

            // Button for 單觸式
            buttonDown_1 = true;
            buttonDown_2 = true;
            buttonDown_3 = true;
            buttonDown_4 = true;
        }
    }
}

/*********************************************************************************************************/ /**
                                                                                                             * @brief  Compare two buffers.
                                                                                                             * @param  Buffer1, Buffer2: buffers to be compared.
                                                                                                             * @param  BufferLength: buffer's length
                                                                                                             * @retval None
                                                                                                             ***********************************************************************************************************/
// TestResult CmpBuffer(u8* Buffer1, u8* Buffer2, u32 BufferLength)
//{
//   while (BufferLength--)
//   {
//     if (*Buffer1 != *Buffer2)
//     {
//       return Fail;
//     }

//    Buffer1++;
//    Buffer2++;
//  }

//  return Pass;
//}

#if (HT32_LIB_DEBUG == 1)
/*********************************************************************************************************/ /**
                                                                                                             * @brief  Report both the error name of the source file and the source line number.
                                                                                                             * @param  filename: pointer to the source file name.
                                                                                                             * @param  uline: error line source number.
                                                                                                             * @retval None
                                                                                                             ***********************************************************************************************************/
void assert_error(u8 *filename, u32 uline)
{
    /*
       This function is called by IP library that the invalid parameters has been passed to the library API.
       Debug message can be added here.
       Example: printf("Parameter Error: file %s on line %d\r\n", filename, uline);
    */

    while (1)
    {
    }
}
#endif

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */