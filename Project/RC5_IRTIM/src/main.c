/**
  ******************************************************************************
  * @file main.c
  * @brief This file contains the main function for AN.
  * @author STMicroelectronics - MCD Application Team
  * @version V1.1.0
  * @date    09/14/2009
  ******************************************************************************
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  * @image html logo.bmp
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm8l15x.h"
#include "rc5.h"

/**
  * @addtogroup IRTIM_AN
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
#define PRESS_DELAY 100
/* Private define ------------------------------------------------------------*/
#define BUTTON_PORT (GPIOB)
#define BUTTON_EXTI_PORT    EXTI_Port_B
#define BUTTON_S1_UP    (GPIO_Pin_4)
#define BUTTON_S2_DOWN  (GPIO_Pin_3)
#define BUTTON_S3_CONF  (GPIO_Pin_5)
#define BUTTON_S4_ADD   (GPIO_Pin_7)
#define BUTTON_S5_SUB   (GPIO_Pin_6)
#define BUTTON_S6_BACK  (GPIO_Pin_2)


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
 RC5_Address_TypeDef Address;
 RC5_Instruction_TypeDef Instruction;
 RC5_Ctrl_TypeDef RC5_Ctrl1 = RC5_Ctrl_Reset;
 RC5_Ctrl_TypeDef RC5_Ctrl2 = RC5_Ctrl_Reset;
/* Private function prototypes -----------------------------------------------*/
void Delay (uint16_t nCount);
void SendFrame(RC5_Address_TypeDef RC5_Address,
               RC5_Instruction_TypeDef RC5_Instruction, 
			         RC5_Ctrl_TypeDef RC5_Ctrl);
void	CLK_Config(void);
void	TIM2_Config(void);
void	TIM3_Config(void);
void	IRTIM_Config(void);
void    DelayMs(uint16_t ms);
void    key_init(void);
void    enter_low_power_mode(void);
/* Public variables ---------------------------------------------------------*/
 extern uint8_t Send_Operation_Completed  ; /* RC5 Flag*/
 extern uint8_t Send_Operation_Ready ;      /* RC5 Flag*/
 extern uint32_t RC5_FrameManchestarFormat;
 
uint32_t count_time; 
/**
  * @brief Application main entry point.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Library functions called:
  */
void main(void)
{
	while(1){
        /* Clock configuration */
        CLK_Config();
      
      /* TIM2 configuration */
        TIM2_Config();
        
        /* TIM3 configuration */
        TIM3_Config();
        
        /* IRTIM configuration */
        IRTIM_Config();
        
        key_init();
        /* Initialize I/Os in Output Mode */
      //GPIO_Init(LEDS_PORT, LEDS_PINS, GPIO_Mode_Out_PP_Low_Fast);
    
        IWDG_Enable();
        IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
        IWDG_SetPrescaler(IWDG_Prescaler_256);
        IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
        IWDG_SetReload(0xff);
        IWDG_ReloadCounter();
        if ((GPIO_ReadInputData(BUTTON_PORT) & (BUTTON_S1_UP|BUTTON_S2_DOWN)) == (uint8_t)0x00){
            DelayMs(3);
            if ((GPIO_ReadInputData(BUTTON_PORT) & (BUTTON_S1_UP|BUTTON_S2_DOWN)) == (uint8_t)0x00){
                /* Button is pressed */
                Address = RC5_Address_Reserved00; 
                Instruction = RC5_Instruction_MenuOn;
                SendFrame(Address, Instruction, RC5_Ctrl1);
                while ((GPIO_ReadInputData(BUTTON_PORT) & (BUTTON_S1_UP|BUTTON_S2_DOWN)) == (uint8_t)0x00) {
                    IWDG_ReloadCounter();
                }
                DelayMs(PRESS_DELAY);
            }
        }
        
        if ((GPIO_ReadInputData(BUTTON_PORT) & (BUTTON_S4_ADD|BUTTON_S5_SUB)) == (uint8_t)0x00){
            DelayMs(3);
            if ((GPIO_ReadInputData(BUTTON_PORT) & (BUTTON_S4_ADD|BUTTON_S5_SUB)) == (uint8_t)0x00){
                /* Button is pressed */
                Address = RC5_Address_Reserved00; 
                Instruction = RC5_Instruction_MenuOff;
                SendFrame(Address, Instruction, RC5_Ctrl1);
                while ((GPIO_ReadInputData(BUTTON_PORT) & (BUTTON_S4_ADD|BUTTON_S5_SUB)) == (uint8_t)0x00) {
                    IWDG_ReloadCounter();
                }
                DelayMs(PRESS_DELAY);
            }
        }
        
		/* Check button status */
        if ((GPIO_ReadInputData(BUTTON_PORT) & BUTTON_S1_UP) == (uint8_t)0x00){
            DelayMs(3);
            if ((GPIO_ReadInputData(BUTTON_PORT) & BUTTON_S1_UP) == (uint8_t)0x00){
                /* Button is pressed */
                Address = RC5_Address_Reserved00; 
                Instruction = RC5_Instruction_VolumeUp;
                SendFrame(Address, Instruction, RC5_Ctrl1);
                while ((GPIO_ReadInputData(BUTTON_PORT) & BUTTON_S1_UP) == (uint8_t)0x00) {
                    IWDG_ReloadCounter();
                }
                DelayMs(PRESS_DELAY);
            }
        }
        
        if ((GPIO_ReadInputData(BUTTON_PORT) & BUTTON_S2_DOWN) == (uint8_t)0x00){
            DelayMs(3);
            if ((GPIO_ReadInputData(BUTTON_PORT) & BUTTON_S2_DOWN) == (uint8_t)0x00){
                /* Button is pressed */
                Address = RC5_Address_Reserved00; 
                Instruction = RC5_Instruction_VolumeDown;
                SendFrame(Address, Instruction, RC5_Ctrl1);
                while ((GPIO_ReadInputData(BUTTON_PORT) & BUTTON_S2_DOWN) == (uint8_t)0x00) {
                    IWDG_ReloadCounter();
                }
                DelayMs(PRESS_DELAY);
            }
        }
        
        if ((GPIO_ReadInputData(BUTTON_PORT) & BUTTON_S3_CONF) == (uint8_t)0x00){
            DelayMs(3);
            if ((GPIO_ReadInputData(BUTTON_PORT) & BUTTON_S3_CONF) == (uint8_t)0x00){
                /* Button is pressed */
                Address = RC5_Address_Reserved00; 
                Instruction = RC5_Instruction_ColorSaturUp;
                SendFrame(Address, Instruction, RC5_Ctrl1);
                while ((GPIO_ReadInputData(BUTTON_PORT) & BUTTON_S3_CONF) == (uint8_t)0x00) {
                    IWDG_ReloadCounter();
                }
                DelayMs(PRESS_DELAY);
            }
        }
        
        if ((GPIO_ReadInputData(BUTTON_PORT) & BUTTON_S4_ADD) == (uint8_t)0x00){
            DelayMs(3);
            if ((GPIO_ReadInputData(BUTTON_PORT) & BUTTON_S4_ADD) == (uint8_t)0x00){
                /* Button is pressed */
                Address = RC5_Address_Reserved00; 
                Instruction = RC5_Instruction_BrightnessUp;
                SendFrame(Address, Instruction, RC5_Ctrl1);
                count_time = 0;
                while ((GPIO_ReadInputData(BUTTON_PORT) & BUTTON_S4_ADD) == (uint8_t)0x00) {
                    count_time ++;
                    IWDG_ReloadCounter();
                    if(count_time > 100000) {
                        while ((GPIO_ReadInputData(BUTTON_PORT) & BUTTON_S4_ADD) == (uint8_t)0x00) {
                            Address = RC5_Address_Reserved00; 
                            Instruction = RC5_Instruction_BrightnessUp;
                            SendFrame(Address, Instruction, RC5_Ctrl1);
                            DelayMs(50);
                            IWDG_ReloadCounter();
                        }
                    }
                }
                DelayMs(PRESS_DELAY);
                
            }
        }
        
        if ((GPIO_ReadInputData(BUTTON_PORT) & BUTTON_S5_SUB) == (uint8_t)0x00){
            DelayMs(3);
            if ((GPIO_ReadInputData(BUTTON_PORT) & BUTTON_S5_SUB) == (uint8_t)0x00){
                /* Button is pressed */
                Address = RC5_Address_Reserved00; 
                Instruction = RC5_Instruction_BrightnessDown;
                SendFrame(Address, Instruction, RC5_Ctrl1);
                count_time = 0;
                while ((GPIO_ReadInputData(BUTTON_PORT) & BUTTON_S5_SUB) == (uint8_t)0x00) {
                    count_time ++;
                    IWDG_ReloadCounter();
                    if(count_time > 80000) {
                        while ((GPIO_ReadInputData(BUTTON_PORT) & BUTTON_S5_SUB) == (uint8_t)0x00) {
                            Address = RC5_Address_Reserved00; 
                            Instruction = RC5_Instruction_BrightnessDown;
                            SendFrame(Address, Instruction, RC5_Ctrl1);
                            DelayMs(50);
                            IWDG_ReloadCounter();
                        }
                    }
                }
                DelayMs(PRESS_DELAY);
            }
        }
        
        if ((GPIO_ReadInputData(BUTTON_PORT) & BUTTON_S6_BACK) == (uint8_t)0x00){
            DelayMs(3);
            if ((GPIO_ReadInputData(BUTTON_PORT) & BUTTON_S6_BACK) == (uint8_t)0x00){
                /* Button is pressed */
                Address = RC5_Address_Reserved00; 
                Instruction = RC5_Instruction_ColorSaturDown;
                SendFrame(Address, Instruction, RC5_Ctrl1);
                while ((GPIO_ReadInputData(BUTTON_PORT) & BUTTON_S6_BACK) == (uint8_t)0x00) {
                    IWDG_ReloadCounter();
                }
                DelayMs(PRESS_DELAY);
            }
        }
        
        
        enter_low_power_mode();
        halt();
    }
}
/* Private functions ---------------------------------------------------------*/
/**
  * @brief Generate a delay
  * @param[in] nCount : value of the delay
  * @retval  None
  */
void Delay(uint16_t nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {
    nCount--;
  }
}
void DelayMs(uint16_t ms)
{
    uint16_t c = 1000;

    while(ms--){
        while(c--){}
        c = 1000;
    }
}
/**
  * @brief Generate and Send the RC5 frame.
  * @param[in] RC5_Address : the RC5 Device destination 
	* (a member of @ref RC5_Address_TypeDef enumeration).
  * @param[in] RC5_Instruction : the RC5 command instruction 
	* (a member of @ref RC5_Address_TypeDef enumeration).
	* @param[in] RC5_Ctrl : the RC5 Control bit
	* (a member of @ref RC5_Ctrl_TypeDef enumeration).
  * @retval  None
  */
void SendFrame(RC5_Address_TypeDef RC5_Address, 
                           RC5_Instruction_TypeDef RC5_Instruction, 
								           RC5_Ctrl_TypeDef RC5_Ctrl)
{
	uint16_t RC5_FrameBinaryFormat =0;
	
	/* Generate a binary format of the Frame */
	RC5_FrameBinaryFormat = RC5_BinFrameGeneration(RC5_Address, RC5_Instruction, RC5_Ctrl);
	
	/* Disable Interrupts */
	disableInterrupts();
	
	/* Generate a Manchester format of the Frame */
	RC5_FrameManchestarFormat = RC5_ManchesterConvert(RC5_FrameBinaryFormat);
	
	/* Set the Send operation Ready flag to indicate that the frame is ready to be sent */
	Send_Operation_Ready = 1;
	
  /* Enable Interrupts */
	enableInterrupts();

}

/**
  * @brief Configurate the Clock & enable the clock of TIM2 and TIM3.
  * @param[in] None
  * @retval  None
  */
void CLK_Config(void)
{
	/* Clock Master = 8 Mhz */
	//CLK_MasterPrescalerConfig(CLK_MasterPrescaler_HSIDiv1);
    CLK_HSEConfig(CLK_HSE_ON);
    CLK_SYSCLKSourceSwitchCmd(ENABLE);
    CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSE);
    CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);
    while (CLK_GetSYSCLKSource() != CLK_SYSCLKSource_HSE){}
  
	/* Enable TIM2 clock */
	CLK_PeripheralClockConfig(CLK_Peripheral_TIM2, ENABLE);
	
	/* Enable TIM3 clock */
	CLK_PeripheralClockConfig(CLK_Peripheral_TIM3, ENABLE);
}
/**
  * @brief Configurate TIM2.
  * @param[in] None
  * @retval  None
  */
void TIM2_Config(void)
{
	/* DeInit TIM2 */
	TIM2_DeInit();
	
	/* TIM2 Time base configuration */
  TIM2_TimeBaseInit(TIM2_Prescaler_1, TIM2_CounterMode_Up, 219 );
	
	/* TIM2 channel 1 configuration */
	TIM2_OC1Init(TIM2_OCMode_PWM1, TIM2_OutputState_Enable, 55, TIM2_OCPolarity_High, TIM2_OCIdleState_Reset );
	
	/* TIM2 Counter Enable */
	TIM2_Cmd(ENABLE);
	
	/* Enable the TIM2 channel1 output to be connected internly to the IRTIM*/
	TIM2_CtrlPWMOutputs(ENABLE);
}
/**
  * @brief Configurate TIM3.
  * @param[in] None
  * @retval  None
  */
void TIM3_Config(void)
{
	/* DeInit TIM3 */
	TIM3_DeInit();
  
	/* TIM3 Time base configuration */
	TIM3_TimeBaseInit(TIM3_Prescaler_1, TIM3_CounterMode_Up, 7112 );
  
	/* TIM3 channel 1 configuration */
	TIM3_OC1Init(TIM3_OCMode_Timing, TIM3_OutputState_Enable, 7000, TIM3_OCPolarity_High, TIM3_OCIdleState_Reset );
	
	/* TIM3 interrupt Update Enable */
	TIM3_ITConfig(TIM3_IT_Update, ENABLE);
	
	/* TIM3 Counter Enable */
	TIM3_Cmd(ENABLE);
	/* Enable the TIM3 channel1 output to be connected internly to the IRTIM*/
	TIM3_CtrlPWMOutputs(ENABLE);
}
/**
  * @brief Configurate IRTIM.
  * @param[in] None
  * @retval  None
  */
void IRTIM_Config(void)
{
	/* DeInit IRTIM */
	IRTIM_DeInit();
	/* Enable IRTIM */
	IRTIM_Cmd(ENABLE);
}

void key_init(void)
{
    
    GPIO_Init(BUTTON_PORT, BUTTON_S1_UP|BUTTON_S2_DOWN|BUTTON_S3_CONF|
            BUTTON_S4_ADD|BUTTON_S5_SUB|BUTTON_S6_BACK, GPIO_Mode_In_FL_IT);  
          
    
    EXTI_SetPinSensitivity(EXTI_Pin_2, EXTI_Trigger_Falling);  
    EXTI_SetPinSensitivity(EXTI_Pin_3, EXTI_Trigger_Falling);  
    EXTI_SetPinSensitivity(EXTI_Pin_4, EXTI_Trigger_Falling);  
    EXTI_SetPinSensitivity(EXTI_Pin_5, EXTI_Trigger_Falling);  
    EXTI_SetPinSensitivity(EXTI_Pin_6, EXTI_Trigger_Falling);  
    EXTI_SetPinSensitivity(EXTI_Pin_7, EXTI_Trigger_Falling);  


    enableInterrupts();  
}

void enter_low_power_mode()
{
    //CLK_DeInit();              
  
    // ±÷”8∑÷∆µ£¨2MHz
    //CLK_MasterPrescalerConfig(CLK_MasterPrescaler_HSIDiv8);  
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM2, DISABLE);
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM3, DISABLE);  
    /*
    GPIO_Init(BUTTON_PORT, BUTTON_S1_UP|BUTTON_S2_DOWN|BUTTON_S3_CONF|
            BUTTON_S4_ADD|BUTTON_S5_SUB|BUTTON_S6_BACK, GPIO_Mode_Out_PP_High_Fast);  
    GPIO_WriteBit(BUTTON_PORT, BUTTON_S1_UP|BUTTON_S2_DOWN|BUTTON_S3_CONF|
            BUTTON_S4_ADD|BUTTON_S5_SUB|BUTTON_S6_BACK, RESET);
            */
}

/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
