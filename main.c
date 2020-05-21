#include <stdbool.h>
#include "stm32f10x.h"
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
I2C_InitTypeDef   I2C_InitStructure;
TIM_TimeBaseInitTypeDef timerInitStructure;

/* Private define ------------------------------------------------------------*/

#define AS7262_V_CAL 0x14
#define AS7262_B_CAL 0x18
#define AS7262_G_CAL 0x1C
#define AS7262_Y_CAL 0x20
#define AS7262_O_CAL 0x24
#define AS7262_R_CAL 0x28

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

int CHIP1=1;
int CHIP2=2;
float humidityValue;
float temparatureValue;

char str[];
//AS7262 
uint8_t readVal=0;
uint8_t  Status=0;
uint8_t b0,b1,b2,b3;
float violet=0;
float blue=0;
float green=0;
float yellow=0;
float orange=0;
float red=0;
float myFloat; 
uint8_t RegVal = 0;
volatile uint8_t result=0;
/* Private function prototypes -----------------------------------------------*/
 //void delay(int nCount);
//void Delay(__IO uint32_t nCount);
void Timer_Configuration(void);
void Delay_ms(int ms);
void Enable_Clock(void);
void Config_GPIO(void);
void Init_I2C(void);

//AS7262 functions
void Timer_Configuration();
void Delay_ms(int);
uint8_t virtualReadReg(uint8_t reg);
void virtualWriteReg(uint8_t , uint8_t );
void clearDataAvailable();
bool dataAvailable();
float getCalibratedViolet();
float getCalibratedBlue();
float getCalibratedGreen();
float getCalibratedYellow();
float getCalibratedOrange();
float getCalibratedRed();
float getCalibratedValue(uint8_t );
void writeRegister(uint8_t ,uint8_t );
uint8_t readRegisterOneByte(int8_t reg, int8_t dAddr);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
 
int main(void)
{
  
  uint8_t a= RCC_GetSYSCLKSource();
  RCC_ClocksTypeDef A;
  RCC_GetClocksFreq(&A);
  
  Enable_Clock();
  Config_GPIO();
  Timer_Configuration();
  I2C_DeInit(I2C1);
  Init_I2C();

  
      //AS7262
    
     GPIO_ResetBits( GPIOA, GPIO_Pin_10);
     Delay_ms(150);
     GPIO_SetBits( GPIOA, GPIO_Pin_10);
     Delay_ms(1000);
     result=virtualReadReg(0x01);
     //Setting IntegrationTime
     virtualWriteReg(0x05,50);
     //Setting the control register
     //setting Gain= 3
     uint8_t gain=0x03;
     uint8_t value=virtualReadReg(0x04);
     value &= 0xCF;   //clear gain bits
     value|=(gain<<4);//setting gain bits
     virtualWriteReg(0x04,value);
      //---------------------
      //Setting the control register
      //setting measurement Mode =3
      uint8_t mode =0x03;
      uint8_t valueGain= virtualReadReg(0x04);
      valueGain &= 0xF3;//clear gain bits
      valueGain |= (mode<<2);//set gain bits
      virtualWriteReg(0x04,valueGain);
      //******Done setting******//
   
  while (1)
  {
        clearDataAvailable();
        while (dataAvailable()==0){
             // delay(5);
          Delay_ms(5);
        };
        violet=getCalibratedViolet();
        blue=getCalibratedBlue();
        green=getCalibratedGreen();
        yellow=getCalibratedYellow();
        orange=getCalibratedOrange();
        red=getCalibratedRed();
        
     
  }
}



void Init_I2C(void){
    
  /*!< LM75_I2C Init */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C; 
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = 100000;
  I2C_Init(I2C1, &I2C_InitStructure);

  /*!< Enable SMBus Alert interrupt */
  I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);

  /*!< LM75_I2C Init */
  I2C_Cmd(I2C1, ENABLE);
  
}

void Config_GPIO(){
 
  
  //Config SCL
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_Init(GPIOB,&GPIO_InitStructure);
    
  //Config SDA
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
  GPIO_Init(GPIOB,&GPIO_InitStructure);   
  
  //PA10
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA,&GPIO_InitStructure);
}

void Enable_Clock(){
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
}

 void Timer_Configuration()
  {
   
    timerInitStructure.TIM_Prescaler = 23999;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period =65535;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &timerInitStructure);

  }

void Delay_ms(int ms)
  {
    TIM_SetCounter(TIM2,0);
    TIM_Cmd(TIM2,ENABLE);
    while(TIM_GetCounter(TIM2)<ms){
    }
    TIM_Cmd(TIM2,DISABLE);
  }



/**
  * @brief  Inserts a delay time.
  * @param  nCount: specifies the delay time length.
  * @retval None
  */
/*void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}*/



 void clearDataAvailable(){
    uint8_t value = virtualReadReg(0x04);
    value &= ~(1 << 1); //Set the DATA_RDY bit
    virtualWriteReg(0x04,value);
  }

  bool dataAvailable(){
    uint8_t value = virtualReadReg(0x04);
    return (value & (1 << 1));  
  }

float getCalibratedViolet(){
    float result=getCalibratedValue(AS7262_V_CAL);
    return result;
  }
  
  float getCalibratedBlue(){
    float result=getCalibratedValue(AS7262_B_CAL);
    return result;
  }
  
  float getCalibratedGreen(){
    float result=getCalibratedValue(AS7262_G_CAL);
    return result;
  }
  
  float getCalibratedYellow(){
    float result=getCalibratedValue(AS7262_Y_CAL);
    return result;
  }
  
  float getCalibratedOrange(){
    float result=getCalibratedValue(AS7262_O_CAL);
    return result;
  }
  
  float getCalibratedRed(){
    float result=getCalibratedValue(AS7262_R_CAL);
    return result;
  }

float getCalibratedValue(uint8_t colorAddr){
   
    uint32_t calBytes=0;
    
    b0=virtualReadReg(colorAddr+0);
    b1=virtualReadReg(colorAddr+1);
    b2=virtualReadReg(colorAddr+2);
    b3=virtualReadReg(colorAddr+3);

    calBytes |= ((uint32_t)b0<<24);
    calBytes |= ((uint32_t)b1<<16);
    calBytes |= ((uint32_t)b2<<8);
    calBytes |= ((uint32_t)b3<<0);

    memcpy(&myFloat,&calBytes,4);
    return myFloat;
    
  }

uint8_t virtualReadReg(uint8_t reg){

    Status=readRegisterOneByte(0x00,0x92);

    while(1){
      if ((Status&0x02)==0){//could write
        break;
        //delay(5);
        Delay_ms(5);
      }
    }
   
    writeRegister(0x01,reg);
    //delay(50);
    Delay_ms(50);
    Status=readRegisterOneByte(0x00,0x92);
    while(1){
      if ((Status&0x01)!=0){//could read if bit 0 is 1
        break;
      }
     // delay(5);
      Delay_ms(5);
    }
    readVal= readRegisterOneByte(0x02,0x92);
    return readVal;
  }

  
  void virtualWriteReg(uint8_t virtualAddr, uint8_t val){
    
   // GPIO_ResetBits( GPIOE, GPIO_Pin_10);
    Status=readRegisterOneByte(0x00,0x92);
    while(1){
      if ((Status&0x02)==0){//could write if bit 1 is 0
        break;
       // delay(5);
        Delay_ms(5);
      }
    }
    writeRegister(0x01,(virtualAddr|0x80));//tndicate that there is pending writing
    Status=readRegisterOneByte(0x00,0x92);
    while(1){
      if ((Status&0x02)==0){//could write if bit 1 is 0
        break;
        //delay(5);
        Delay_ms(5);
      }
    }
    
    writeRegister(0x01,val);
    
  }

  uint8_t readRegisterOneByte(int8_t reg, int8_t dAddr){
   
   
     I2C_GenerateSTART(I2C1, ENABLE);
    
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)){
    }
     I2C_Send7bitAddress(I2C1, dAddr, I2C_Direction_Transmitter);
    

    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
    }
    
    I2C_SendData(I2C1, reg); 
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
    }
   
    /* Send START condition a second time */  
    I2C_GenerateSTART(I2C1, ENABLE);
    
    /* Test on SB Flag */
   
    
     while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)){
    }
    /* Send LM75 address for read */
    I2C_Send7bitAddress(I2C1, dAddr|0x01, I2C_Direction_Receiver);
    
    /* Test on ADDR Flag */

    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){
    }
     
    
    while( !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)){
    }

    RegVal = I2C_ReceiveData(I2C1);
     

    while( !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)){
    }
           
    I2C_AcknowledgeConfig(I2C1, DISABLE);

    I2C_GenerateSTOP(I2C1,ENABLE);

    while (I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY)){
    }
 
    return RegVal;
      
 }

void writeRegister(uint8_t wrtRegAdd,uint8_t VAddr){
  
    I2C_GenerateSTART(I2C1, ENABLE);
    
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    
    /* Send device address for write */
    I2C_Send7bitAddress(I2C1, 0x92, I2C_Direction_Transmitter);
    
    /* Test on ADDR Flag */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
    }
    
    /* Send the device's internal address to write to */
    I2C_SendData(I2C1, wrtRegAdd); 
     
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
    }
    /* Send the device's virtual address to write to */
    I2C_SendData(I2C1, VAddr); 
    
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_GenerateSTOP(I2C1, ENABLE);

    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
}




/*
void delay(int nCount)
 {
    int index = 0; 
    for(index = (34000 * nCount); index != 0; index--)
    {
    }
 }*/


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
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

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
