#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "sn8200_api.h"
#include "sn8200_core.h"
#include "delay.h"
#include "algorithm_by_RF.h"
#include "max30102.h"
#include "qpc.h"

//GPIO and I2C Peripheral (I2C3 Configuration)
#define I2Cx                      I2C3  //Selected I2C peripheral
#define RCC_APB1Periph_I2Cx       RCC_APB1Periph_I2C3 //Bus where the peripheral is connected
#define RCC_AHB1Periph_GPIO_SCL   RCC_AHB1Periph_GPIOA  //Bus for GPIO Port of SCL
#define RCC_AHB1Periph_GPIO_SDA   RCC_AHB1Periph_GPIOC  //Bus for GPIO Port of SDA
#define RCC_AHB1Periph_GPIO_INT		RCC_AHB1Periph_GPIOC	//Bus for GPIO Port of INT
#define RCC_AHB1Periph_GPIO_LED		RCC_AHB1Periph_GPIOD	//Bus for GPIO Port of LEDs
#define RCC_AHB1Periph_GPIO_BUTTON		RCC_AHB1Periph_GPIOA	//Bus for GPIO Port of INT
#define GPIO_AF_I2Cx              GPIO_AF_I2C3    //Alternate function for GPIO pins
#define GPIO_SCL                  GPIOA
#define GPIO_SDA                  GPIOC
#define GPIO_INT									GPIOC
#define GPIO_LED									GPIOD
#define GPIO_BUTTON								GPIOD
#define GPIO_Pin_SCL              GPIO_Pin_8
#define GPIO_Pin_SDA              GPIO_Pin_9
#define GPIO_Pin_INT							GPIO_Pin_1
#define GPIO_Pin_LED_GREEN				GPIO_Pin_12
#define GPIO_Pin_LED_ORANGE				GPIO_Pin_13
#define GPIO_Pin_LED_RED					GPIO_Pin_14
#define GPIO_Pin_LED_BLUE					GPIO_Pin_15
#define GPIO_Pin_BUTTON						GPIO_Pin_0
#define GPIO_PinSource_SCL        GPIO_PinSource8
#define GPIO_PinSource_SDA        GPIO_PinSource9
#define GPIO_PinSource_INT				GPIO_PinSource1
#define GPIO_PinSource_LED_GREEN	GPIO_PinSource12
#define GPIO_PinSource_LED_ORANGE	GPIO_PinSource13
#define GPIO_PinSource_LED_RED		GPIO_PinSource14
#define GPIO_PinSource_LED_BLUE		GPIO_PinSource15
#define GPIO_PinSource_BUTTON			GPIO_PinSource_0

uint32_t elapsedTime, timeStart;
uint32_t aun_ir_buffer[BUFFER_SIZE];			// IR LED Sensor Data
uint32_t aun_red_buffer[BUFFER_SIZE];			// Red LED Sensor Data
float old_n_spO2;													// SpO2 Value
uint8_t uch_dummy, k;											// Dummy variable to clear status register

// MAX30102 MODULES

bool maxim_max30102_write_reg(uint8_t uch_addr, uint8_t uch_data)
/**
* \brief        Write a value to a MAX30102 register
* \par          Details
*               This function writes a value to a MAX30102 register
*
* \param[in]    uch_addr    - register address
* \param[in]    uch_data    - register data
*
* \retval       true on success
*/
{
  char ach_i2c_data[2];
  ach_i2c_data[0]=uch_addr;
  ach_i2c_data[1]=uch_data;

	I2C_WrBuf(I2C_DEV_ADDR, (uint8_t *) ach_i2c_data, 2);
  //if(HAL_I2C_Master_Transmit(&hi2c3, I2C_WRITE_ADDR, (uint8_t *) ach_i2c_data, 2, HAL_MAX_DELAY) == HAL_OK) {
	//printf("Register Write Successful\n\r");
	return true;
  //} else {
  //  printf("Register Write Failed\n\r");
  // return false;
  //}
}

bool maxim_max30102_read_reg(uint8_t uch_addr, uint8_t *puch_data)
/**
* \brief        Read a MAX30102 register
* \par          Details
*               This function reads a MAX30102 register
*
* \param[in]    uch_addr    - register address
* \param[out]   puch_data    - pointer that stores the register data
*
* \retval       true on success
*/
{
  char ch_i2c_data;
  ch_i2c_data=uch_addr;
	
	I2C_WrBuf(I2C_DEV_ADDR, (uint8_t *) &ch_i2c_data, 1);
	I2C_RdBufEasy(I2C_DEV_ADDR, (uint8_t *) &ch_i2c_data, 1);
	*puch_data=(uint8_t) ch_i2c_data;
	return true;
}

bool maxim_max30102_init()
/**
* \brief        Initialize the MAX30102
* \par          Details
*               This function initializes the MAX30102
*
* \param        None
*
* \retval       true on success
*/
{
  if(!maxim_max30102_write_reg(REG_INTR_ENABLE_1,0xc0)) // INTR setting
    return false;
  if(!maxim_max30102_write_reg(REG_INTR_ENABLE_2,0x00))
    return false;
  if(!maxim_max30102_write_reg(REG_FIFO_WR_PTR,0x00))  //FIFO_WR_PTR[4:0]
    return false;
  if(!maxim_max30102_write_reg(REG_OVF_COUNTER,0x00))  //OVF_COUNTER[4:0]
    return false;
  if(!maxim_max30102_write_reg(REG_FIFO_RD_PTR,0x00))  //FIFO_RD_PTR[4:0]
    return false;
  if(!maxim_max30102_write_reg(REG_FIFO_CONFIG,0x0f))  //sample avg = 1, fifo rollover=false, fifo almost full = 17
    return false;
  if(!maxim_max30102_write_reg(REG_MODE_CONFIG,0x03))   //0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
    return false;
  if(!maxim_max30102_write_reg(REG_SPO2_CONFIG,0x2B))  // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)
    return false;
  if(!maxim_max30102_write_reg(REG_LED1_PA,0x24))   //Choose value for ~ 7mA for LED1
    return false;
  if(!maxim_max30102_write_reg(REG_LED2_PA,0x24))   // Choose value for ~ 7mA for LED2
    return false;
  if(!maxim_max30102_write_reg(REG_PILOT_PA,0x7f))   // Choose value for ~ 25mA for Pilot LED
    return false;
	
  return true;
}

void maxim_max30102_read_temp(float *pun_temp) {	
	int8_t tempInt;
	uint8_t tempFrac;
	uint8_t response;
	
//	QF_CRIT_STAT_TYPE istat;
//	
//	QF_CRIT_ENTRY(istat);
	maxim_max30102_write_reg(REG_TEMP_CONFIG, 0x01);
	maxim_max30102_read_reg(REG_INTR_STATUS_2, &response);
	
	maxim_max30102_read_reg(REG_TEMP_INTR, (uint8_t *) &tempInt);
	maxim_max30102_read_reg(REG_TEMP_FRAC, &tempFrac);
	
	*pun_temp = (float)tempInt + ((float) tempFrac * 0.0625);
	
//	QF_CRIT_EXIT(istat);
}

bool maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
/**
* \brief        Read a set of samples from the MAX30102 FIFO register
* \par          Details
*               This function reads a set of samples from the MAX30102 FIFO register
*
* \param[out]   *pun_red_led   - pointer that stores the red LED reading data
* \param[out]   *pun_ir_led    - pointer that stores the IR LED reading data
*
* \retval       true on success
*/
{	
  uint32_t un_temp;
  unsigned char uch_temp;
	char ach_i2c_data[6];
//	QF_CRIT_STAT_TYPE istat;
//	QF_CRIT_ENTRY(istat);
	
  *pun_red_led=0;
  *pun_ir_led=0;
	
  // Read and clear status register
  maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_temp);
  maxim_max30102_read_reg(REG_INTR_STATUS_2, &uch_temp);

  ach_i2c_data[0]=REG_FIFO_DATA;
	
	I2C_WrBuf(I2C_DEV_ADDR, (uint8_t *) ach_i2c_data, 1);

	I2C_RdBufEasy(I2C_DEV_ADDR, (uint8_t *) ach_i2c_data, 6);
	
  un_temp=(unsigned char) ach_i2c_data[0];
  un_temp<<=16;
  *pun_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[1];
  un_temp<<=8;
  *pun_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[2];
  *pun_red_led+=un_temp;

  un_temp=(unsigned char) ach_i2c_data[3];
  un_temp<<=16;
  *pun_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[4];
  un_temp<<=8;
  *pun_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[5];
  *pun_ir_led+=un_temp;
  *pun_red_led&=0x03FFFF;  //Mask MSB [23:18]
  *pun_ir_led&=0x03FFFF;  //Mask MSB [23:18]
	
//	QF_CRIT_ENTRY(istat);
  return true;
}

bool maxim_max30102_reset()
/**
* \brief        Reset the MAX30102
* \par          Details
*               This function resets the MAX30102
*
* \param        None
*
* \retval       true on success
*/
{
    if(!maxim_max30102_write_reg(REG_MODE_CONFIG,0x40)) {
				printf("Reset Failed\n\r");
        return false;
    } else {
				printf("Reset Successful\n\r");
        return true;
    }
}

void I2C_LL_Init(void) {
  GPIO_InitTypeDef  GPIO_InitStructure;
  I2C_InitTypeDef   I2C_InitStructure;
  
  //Enable the i2c
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2Cx, ENABLE);
  //Reset the Peripheral
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2Cx, ENABLE);
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2Cx, DISABLE);
  
  //Enable the GPIOs for the SCL/SDA Pins
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIO_SCL | RCC_AHB1Periph_GPIO_SDA, ENABLE);
  
  //Configure and initialize the GPIOs
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SCL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIO_SCL, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SDA;
  GPIO_Init(GPIO_SDA, &GPIO_InitStructure);
  
  //Connect GPIO pins to peripheral
  GPIO_PinAFConfig(GPIO_SCL, GPIO_PinSource_SCL, GPIO_AF_I2Cx);
	GPIO_PinAFConfig(GPIO_SDA, GPIO_PinSource_SDA, GPIO_AF_I2Cx);
  
  //Configure and Initialize the I2C
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00; //We are the master. We don't need this
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = 100000;  //400kHz (Fast Mode)
  
  //Initialize the Peripheral
  I2C_Init(I2Cx, &I2C_InitStructure);
  // I2C Peripheral Enable
  I2C_Cmd(I2Cx, ENABLE);
  
  return; 
}

void GPIO_INT_Init(void) {
//	GPIO_InitTypeDef GPIO_InitStruct;
//	EXTI_InitTypeDef EXTI_InitStruct;
//	NVIC_InitTypeDef NVIC_InitStruct;
//	
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIO_INT, ENABLE);
//	RCC_AHB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
//	
//	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_INT;
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOC, &GPIO_InitStruct);
//	
//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);
//	
//	EXTI_InitStruct.EXTI_Line = EXTI_Line1;
//	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
//	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStruct);
//	
//	NVIC_InitStruct.NVIC_IRQChannel = EXTI1_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = QF_AWARE_ISR_CMSIS_PRI;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = QF_AWARE_ISR_CMSIS_PRI;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);
	GPIO_InitTypeDef GPIO_InitStruct;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIO_INT, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_INT;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void GPIO_LED_Init(void) {
	GPIO_InitTypeDef LED_GREEN_InitStruct;
	GPIO_InitTypeDef LED_ORANGE_InitStruct;
	GPIO_InitTypeDef LED_RED_InitStruct;
	GPIO_InitTypeDef LED_BLUE_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIO_LED, ENABLE);
	
	LED_GREEN_InitStruct.GPIO_Pin = GPIO_Pin_LED_GREEN;
	LED_GREEN_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	LED_GREEN_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	LED_GREEN_InitStruct.GPIO_OType = GPIO_OType_PP;
	LED_GREEN_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	
	LED_ORANGE_InitStruct.GPIO_Pin = GPIO_Pin_LED_ORANGE;
	LED_ORANGE_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	LED_ORANGE_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	LED_ORANGE_InitStruct.GPIO_OType = GPIO_OType_PP;
	LED_ORANGE_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	
	LED_RED_InitStruct.GPIO_Pin = GPIO_Pin_LED_RED;
	LED_RED_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	LED_RED_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	LED_RED_InitStruct.GPIO_OType = GPIO_OType_PP;
	LED_RED_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	
	LED_BLUE_InitStruct.GPIO_Pin = GPIO_Pin_LED_BLUE;
	LED_BLUE_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	LED_BLUE_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	LED_BLUE_InitStruct.GPIO_OType = GPIO_OType_PP;
	LED_BLUE_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	
	GPIO_Init(GPIO_LED, &LED_GREEN_InitStruct);
	GPIO_Init(GPIO_LED, &LED_ORANGE_InitStruct);
	GPIO_Init(GPIO_LED, &LED_RED_InitStruct);
	GPIO_Init(GPIO_LED, &LED_BLUE_InitStruct);
}

void GPIO_BUTTON_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIO_INT, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_BUTTON;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIO_BUTTON, &GPIO_InitStruct);
}

uint32_t I2C_WrData(uint8_t DevAddr, uint8_t RegAddr, uint8_t data){
  //Write a single byte data to the given register address
  
  //Generate a Start condition
  I2C_Start();
  
  //Send I2C device Address and clear ADDR
  I2C_Addr(DevAddr, I2C_Direction_Transmitter);
  (void) I2Cx->SR2;
  
  //Send Data
  I2Cx->DR = data;
  WaitSR1FlagsSet(I2C_SR1_BTF);  //wait till the data is actually written.
  
  //Generate Stop
  I2Cx->CR1 |= I2C_CR1_STOP;
  
  //Wait to be sure that line is iddle
  WaitLineIdle();
  
  return 0;
}


uint32_t I2C_RdData(uint8_t DevAddr, uint8_t RegAddr, uint8_t *buf, uint32_t cnt) {
  //Reads "cnt" number of data starting from RegAddr
  
  //Send the Register Addres
  I2C_Start();
  I2C_Addr(DevAddr, I2C_Direction_Transmitter);
  (void) I2Cx->SR2;
  I2Cx->DR = RegAddr;
  WaitSR1FlagsSet(I2C_SR1_BTF);
  
  //Start Reading
  I2C_RdBuf(DevAddr, buf, cnt);
  
  return 0;
}


uint32_t I2C_WrBuf(uint8_t DevAddr, uint8_t *buf, uint32_t cnt) {
  
  //Generate a Start condition
  I2C_Start();
  
  //Send I2C device Address
  I2C_Addr(DevAddr, I2C_Direction_Transmitter);
  //Unstretch the clock by just reading SR2 (Physically the clock is continued to be strectehed because we have not written anything to the DR yet.)
  (void) I2Cx->SR2; 
  
  //Start Writing Data
  while (cnt--) {
    I2C_Write(*buf++);
  }
  
  //Wait for the data on the shift register to be transmitted completely
  WaitSR1FlagsSet(I2C_SR1_BTF);
  //Here TXE=BTF=1. Therefore the clock stretches again.
  
  //Order a stop condition at the end of the current tranmission (or if the clock is being streched, generate stop immediatelly)
  I2Cx->CR1 |= I2C_CR1_STOP;
  //Stop condition resets the TXE and BTF automatically.
  
  //Wait to be sure that line is iddle
  WaitLineIdle();
  
  return 0;
}


uint32_t I2C_RdBufEasy (uint8_t DevAddr, uint8_t *buf, uint32_t cnt) {
  //The easy read.
  //We assume that we will reset ACK and order a stop condition while the last byte is being received by the shift register.
  //If this can't be done on time (during last byte reception), the slave will continue to send at least 1 more byte than cnt.
  //In most cases, such a condition does not hurt at all. Therefore people uses this method exclusively.
  
  //Note that it is impossible to guarantee the timig requirement only for single byte reception.
  //For N>=2, the timing is almost always satisfied. (if there is no interrupt, it will definetely be satisfied)
  
  //Generate Start
  I2C_Start();
  
  //Send I2C Device Address and clear ADDR
  I2C_Addr(DevAddr, I2C_Direction_Receiver);
  (void)I2Cx->SR2;
  
  while ((cnt--)>1) {
    I2C_Read(buf++);
  }
  
  //At this point we assume last byte is being received by the shift register. (reception has not been completed yet)
  //Reset ack
  I2Cx->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_ACK);
  
  //Order a stop condition
  I2Cx->CR1 |= I2C_CR1_STOP;
  
  //Now read the final byte
  I2C_Read(buf);
  
  //Make Sure Stop bit is cleared and Line is now Iddle
  WaitLineIdle();
    
  //Enable the Acknowledgement
  I2Cx->CR1 |= ((uint16_t)I2C_CR1_ACK);
  
  return 0;
}


uint32_t I2C_RdBuf (uint8_t DevAddr, uint8_t *buf, uint32_t cnt) {
  //Generate Start
  I2C_Start();
  
  //Send I2C Device Address
  I2C_Addr(DevAddr, I2C_Direction_Receiver);
  
  if (cnt==1) {//We are going to read only 1 byte
    //Before Clearing Addr bit by reading SR2, we have to cancel ack.
    I2Cx->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_ACK);
    
    //Now Read the SR2 to clear ADDR
    (void)I2Cx->SR2;
    
    //Order a STOP condition
    //Note: Spec_p583 says this should be done just after clearing ADDR
    //If it is done before ADDR is set, a STOP is generated immediately as the clock is being streched
    I2Cx->CR1 |= I2C_CR1_STOP;
    //Be carefull that till the stop condition is actually transmitted the clock will stay active even if a NACK is generated after the next received byte.
    
    //Read the next byte
    I2C_Read(buf);
    
    //Make Sure Stop bit is cleared and Line is now Iddle
    WaitLineIdle();
    
    //Enable the Acknowledgement again
    I2Cx->CR1 |= ((uint16_t)I2C_CR1_ACK);
  }
  
  else if (cnt==2) {  //We are going to read 2 bytes (See: Spec_p584)
    //Before Clearing Addr, reset ACK, set POS
    I2Cx->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_ACK);
    I2Cx->CR1 |= I2C_CR1_POS;
    
    //Read the SR2 to clear ADDR
    (void)I2Cx->SR2;
    
    //Wait for the next 2 bytes to be received (1st in the DR, 2nd in the shift register)
    WaitSR1FlagsSet(I2C_SR1_BTF);
    //As we don't read anything from the DR, the clock is now being strecthed.
    
    //Order a stop condition (as the clock is being strecthed, the stop condition is generated immediately)
    I2Cx->CR1 |= I2C_CR1_STOP;
    
    //Read the next two bytes
    I2C_Read(buf++);
    I2C_Read(buf);
    
    //Make Sure Stop bit is cleared and Line is now Iddle
    WaitLineIdle();
    
    //Enable the ack and reset Pos
    I2Cx->CR1 |= ((uint16_t)I2C_CR1_ACK);
    I2Cx->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_POS);
  }
  else { //We have more than 2 bytes. See spec_p585
    //Read the SR2 to clear ADDR
    (void)I2Cx->SR2;
     
    while((cnt--)>3) {//Read till the last 3 bytes
      I2C_Read(buf++);
    }
    
    //3 more bytes to read. Wait till the next to is actually received
    WaitSR1FlagsSet(I2C_SR1_BTF);
    //Here the clock is strecthed. One more to read.
    
    //Reset Ack
    I2Cx->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_ACK);
    
    //Read N-2
    I2C_Read(buf++);
    //Once we read this, N is going to be read to the shift register and NACK is generated
    
    //Wait for the BTF
    WaitSR1FlagsSet(I2C_SR1_BTF); //N-1 is in DR, N is in shift register
    //Here the clock is stretched
    
    //Generate a stop condition
    I2Cx->CR1 |= I2C_CR1_STOP;
    
    //Read the last two bytes (N-1 and N)
    //Read the next two bytes
    I2C_Read(buf++);
    I2C_Read(buf);
    
    //Make Sure Stop bit is cleared and Line is now Iddle
    WaitLineIdle();
    
    //Enable the ack
    I2Cx->CR1 |= ((uint16_t)I2C_CR1_ACK);
  }
  
  return 0;
}

///////////////PRIVATE FUNCTIONS/////////////////////
static uint32_t I2C_Read(uint8_t *pBuf) {
    uint32_t err;
    
    //Wait till new data is ready to be read
    err=WaitSR1FlagsSet(I2C_SR1_RXNE);
        
    if (!err) {
      *pBuf = I2Cx->DR;   //This clears the RXNE bit. IF both RXNE and BTF is set, the clock stretches
      return 0;
    }
    else {return err;}
}


static uint32_t I2C_Write(uint8_t byte){
  
  //Write the byte to the DR
  I2Cx->DR = byte;
  
  //Wait till the content of DR is transferred to the shift Register.
  return WaitSR1FlagsSet(I2C_SR1_TXE);
  //At this point point DR is available for the next byte even if it is not actually transmitted from the shift register
  //TXE will be reset automatically when DR is written again (TXE does not strecth the clock unless BTF is also set)
}

static uint32_t I2C_Addr(uint8_t DevAddr, uint8_t dir) {
  
  //Write address to the DR (to the bus)
  I2Cx->DR = (DevAddr << 1) | dir;
  
  //Wait till ADDR is set (ADDR is set when the slave sends ACK to the address).
  //Clock streches till ADDR is Reset. To reset the hardware i)Read the SR1 ii)Wait till ADDR is Set iii)Read SR2
  //Note1:Spec_p602 recommends the waiting operation
  //Note2:We don't read SR2 here. Therefore the clock is going to be streched even after return from this function
  return WaitSR1FlagsSet(I2C_SR1_ADDR); 
}


static uint32_t I2C_Start(void) {
  
  //Generate a start condition. (As soon as the line becomes idle, a Start condition will be generated)
  I2Cx->CR1 |= I2C_CR1_START;
  
  //When start condition is generated SB is set and clock is stretched.
  //To activate the clock again i)read SR1 ii)write something to DR (e.g. address)
  return WaitSR1FlagsSet(I2C_SR1_SB);  //Wait till SB is set
}


static uint32_t WaitSR1FlagsSet (uint32_t Flags) {
  //Wait till the specified SR1 Bits are set
  //More than 1 Flag can be "or"ed. This routine reads only SR1.
  uint32_t TimeOut = HSI_VALUE;
  while(!((I2Cx->SR1) & Flags)) {
    if (!(TimeOut--)) {
			printf("I2C Error\n\r");
      return 1;
    }
  } 
  return 0;
}


static uint32_t WaitLineIdle(void) {
  //Wait till the Line becomes idle.
  
  uint32_t TimeOut = HSI_VALUE;
  //Check to see if the Line is busy
  //This bit is set automatically when a start condition is broadcasted on the line (even from another master)
  //and is reset when stop condition is detected.
  while((I2Cx->SR2) & (I2C_SR2_BUSY)) {
    if (!(TimeOut--)) {
      return 1;
    }
  }
  return 0;
}
