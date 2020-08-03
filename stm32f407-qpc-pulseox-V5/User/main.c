/******************** (C) COPYRIGHT 2009 Embest Info&Tech Co.,LTD. ************
* File Name          : main.c
* Author             : Wuhan R&D Center, Embest
* Date First Issued  : 28/03/2013
* Description        : Main program body
*******************************************************************************
*******************************************************************************
* History:
* 28/03/2013		 : V1		   initial version
* 13/06/2013		 : V2
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
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
#include "bsp.h"

Q_DEFINE_THIS_FILE

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MAX_BRIGHTNESS 255
#define DBGU_RX_BUFFER_SIZE 256
#define TEST_BUFFERSIZE 128
#define UDP_NUM_PKT 10

//GPIO and I2C Peripheral (I2C3 Configuration)
#define I2Cx                      I2C3  //Selected I2C peripheral
#define RCC_APB1Periph_I2Cx       RCC_APB1Periph_I2C3 //Bus where the peripheral is connected
#define RCC_AHB1Periph_GPIO_SCL   RCC_AHB1Periph_GPIOA  //Bus for GPIO Port of SCL
#define RCC_AHB1Periph_GPIO_SDA   RCC_AHB1Periph_GPIOC  //Bus for GPIO Port of SDA
#define RCC_AHB1Periph_GPIO_INT		RCC_AHB1Periph_GPIOC	//Bus for GPIO Port of INT
#define GPIO_AF_I2Cx              GPIO_AF_I2C3    //Alternate function for GPIO pins
#define GPIO_SCL                  GPIOA
#define GPIO_SDA                  GPIOC
#define GPIO_INT									GPIOC
#define GPIO_Pin_SCL              GPIO_Pin_8
#define GPIO_Pin_SDA              GPIO_Pin_9
#define GPIO_Pin_INT							GPIO_Pin_1
#define GPIO_PinSource_SCL        GPIO_PinSource8
#define GPIO_PinSource_SDA        GPIO_PinSource9
#define GPIO_PinSource_INT				GPIO_PinSource1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
char rxData[30];													// UART RX Data
char txData[128];													// UART TX Data

extern uint32_t elapsedTime, timeStart;
extern uint32_t aun_ir_buffer[BUFFER_SIZE];			// IR LED Sensor Data
extern uint32_t aun_red_buffer[BUFFER_SIZE];			// Red LED Sensor Data
extern float old_n_spO2;													// SpO2 Value
extern uint8_t uch_dummy, k;											// Dummy variable to clear status register

uint8_t DBGU_RxBuffer[DBGU_RX_BUFFER_SIZE];
uint32_t DBGU_RxBufferTail = 0;
uint32_t DBGU_RxBufferHead = 0;
bool DBGU_InputReady = false;
bool quit_flag = false;

uint8_t key;
uint8_t seqNo = 0;

int8_t mysock = -1;
int8u TxBuf[TEST_BUFFERSIZE];

extern int ipok, joinok;
extern int destIP, srcIP;
extern long int destPort, srcPort;
extern int32u pktcnt;

extern char domain[100];
extern char Portstr[8];
char uri[100]={0};
char sockConnected = -1;
char sockClosed = -1;
int timeout1 = 5;
extern bool IsCreateSocketResponsed ;
extern int32u timeout;
extern bool IsWIFIJoinResponsed ;

float n_spO2, ratio, correl;  	// SpO2 Value
int8_t ch_spO2_valid;  					// SpO2 Valid Flag
int32_t n_heart_rate; 					// Heart Rate Value
int8_t  ch_hr_valid;  					// Heart Rate Valid Flag
float temp_c;										// Temperature in Celsius
char alarmFlag;														// Alarm Flag
int32_t i;
char sockstr[8];
int32u sock;
int len;



#define GET_REQUEST \
    "GET / HTTP/1.1\r\n" \
    "Host: 192.168.2.125\r\n" \
    "Accept: text/html\r\n" \
    "\r\n"

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

void DBGU_Init(void);
bool DBGU_RxBufferEmpty(void);
uint8_t DBGU_GetChar(void);

QXSemaphore finger_valid_sema;

uint32_t stack_Thread1[1000];
QXThread Thread1;
void main_Thread1(QXThread* const me) {	
	while (1) {
		//printf("Hello from thread 1\r\n");
		BSP_ledBlueToggle();
		
		maxim_max30102_read_fifo((aun_red_buffer), (aun_ir_buffer));
		
		if (aun_red_buffer[0] < 110000) {
			//printf("Place finger on sensor...\n\r");
		} else {
			QXSemaphore_signal(&finger_valid_sema);
		}
	}
}

uint32_t stack_Thread2[2000];
QXThread Thread2;
void main_Thread2(QXThread* const me) {
	while (1) {
		QXSemaphore_wait(&finger_valid_sema, QXTHREAD_NO_TIMEOUT);
		//printf("Hello from thread 2\r\n");
		BSP_ledRedToggle();
		
		// Store ST seconds of samples at FS samples per second into buffer of length BUFFER_SIZE
		// Read BUFFER_SIZE samples, and determine the signal range
		for (i = 0; i < BUFFER_SIZE; i++) {
			while(GPIO_ReadInputData(GPIO_INT) & GPIO_Pin_INT);			// Wait for interrupt pin to assert
			maxim_max30102_read_fifo((aun_red_buffer + i), (aun_ir_buffer + i));
			maxim_max30102_read_temp(&temp_c);
		}
		// Calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples)
		rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spO2, &ch_spO2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl);
		alarmFlag = n_spO2 < 95.0 && ch_spO2_valid;
		sprintf(txData, "%i,%i,%f,%i,%f,%i,\n\r", n_heart_rate, ch_hr_valid, n_spO2, ch_spO2_valid, temp_c, alarmFlag);
		// Transmit TCP packet every 500 samples
		len = (int)strlen(txData);
		sendFromSock(sock, (int8u*)txData, len, 2, seqNo++);
		printf(txData);
	}
}

uint32_t stack_Thread3[1000];
QXThread Thread3;
void main_Thread3(QXThread* const me) {
	while (1) {
		printf("Hello from thread 3\r\n");
		BSP_ledGreenToggle();
		
		if(SN8200_API_HasInput()) {
				ProcessSN8200Input();
		}

		if(quit_flag)
				break;
		
		QXThread_delay(10);
	}
}

uint32_t stack_Thread4[200];
QXThread Thread4;
void main_Thread4(QXThread* const me) {
	while (1) {
		printf("Hello from thread 4\r\n");
		BSP_ledOrangeToggle();
		QXThread_delay(10U);
	}
}


/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
		uint32_t volatile run = 0U;
		
		// Initialization ----------------------------------------------------------	
		DBGU_Init();
		I2C_LL_Init();
		GPIO_INT_Init();
		GPIO_LED_Init();
		GPIO_BUTTON_Init();
    SN8200_API_Init(921600);
		maxim_max30102_reset();									// Reset the MAX30102
		maxim_max30102_read_reg(0, &uch_dummy); // Clear registers
		maxim_max30102_init();									// Initialize MAX30102
		old_n_spO2 = 0.0;												// Initialize old SpO2 Value

		// Establish Wifi and TCP Server -------------------------------------------
    WifiOn(seqNo++);
    printf("\n\r");
		WifiDisconn(seqNo++);
		printf("\n\r");
    WifiJoin(seqNo++);
		printf("\n\r");
    SnicInit(seqNo++);
		printf("\n\r");
    SnicIPConfig(seqNo++);
		printf("\n\r");
		mysock = -1;
		tcpCreateSocket(0, 0xFF, 0xFF, seqNo++, SNIC_TCP_CREATE_SOCKET_REQ);
		if (mysock != -1) {
			// This connection can receive data upto 0x0400=1K bytes at a time.
			getTCPinfo();
			tcpConnectToServer(mysock, destIP, (unsigned short)destPort, 0x0400, 0x5, seqNo++);
		}
		sprintf(sockstr, "4");
		sock = strtol(sockstr, NULL, 0);
		
		// Begin Multi-Threading --------------------------------------------------
		QF_init();
		
		QXSemaphore_init(&finger_valid_sema, 0U, 1U);
		
		QXThread_ctor(&Thread1, &main_Thread1, 0);
		QXTHREAD_START(&Thread1,
									 3U,
									 (void*)0, 0,
									 stack_Thread1, sizeof(stack_Thread1),
									 (void*)0);
									 
		QXThread_ctor(&Thread2, &main_Thread2, 0);
		QXTHREAD_START(&Thread2,
									 4U,
									 (void*)0, 0,
									 stack_Thread2, sizeof(stack_Thread2),
									 (void*)0);
									 
		QXThread_ctor(&Thread3, &main_Thread3, 0);
		QXTHREAD_START(&Thread3,
									 2U,
									 (void*)0, 0,
									 stack_Thread3, sizeof(stack_Thread3),
									 (void*)0);
									 
		QXThread_ctor(&Thread4, &main_Thread4, 0);
		QXTHREAD_START(&Thread4,
									 1U,
									 (void*)0, 0,
									 stack_Thread4, sizeof(stack_Thread4),
									 (void*)0);
									 
		// Transfer control to qpc to run threads
		QF_run();
}

// SN8200 MODULES
void DBGU_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable GPIO clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);


    /* Connect PXx to USARTx_Tx*/
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

    /* Connect PXx to USARTx_Rx*/
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    /* Configure USART Tx as alternate function  */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure USART Rx as alternate function  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    /* Enable the USART2 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* USART configuration */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART2, &USART_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    /* Enable USART */
    USART_Cmd(USART2, ENABLE);
}


bool DBGU_RxBufferEmpty(void)
{
    return (DBGU_RxBufferHead == DBGU_RxBufferTail);
}


void USART2_IRQHandler(void)
{
    uint8_t ch;
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        ch = USART_ReceiveData(USART2);
        switch (ch) {
        case 0x7F:
            if(DBGU_RxBufferHead != DBGU_RxBufferTail) {
                DBGU_RxBufferHead = (DBGU_RxBufferHead - 1) % DBGU_RX_BUFFER_SIZE;
                USART_SendData(USART2, 0x7F);
            }
            break;
        case 0x0D:
            DBGU_RxBuffer[DBGU_RxBufferHead] = ch;
            USART_SendData(USART2, 0x0D);
            DBGU_RxBufferHead = (DBGU_RxBufferHead + 1) % DBGU_RX_BUFFER_SIZE;
            DBGU_InputReady = true;
            break;
        default:
            DBGU_RxBuffer[DBGU_RxBufferHead] = ch;
            USART_SendData(USART2, ch);
            DBGU_RxBufferHead = (DBGU_RxBufferHead + 1) % DBGU_RX_BUFFER_SIZE;
            break;
        }
    }
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE {
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    USART_SendData(USART2, (uint8_t) ch);

    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) {
    }

    return ch;
}

uint8_t DBGU_GetChar(void)
{
    uint8_t ch = 0;

    if(DBGU_RxBufferHead != DBGU_RxBufferTail) {
        ch = DBGU_RxBuffer[DBGU_RxBufferTail];
        DBGU_RxBufferTail = (DBGU_RxBufferTail + 1) % DBGU_RX_BUFFER_SIZE;
    } else {
        DBGU_InputReady = false;
    }

    return ch;
}


int fgetc(FILE *f)
{
    uint8_t ch = 0;

    while (!DBGU_InputReady);
    while(DBGU_RxBufferHead == DBGU_RxBufferTail);
    ch = DBGU_RxBuffer[DBGU_RxBufferTail];
    DBGU_RxBufferTail = (DBGU_RxBufferTail + 1) % DBGU_RX_BUFFER_SIZE;
    if (DBGU_RxBufferHead == DBGU_RxBufferTail)
        DBGU_InputReady = false;

    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);

    return ch;
}

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
    while (1) {
    }
}
#endif


/************* (C) COPYRIGHT 2013 Wuhan R&D Center, Embest *****END OF FILE****/
