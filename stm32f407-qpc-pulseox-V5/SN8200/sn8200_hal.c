#include "stm32f4xx.h"
#include <stdbool.h>

#define RX_BUFFER_SIZE 4096
uint8_t RxBuffer[RX_BUFFER_SIZE];
uint32_t RxBufferTail = 0;
uint32_t RxBufferHead = 0;


void SN8200_HAL_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable GPIO clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* Enable UART clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);


    /* Connect PXx to USARTx_Tx*/
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);

    /* Connect PXx to USARTx_Rx*/
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

    /* Configure USART Tx as alternate function  */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Configure USART Rx as alternate function  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* USART configuration */
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* Enable USART */
    USART_Cmd(USART1, ENABLE);
}


void SN8200_HAL_SendData(unsigned char *buf, int len)
{
    int i;

    for (i = 0; i < len; i++) {
        USART_SendData(USART1, buf[i]);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    }
}

bool SN8200_HAL_RxBufferEmpty(void)
{
    return (RxBufferHead == RxBufferTail);
}

uint8_t SN8200_HAL_ReadByte(void)
{
    uint8_t data = 0;

    if(RxBufferHead != RxBufferTail) {
        data = RxBuffer[RxBufferTail];
        RxBufferTail = (RxBufferTail + 1) % RX_BUFFER_SIZE;
    }

    return data;
}

void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        RxBuffer[RxBufferHead] = USART_ReceiveData(USART1);
        RxBufferHead = (RxBufferHead + 1) % RX_BUFFER_SIZE;
    }
}
