#ifndef __SN8200_API_H
#define __SN8200_API_H
#include <stdbool.h>
#include <rtl.h>
#include <stdint.h>

#define PORT_NONE 0
#define CMD_ERROR -1
#define SUB_CMD_RESP_MASK 0x80 // Bit 7: 0 for original command, 1 for response 
#define MAX_CONNECTION_PER_SOCK 4   // max connection per listening socket
#define MAX_BUFFER_SIZE 0x800

typedef  unsigned char   u8_t;
typedef  signed char     s8_t;
typedef  unsigned short  u16_t;
typedef  signed short    s16_t;
typedef  unsigned long   u32_t;
typedef  signed long     s32_t;


#define int8u unsigned char
#define int8s char
#define int16u unsigned short
#define int16s short
#define int32u unsigned int
#define int32s int


int inet_aton(const char *cp, struct in_addr *addr);
u32_t inet_addr(const char *cp);


int getTCPinfo(void);
int setTCPinfo(void);

void SN8200_API_Init(uint32_t baudrate);
bool SN8200_API_HasInput(void);
void WifiOn(int8u seq);
void WifiOff(int8u seq);
void ApOnOff(int8u seq);
void WifiJoin(int8u seq);
void WifiDisconn(int8u seq);
void SnicInit(int8u seq);
void SnicCleanup(int8u seq);
void SnicIPConfig(int8u seq);
void SnicGetDhcp(int8u seq);
int tcpCreateSocket(uint8_t bindOption, uint32_t localIp, uint16_t port, uint8_t seq, int8u ssl);
int closeSocket(uint8_t shortSocket, uint8_t seq);
int tcpConnectToServer(int8u shortSock, int32u ip, int16u port, int16u bufsize, int8u timeout, int8u seq);
int sendFromSock(int8u shortSocket, int8u * sendBuf, int16u len, int8u timeout, int8u seq);
int tcpCreateConnection(int8u shortSock, int16u size, int8u maxClient, int8u seq);

void ProcessSN8200Input(void);
void handleRxWiFi(int8u* buf, int len);
void handleRxSNIC(int8u* buf, int len);
#endif
