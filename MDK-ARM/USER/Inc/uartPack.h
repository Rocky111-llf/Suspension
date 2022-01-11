/**
 * @file uartPack.h
 * @brief see uartPack.c for details.
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2021-12-19
 *
 * THINK DIFFERENTLY
 */

#ifndef _UART_PACK_H_
#define _UART_PACK_H_
#include "main.h"
// private define
#define _REDIRECT_UART_PORT huart1 //重定向串口目标

//typedef
// typedef char* va_list;

// constants
#define RX_BUFFER_SIZE 128
#define _RX_DEFAILT_TIMEOUT 10
#define _RX_DEFAILT_ENDBIT '\n'

// typedef
typedef struct {                      //超时型UART控制结构体
  __IO uint8_t rxFlag;                //正在接收标志位
  __IO uint8_t rxSaveFlag;            //接收完成标志位
  uint8_t rxData[2];                  //接收到的数据
  uint8_t rxBuf[RX_BUFFER_SIZE];      //接收缓冲区
  __IO uint8_t rxBufIndex;            //接收缓冲区索引
  __IO uint8_t rxSaveCounter;         //接收保存区计数器
  uint8_t rxSaveBuf[RX_BUFFER_SIZE];  //接收保存缓冲区
  __IO uint32_t rxTick;               //接收超时计时器
  uint32_t rxTimeout;                 //接收超时时间
} uart_o_ctrl_t;

typedef struct {                      //单结束位型UART控制结构体
  __IO uint8_t rxFlag;                //正在接收标志位
  __IO uint8_t rxSaveFlag;            //接收完成标志位
  uint8_t rxData[2];                  //接收到的数据
  uint8_t rxBuf[RX_BUFFER_SIZE];      //接收缓冲区
  __IO uint8_t rxBufIndex;            //接收缓冲区索引
  __IO uint8_t rxSaveCounter;         //接收保存区计数器
  uint8_t rxSaveBuf[RX_BUFFER_SIZE];  //接收保存缓冲区
  uint8_t rxEndBit;                   //接收结束位
} uart_e_ctrl_t;

// public functions

int printft(UART_HandleTypeDef *huart, char *fmt, ...);
void Enable_Uart_O_Control(UART_HandleTypeDef *huart, uart_o_ctrl_t *ctrl);
void Enable_Uart_E_Control(UART_HandleTypeDef *huart, uart_e_ctrl_t *ctrl);
uint8_t Uart_O_Data_Process(UART_HandleTypeDef *huart, uart_o_ctrl_t *ctrl);
uint8_t Uart_O_Timeout_Check(UART_HandleTypeDef *huart, uart_o_ctrl_t *ctrl);
uint8_t Uart_E_Data_Process(UART_HandleTypeDef *huart, uart_e_ctrl_t *ctrl);
#endif
