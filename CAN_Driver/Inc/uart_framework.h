#ifndef UART_FRAMEWORK_H
#define UART_FRAMEWORK_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ================= CONFIG ================= */

#define MAX_PAYLOAD_RX_SIZE    30
#define MAX_PAYLOAD_TX_SIZE    80
#define MAX_FRAME_TX_SIZE      (MAX_PAYLOAD_TX_SIZE + 4)
#define RX_BUFFER_SIZE         256

#define SOF_fr                 0xAA
#define EOF_fr                 0x55


/* ================= PARSER STATE ================= */

typedef enum
{
    WAIT_SOF,
    IDENTIFY_LEN,
    READ_FRAME

} Parser_State_t;


/* ================= RX DATA ================= */

typedef struct
{
    uint8_t payload[MAX_PAYLOAD_RX_SIZE];
    uint8_t payload_len;
    uint8_t seq;
    uint8_t newData;

} UART_Output_t;


/* ================= TX DATA ================= */

typedef struct
{
    uint8_t payload[MAX_PAYLOAD_TX_SIZE];
    uint8_t payload_len;
    uint8_t newData;

} UART_Input_t;


/* ================= API ================= */

void UART1_Init(void);

void UART1_Parse(void);

bool UART1_GetReceivedData(UART_Output_t *data);

bool UART1_SendExpectedData(uint8_t *data, uint8_t len);

void UART1_Send(void);

#endif /* UART_FRAMEWORK_H */
