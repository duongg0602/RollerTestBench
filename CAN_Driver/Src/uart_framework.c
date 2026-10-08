#include "uart_framework.h"
#include <string.h>

/* huart1 is defined in main.c */
extern UART_HandleTypeDef huart1;


/* =========================================================
 * RX
 * ========================================================= */

static uint8_t uartRxByte;

static uint8_t rxBuffer[RX_BUFFER_SIZE + 1];

static uint16_t head = 0;
static uint16_t tail = 0;


/* ================= Parser ================= */

static Parser_State_t parser_state = WAIT_SOF;

static uint8_t parserPayload[MAX_PAYLOAD_RX_SIZE];
static uint8_t parserPayloadLen;
static uint8_t parserSeq;
static uint8_t parserIdx;


/* ================= RX Output ================= */

static UART_Output_t rxOutput;


/* =========================================================
 * TX
 * ========================================================= */

static UART_Input_t txInput;

static uint8_t txFrame[MAX_FRAME_TX_SIZE];

static uint8_t txSeq = 0;


/* =========================================================
 * UART INIT
 * ========================================================= */

void UART1_Init(void)
{
    /* Start receiving one byte using interrupt */
    HAL_UART_Receive_IT(&huart1, &uartRxByte, 1);
}


/* =========================================================
 * RX CALLBACK
 * ========================================================= */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint16_t nextHead =
            (head + 1) % (RX_BUFFER_SIZE + 1);

        if (nextHead != tail)
        {
            rxBuffer[head] = uartRxByte;
            head = nextHead;
        }

        /* Re-arm UART reception */
        HAL_UART_Receive_IT(&huart1, &uartRxByte, 1);
    }
}


/* =========================================================
 * PARSER
 * ========================================================= */

void UART1_Parse(void)
{
    while (tail != head)
    {
        uint8_t byte = rxBuffer[tail];

        tail = (tail + 1) % (RX_BUFFER_SIZE + 1);

        switch (parser_state)
        {
            /* -------------------------------------------------
             * WAIT FOR SOF
             * ------------------------------------------------- */

            case WAIT_SOF:

                if (byte == SOF_fr)
                {
                    parserIdx = 0;
                    parser_state = IDENTIFY_LEN;
                }

                break;


            /* -------------------------------------------------
             * IDENTIFY PAYLOAD LENGTH
             * ------------------------------------------------- */

            case IDENTIFY_LEN:

                if (byte <= MAX_PAYLOAD_RX_SIZE)
                {
                    parserPayloadLen = byte;
                    parserIdx = 0;

                    parser_state = READ_FRAME;
                }
                else
                {
                    parser_state = WAIT_SOF;
                }

                break;


            /* -------------------------------------------------
             * READ PAYLOAD + SEQ + EOF
             * ------------------------------------------------- */

            case READ_FRAME:

                if (parserIdx < parserPayloadLen)
                {
                    /* Read payload */
                    parserPayload[parserIdx] = byte;
                    parserIdx++;
                }
                else if (parserIdx == parserPayloadLen)
                {
                    /* Read sequence number */
                    parserSeq = byte;
                    parserIdx++;
                }
                else
                {
                    /* Read EOF */

                    if (byte == EOF_fr)
                    {
                        /* Complete valid frame */

                        memcpy(rxOutput.payload,
                               parserPayload,
                               parserPayloadLen);

                        rxOutput.payload_len = parserPayloadLen;
                        rxOutput.seq = parserSeq;
                        rxOutput.newData = 1;
                    }

                    /* Frame finished */
                    parser_state = WAIT_SOF;
                }

                break;
        }
    }
}


/* =========================================================
 * GET RECEIVED DATA
 * ========================================================= */

bool UART1_GetReceivedData(UART_Output_t *data)
{
    if (rxOutput.newData == 0)
    {
        return false;
    }

    __disable_irq();

    *data = rxOutput;

    rxOutput.newData = 0;

    __enable_irq();

    return true;
}


/* =========================================================
 * SEND EXPECTED DATA
 * ========================================================= */

bool UART1_SendExpectedData(uint8_t *data, uint8_t len)
{
    if (len > MAX_PAYLOAD_TX_SIZE)
    {
        return false;
    }

    __disable_irq();

    memcpy(txInput.payload, data, len);

    txInput.payload_len = len;

    txInput.newData = 1;

    __enable_irq();

    return true;
}


/* =========================================================
 * TX
 * ========================================================= */

void UART1_Send(void)
{
    if (txInput.newData == 0)
    {
        return;
    }

    uint8_t len = txInput.payload_len;


    /* ---------------------------------------------------------
     * Build frame
     *
     * SOF | DataLen | Payload | Seq | EOF
     * --------------------------------------------------------- */

    txFrame[0] = SOF_fr;

    txFrame[1] = len;

    memcpy(&txFrame[2],
           txInput.payload,
           len);

    txFrame[2 + len] = txSeq;

    txFrame[3 + len] = EOF_fr;


    uint8_t frameLen = len + 4;


    /*
     * Request has been consumed.
     *
     * Even if HAL_UART_Transmit_IT() returns BUSY,
     * this request is discarded.
     */

    txInput.newData = 0;


    /* Start asynchronous transmission */

    if (HAL_UART_Transmit_IT(&huart1,
                             txFrame,
                             frameLen) == HAL_OK)
    {
        txSeq++;
    }
}


/* =========================================================
 * TX COMPLETE CALLBACK
 * ========================================================= */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /*
         * Transmission of the complete frame is finished.
         *
         * No additional action is required.
         */
    }
}
