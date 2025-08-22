/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

#ifndef MCP2515_H
#define MCP2515_H

#include "digio.h"
#include <libopencm3/stm32/spi.h>
#include <stdbool.h>
#include <stdint.h>

// MCP2515 SPI Instruction Set
#define MCP2515_RESET 0xC0

#define MCP2515_READ 0x03
#define MCP2515_READ_RXB0SIDH 0x90
#define MCP2515_READ_RXB0D0 0x92
#define MCP2515_READ_RXB1SIDH 0x94
#define MCP2515_READ_RXB1D0 0x96

#define MCP2515_WRITE 0x02
#define MCP2515_LOAD_TXB0SIDH 0x40
#define MCP2515_LOAD_TXB0D0 0x41
#define MCP2515_LOAD_TXB1SIDH 0x42
#define MCP2515_LOAD_TXB1D0 0x43
#define MCP2515_LOAD_TXB2SIDH 0x44
#define MCP2515_LOAD_TXB2D0 0x45

#define MCP2515_RTS_TX0 0x81
#define MCP2515_RTS_TX1 0x82
#define MCP2515_RTS_TX2 0x84
#define MCP2515_RTS_ALL 0x87
#define MCP2515_READ_STATUS 0xA0
#define MCP2515_RX_STATUS 0xB0
#define MCP2515_BIT_MOD 0x05

// MCP25152515 Register Adresses
#define MCP2515_RXF0SIDH 0x00
#define MCP2515_RXF0SIDL 0x01
#define MCP2515_RXF0EID8 0x02
#define MCP2515_RXF0EID0 0x03
#define MCP2515_RXF1SIDH 0x04
#define MCP2515_RXF1SIDL 0x05
#define MCP2515_RXF1EID8 0x06
#define MCP2515_RXF1EID0 0x07
#define MCP2515_RXF2SIDH 0x08
#define MCP2515_RXF2SIDL 0x09
#define MCP2515_RXF2EID8 0x0A
#define MCP2515_RXF2EID0 0x0B
#define MCP2515_CANSTAT 0x0E
#define MCP2515_CANCTRL 0x0F

#define MCP2515_RXF3SIDH 0x10
#define MCP2515_RXF3SIDL 0x11
#define MCP2515_RXF3EID8 0x12
#define MCP2515_RXF3EID0 0x13
#define MCP2515_RXF4SIDH 0x14
#define MCP2515_RXF4SIDL 0x15
#define MCP2515_RXF4EID8 0x16
#define MCP2515_RXF4EID0 0x17
#define MCP2515_RXF5SIDH 0x18
#define MCP2515_RXF5SIDL 0x19
#define MCP2515_RXF5EID8 0x1A
#define MCP2515_RXF5EID0 0x1B
#define MCP2515_TEC 0x1C
#define MCP2515_REC 0x1D

#define MCP2515_RXM0SIDH 0x20
#define MCP2515_RXM0SIDL 0x21
#define MCP2515_RXM0EID8 0x22
#define MCP2515_RXM0EID0 0x23
#define MCP2515_RXM1SIDH 0x24
#define MCP2515_RXM1SIDL 0x25
#define MCP2515_RXM1EID8 0x26
#define MCP2515_RXM1EID0 0x27
#define MCP2515_CNF3 0x28
#define MCP2515_CNF2 0x29
#define MCP2515_CNF1 0x2A
#define MCP2515_CANINTE 0x2B
#define MCP2515_CANINTF 0x2C
#define MCP2515_EFLG 0x2D

#define MCP2515_TXB0CTRL 0x30
#define MCP2515_TXB1CTRL 0x40
#define MCP2515_TXB2CTRL 0x50
#define MCP2515_RXB0CTRL 0x60
#define MCP2515_RXB0SIDH 0x61
#define MCP2515_RXB1CTRL 0x70
#define MCP2515_RXB1SIDH 0x71

// Defines for Rx Status
#define MSG_IN_RXB0 0x01
#define MSG_IN_RXB1 0x02
#define MSG_IN_BOTH_BUFFERS 0x03

typedef union {
  struct {
    bool RX0IF;
    bool RX1IF;
    bool TXB0REQ;
    bool TX0IF;
    bool TXB1REQ;
    bool TX1IF;
    bool TXB2REQ;
    bool TX2IF;
  } ctrl;
  uint8_t ctrl_status;
} ctrl_status_t;

typedef union {
  struct {
    unsigned filter : 3;
    unsigned msgType : 2;
    unsigned unusedBit : 1;
    unsigned rxBuffer : 2;
  } ctrlRx;
  uint8_t ctrl_rx_status;
} ctrl_rx_status_t;

typedef union {
  struct {
    unsigned EWARN : 1;
    unsigned RXWAR : 1;
    unsigned TXWAR : 1;
    unsigned RXEP : 1;
    unsigned TXEP : 1;
    unsigned TXBO : 1;
    unsigned RX0OVR : 1;
    unsigned RX1OVR : 1;
  } ErrorF;
  uint8_t error_flag_reg;
} ctrl_error_status_t;

typedef union {
  struct {
    uint8_t RXBnSIDH;
    uint8_t RXBnSIDL;
    uint8_t RXBnEID8;
    uint8_t RXBnEID0;
    uint8_t RXBnDLC;
    uint8_t RXBnD0;
    uint8_t RXBnD1;
    uint8_t RXBnD2;
    uint8_t RXBnD3;
    uint8_t RXBnD4;
    uint8_t RXBnD5;
    uint8_t RXBnD6;
    uint8_t RXBnD7;
  } RxReg;
  uint8_t rx_reg_array[13];
} rx_reg_t;

// MXP2515 Registers
typedef struct {
  uint8_t RXF0SIDH;
  uint8_t RXF0SIDL;
  uint8_t RXF0EID8;
  uint8_t RXF0EID0;
} RXF0;

typedef struct {
  uint8_t RXF1SIDH;
  uint8_t RXF1SIDL;
  uint8_t RXF1EID8;
  uint8_t RXF1EID0;
} RXF1;

typedef struct {
  uint8_t RXF2SIDH;
  uint8_t RXF2SIDL;
  uint8_t RXF2EID8;
  uint8_t RXF2EID0;
} RXF2;

typedef struct {
  uint8_t RXF3SIDH;
  uint8_t RXF3SIDL;
  uint8_t RXF3EID8;
  uint8_t RXF3EID0;
} RXF3;

typedef struct {
  uint8_t RXF4SIDH;
  uint8_t RXF4SIDL;
  uint8_t RXF4EID8;
  uint8_t RXF4EID0;
} RXF4;

typedef struct {
  uint8_t RXF5SIDH;
  uint8_t RXF5SIDL;
  uint8_t RXF5EID8;
  uint8_t RXF5EID0;
} RXF5;

typedef struct {
  uint8_t RXM0SIDH;
  uint8_t RXM0SIDL;
  uint8_t RXM0EID8;
  uint8_t RXM0EID0;
} RXM0;

typedef struct {
  uint8_t RXM1SIDH;
  uint8_t RXM1SIDL;
  uint8_t RXM1EID8;
  uint8_t RXM1EID0;
} RXM1;

typedef struct {
  uint8_t tempSIDH;
  uint8_t tempSIDL;
  uint8_t tempEID8;
  uint8_t tempEID0;
} id_reg_t;

void MCP2515_Initialize(void);
void MCP2515_SetTo_ConfigMode(void);
void MCP2515_SetTo_NormalMode(void);
void MCP2515_SetTo_Sleep_Mode(void);
void MCP2515_Reset(void);

uint8_t MCP2515_Read_Byte(uint8_t readAddress);
uint8_t MCP2515_Read_RxBuffer(uint8_t readRxBuffInst);
void MCP2515_Read_RxbSequence(uint8_t readRxBuffInst, uint8_t rxLength,
                              uint8_t *rxData);

void MCP2515_Write_Byte(uint8_t writeAddress, uint8_t writeData);
void MCP2515_Write_ByteSequence(uint8_t startAddress, uint8_t endAddress,
                                uint8_t *data);
void MCP2515_Load_TxSequence(uint8_t loadtxBnSidhInst, uint8_t *idReg,
                             uint8_t dlc, uint8_t *txData);
void MCP2515_Load_TxBuffer(uint8_t loadTxBuffInst, uint8_t txBufferData);
void MCP2515_RequestToSend(uint8_t rtsTxBuffInst);

uint8_t MCP2515_Read_Status(void);
uint8_t MCP2515_Get_RxStatus(void);

void MCP2515_Bit_Modify(uint8_t regAddress, uint8_t maskByte, uint8_t dataByte);

#endif /* MCP2515_H */
