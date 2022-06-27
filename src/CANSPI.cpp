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

//#include "can_spi.h"    /* Modified manually, Mysil */
#include "CANSPI.h"
#include "MCP2515.h"

/**
    Local Function Prototypes
*/
uint32_t convertReg2ExtendedCANid(uint8_t tempRXBn_EIDH, uint8_t tempRXBn_EIDL, uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL);
uint32_t convertReg2StandardCANid(uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL) ;
void convertCANid2Reg(uint32_t tempPassedInID, uint8_t canIdType, id_reg_t *passedIdReg);

/**
    Local Variables
*/
ctrl_status_t ctrlStatus;
ctrl_error_status_t errorStatus;
id_reg_t idReg;

/**
 CAN SPI APIs
*/

void CANSPI_ENRx_IRQ(void)
{
   MCP2515_Bit_Modify(MCP2515_CANINTF, 0x43, 0x00);        //clear irqs
   MCP2515_Bit_Modify(MCP2515_CANINTE, 0x43, 0x43);        //Enable Receive and wake interrupts

}
//test
void CANSPI_CLR_IRQ(void)
{
   MCP2515_Bit_Modify(MCP2515_CANINTF, 0x43, 0x00);        //clear irqs

}

void CANSPI_Sleep(void)
{
   MCP2515_Bit_Modify(MCP2515_CANINTF, 0x40, 0x00);        //clear CAN bus wakeup interrupt
   MCP2515_Bit_Modify(MCP2515_CANINTE, 0x40, 0x40);        //enable CAN bus activity wakeup
   MCP2515_SetTo_Sleep_Mode();
}

void CANSPI_Initialize(void)
{
   RXF0 RXF0reg;
   RXF1 RXF1reg;
   RXF2 RXF2reg;
   RXF3 RXF3reg;
   RXF4 RXF4reg;
   RXF5 RXF5reg;
   RXM0 RXM0reg;
   RXM1 RXM1reg;

   /**
   Mask and Filter definitions
   .............................................................
   CAN ID		Mask				Filter		Buffer
   .............................................................
   0x444		Acceptance Mask 0		Filter 0	RXB0
   0x555		Acceptance Mask 0		Filter 0	RXB0
   0x666		Acceptance Mask 1		Filter 1	RXB1
   .............................................................
   */

   // Initialize Rx Mask values
   RXM0reg.RXM0SIDH = 0x00;
   RXM0reg.RXM0SIDL = 0x00;
   RXM0reg.RXM0EID8 = 0x00;
   RXM0reg.RXM0EID0 = 0x00;

   RXM1reg.RXM1SIDH = 0xFF;
   RXM1reg.RXM1SIDL = 0xFF;
   RXM1reg.RXM1EID8 = 0xFF;
   RXM1reg.RXM1EID0 = 0xFF;

   // Initialize Rx Filter values
   RXF0reg.RXF0SIDH = 0x88;
   RXF0reg.RXF0SIDL = 0x80;
   RXF0reg.RXF0EID8 = 0x00;
   RXF0reg.RXF0EID0 = 0x00;

   RXF1reg.RXF1SIDH = 0xCC;
   RXF1reg.RXF1SIDL = 0xC0;
   RXF1reg.RXF1EID8 = 0x00;
   RXF1reg.RXF1EID0 = 0x00;

   RXF2reg.RXF2SIDH = 0x00;
   RXF2reg.RXF2SIDL = 0x00;
   RXF2reg.RXF2EID8 = 0x00;
   RXF2reg.RXF2EID0 = 0x00;

   RXF3reg.RXF3SIDH = 0x00;
   RXF3reg.RXF3SIDL = 0x00;
   RXF3reg.RXF3EID8 = 0x00;
   RXF3reg.RXF3EID0 = 0x00;

   RXF4reg.RXF4SIDH = 0x00;
   RXF4reg.RXF4SIDL = 0x00;
   RXF4reg.RXF4EID8 = 0x00;
   RXF4reg.RXF4EID0 = 0x00;

   RXF5reg.RXF5SIDH = 0x00;
   RXF5reg.RXF5SIDL = 0x00;
   RXF5reg.RXF5EID8 = 0x00;
   RXF5reg.RXF5EID0 = 0x00;

   MCP2515_Initialize();
   MCP2515_SetTo_ConfigMode();

   // Write Filter and Mask values to CAN controller
   MCP2515_Write_ByteSequence(MCP2515_RXM0SIDH, MCP2515_RXM0EID0, &(RXM0reg.RXM0SIDH));
   MCP2515_Write_ByteSequence(MCP2515_RXM1SIDH, MCP2515_RXM1EID0, &(RXM1reg.RXM1SIDH));
   MCP2515_Write_ByteSequence(MCP2515_RXF0SIDH, MCP2515_RXF0EID0, &(RXF0reg.RXF0SIDH));
   MCP2515_Write_ByteSequence(MCP2515_RXF1SIDH, MCP2515_RXF1EID0, &(RXF1reg.RXF1SIDH));
   MCP2515_Write_ByteSequence(MCP2515_RXF2SIDH, MCP2515_RXF2EID0, &(RXF2reg.RXF2SIDH));
   MCP2515_Write_ByteSequence(MCP2515_RXF3SIDH, MCP2515_RXF3EID0, &(RXF3reg.RXF3SIDH));
   MCP2515_Write_ByteSequence(MCP2515_RXF4SIDH, MCP2515_RXF4EID0, &(RXF4reg.RXF4SIDH));
   MCP2515_Write_ByteSequence(MCP2515_RXF5SIDH, MCP2515_RXF5EID0, &(RXF5reg.RXF5SIDH));

   // Initialize CAN Timings


       MCP2515_Write_Byte(MCP2515_CNF1, 0x40);//500kbps at 16HMz xtal.
       MCP2515_Write_Byte(MCP2515_CNF2, 0xe5);
       MCP2515_Write_Byte(MCP2515_CNF3, 0x83);

   /*MCP2515_Write_Byte(MCP2515_CNF1, 0x4E);//33kbps at 16HMz xtal.
   MCP2515_Write_Byte(MCP2515_CNF2, 0xe5);
   MCP2515_Write_Byte(MCP2515_CNF3, 0x83);*/

   MCP2515_SetTo_NormalMode();
}

uint8_t CANSPI_Transmit(uCAN_MSG *tempCanMsg)
{
   uint8_t returnValue = 0;

   idReg.tempSIDH = 0;
   idReg.tempSIDL = 0;
   idReg.tempEID8 = 0;
   idReg.tempEID0 = 0;

   ctrlStatus.ctrl_status = MCP2515_Read_Status();

   if (ctrlStatus.ctrl.TXB0REQ ==0)
   {
      convertCANid2Reg(tempCanMsg->frame.id, tempCanMsg->frame.idType, &idReg);

      MCP2515_Load_TxSequence(MCP2515_LOAD_TXB0SIDH, &(idReg.tempSIDH), tempCanMsg->frame.dlc, &(tempCanMsg->frame.data0));
      MCP2515_RequestToSend(MCP2515_RTS_TX0);

      returnValue = 1;
   }
   else if (ctrlStatus.ctrl.TXB1REQ ==0)
   {
      convertCANid2Reg(tempCanMsg->frame.id, tempCanMsg->frame.idType, &idReg);

      MCP2515_Load_TxSequence(MCP2515_LOAD_TXB1SIDH, &(idReg.tempSIDH), tempCanMsg->frame.dlc, &(tempCanMsg->frame.data0));
      MCP2515_RequestToSend(MCP2515_RTS_TX1);

      returnValue = 1;
   }
   else if (ctrlStatus.ctrl.TXB2REQ ==0)
   {
      convertCANid2Reg(tempCanMsg->frame.id, tempCanMsg->frame.idType, &idReg);

      MCP2515_Load_TxSequence(MCP2515_LOAD_TXB2SIDH, &(idReg.tempSIDH), tempCanMsg->frame.dlc, &(tempCanMsg->frame.data0));
      MCP2515_RequestToSend(MCP2515_RTS_TX2);

      returnValue = 1;
   }

   return (returnValue);
}

uint8_t CANSPI_receive(uCAN_MSG *tempCanMsg)
{
   uint8_t returnValue = 0;
   rx_reg_t rxReg;
   ctrl_rx_status_t rxStatus;

   rxStatus.ctrl_rx_status = MCP2515_Get_RxStatus();

   //check to see if we received a CAN message
   if (rxStatus.ctrlRx.rxBuffer != 0)
   {
      //check which buffer the CAN message is in
      if ((rxStatus.ctrlRx.rxBuffer == MSG_IN_RXB0)|(rxStatus.ctrlRx.rxBuffer == MSG_IN_BOTH_BUFFERS))
      {
         MCP2515_Read_RxbSequence(MCP2515_READ_RXB0SIDH, sizeof(rxReg.rx_reg_array), rxReg.rx_reg_array);
      }
      else if (rxStatus.ctrlRx.rxBuffer == MSG_IN_RXB1)
      {
         MCP2515_Read_RxbSequence(MCP2515_READ_RXB1SIDH, sizeof(rxReg.rx_reg_array), rxReg.rx_reg_array);
      }

      if (rxStatus.ctrlRx.msgType == dEXTENDED_CAN_MSG_ID_2_0B)
      {
         tempCanMsg->frame.idType = (uint8_t) dEXTENDED_CAN_MSG_ID_2_0B;
         tempCanMsg->frame.id = convertReg2ExtendedCANid(rxReg.RxReg.RXBnEID8, rxReg.RxReg.RXBnEID0, rxReg.RxReg.RXBnSIDH, rxReg.RxReg.RXBnSIDL);
      }
      else
      {
         tempCanMsg->frame.idType = (uint8_t) dSTANDARD_CAN_MSG_ID_2_0B;
         tempCanMsg->frame.id = convertReg2StandardCANid(rxReg.RxReg.RXBnSIDH, rxReg.RxReg.RXBnSIDL);
      }

      tempCanMsg->frame.dlc   = rxReg.RxReg.RXBnDLC;
      tempCanMsg->frame.data0 = rxReg.RxReg.RXBnD0;
      tempCanMsg->frame.data1 = rxReg.RxReg.RXBnD1;
      tempCanMsg->frame.data2 = rxReg.RxReg.RXBnD2;
      tempCanMsg->frame.data3 = rxReg.RxReg.RXBnD3;
      tempCanMsg->frame.data4 = rxReg.RxReg.RXBnD4;
      tempCanMsg->frame.data5 = rxReg.RxReg.RXBnD5;
      tempCanMsg->frame.data6 = rxReg.RxReg.RXBnD6;
      tempCanMsg->frame.data7 = rxReg.RxReg.RXBnD7;

      returnValue = 1;
   }
   return (returnValue);
}

uint8_t CANSPI_messagesInBuffer(void)
{
   uint8_t messageCount = 0;

   ctrlStatus.ctrl_status = MCP2515_Read_Status();
   if(ctrlStatus.ctrl.RX0IF != 0)
   {
      messageCount++;
   }
   if(ctrlStatus.ctrl.RX1IF != 0)
   {
      messageCount++;
   }

   return (messageCount);
}

uint8_t CANSPI_isBussOff(void)
{
   uint8_t returnValue = 0;

   errorStatus.error_flag_reg = MCP2515_Read_Byte(MCP2515_EFLG);

   if(errorStatus.ErrorF.TXBO == 1)
   {
      returnValue = 1;
   }

   return (returnValue);
}

uint8_t CANSPI_isRxErrorPassive(void)
{
   uint8_t returnValue = 0;

   errorStatus.error_flag_reg = MCP2515_Read_Byte(MCP2515_EFLG);

   if(errorStatus.ErrorF.RXEP == 1)
   {
      returnValue = 1;
   }

   return (returnValue);
}

uint8_t CANSPI_isTxErrorPassive(void)
{
   uint8_t returnValue = 0;

   errorStatus.error_flag_reg = MCP2515_Read_Byte(MCP2515_EFLG);

   if(errorStatus.ErrorF.TXEP == 1)
   {
      returnValue = 1;
   }

   return (returnValue);
}

uint32_t convertReg2ExtendedCANid(uint8_t tempRXBn_EIDH, uint8_t tempRXBn_EIDL, uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL)
{
   uint32_t returnValue = 0;
   uint32_t ConvertedID = 0;
   uint8_t CAN_standardLo_ID_lo2bits;
   uint8_t CAN_standardLo_ID_hi3bits;

   CAN_standardLo_ID_lo2bits = (tempRXBn_SIDL & 0x03);
   CAN_standardLo_ID_hi3bits = (tempRXBn_SIDL >> 5);
   ConvertedID = (tempRXBn_SIDH << 3);
   ConvertedID = ConvertedID + CAN_standardLo_ID_hi3bits;
   ConvertedID = (ConvertedID << 2);
   ConvertedID = ConvertedID + CAN_standardLo_ID_lo2bits;
   ConvertedID = (ConvertedID << 8);
   ConvertedID = ConvertedID + tempRXBn_EIDH;
   ConvertedID = (ConvertedID << 8);
   ConvertedID = ConvertedID + tempRXBn_EIDL;
   returnValue = ConvertedID;
   return (returnValue);
}

uint32_t convertReg2StandardCANid(uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL)
{
   uint32_t returnValue = 0;
   uint32_t ConvertedID;

   ConvertedID = (tempRXBn_SIDH << 3);
   ConvertedID = ConvertedID + (tempRXBn_SIDL >> 5);
   returnValue = ConvertedID;

   return (returnValue);
}


void convertCANid2Reg(uint32_t tempPassedInID, uint8_t canIdType, id_reg_t *passedIdReg)
{
   uint8_t wipSIDL = 0;

   if (canIdType == dEXTENDED_CAN_MSG_ID_2_0B)
   {
      //EID0
      passedIdReg->tempEID0 = 0xFF & tempPassedInID;
      tempPassedInID = tempPassedInID >> 8;

      //EID8
      passedIdReg->tempEID8 = 0xFF & tempPassedInID;
      tempPassedInID = tempPassedInID >> 8;

      //SIDL
      wipSIDL = 0x03 & tempPassedInID;
      tempPassedInID = tempPassedInID << 3;
      wipSIDL = (0xE0 & tempPassedInID) + wipSIDL;
      wipSIDL = wipSIDL + 0x08;
      passedIdReg->tempSIDL = 0xEB & wipSIDL;

      //SIDH
      tempPassedInID = tempPassedInID >> 8;
      passedIdReg->tempSIDH = 0xFF & tempPassedInID;
   }
   else
   {
      passedIdReg->tempEID8 = 0;
      passedIdReg->tempEID0 = 0;
      tempPassedInID = tempPassedInID << 5;
      passedIdReg->tempSIDL = 0xFF & tempPassedInID;
      tempPassedInID = tempPassedInID >> 8;
      passedIdReg->tempSIDH = 0xFF & tempPassedInID;
   }
}
