#include <GS450H.h>



static uint8_t dma_complete;
static uint8_t htm_state = 0;
static uint8_t inv_status = 1;
static bool htm_sent=0, mth_good=0;
//uint16_t rx_buffer_count;
uint16_t counter;
static uint16_t htm_checksum;
static int16_t mg1_torque, mg2_torque, speedSum;
bool statusInv=0;
int16_t GS450H::dc_bus_voltage;
int16_t GS450H::temp_inv_water;
int16_t GS450H::temp_inv_inductor;
int16_t GS450H::mg1_speed;
int16_t GS450H::mg2_speed;


//80 bytes out and 100 bytes back in (with offset of 8 bytes.
static uint8_t mth_data[100];
static uint8_t htm_data_setup[80]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,25,0,0,0,0,0,0,0,128,0,0,0,128,0,0,0,37,1};
static uint8_t htm_data[80]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,255,0,0,0,0,0,0,0,0,0};







///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Usart 2 DMA Transmitt and Receive Section
//////////////////////////////////////////////////////////////////////////

static void dma_write(uint8_t *data, int size)
{
	/*
	 * Using channel 7 for USART2_TX
	 */

	/* Reset DMA channel*/
	dma_channel_reset(DMA1, DMA_CHANNEL7);

	dma_set_peripheral_address(DMA1, DMA_CHANNEL7, (uint32_t)&USART2_DR);
	dma_set_memory_address(DMA1, DMA_CHANNEL7, (uint32_t)data);
	dma_set_number_of_data(DMA1, DMA_CHANNEL7, size);
	dma_set_read_from_memory(DMA1, DMA_CHANNEL7);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL7);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL7, DMA_CCR_PSIZE_8BIT);
	dma_set_memory_size(DMA1, DMA_CHANNEL7, DMA_CCR_MSIZE_8BIT);
	dma_set_priority(DMA1, DMA_CHANNEL7, DMA_CCR_PL_MEDIUM);

	//dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL7);
    //dma_clear_interrupt_flags(DMA1, DMA_CHANNEL7, DMA_TCIF);
	dma_enable_channel(DMA1, DMA_CHANNEL7);

        usart_enable_tx_dma(USART2);
}

volatile int transfered = 0;

void dma1_channel7_isr(void)
{


	if ((DMA1_ISR &DMA_ISR_TCIF7) != 0)
        {
		DMA1_IFCR |= DMA_IFCR_CTCIF7;//Interrupt Flag Clear Register

		transfered = 1;
	}

	dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL7);

	usart_disable_tx_dma(USART2);

	dma_disable_channel(DMA1, DMA_CHANNEL7);


}

static void dma_read(uint8_t *data, int size)
{
	/*
	 * Using channel 6 for USART2_RX
	 */

	/* Reset DMA channel*/
	dma_channel_reset(DMA1, DMA_CHANNEL6);

	dma_set_peripheral_address(DMA1, DMA_CHANNEL6, (uint32_t)&USART2_DR);
	dma_set_memory_address(DMA1, DMA_CHANNEL6, (uint32_t)data);
	dma_set_number_of_data(DMA1, DMA_CHANNEL6, size);
	dma_set_read_from_peripheral(DMA1, DMA_CHANNEL6);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL6);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL6, DMA_CCR_PSIZE_8BIT);
	dma_set_memory_size(DMA1, DMA_CHANNEL6, DMA_CCR_MSIZE_8BIT);
	dma_set_priority(DMA1, DMA_CHANNEL6, DMA_CCR_PL_LOW);

	//dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL6);
    // dma_clear_interrupt_flags(DMA1, DMA_CHANNEL6, DMA_TCIF);
	dma_enable_channel(DMA1, DMA_CHANNEL6);

        usart_enable_rx_dma(USART2);
}

volatile int received = 0;

void dma1_channel6_isr(void)
{
	if ((DMA1_ISR &DMA_ISR_TCIF6) != 0) {
		DMA1_IFCR |= DMA_IFCR_CTCIF6;

		received = 1;
	}

	dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL6);

	usart_disable_rx_dma(USART2);

	dma_disable_channel(DMA1, DMA_CHANNEL6);
}






//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Dilbert's code here
//////////////////////////////////////////////////////////////////////////////////////////////////////


uint8_t CalcMTHChecksum(void){

	uint16_t mth_checksum=0;

	for(int i=0;i<98;i++)mth_checksum+=mth_data[i];


  if(mth_checksum==(mth_data[98]|(mth_data[99]<<8))) return 1;
  else return 0;

}

void CalcHTMChecksum(void){

uint16_t htm_checksum=0;

  for(int i=0;i<78;i++)htm_checksum+=htm_data[i];
  htm_data[78]=htm_checksum&0xFF;
  htm_data[79]=htm_checksum>>8;

}



void GS450H::UpdateHTMState1Ms(int8_t gear, int16_t torque)
{


switch(htm_state){

case 0:{
dma_read(mth_data,100);//read in mth data via dma. Probably need some kind of check dma complete flag here
 DigIo::req_out.Clear(); //HAL_GPIO_WritePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin, 0);
htm_state++;
}break;

case 1:{
 DigIo::req_out.Set();  //HAL_GPIO_WritePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin, 1);

if(inv_status==0){
dma_write(htm_data,80); //HAL_UART_Transmit_IT(&huart2, htm_data, 80);
}
else {
dma_write(htm_data_setup,80);   //HAL_UART_Transmit_IT(&huart2, htm_data_setup, 80);
if(mth_data[1]!=0)
inv_status--;
}
htm_state++;
break;

case 2:
htm_state++;
}break;

case 3:{
    //
   // dma_get_interrupt_flag(DMA1, DMA_CHANNEL6, DMA_TCIF);
if(CalcMTHChecksum()==0 || dma_get_interrupt_flag(DMA1, DMA_CHANNEL6, DMA_TCIF)==0){
//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1 );
statusInv=0;
}
else{
//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0 );
//exchange data and prepare next HTM frame
dma_clear_interrupt_flags(DMA1, DMA_CHANNEL6, DMA_TCIF);
statusInv=1;
dc_bus_voltage=(((mth_data[82]|mth_data[83]<<8)-5)/2);
temp_inv_water=(mth_data[42]|mth_data[43]<<8);
temp_inv_inductor=(mth_data[86]|mth_data[87]<<8);
mg1_speed=mth_data[6]|mth_data[7]<<8;
mg2_speed=mth_data[31]|mth_data[32]<<8;
}

mth_data[98]=0;
mth_data[99]=0;

htm_state++;
}break;

case 4:{

    // -3500 (reverse) to 3500 (forward)
    if(gear==0) mg2_torque=0;//Neutral
    if(gear==32) mg2_torque=torque;//Drive
    if(gear==-32) mg2_torque=torque*-1;//Reverse

  mg1_torque=((mg2_torque*5)/4);
  if(gear=-32) mg1_torque=0; //no mg1 torque in reverse.
  Param::SetInt(Param::torque,mg2_torque);//post processed final torue value sent to inv to web interface

	//speed feedback
	speedSum=mg2_speed+mg1_speed;
	speedSum/=113;
    uint8_t speedSum2=speedSum;
    htm_data[0]=speedSum2;
	htm_data[75]=(mg1_torque*4)&0xFF;
	htm_data[76]=((mg1_torque*4)>>8);

	//mg1
	htm_data[5]=(mg1_torque*-1)&0xFF;  //negative is forward
	htm_data[6]=((mg1_torque*-1)>>8);
	htm_data[11]=htm_data[5];
	htm_data[12]=htm_data[6];

	//mg2
	htm_data[26]=(mg2_torque)&0xFF; //positive is forward
	htm_data[27]=((mg2_torque)>>8);
	htm_data[32]=htm_data[26];
	htm_data[33]=htm_data[27];

	//checksum
	htm_checksum=0;
	for(int i=0;i<78;i++)htm_checksum+=htm_data[i];
	htm_data[78]=htm_checksum&0xFF;
	htm_data[79]=htm_checksum>>8;

if(counter>100){
//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin );
counter = 0;
}
else{
counter++;
}

htm_state=0;
}break;

case 5:{

}break;


}
}


bool GS450H::statusFB()
{
  return statusInv;
}
//////////////////////////////////////////////////////////////

