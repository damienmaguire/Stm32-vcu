#include "GS450H.h"
#include "hwinit.h"
#include "temp_meas.h"
#include <libopencm3/stm32/timer.h>
#include "anain.h"
#include "my_math.h"
#include "utils.h"

#define  LOW_Gear  0
#define  HIGH_Gear  1
#define  AUTO_Gear  2

volatile int received = 0;
static uint8_t htm_state = 0;
static uint8_t inv_status = 1;//must be 1 for gs450h and gs300h
uint16_t counter;
static uint16_t htm_checksum;
static uint8_t frame_count;
static uint8_t GearSW;
static int16_t mg1_torque, mg2_torque, speedSum;
static bool statusInv=0;
static bool TorqueCut=0;
static uint16_t Lexus_Oil2=0;

static void dma_read(uint8_t *data, int size);
static void dma_write(uint8_t *data, int size);

//80 bytes out and 100 bytes back in (with offset of 8 bytes.
static uint8_t mth_data[140];
static uint8_t htm_data_setup[100]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,25,0,0,0,0,0,0,0,128,0,0,0,128,0,0,0,37,1};
static uint8_t htm_data[105]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,255,0,0,0,0,0,0,0,0,0};
static uint8_t htm_data_GS300H[105]={0,14,0,2,0,0,0,0,0,0,0,0,0,23,0,97,0,0,0,0,0,0,0,248,254,8,1,0,0,0,0,0,0,22,0,0,0,0,0,23,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,23,0,75,22,47,250,137,14,0,0,23,0,0,0,0,201,0,218,0,16,0,0,0,29,0,0,0,0,0,0
};
#if 0
// Not currently used
static uint8_t htm_data_setup_auris[100]= {0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88, 0x00, 0x00, 0x00, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5F, 0x01};
#endif

uint8_t htm_data_init[7][100]=
{
   {0,14,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,4,0,25,0,0,0,0,0,0,0,0,0,0,136,0,0,0,160,0,0,0,0,0,0,0,95,1},
   {0,14,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,4,0,25,0,0,0,0,0,0,0,0,0,0,136,0,0,0,160,0,0,0,0,0,0,0,95,1},
   {0,30,0,0,0,0,0,18,0,154,250,0,0,0,0,97,4,0,0,0,0,0,173,255,82,0,0,0,0,0,0,0,16,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,4,75,25,60,246,52,8,0,0,0,0,0,0,138,0,0,0,168,0,0,0,1,0,0,0,72,7},
   {0,30,0,0,0,0,0,18,0,154,250,0,0,0,0,97,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,16,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,4,75,25,60,246,52,8,0,0,0,0,0,0,138,0,0,0,168,0,0,0,2,0,0,0,75,5},
   {0,30,0,0,0,0,0,18,0,154,250,0,0,0,0,97,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,16,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,4,75,25,60,246,52,8,0,0,0,0,0,0,138,0,0,0,168,0,0,0,2,0,0,0,75,5},
   {0,30,0,0,0,0,0,18,0,154,250,0,0,255,0,97,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,16,0,0,0,0,0,255,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,255,4,73,25,60,246,52,8,0,0,255,0,0,0,138,0,0,0,168,0,0,0,3,0,0,0,70,9},
   {0,30,0,2,0,0,0,18,0,154,250,0,0,16,0,97,0,0,0,0,0,0,200,249,56,6,165,0,136,0,63,0,16,0,0,0,63,0,16,0,3,128,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,16,0,75,12,45,248,21,6,0,0,16,0,0,0,202,0,211,0,16,0,0,0,134,16,0,0,130,10}
};

uint8_t  htm_data_Init_GS300H[6][105]=
{
     {0,14,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,4,0,25,0,0,0,0,0,0,0,0,0,0,0,136,0,0,0,160,0,0,0,0,0,0,0,0,95,1},
     {0,14,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,4,0,25,0,0,0,0,0,0,0,0,0,0,0,136,0,0,0,160,0,0,0,0,0,0,0,0,95,1},
     {0,14,0,0,0,0,0,0,0,0,0,0,0,0,0,97,4,0,0,0,0,0,0,173,255,82,0,0,0,0,0,0,0,22,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,4,75,25,212,254,210,15,0,0,0,0,0,0,0,137,0,0,0,168,0,0,0,1,0,0,0,0,220,6},
     {0,14,0,0,0,0,0,0,0,0,0,0,0,0,0,97,4,0,0,0,0,0,0,173,255,82,0,0,0,0,0,0,0,22,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,4,75,25,212,254,210,15,0,0,0,0,0,0,0,137,0,0,0,168,0,0,0,1,0,0,0,0,220,6},
     {0,14,0,0,0,0,0,0,0,0,0,0,0,0,0,97,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,22,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,4,75,25,212,254,190,15,0,0,0,0,0,0,0,137,0,0,0,168,0,0,0,2,0,0,0,0,203,4},
     {0,14,0,0,0,0,0,0,0,0,0,0,0,0,0,97,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,22,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,4,75,25,212,254,190,15,0,0,0,0,0,0,0,137,0,0,0,168,0,0,0,2,0,0,0,0,203,4}
};

void GS450HClass::SetTorque(float torquePercent)
{
   if(!TorqueCut)
   {
   scaledTorqueTarget = (torquePercent * 3500) / 100.0f;
   mg2_torque = this->scaledTorqueTarget;

   if (scaledTorqueTarget < 0) mg1_torque = 0;
   else mg1_torque=((mg2_torque*5)/4);
   }
   else
   {
       mg2_torque =0;
       mg1_torque=0;
   }

}

float GS450HClass::GetMotorTemperature()
{
   int tmpmg1 = AnaIn::MG1_Temp.Get();//in the gs450h case we must read the analog temp values from sensors in the gearbox
   int tmpmg2 = AnaIn::MG2_Temp.Get();
   Param::SetFloat(Param::MG1Raw,tmpmg1);//-0.01982833(x) +56.678789
   Param::SetFloat(Param::MG2Raw,tmpmg2);

   float t1 = (tmpmg1*(-0.02058758))+56.56512898;//Trying a best fit line approach.
   float t2 = (tmpmg2*(-0.02058758))+56.56512898;;
   float tmpm = MAX(t1, t2);//which ever is the hottest gets displayed

   return float(tmpm);
    // return 20;
}

// 100 ms code
void GS450HClass::Task100Ms()
{
   //Param::SetInt(Param::InvStat, GS450HClass::statusFB()); //update inverter status on web interface
   gear=(Param::GetInt(Param::GEAR));

   if (gear == 1)
   {
      DigIo::SP_out.Clear();
      DigIo::SL1_out.Clear();
      DigIo::SL2_out.Clear();

    //  Param::SetInt(Param::GearFB,HIGH_Gear);// set high gear
   }

   if (gear == 0)
   {
      DigIo::SP_out.Clear();
      DigIo::SL1_out.Set();
      DigIo::SL2_out.Set();

    //  Param::SetInt(Param::GearFB,LOW_Gear);// set low gear
   }
  // setTimerState(true);
   if (Param::GetInt(Param::opmode) == MOD_OFF) Lexus_Oil2 =0;
   if (Param::GetInt(Param::opmode) == MOD_RUN) Lexus_Oil2 = Param::GetInt(Param::OilPump);
   Lexus_Oil2 = utils::change(Lexus_Oil2, 10, 80, 1875, 425); //map oil pump pwm to timer
   timer_set_oc_value(TIM1, TIM_OC1, Lexus_Oil2);//duty. 1000 = 52% , 500 = 76% , 1500=28%

   Param::SetInt(Param::Gear1,!DigIo::gear1_in.Get());//update web interface with status of gearbox PB feedbacks for diag purposes.
   Param::SetInt(Param::Gear2,!DigIo::gear2_in.Get());
   Param::SetInt(Param::Gear3,!DigIo::gear3_in.Get());
   GearSW=((!DigIo::gear3_in.Get()<<2)|(!DigIo::gear2_in.Get()<<1)|(!DigIo::gear1_in.Get()));
   if(GearSW==6) Param::SetInt(Param::GearFB,LOW_Gear);// set low gear
   if(GearSW==5) Param::SetInt(Param::GearFB,HIGH_Gear);// set high gear
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Dilbert's code here
//////////////////////////////////////////////////////////////////////////////////////////////////////

void GS450HClass::SetPrius()
{
    setTimerState(true);//start toyota timers
   if (htm_state<5)
   {
      htm_state = 5;
      inv_status = 0;//must be 0 for prius
   }
}

void GS450HClass::SetGS450H()
{
    setTimerState(true);//start toyota timers
   if (htm_state>4)
   {
      htm_state = 0;
      inv_status = 1;//must be 1 for gs450h
   }
}

void GS450HClass::SetGS300H()
{
    setTimerState(true);//start toyota timers
   if (htm_state<10)
   {
      htm_state = 10;

   }
    inv_status = 0;//must be 0 for gs300h
   for(int i=0; i<105; i++)htm_data[i] = htm_data_GS300H[i];
}

uint8_t GS450HClass::VerifyMTHChecksum(uint16_t len)
{

   uint16_t mth_checksum=0;

   for(int i=0; i<(len-2); i++)
      mth_checksum+=mth_data[i];


   if(mth_checksum==(mth_data[len-2]|(mth_data[len-1]<<8))) return 1;
   else return 0;

}

void GS450HClass::CalcHTMChecksum(uint16_t len)
{

   uint16_t htm_checksum=0;

   for(int i=0; i<(len-2); i++)htm_checksum+=htm_data[i];
   htm_data[len-2]=htm_checksum&0xFF;
   htm_data[len-1]=htm_checksum>>8;

}


void GS450HClass::Task1Ms()
{
   uint8_t speedSum2;

   switch(htm_state)
   {
   case 0:
      dma_read(mth_data,100);//read in mth data via dma. Probably need some kind of check dma complete flag here
      DigIo::req_out.Clear(); //HAL_GPIO_WritePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin, 0);
      htm_state++;
      break;
   case 1:
      DigIo::req_out.Set();  //HAL_GPIO_WritePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin, 1);

      if(inv_status==0)
      {
         if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL7, DMA_TCIF))// if the transfer complete flag is set then send another packet
         {
            dma_clear_interrupt_flags(DMA1, DMA_CHANNEL7, DMA_TCIF);//clear the flag.
            dma_write(htm_data,80); //HAL_UART_Transmit_IT(&huart2, htm_data, 80);
         }

      }
      else
      {
         dma_write(htm_data_setup,80);   //HAL_UART_Transmit_IT(&huart2, htm_data_setup, 80);
         if(mth_data[1]!=0) inv_status--;
         if(mth_data[1]==0) inv_status=1;
      }
      htm_state++;
      break;
   case 2:
      htm_state++;
      break;
   case 3:
      if(VerifyMTHChecksum(100)==0 || dma_get_interrupt_flag(DMA1, DMA_CHANNEL6, DMA_TCIF)==0)
      {
         statusInv=0;
      }
      else
      {
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
      break;
   case 4:
      // -3500 (reverse) to 3500 (forward)
      Param::SetInt(Param::torque,mg2_torque);//post processed final torue value sent to inv to web interface

      //speed feedback
      speedSum=mg2_speed+mg1_speed;
      speedSum/=113;
      speedSum2=speedSum;
      htm_data[0]=speedSum2;
      htm_data[75]=(mg1_torque*4) & 0xFF;
      htm_data[76]=((mg1_torque*4)>>8) & 0xFF;

      //mg1
      htm_data[5]=(-mg1_torque) & 0xFF;  //negative is forward
      htm_data[6]=((-mg1_torque) >> 8);
      htm_data[11]=htm_data[5];
      htm_data[12]=htm_data[6];

      //mg2
      htm_data[26]=(mg2_torque) & 0xFF; //positive is forward
      htm_data[27]=((mg2_torque)>>8) & 0xFF;
      htm_data[32]=htm_data[26];
      htm_data[33]=htm_data[27];

      htm_data[63]=(-5000)&0xFF;  // regen ability of battery
      htm_data[64]=((-5000)>>8);

      htm_data[65]=(27500)&0xFF;  // discharge ability of battery
      htm_data[66]=((27500)>>8);

      //checksum
      htm_checksum=0;

      for (int i = 0; i < 78; i++)
         htm_checksum += htm_data[i];

      htm_data[78]=htm_checksum&0xFF;
      htm_data[79]=htm_checksum>>8;

      if(counter>100)
      {
//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin );
         counter = 0;
      }
      else
      {
         counter++;
      }

      htm_state=0;
      break;

   /***** Demo code for Gen3 Prius/Auris direct communications! */
   case 5:
      dma_read(mth_data,120);//read in mth data via dma. Probably need some kind of check dma complete flag here
      DigIo::req_out.Clear(); //HAL_GPIO_WritePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin, 0);
      htm_state++;
      break;
   case 6:
      DigIo::req_out.Set();  //HAL_GPIO_WritePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin, 1);

      if(inv_status>5)
      {
         if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL7, DMA_TCIF))// if the transfer complete flag is set then send another packet
         {
            dma_clear_interrupt_flags(DMA1, DMA_CHANNEL7, DMA_TCIF);//clear the flag.
            dma_write(htm_data,100); //HAL_UART_Transmit_IT(&huart2, htm_data, 80);
         }
      }
      else
      {
         dma_write(&htm_data_init[ inv_status ][0],100); //HAL_UART_Transmit_IT(&huart2, htm_data_setup, 80);

         inv_status++;
         if(inv_status==6)
         {
            //memcpy(htm_data, &htm_data_init[ inv_status ][0], 100);
         }
      }
      htm_state++;
      break;
   case 7:
      htm_state++;
      break;
   case 8:
      if(VerifyMTHChecksum(120)==0 || dma_get_interrupt_flag(DMA1, DMA_CHANNEL6, DMA_TCIF)==0)
      {

         statusInv=0;
      }
      else
      {

         //exchange data and prepare next HTM frame
         dma_clear_interrupt_flags(DMA1, DMA_CHANNEL6, DMA_TCIF);
         statusInv=1;
         dc_bus_voltage=(((mth_data[100]|mth_data[101]<<8)-5)/2);
         temp_inv_water=(mth_data[42]|mth_data[43]<<8);
         temp_inv_inductor=(mth_data[86]|mth_data[87]<<8);
         mg1_speed=mth_data[6]|mth_data[7]<<8;
         mg2_speed=mth_data[38]|mth_data[39]<<8;
      }

      mth_data[98]=0;
      mth_data[99]=0;

      htm_state++;
      break;
   case 9:
      Param::SetInt(Param::torque,mg2_torque);//post processed final torue value sent to inv to web interface

      //speed feedback
      speedSum=mg2_speed+mg1_speed;
      speedSum/=113;
      //Possibly not needed
      //uint8_t speedSum2=speedSum;
      //htm_data[0]=speedSum2;

      //these bytes are used, and seem to be MG1 for startup, but can't work out the relatino to the
      //bytes earlier in the stream, possibly the byte order has been flipped on these 2 bytes
      //could be a software bug ?
      htm_data[76]=(mg1_torque*4) & 0xFF;
      htm_data[75]=((mg1_torque*4)>>8) & 0xFF;

      //mg1
      htm_data[5]=(mg1_torque)&0xFF;  //negative is forward
      htm_data[6]=((mg1_torque)>>8);
      htm_data[11]=htm_data[5];
      htm_data[12]=htm_data[6];

      //mg2 the MG2 values are now beside each other!
      htm_data[30]=(mg2_torque) & 0xFF; //positive is forward
      htm_data[31]=((mg2_torque)>>8) & 0xFF;

      if(scaledTorqueTarget > 0)
      {
         //forward direction these bytes should match
         htm_data[26]=htm_data[30];
         htm_data[27]=htm_data[31];
         htm_data[28]=(mg2_torque/2) & 0xFF; //positive is forward
         htm_data[29]=((mg2_torque/2)>>8) & 0xFF;
      }

      if(scaledTorqueTarget < 0)
      {
         //reverse direction these bytes should match
         htm_data[28]=htm_data[30];
         htm_data[29]=htm_data[31];
         htm_data[26]=(mg2_torque/2) & 0xFF; //positive is forward
         htm_data[27]=((mg2_torque/2)>>8) & 0xFF;
      }

      //This data has moved!

      htm_data[85]=(-5000)&0xFF;  // regen ability of battery
      htm_data[86]=((-5000)>>8);

      htm_data[87]=(-10000)&0xFF;  // discharge ability of battery
      htm_data[88]=((-10000)>>8);

      //checksum
      if(++frame_count & 0x01)
      {
         htm_data[94]++;
      }

      CalcHTMChecksum(100);

      htm_state=5;
      break;


         /***** Code for Lexus GS300H */
   case 10:
      if (Param::GetInt(Param::opmode) == MOD_OFF) inv_status = 0;
      dma_read(mth_data,140);//read in mth data via dma.
      DigIo::req_out.Clear(); //HAL_GPIO_WritePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin, 0);
      htm_state++;
      break;
   case 11:
      DigIo::req_out.Set();  //HAL_GPIO_WritePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin, 1);
      if(inv_status>6)
      {
         if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL7, DMA_TCIF))// if the transfer complete flag is set then send another packet
         {
            dma_clear_interrupt_flags(DMA1, DMA_CHANNEL7, DMA_TCIF);//clear the flag.
            dma_write(htm_data,105); //HAL_UART_Transmit_IT(&huart2, htm_data, 80);
         }
      }
      else
      {
         dma_write(&htm_data_Init_GS300H[ inv_status ][0],105); //HAL_UART_Transmit_IT(&huart2, htm_data_setup, 80);

         inv_status++;

      }
      htm_state++;
      break;
   case 12:
      htm_state++;
      break;
   case 13:
      if(VerifyMTHChecksum(140)==0 || dma_get_interrupt_flag(DMA1, DMA_CHANNEL6, DMA_TCIF)==0)
      {

         statusInv=0;
         inv_status=0;
      }
      else
      {

         //exchange data and prepare next HTM frame
         dma_clear_interrupt_flags(DMA1, DMA_CHANNEL6, DMA_TCIF);
         statusInv=1;
         dc_bus_voltage=(((mth_data[117]|mth_data[118]<<8))/2);
         temp_inv_water=(mth_data[20]|mth_data[21]<<8);
         temp_inv_inductor=(mth_data[25]|mth_data[26]<<8);
         mg1_speed=mth_data[10]|mth_data[11]<<8;
         mg2_speed=mth_data[43]|mth_data[44]<<8;
      }

     // mth_data[98]=0;
     // mth_data[99]=0;

      htm_state++;
      break;
   case 14:
      Param::SetInt(Param::torque,mg2_torque);//post processed final torue value sent to inv to web interface

      //speed feedback
      speedSum=mg2_speed+mg1_speed;
      speedSum/=113;
      //Possibly not needed
      //uint8_t speedSum2=speedSum;
      //htm_data[0]=speedSum2;

      //these bytes are used, and seem to be MG1 for startup, but can't work out the relatino to the
      //bytes earlier in the stream, possibly the byte order has been flipped on these 2 bytes
      //could be a software bug ?
     // htm_data[76]=(mg1_torque*4) & 0xFF;
     // htm_data[75]=((mg1_torque*4)>>8) & 0xFF;

    //mg1
    htm_data[5]=(mg1_torque*-1)&0xFF;  //negative is forward
    htm_data[6]=((mg1_torque*-1)>>8);
    htm_data[11]=htm_data[5];
    htm_data[12]=htm_data[6];

    //mg2
    htm_data[31]=(mg2_torque)&0xFF; //positive is forward
    htm_data[32]=((mg2_torque)>>8);
    htm_data[37]=htm_data[26];
    htm_data[38]=htm_data[27];
/*
      if(scaledTorqueTarget > 0)
      {
         //forward direction these bytes should match
         htm_data[26]=htm_data[30];
         htm_data[27]=htm_data[31];
         htm_data[28]=(mg2_torque/2) & 0xFF; //positive is forward
         htm_data[29]=((mg2_torque/2)>>8) & 0xFF;
      }

      if(scaledTorqueTarget < 0)
      {
         //reverse direction these bytes should match
         htm_data[28]=htm_data[30];
         htm_data[29]=htm_data[31];
         htm_data[26]=(mg2_torque/2) & 0xFF; //positive is forward
         htm_data[27]=((mg2_torque/2)>>8) & 0xFF;
      }

      //This data has moved!

      htm_data[85]=(-5000)&0xFF;  // regen ability of battery
      htm_data[86]=((-5000)>>8);

      htm_data[87]=(-10000)&0xFF;  // discharge ability of battery
      htm_data[88]=((-10000)>>8);

      //checksum
     if(++frame_count & 0x01)
      {
         htm_data[94]++;
      }
*/
      CalcHTMChecksum(105);

      htm_state=10;
      break;
   }
}

void GS450HClass::setTimerState(bool desiredTimerState)
{
   if (desiredTimerState != this->timerIsRunning)
   {
      if (desiredTimerState)
      {
         tim_setup(); //toyota hybrid oil pump pwm timer
         tim2_setup(); //TOYOTA HYBRID INVERTER INTERFACE CLOCK
         this->timerIsRunning=true; //timers are now running
      }
      else
      {
         // These are only used with the Totoa hybrid option.
         timer_disable_counter(TIM2); //TOYOTA HYBRID INVERTER INTERFACE CLOCK
         timer_disable_counter(TIM1); //toyota hybrid oil pump pwm timer
         this->timerIsRunning=false; //timers are now stopped
      }
   }
}

int GS450HClass::GetInverterState()
{
   return statusInv;
}
//////////////////////////////////////////////////////////////


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

   dma_enable_channel(DMA1, DMA_CHANNEL7);

   usart_enable_tx_dma(USART2);
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

   dma_enable_channel(DMA1, DMA_CHANNEL6);

   usart_enable_rx_dma(USART2);
}

