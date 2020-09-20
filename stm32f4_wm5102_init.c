// stm32f4_wm5102_init.c

#include "stm32f4_wm5102_init.h"

 typedef union 
{
 uint16_t value;
 struct 
 {
	 unsigned char bit0 : 1;
	 unsigned char bit1 : 1;
	 unsigned char bit2 : 1;
	 unsigned char bit3 : 1;
	 unsigned char bit4 : 1;
	 unsigned char bit5 : 1;
	 unsigned char bit6 : 1;
	 unsigned char bit7 : 1;
	 unsigned char bit8 : 1;
	 unsigned char bit9 : 1;
	 unsigned char bit10 : 1;
	 unsigned char bit11 : 1;
	 unsigned char bit12 : 1;
	 unsigned char bit13 : 1;
	 unsigned char bit14 : 1;
	 unsigned char bit15 : 1;
  } bits;
} shift_register;

shift_register sreg = {0x0001};

short prbs(int16_t noise_level)
{
  char fb;
  
	fb =((sreg.bits.bit15)+(sreg.bits.bit14)+(sreg.bits.bit3)+(sreg.bits.bit1))%2;
  sreg.value = sreg.value << 1;
  sreg.bits.bit0 = fb;			      
  if(fb == 0)	return(-noise_level); else return(noise_level);
}

uint32_t prand_seed = 1;       // used in function prand()

uint32_t rand31_next()
{
  uint32_t hi, lo;

  lo = 16807 * (prand_seed & 0xFFFF);
  hi = 16807 * (prand_seed >> 16);

  lo += (hi & 0x7FFF) << 16;
  lo += hi >> 15;

  if (lo > 0x7FFFFFFF) lo -= 0x7FFFFFFF;

  return(prand_seed = (uint32_t)lo);
}

int16_t prand()
{
return ((int16_t)(rand31_next()>>18)-4096);
}

uint16_t pingIN[BUFSIZE], pingOUT[BUFSIZE], pongIN[BUFSIZE], pongOUT[BUFSIZE];

__IO uint32_t CODECTimeout = CODEC_LONG_TIMEOUT;
__IO uint8_t OutputDev = 0;

static  void DonaldDelay( __IO uint32_t nCount)
{ for (; nCount > 0; nCount--);
}

uint32_t Codec_TIMEOUT_UserCallback(void)
{
while(1);
}

uint32_t Codec_WriteRegister(uint32_t RegisterAddr, uint16_t RegisterValue)
{
	uint32_t result = 0;
	
// assemble 6-byte data in WM5102 format - transmit in 8-bit bytes
// WM5102 version has 32-bit address and 16-bit data
  uint8_t Byte1 = (RegisterAddr>>24) & 0xFF;
  uint8_t Byte2 = (RegisterAddr>>16) & 0xFF;
  uint8_t Byte3 = (RegisterAddr>>8) & 0xFF;
  uint8_t Byte4 = (RegisterAddr) & 0xFF;
  uint8_t Byte5 = (RegisterValue>>8) & 0xFF;
  uint8_t Byte6 = (RegisterValue) & 0xFF;

	CODECTimeout = CODEC_LONG_TIMEOUT;

	while(I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BUSY))
	{
		if((CODECTimeout--) == 0) return Codec_TIMEOUT_UserCallback();
	}

// start configuration sequence 
	I2C_GenerateSTART(CODEC_I2C, ENABLE);
// test on EV5 and clear it 
	CODECTimeout = CODEC_FLAG_TIMEOUT;
	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if((CODECTimeout--) == 0)
			return Codec_TIMEOUT_UserCallback();
	}

//transmit the slave address and enable writing operation
	I2C_Send7bitAddress(CODEC_I2C, (W5102_ADDR_0<<1), I2C_Direction_Transmitter);
// test on EV6 and clear it
	CODECTimeout = CODEC_FLAG_TIMEOUT;
	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		if((CODECTimeout--) == 0)
			return Codec_TIMEOUT_UserCallback();
	}

// transmit the first address for write operation
	I2C_SendData(CODEC_I2C, Byte1);
// test on EV8 and clear it
	CODECTimeout = CODEC_FLAG_TIMEOUT;
	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
	{
		if((CODECTimeout--) == 0)
			return Codec_TIMEOUT_UserCallback();
	}
// transmit the first address for write operation
	I2C_SendData(CODEC_I2C, Byte2);
// test on EV8 and clear it
	CODECTimeout = CODEC_FLAG_TIMEOUT;
	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
	{
		if((CODECTimeout--) == 0)
			return Codec_TIMEOUT_UserCallback();
	}
// transmit the first address for write operation
	I2C_SendData(CODEC_I2C, Byte3);
// test on EV8 and clear it
	CODECTimeout = CODEC_FLAG_TIMEOUT;
	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
	{
		if((CODECTimeout--) == 0)
			return Codec_TIMEOUT_UserCallback();
	}
// transmit the first address for write operation
	I2C_SendData(CODEC_I2C, Byte4);
// test on EV8 and clear it
	CODECTimeout = CODEC_FLAG_TIMEOUT;
	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
	{
		if((CODECTimeout--) == 0)
			return Codec_TIMEOUT_UserCallback();
	}
// prepare the register value to be sent
	I2C_SendData(CODEC_I2C, Byte5);
// wait till all data have been physically transferred on the bus
	CODECTimeout = CODEC_LONG_TIMEOUT;
	while(!I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BTF))
	{
		if((CODECTimeout--) == 0)
			return Codec_TIMEOUT_UserCallback();
	}
// prepare the register value to be sent
	I2C_SendData(CODEC_I2C, Byte6);
// wait till all data have been physically transferred on the bus
	CODECTimeout = CODEC_LONG_TIMEOUT;
	while(!I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BTF))
	{
		if((CODECTimeout--) == 0)
			return Codec_TIMEOUT_UserCallback();
	}
// end the configuration sequence
	I2C_GenerateSTOP(CODEC_I2C, ENABLE);  
// return the verifying value: 0 (Passed) or 1 (Failed)
	return result;  
}

void Codec_CtrlInterface_Init(void)
{
	I2C_InitTypeDef I2C_InitStructure;

	RCC_APB1PeriphClockCmd(CODEC_I2C_CLK, ENABLE);

	I2C_DeInit(CODEC_I2C);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x33;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;

	I2C_Cmd(CODEC_I2C, ENABLE);  
	I2C_Init(CODEC_I2C, &I2C_InitStructure);
}


void configure_codec(uint16_t fs, int select_input)
{
	Codec_WriteRegister(0x0000, 0x0000);   // reset codec
  DonaldDelay(30000);                    // may or may not need delay here

  Codec_WriteRegister(0x0019, 0x0001);   // patch codec (supplied by Wolfson)  
	Codec_WriteRegister(0x0080, 0x0003);
	Codec_WriteRegister(0x0081, 0xE022);
	Codec_WriteRegister(0x0410, 0x6080);
	Codec_WriteRegister(0x0418, 0xa080);
	Codec_WriteRegister(0x0420, 0xa080);
	Codec_WriteRegister(0x0428, 0xE000);
	Codec_WriteRegister(0x0443, 0xDC1A);
	Codec_WriteRegister(0x04B0, 0x0066);
	Codec_WriteRegister(0x0458, 0x000B);
	Codec_WriteRegister(0x0212, 0x0000);
	Codec_WriteRegister(0x0171, 0x0000);
	Codec_WriteRegister(0x035E, 0x000C);
	Codec_WriteRegister(0x02D4, 0x0000);
	Codec_WriteRegister(0x0080, 0x0000);  // end of patch

	Codec_WriteRegister(0x192, 0x8008);   // FLL2 12.000 MHz MCLK1 -> 24.576 MHz SYSCLK
	Codec_WriteRegister(0x193, 0x0018);   // could make this part of a switch in order
	Codec_WriteRegister(0x194, 0x007D);   // to allow 44.1kHz-related sample rates
	Codec_WriteRegister(0x195, 0x0008);
	Codec_WriteRegister(0x196, 0x0000);
	Codec_WriteRegister(0x191, 0x0001);

  DonaldDelay(30000);                   // may or may not be needed

	Codec_WriteRegister(0x101, 0x0245);   // clock set up as SYSCLK = 24.576 MHz, src FLL2, SYSCLK_FRAC = 0
	                                      // SYSCLK enabled

	switch(fs) // set sample rate 1 - currently only 48kHz-related sample rates allowed
	{
		case FS_8000_HZ:
			Codec_WriteRegister(0x102, 0x11);
			Codec_WriteRegister(0x580, 0x26);
		  break;
		case FS_12000_HZ:
			Codec_WriteRegister(0x102, 0x01);
			Codec_WriteRegister(0x580, 0x27);
		  break;
		case FS_16000_HZ:
			Codec_WriteRegister(0x102, 0x12);
			Codec_WriteRegister(0x580, 0x28);
		  break;
		case FS_24000_HZ:
			Codec_WriteRegister(0x102, 0x02);
			Codec_WriteRegister(0x580, 0x29);
		  break;
		case FS_32000_HZ:
			Codec_WriteRegister(0x102, 0x13);
			Codec_WriteRegister(0x580, 0x2A);
		  break;
		case FS_48000_HZ:
			Codec_WriteRegister(0x102, 0x03);
			Codec_WriteRegister(0x580, 0x2B);
		  break;
		default:
			Codec_WriteRegister(0x102, 0x11); // default is 8kHz sample rate
			Codec_WriteRegister(0x580, 0x26);
		  break;
	}

	Codec_WriteRegister(0x458, 0x0009);   // output noise gate enabled, threshold -84dB (important??)

	Codec_WriteRegister(0x200, 0x0001);   // not used prior to 20 March but I think necessary for CP2/LDO2 - analog inputs
                                        // Wolfson example write 0x0007 but bit 0 is CP2_ENA
	Codec_WriteRegister(0x210, 0x00D5);   // LDO1 control 0x00D5 -> LDO1 enabled, normal, 1V2

  Codec_WriteRegister(0x584, 0x0002);   // AIF3 I2S format
  Codec_WriteRegister(0x582, 0x0005);   // AIF3 LRCLK master - this takes LRC high, reliably I hope...
  DonaldDelay(30000);
  I2S_Cmd(I2Sx, ENABLE);                // ...because it's necessary for correct startup of STM32F4
  I2S_Cmd(I2Sxext, ENABLE);             // I2S interface (STM32F4 I2S module MUST be enabled while
  DonaldDelay(30000);                   // LRC is high - as per October 2013 errata)

  Codec_WriteRegister(0x587, 0x1010);   // AIF3 TX WL and SLOT_LEN both 16-bit
	Codec_WriteRegister(0x588, 0x1010);   // AIF3 RX WL and SLOT_LEN both 16-bit
	Codec_WriteRegister(0x59A, 0x0003);   // enable AIF3 RX channels (L and R)
	Codec_WriteRegister(0x599, 0x0003);   // enable AIF3 TX channels (L and R)
	Codec_WriteRegister(0x585, 0x0020);   // AIF3 32 BCLK cycles per LRC TX frame
	Codec_WriteRegister(0x586, 0x0020);   // AIF3 32 BCLK cycles per LRC RX frame

                                        // LINE OUT and HP OUT enabled in parallel 
  Codec_WriteRegister(0x690, 0x0030);   // OUT2L (LINE OUT) mixer input is AIF3 RX1 (from I2S) 30
  Codec_WriteRegister(0x691, 0x0080);   // associated volume is 0dB

  Codec_WriteRegister(0x698, 0x0031);   // OUT2R (LINE OUT) mixer input is AIF3 RX2 (from I2S) 31
  Codec_WriteRegister(0x699, 0x0080);   // associated volume is 0dB

  Codec_WriteRegister(0x680, 0x0030);   // OUT1L (HP OUT) mixer input is AIF3 RX1 (from I2S)
  Codec_WriteRegister(0x681, 0x0080);   // associated volume is 0dB

  Codec_WriteRegister(0x688, 0x0031);   // OUT1R (HP OUT) mixer input is AIF3 RX2 (from I2S)
  Codec_WriteRegister(0x689, 0x0080);   // associated volume is 0dB

// route LHPFs to AIF3TX
  Codec_WriteRegister(0x0780, 0x0060);   // AIF3TX mixer from LHPF1 60
	Codec_WriteRegister(0x0788, 0x0061);   // AIF3TX mixer from LHPF2 61
  Codec_WriteRegister(0x0781, 0x0080);   // AIF3TX mixer gain 0dB
	Codec_WriteRegister(0x0789, 0x0080);   // AIF3TX mixer gain 0dB

  Codec_WriteRegister(0x0EC0, 0x0003);   // LHPF1 HPF enabled
  Codec_WriteRegister(0x0EC1, 0xF09e);   // LHPF1 cutoff frequency in Hz depends on fs
  Codec_WriteRegister(0x0EC4, 0x0003);   // LHPF2 HPF enabled
  Codec_WriteRegister(0x0EC5, 0xF09E);   // LHPF2 cutoff frequency in Hz depends on fs

  Codec_WriteRegister(0x0901, 0x0080);   // LHPF1 mixer source 1 gain 0dB
  Codec_WriteRegister(0x0909, 0x0080);   // LHPF2 mixer source 1 gain 0dB

  switch(select_input)
  {
	  case WM5102_LINE_IN:
      Codec_WriteRegister(0x0900, 0x0014); // LHPF1 mixer from IN3 (LINE IN)
	    Codec_WriteRegister(0x0908, 0x0015); // LHPF2 mixer from IN3
  	  Codec_WriteRegister(0x0300, 0x0030); // enable IN3L and IN3R 0030 LINE IN
	    Codec_WriteRegister(0x0320, 0x2290); // IN3L PGA gain +8.0dB LINE IN (potential divider comp.)
	    Codec_WriteRegister(0x0321, 0x0280); // IN3L ADC volume 0dB LINE IN
	    Codec_WriteRegister(0x0324, 0x0090); // IN3R PGA gain +8.0dB LINE IN (potential divider comp.)
	    Codec_WriteRegister(0x0325, 0x0280); // IN3R ADC volume 0dB LINE IN
    break;
	  case WM5102_DMIC_IN:
      Codec_WriteRegister(0x0900, 0x0012); // LHPF1 mixer from IN2 (DMIC IN)
  	  Codec_WriteRegister(0x0908, 0x0013); // LHPF2 mixer from IN2
      Codec_WriteRegister(0x0300, 0x000C); // enable IN2L and IN2R 000C DMIC IN
	    Codec_WriteRegister(0x0318, 0x3480); // IN2 DMIC IN IN2L PGA vol 0dB
	    Codec_WriteRegister(0x031D, 0x0280); // IN2R ADC volume 0dB DMIC IN
	    Codec_WriteRegister(0x0319, 0x0280); // IN2L ADC volume 0dB DMIC IN
	    Codec_WriteRegister(0x0219, 0x01A7); // MICBIAS2 enable DMIC IN
    break;
	  case WM5102_MIC_IN:
      Codec_WriteRegister(0x0900, 0x0010); // LHPF1 mixer from IN1 (MIC IN)
  	  Codec_WriteRegister(0x0908, 0x0011); // LHPF2 mixer from IN1
  	  Codec_WriteRegister(0x0300, 0x0003); // enable IN1L and IN1R 0003 MIC IN
	    Codec_WriteRegister(0x0310, 0x2A80); // IN1L PGA vol 0dB MIC IN
	    Codec_WriteRegister(0x0314, 0x0080); // IN1R PGA volume 0dB MIC IN
	    Codec_WriteRegister(0x0311, 0x0280); // IN1L ADC volume 0dB MIC IN
	    Codec_WriteRegister(0x0315, 0x0280); // IN1R ADC volume 0dB DMIC IN
	    Codec_WriteRegister(0x0218, 0x01A7); // MICBIAS1 enable MIC IN
    break;
	  default:
      Codec_WriteRegister(0x0900, 0x0014); // LHPF1 mixer from IN3 (LINE IN)
	    Codec_WriteRegister(0x0908, 0x0015); // LHPF2 mixer from IN3
  	  Codec_WriteRegister(0x0300, 0x0030); // enable IN3L and IN3R 0030 LINE IN
	    Codec_WriteRegister(0x0320, 0x2290); // IN3L PGA gain +8.0dB LINE IN (potential divider comp.)
	    Codec_WriteRegister(0x0321, 0x0280); // IN3L ADC volume 0dB LINE IN
	    Codec_WriteRegister(0x0324, 0x0090); // IN3R PGA gain +8.0dB LINE IN (potential divider comp.)
	    Codec_WriteRegister(0x0325, 0x0280); // IN3R ADC volume 0dB LINE IN
    break;
  }		

	Codec_WriteRegister(0x419, 0x0280); // DAC 2 volume L 0dB (LINE OUT)
	Codec_WriteRegister(0x41D, 0x0280); // DAC 2 volume R 0dB (LINE OUT)
	Codec_WriteRegister(0x411, 0x0280); // DAC 1 volume L 0dB (HP OUT)
	Codec_WriteRegister(0x415, 0x0280); // DAC 1 volume R 0dB (HP OUT)

	Codec_WriteRegister(0x400, 0x000F); // enable outputs OUT2L, OUT2R, OUT1L, OUT1R
}

// overall initialisation routine

void stm32_wm5102_init(uint16_t fs, int select_input, int io_method)
{
// declare structures for I2S, NVIC, and GPIO initialisation
  I2S_InitTypeDef I2S_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;

// GPIOB, GPIOC and GPIOD clocks on  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);

	// added 21 Sept 2014 for pushbutton input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


// configure PD15 and PD11 in output pushpull mode 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_11 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
// set up GPIO initialisation structure for I2C pins SCL and SDA
  GPIO_InitStructure.GPIO_Pin = CODEC_I2C_SCL_PIN | CODEC_I2C_SDA_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
// initialise I2C pins SCL and SDA
  GPIO_Init(CODEC_I2C_GPIO, &GPIO_InitStructure);
// configure GPIO pins for I2C alternative functions
	GPIO_PinAFConfig(CODEC_I2C_GPIO, CODEC_I2C_SCL_PINSRC, CODEC_I2C_GPIO_AF); 
	GPIO_PinAFConfig(CODEC_I2C_GPIO, CODEC_I2C_SDA_PINSRC, CODEC_I2C_GPIO_AF);  
// set up GPIO initialisation structure for I2S pins WS, CK, SD and SD
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

  GPIO_InitStructure.GPIO_Pin = I2Sx_WS_PIN | I2Sx_CK_PIN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = I2Sx_SD_PIN | I2Sxext_SD_PIN ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

// connect I2S pins to alternative functions 
  GPIO_PinAFConfig(I2Sx_WS_GPIO_PORT, I2Sx_WS_SOURCE, I2Sx_WS_AF);
  GPIO_PinAFConfig(I2Sx_CK_GPIO_PORT, I2Sx_CK_SOURCE, I2Sx_CK_AF);
  GPIO_PinAFConfig(I2Sx_SD_GPIO_PORT, I2Sx_SD_SOURCE, I2Sx_SD_AF);
  GPIO_PinAFConfig(I2Sxext_SD_GPIO_PORT, I2Sxext_SD_SOURCE, I2Sxext_SD_AF);
	
  I2S_Cmd(I2Sx, DISABLE);
  I2S_Cmd(I2Sxext, DISABLE);

// I2S clocks on
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  I2S_InitStructure.I2S_Standard = I2S_Standard_Phillips;
  I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_16b;
  I2S_InitStructure.I2S_CPOL = I2S_CPOL_High;
  I2S_InitStructure.I2S_Mode = I2S_Mode_SlaveRx;

  I2S_Init(I2Sx, &I2S_InitStructure);

  I2S_FullDuplexConfig(I2Sxext, &I2S_InitStructure); // enable the I2Sx_ext peripheral for Full Duplex mode */ 

  DonaldDelay(30000);

  GPIO_SetBits(GPIOD, GPIO_Pin_11); // pi board reset high

  Codec_CtrlInterface_Init();
	configure_codec(fs, select_input);
 
  DonaldDelay(30000);

  switch(io_method)
  {
    case IO_METHOD_POLL:
	  break;

	  case IO_METHOD_INTR:
      NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
      NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
      DonaldDelay(30000);
      SPI_I2S_ClearFlag(I2Sx, I2S_FLAG_CHSIDE); 
      SPI_I2S_ClearFlag(I2Sxext, I2S_FLAG_CHSIDE); 
  // enable the Rx buffer not empty interrupt
      SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
    break;

		case IO_METHOD_DMA:
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
      DMA_Cmd(DMA1_Stream3,DISABLE);
      DMA_DeInit(DMA1_Stream3);                                                    // parameters for stream 3
      DMA_InitStructure.DMA_Channel = DMA_Channel_0;                               // DMA channel #0
      DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                       // FIFO mode not used 
      DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                      // from I2S peripheral to memory
      DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;          // 16-bit sample values
      DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                              // circular mode later
		                                                                               // forced by double-buffer mode
      DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;             // I2S peripheral address is fixed
      DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                      // increment memory address
      DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(SPI2_BASE + 0x0C);     // address of I2S2 peripheral
      DMA_InitStructure.DMA_Priority = DMA_Priority_High;                          // set high priority
      DMA_InitStructure.DMA_BufferSize = BUFSIZE;                                  // BUFSIZE sample values (L+R)
		                                                                               // in BUFSIZE/2 sample instants
      DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;          // don't use burst 
      DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                  // don't use burst 
      DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;                // set FIFO threshold
      DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  // 16-bit sample values
      DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) pingIN;                   // address of memory buffer
 
      DMA_Init(DMA1_Stream3,&DMA_InitStructure);                                   // initialise stream 3
 
      DMA_DoubleBufferModeConfig(DMA1_Stream3, (uint32_t) pongIN, DMA_Memory_0);   // set up double-buffer mode
      DMA_DoubleBufferModeCmd(DMA1_Stream3, ENABLE);
                                                                                   // parameters for stream 4
      DMA_InitStructure.DMA_Channel = DMA_Channel_2;                               // DMA channel #2
      DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                       // FIFO mode not used 
      DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                      // from memory to I2S peripheral
      DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;          // 16-bit sample values
      DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                              // circular mode later
			                                                                             // forced by double-buffer mode
      DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;             // I2S peripheral address is fixed
      DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                      // increment memory address 
      DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(I2S2ext_BASE + 0x0C);  // address of I2S2_ext peripheral
      DMA_InitStructure.DMA_Priority = DMA_Priority_High;                          // set high priority
      DMA_InitStructure.DMA_BufferSize = BUFSIZE;                                  // BUFSIZE sample values (L+R)
			                                                                             // in BUFSIZE/2 sample instants
      DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;          // don't use burst 
      DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;                // set FIFO threshold
      DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  // 16-bit sample values
      DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) pingOUT;                  // address of memory buffer
 
      DMA_Init(DMA1_Stream4,&DMA_InitStructure);                                   // initialise stream 3
 
      DMA_DoubleBufferModeConfig(DMA1_Stream4, (uint32_t) pongOUT, DMA_Memory_0);  // set up double-buffer mode
      DMA_DoubleBufferModeCmd(DMA1_Stream4, ENABLE);
 
      DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
      DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);
 
	    SPI_I2S_DMACmd(I2Sx, SPI_I2S_DMAReq_Rx, ENABLE);
  	  DMA_Cmd(DMA1_Stream3,ENABLE);
	    SPI_I2S_DMACmd(I2Sxext, SPI_I2S_DMAReq_Tx, ENABLE);
  	  DMA_Cmd(DMA1_Stream4,ENABLE);
   
      NVIC_EnableIRQ(DMA1_Stream3_IRQn);
      NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    break;

		default:
      NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
      NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
      DonaldDelay(30000);
      SPI_I2S_ClearFlag(I2Sx, I2S_FLAG_CHSIDE); 
      SPI_I2S_ClearFlag(I2Sxext, I2S_FLAG_CHSIDE); 
// enable the Rx buffer not empty interrupt
      SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
    break;
  }
}


