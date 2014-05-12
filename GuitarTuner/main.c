
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdio.h>
#include "arm_math.h"               // Required to use float32_t type
#include "main.h"
#include "pdm_filter.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_lcd.h"
#include "stm32f4xx.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t  PDM_Input_Buffer[PDM_Input_Buffer_SIZE];
uint16_t PCM_Output_Buffer[PCM_Output_Buffer_SIZE];

float32_t buffer_input[1024];
float32_t buffer_input_copy[256];
float32_t buffer_output[1024];
float32_t buffer_output_mag[1024];
float32_t buffer_output_mag_copy[256];
float32_t maxvalue;
int  maxvalueindex;
float32_t test;
uint8_t   mode;

uint16_t  g_SPECTRUM_Xpos;
uint8_t   g_SPECTRUM_Ypos;
char      text[100];

arm_rfft_instance_f32 S;
arm_cfft_radix4_instance_f32 S_CFFT;
PDMFilter_InitStruct Filter;

/* Private function prototypes -----------------------------------------------*/
static void GPIO_Configure(void);
static void I2S_Configure(void);
static void NVIC_Configure(void);
static void RCC_Configure(void);



/* Functions -----------------------------------------------------------------*/
int main(void)
{
	extern uint32_t Data_Status;

	unsigned int freq, i, z;
	float dt;

	RCC_Configure();
	NVIC_Configure();
	GPIO_Configure();
	I2S_Configure();

	// Initialize PDM filter
	Filter.Fs = OUT_FREQ;
	Filter.HP_HZ = 10;
	Filter.LP_HZ = 16000;
	Filter.In_MicChannels = 1;
	Filter.Out_MicChannels = 1;
	PDM_Filter_Init(&Filter);

	arm_rfft_init_f32(&S, &S_CFFT, 512, 0, 1);

	// Initialize User Button
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO);


	while(STM_EVAL_PBGetState(BUTTON_USER) == RESET);
	while(STM_EVAL_PBGetState(BUTTON_USER) == SET);

	while(1)
	{
		// Clear arrays
		for(i=0; i<256; ++i)
		{
			buffer_input_copy[i] = 0;
			buffer_output_mag_copy[i] = 0;
		}

		dt = 0.001;
		freq = 0;
		mode = 1;

		while(mode == 1 || mode == 2 || mode == 3 || mode == 4)
		{
			++freq;
			if(freq > 512*2)
				freq = 1;

			if(STM_EVAL_PBGetState(BUTTON_USER) == SET)
			{
				while(STM_EVAL_PBGetState(BUTTON_USER) == SET);

				++mode;

				if(mode > 4)
					break;

				freq = 1;
			}

			// ************************************************************
			// Generate signal
			if(mode == 1)
			{
				for(i=0; i<512; ++i)
				{
					buffer_input[i] = (float32_t) 30*sin(2*PI*freq*i*dt);
				}
			}
			else if(mode == 2)
			{
				for(i=0; i<512; ++i)
				{
					buffer_input[i] = (float32_t) 20*sin(2*PI*freq*i*dt) + 10*sin(2*PI*2*freq*i*dt);
				}
			}
			else if(mode == 3)
			{
				for(i=0; i<512; ++i)
				{
					buffer_input[i] = (float32_t) 15*sin(2*PI*freq*i*dt) + 10*sin(2*PI*2*freq*i*dt) + 5*sin(2*PI*3*freq*i*dt);
				}
			}
			else if(mode == 4)
			{
				for(i=0; i<512; ++i)
				{
					buffer_input[i] = (float32_t) 15*sin(2*PI*freq*i*dt) + 10*sin(2*PI*2*freq*i*dt) + 3*sin(2*PI*3*freq*i*dt) + 2*sin(2*PI*4*freq*i*dt);
				}
			}


			// ************************************************************
			// Update mirror buffers
			for(i=0; i<256; ++i)
			{
				buffer_input_copy[i] = buffer_input[i];
				buffer_output_mag_copy[i] = buffer_output_mag[i];
			}

			// ************************************************************
			// Calculate Real FFT
			arm_rfft_f32(&S, buffer_input, buffer_output);
			// Calculate magnitude
			arm_cmplx_mag_f32(buffer_output, buffer_output_mag, 512);
			// Get maximum value of magnitude
			arm_max_f32(buffer_output_mag, 512, &maxvalue, &maxvalueindex);
			// Scale magnitude values
			for(i=0; i<512; ++i)
			{
				buffer_output_mag[i] = 100*buffer_output_mag[i]/maxvalue;
			}


			// ************************************************************
			// Wait some time
			for(i=0; i<0x20000; ++i)
			{

			}
		}


		// Clear arrays
		for(i=0; i<256; ++i)
		{
			buffer_input_copy[i] = 0;
			buffer_output_mag_copy[i] = 0;
		}

		z = 0;

		I2S_Cmd(SPI2, ENABLE);

		while(1)
		{
			if(STM_EVAL_PBGetState(BUTTON_USER) == SET)
			{
				while(STM_EVAL_PBGetState(BUTTON_USER) == SET);
				break;
			}

			if(Data_Status)
			{
				for(i=0; i<(OUT_FREQ/1000); i++)
				{
					buffer_input[i+(OUT_FREQ/1000)*z] = (float32_t) PCM_Output_Buffer[i];
				}

				++z;
				if(z > 512/(OUT_FREQ/1000))
				{
					z = 0;

					// ************************************************************
					// Calculate Real FFT
					arm_rfft_f32(&S, buffer_input, buffer_output);
					// Calculate magnitude
					arm_cmplx_mag_f32(buffer_output, buffer_output_mag, 512);
					// Get maximum value of magnitude
					arm_max_f32(&(buffer_output_mag[1]), 512, &maxvalue, &maxvalueindex);
					// Scale magnitude values
					for(i=0; i<512; ++i)
					{
						//buffer_output_mag[i] = 100*buffer_output_mag[i]/maxvalue;
						buffer_output_mag[i+1] = 140*buffer_output_mag[i+1]/20000000;
					}

				}
				Data_Status = 0;
			}

			sprintf(text,"F=%2.2f[kHz]   ",(maxvalueindex+1)*32.0/512);
			test = (maxvalueindex+1)*32.0/512;//wypisz

			// Wait some time
			for(i=0; i<0x10000; ++i);
		}

		I2S_Cmd(SPI2, DISABLE);
	}
}



/* Private functions ---------------------------------------------------------*/
static void GPIO_Configure(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// Configure MP45DT02's CLK / I2S2_CLK (PB10) line
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Configure MP45DT02's DOUT / I2S2_DATA (PC3) line
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI2);  // Connect pin 10 of port B to the SPI2 peripheral
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI2);   // Connect pin 3 of port C to the SPI2 peripheral
}

static void I2S_Configure(void)
{
	I2S_InitTypeDef I2S_InitStructure;

	SPI_I2S_DeInit(SPI2);
	I2S_InitStructure.I2S_AudioFreq = OUT_FREQ*2;
	I2S_InitStructure.I2S_Standard = I2S_Standard_LSB;
	I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_16b;
	I2S_InitStructure.I2S_CPOL = I2S_CPOL_High;
	I2S_InitStructure.I2S_Mode = I2S_Mode_MasterRx;
	I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
	I2S_Init(SPI2, &I2S_InitStructure);

	// Enable the Rx buffer not empty interrupt
	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
}

static void NVIC_Configure(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	// Configure the interrupt priority grouping
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	// Configure the SPI2 interrupt channel
	NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
}

static void RCC_Configure(void)
{
	// Enable CRC module - required by PDM Library
	//RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;

	/********/
	/* AHB1 */
	/********/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC |
		RCC_AHB1Periph_CRC, ENABLE);

	/********/
	/* APB1 */
	/********/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	RCC_PLLI2SCmd(ENABLE);
}
