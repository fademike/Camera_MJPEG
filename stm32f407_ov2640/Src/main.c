/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

#include "ov2640.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/



extern unsigned char RX_ready;
extern unsigned char RX_cmd[16];
extern volatile unsigned char RX_cmd_get;
extern unsigned char RX_value[16];
extern unsigned char RX_cmd_cnt;
extern volatile unsigned char RX_pos;
extern unsigned char RX_len;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DCMI_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void SendData(unsigned char * buff, int len);
#define IRQ HAL_GPIO_ReadPin(nRF_IRQ_GPIO_Port, nRF_IRQ_Pin)

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

#include <stdarg.h>
#include <string.h> //strlen()

void Printf(const char *fmt, ...)
{return;
	static char buf[256];
	char *p;
	va_list lst;

	va_start(lst, fmt);
	vsprintf(buf, fmt, lst);
	va_end(lst);

	p = buf;
	HAL_UART_Transmit(&huart2, (unsigned char *)p, strlen(p), 500);
	HAL_UART_Transmit(&huart1, (unsigned char *)p, strlen(p), 500);
	/*while(*p) {
		HAL_UART_Transmit(&huart1, (unsigned char *)p, 1, 500);
		p++;
	}*/
}

int SCCB_Write(uint8_t addr, uint8_t data)	// Send data to the Camera on the I2C interface
{
	uint8_t Data[] = {addr, data};
	int ret = HAL_I2C_Master_Transmit(&hi2c2, OV9655_DEVICE_WRITE_ADDRESS, Data, 2, 100);
	HAL_Delay(2);
	return ret;
}




int OV2640_Settings(int format, int sat , int precision)
{
	int i=0;

	if (format != -1){
		if(format == 1) {//OV2640_352x288_JPEG
				for(i=0; i<(sizeof(OV2640_352x288_JPEG)/2); i++)	//OV2640_320x240_JPEG
				SCCB_Write(OV2640_352x288_JPEG[i][0], OV2640_352x288_JPEG[i][1]);
		}
		else if(format == 2) {
				for(i=0; i<(sizeof(OV2640_640x480_JPEG)/2); i++)
				SCCB_Write(OV2640_640x480_JPEG[i][0], OV2640_640x480_JPEG[i][1]);
		}
		else if(format == 3) {
				for(i=0; i<(sizeof(OV2640_800x600_JPEG)/2); i++)
				SCCB_Write(OV2640_800x600_JPEG[i][0], OV2640_800x600_JPEG[i][1]);
		}
		else if(format == 4) {
				for(i=0; i<(sizeof(OV2640_1024x768_JPEG)/2); i++)
				SCCB_Write(OV2640_1024x768_JPEG[i][0], OV2640_1024x768_JPEG[i][1]);
		}
		else if(format == 5) {
				for(i=0; i<(sizeof(OV2640_1280x1024_JPEG)/2); i++)
				SCCB_Write(OV2640_1280x1024_JPEG[i][0], OV2640_1280x1024_JPEG[i][1]);
		}
		else if(format == 6) {
				for(i=0; i<(sizeof(OV2640_1600x1200_JPEG)/2); i++)
				SCCB_Write(OV2640_1600x1200_JPEG[i][0], OV2640_1600x1200_JPEG[i][1]);
		}


	}
	if (sat != -1){	// Color Saturation
		SCCB_Write(0xff, 0x00);
		SCCB_Write(0x7c, 0x00);
		SCCB_Write(0x7d, 0x02);
		SCCB_Write(0x7c, 0x03);
		if(sat){
			SCCB_Write(0x7d, 0x48);
			SCCB_Write(0x7d, 0x48);
		}
		else{
			SCCB_Write(0x7d, 0x0);
			SCCB_Write(0x7d, 0x0);
		}

	}
	if (precision != -1){
		SCCB_Write(0xFF, 0x00); // {0x44, 0x10},
		SCCB_Write(0x44, precision); //5-63
	}

	return 0;

}

#define BUFFER_SIZE 60*1024

unsigned char Camera_buff_n1[BUFFER_SIZE];	// Buffer 1 for JPEG image
unsigned char Camera_buff_n2[BUFFER_SIZE];	// Buffer 2 for JPEG image

int currDMADataCounter = 0;	// Length of last image
int DataExist = 0;			// Data in Buffer is exist
int NChannel = 0;			// Switch Channel of Buffer


void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	int ndtr = hdcmi->DMA_Handle->Instance->NDTR;
	currDMADataCounter = (BUFFER_SIZE-ndtr)*4;
	HAL_DCMI_Stop(hdcmi);

	__HAL_DCMI_ENABLE_IT(hdcmi, DCMI_IT_FRAME);	 // Enable next irq
	if (NChannel == 0){
		HAL_DCMI_Start_DMA(hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)&Camera_buff_n1, BUFFER_SIZE);	//Start of DMA for get one image in Buffer 1
	}
	else  {
		HAL_DCMI_Start_DMA(hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)&Camera_buff_n2, BUFFER_SIZE);	//Start of DMA for get one image in Buffer 2
	}


	DataExist = 1;	// Data in Buffer Exist

}



void HAL_SYSTICK_Callback(void)
{

}





/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DCMI_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);	// Enable Interrupt UART1
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);	// Enable Interrupt UART2

  HAL_GPIO_WritePin(RESETB_GPIO_Port, RESETB_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PW_DWN_GPIO_Port, PW_DWN_Pin, GPIO_PIN_RESET);

  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_5);	 // set the output frequency to 32 MHz in the Camera

  __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME);		// Enable Interrupt DCMI


  HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  int i=0;


	  int ret = SCCB_Write(0x12, 0x80);	// SET Cmd Power on


	  if (ret == 0){
		  HAL_Delay(1000);

		  for(i=0; (i<(sizeof(OV9655_QVGA)/2) && (ret == 0)); i++)	// init sequence for Camera
		  {
			ret = SCCB_Write(OV9655_QVGA[i][0], OV9655_QVGA[i][1]);
		  }
		  Printf("init %d, cnt=%d\r\n", ret, i);

		  ret = OV2640_Settings(1, 1, 100);	// Set advanced settings


		  if (ret == 0){	// If no problem...

			  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)&Camera_buff_n1, BUFFER_SIZE);	//Start of DMA for get one image in Buffer 1

			  while(1){


				if (RX_cmd_get !=0){	// if rx command for get image
					RX_cmd_get=0;		// clear flag

					while(DataExist == 0){};		// expect image
					if (NChannel == 0){	// Get data from buffer with image (DMA working in another buffer)
						SendData(Camera_buff_n2, currDMADataCounter);	// Send data to output
					}
					else  {
						SendData(Camera_buff_n1, currDMADataCounter);	// Send data to output
					}
					DataExist=0;
					if (++NChannel>=2)NChannel=0;	// Switch Buffer

				}

				if ((RX_ready==1)&&(RX_cmd_cnt != 0)){	// if Rx command to change Settings

					int format = 2;
					int sat = 1;
					int precision = 40;
					int count = 0;
					for (count = 0; count<RX_cmd_cnt; count++){
						if (RX_cmd[count] == 0x00) format = RX_value[count];
						else if (RX_cmd[count] == 0x01) sat = RX_value[count];
						else if (RX_cmd[count] == 0x44) precision = RX_value[count];
					}
					OV2640_Settings(format, sat, precision);	// Set new settings
					RX_cmd_cnt=0; // clear flag
				}


			  }

		  }



	  }
	  else{
		  Printf("error return is %d\n\r", ret);
	  }
	  HAL_Delay(10*1000);

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_5);

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* DCMI init function */
static void MX_DCMI_Init(void)
{

  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200*20;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200*20;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA8   ------> RCC_MCO_1
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RESETB_Pin|PW_DWN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, nRF_CE_Pin|nRF_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RESETB_Pin PW_DWN_Pin */
  GPIO_InitStruct.Pin = RESETB_Pin|PW_DWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : nRF_IRQ_Pin */
  GPIO_InitStruct.Pin = nRF_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(nRF_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : nRF_CE_Pin nRF_CSN_Pin */
  GPIO_InitStruct.Pin = nRF_CE_Pin|nRF_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */




unsigned char BuffToTx[32];

unsigned char ks = 0;				// for crc calculate

void Send_char(char symbol)
{
	ks ^= symbol;
	HAL_UART_Transmit(&huart2, (unsigned char *)&symbol, 1, 100);

}



void SendData(unsigned char * buff, int len)																																																		// SendUART
{

	unsigned int num = len;				// size image
	unsigned char * copy_from = buff;	// bufer image

	unsigned char num_s[4];

	if (num> (BUFFER_SIZE)) return;	// can not be!!

	num_s[0] = ((num>>(3*8))&0xFF);	// convert "int" in 4 byte
	num_s[1] = ((num>>(2*8))&0xFF);
	num_s[2] = ((num>>(1*8))&0xFF);
	num_s[3] = ((num>>(0*8))&0xFF);

	ks = 0;	// Clear CRC calculate

	Send_char(0x01);	// pointer to start cmd

	ks = 0x00;

	// Send cmd...
	Send_char('J');
	Send_char('P');
	Send_char('E');
	Send_char('G');
	Send_char('1');
	Send_char('.');
	Send_char('0');
	Send_char(num_s[0]);
	Send_char(num_s[1]);
	Send_char(num_s[2]);
	Send_char(num_s[3]);
	Send_char(ks);	// Send CRC

	while(num--)	// Send Data of image
	{
		unsigned char cRead = *copy_from++;
		Send_char(cRead);

	}
	Send_char(ks);  // send CRC

}




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
