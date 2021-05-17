
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "uartMsg.h"
#include "ssd1306.h"
#include "i2c-lcd.h"
#include "uart_protocol.h"
#include "ubx.h"
#include "ADRF6820.h"
#include "adrf_6720.h"
#include "adrf_6820.h"
#include "TB334.h"

#define COUNTOF(__BUFFER__)   		(sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
#define TXBUFFERSIZE               	(COUNTOF(aTxBuffer) - 1)
#define RXBUFFERSIZE              	11
#define BufferSize  				750

#define MMIO16(addr)  (*(volatile uint16_t *)(addr))
#define MMIO32(addr)  (*(volatile uint32_t *)(addr))
#define U_ID          0x1ffff7e8

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

IWDG_HandleTypeDef hiwdg;

struct ADRFData adrf6820;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Gps_bridge_ON();
void RFchip_write_reg();
void esp_reset();

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
UartReqPackConf_t        		uartReqPackConf;
UartResPackConf_t        		uartResPackConf;

__IO uint8_t volatile Req_Conf_Flag = 0;
__IO uint8_t volatile Req_ubx_Flag = 0;

uint32_t last_hb_req_tick = 0;
uint32_t last_hb_res_tick = 0;

uint8_t 	DST_Buffer[BufferSize];
uint8_t 	gps_readBuf[BufferSize] = {0};


uint8_t 	BootloaderStart_ID = 0;
uint8_t 	rx_part_complate = 0;
uint8_t 	cnt = 0;
uint32_t 	number_SendBuf = 0x00;
uint8_t		tryAgain;

uint32_t timeout_StartTime = 0;

uint8_t             resPackSeq = 0;

uint32_t            reqSessionId = 0;
UartReqPackHeader_t reqHeaderRead;
uint                runTestForIndex;
uint8_t             singleTestCount = 1;
uint16_t            currentTest = 0;
__IO uint8_t        isRunning = 0;
uint8_t             ledToggleRed = 0;
uint8_t             ledToggleGreen = 0;
uint8_t             ledToggleBlue = 0;
uint8_t             uartRx_buf[UART_PACKET_SIZE] = {0};
uint8_t             uartTx_buf[UART_PACKET_SIZE] = {0};
uint8_t             uartRx_payload[UART_PACKET_SIZE - sizeof(UartReqPackHeader_t)] = {0};

uint8_t 			isDownloding = 0;
volatile uint16_t start_index = 0;

//struct msg_mngmnt_s uart1Mng;
//struct msg_mngmnt_s uart2Mng;
//struct msg_mngmnt_s uart3Mng;

uint8_t msg1[] = "**** uart -> gps   **** \n\r ";
uint8_t msg2[] = "**** uart -> stm32 **** \n\r ";
char sbuf[25];

uint8_t aRxBuffer[RXBUFFERSIZE];

#define UART_BUFFER_SIZE 256
uint8_t             uartRx_buf_irq[UART_BUFFER_SIZE] = {0};
uint8_t             uartTx_buf_irq[UART_BUFFER_SIZE] = {0};
uint8_t uart_toogle = 1;
#define timeout 						5000

/* public */

/* private */
#define line_size             256
static char line_buffer[line_size];

char ipv4[13] = {0};
char mask[13] = {0};
char gw[13]   = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
//static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
//static void MX_IWDG_Init(void);
uint8_t read_line(UART_HandleTypeDef *huart);
uint8_t StartParsingDebugData(void);

void All_nemea_msg_OFF();

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t ADRF6720_isLock = 0;
uint8_t ADRF6820_isLock = 0;
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
size_t sizeOfReqPack(UartReqPackKey_t key) {
  switch(key) {
  case UartReqPack_Conf:         return sizeof(UartReqPackConf_t);

  default: break;
  }
  return 0;
}

uint8_t * reqPack(UartReqPackKey_t key) {
  switch(key) {
  case UartReqPack_Conf:         return (uint8_t*)&uartReqPackConf;
  default: break;
  }
  return (uint8_t*)NULL;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
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
  MX_SPI2_Init();
  MX_I2C1_Init();
  //MX_IWDG_Init();
  //MX_USART2_UART_Init();
  //ikinci uart kullanilmiyor

  /* USER CODE BEGIN 2 */
//HAL_IWDG_Init(&hiwdg);

  SDIO_PIN_Setup();

  MX_SPI1_Init();
  All_nemea_msg_OFF();
  HAL_SPI_DeInit(&hspi1);
//
  Uart_Transmit_Header_Init();

  lcd_init ();
  SSD1306_Init ();
//
  SSD1306_Fill (0);
  SSD1306_UpdateScreen(); //display
  SSD1306_GotoXY (0, 0);
  sprintf(sbuf,"id: %d %d %d %d",uartResPackConf.id[0]
								,uartResPackConf.id[1]
								,uartResPackConf.id[2]
								,uartResPackConf.id[3]);

  SSD1306_Puts ((char*)sbuf, &Font_7x10, 1);
  SSD1306_GotoXY (10, 20);
  SSD1306_Puts ("GPS-Link", &Font_11x18, 1);
  SSD1306_UpdateScreen(); //display
  //HAL_IWDG_Refresh(&hiwdg);
  MX_USART1_UART_Init();
  HAL_Delay(1000);
  esp_reset();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  while(1){
		  if(read_line(&huart1)==HAL_OK){
		 		  if(StartParsingDebugData()==HAL_OK){
		 			   SSD1306_GotoXY(0,40);
		 			   SSD1306_Puts("ipv4:",&Font_7x10,1);
		 			   SSD1306_GotoXY(0,50);
		 			   SSD1306_Puts(ipv4,&Font_7x10,1);
		 			   SSD1306_UpdateScreen();
		 			   break;
		 		  }

		 	  }
		 	  HAL_Delay(100);
		   }
	  HAL_Delay(1000);
	  while(Req_Conf_Flag != 1){
		  SSD1306_GotoXY (0, 20);
		  SSD1306_Puts ("GPS-Link-Ok", &Font_11x18, 1);
		  SSD1306_UpdateScreen(); //display
		  HAL_UART_RxCpltCallback(&huart1);
		  //HAL_IWDG_Refresh(&hiwdg);
	  }
	  //HAL_IWDG_Refresh(&hiwdg);
	  while(Req_Conf_Flag == 1){

		  //HAL_IWDG_Refresh(&hiwdg);
		  Req_Conf_Flag = 0;
		  TB334_GPIO_Init();

		  SSD1306_Fill(SSD1306_COLOR_BLACK);
		  SSD1306_GotoXY (0, 0);
		  sprintf(sbuf,"id: %d %d %d %d",uartResPackConf.id[0]
										,uartResPackConf.id[1]
									    ,uartResPackConf.id[2]
									    ,uartResPackConf.id[3]);

		  SSD1306_Puts ((char*)sbuf, &Font_7x10, 1);

		  if(uartReqPackConf.ADRF_id != 0){
			  if(ADRF6820_isLock != 1 && uartReqPackConf.ADRF_id == 1){
				  SSD1306_GotoXY (10, 20);
				  sprintf(sbuf,"Set-6820");
				  ADRF_Setup_GPS(&adrf6820);
				  ADRF6820_isLock = 1;
				  SSD1306_Puts ((char*)sbuf, &Font_11x18, 1);

			  }
			  if(ADRF6720_isLock != 1 && uartReqPackConf.ADRF_id == 2){
				  SSD1306_GotoXY (10, 20);
				  sprintf(sbuf,"Set-6720");
				  ADRF6720_Setup_GPS();
				  ADRF6720_isLock = 1;
				  SSD1306_Puts ((char*)sbuf, &Font_11x18, 1);

			  }
		  }else if(ADRF6820_isLock != 1 || ADRF6720_isLock != 1){
			  SSD1306_GotoXY (10, 20);
			  sprintf(sbuf,"Set-Non");
			  SSD1306_Puts ((char*)sbuf, &Font_11x18, 1);
		  }


		  SSD1306_GotoXY (0, 43);
		  sprintf(sbuf,"Attenuator set:");
		  SSD1306_Puts ((char*)sbuf, &Font_7x10, 1);

		  SSD1306_GotoXY (0, 53);
		  if(uartReqPackConf.ADRF_id == spi_chip1){
			  ADRF_Setup_GPS(&adrf6820);
		  }
		  else if(uartReqPackConf.ADRF_id == spi_chip2){
			  ADRF6720_Setup_GPS();

	  }

		  sprintf(sbuf,"%d",(uartReqPackConf.TB334_SET));

		  //attenuator set
		  if(attenuator_set(uartReqPackConf.TB334_SET) != HAL_OK)
		  {
			  sprintf(sbuf,"undefined");
		  }else{
			  sprintf(sbuf,"%d",(uartReqPackConf.TB334_SET));
		  }

		  SSD1306_Puts ((char*)sbuf, &Font_7x10, 1);
		  SSD1306_UpdateScreen(); //display
		  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

		  //RFchip_write_reg();

	  }
	  memset(uartTx_buf, 0   , UART_PACKET_SIZE);
	  memcpy(uartTx_buf, (uint8_t*)&uartResPackConf, sizeof(UartResPackConf_t));

	  if(uartReqPackConf.TB334_SET != 50){
	   HAL_UART_Transmit(&huart1, uartTx_buf, sizeof(UartResPackConf_t), 1000);
	  }
	  uart_toogle = 1;
	  HAL_Delay(50);
	  HAL_UART_DeInit(&huart1);
	  esp_reset();
	  HAL_Delay(1000);
	  MX_USART1_UART_Init();

	  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}


/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{

	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	  /** Initializes the CPU, AHB and APB busses clocks
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
	  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Initializes the CPU, AHB and APB busses clocks
	  */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
	  Error_Handler();
  }
  //__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
//  if(HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
//  {
//    Error_Handler();
//  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */

//usart2 kullanilmasi durumunda commenti acin

//static void MX_USART2_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART2_Init 0 */
//
//  /* USER CODE END USART2_Init 0 */
//
//  /* USER CODE BEGIN USART2_Init 1 */
//
//  /* USER CODE END USART2_Init 1 */
//  huart2.Instance = USART2;
//  huart2.Init.BaudRate = 9600;
//  huart2.Init.WordLength = UART_WORDLENGTH_8B;
//  huart2.Init.StopBits = UART_STOPBITS_1;
//  huart2.Init.Parity = UART_PARITY_NONE;
//  huart2.Init.Mode = UART_MODE_TX_RX;
//  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
//  if (HAL_UART_Init(&huart2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART2_Init 2 */
//
//  /* USER CODE END USART2_Init 2 */
//
//}


/* Configure pins as
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_11, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_11 /*| GPIO_PIN_12*/, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 |GPIO_PIN_7 , GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(ATT_CLK_GPIO_Port, ATT_CLK_Pin | ATT_DATA_Pin | ATT_LE_Pin , GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin =  GPIO_PIN_12|GPIO_PIN_11 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13*/
  GPIO_InitStruct.Pin =  GPIO_PIN_13 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

//  GPIO_InitStruct.Pin = ATT_CLK_Pin | ATT_DATA_Pin | ATT_LE_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(ATT_CLK_GPIO_Port, &GPIO_InitStruct);


}

void TB334_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();


  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_7, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin =  GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

uint8_t StartParsingDebugData(void){

	//473766> ip:192.168.43.65,mask:255.255.255.0,gw:192.168.43.1
	//473766> Wifi got ip:192.168.43.65,mask:255.255.255.0,gw:192.168.43.1
	//                    ^ip           ^mask              ^gw
	int j = 0;
	for( j = 0; j <= line_size; j++){
		if (line_buffer[j] == '>' && line_buffer[j+2] == 'i' && line_buffer[j+3] == 'p'){
			break;
		}
	if(j >= 254)return HAL_ERROR;

	}

		char * begin = line_buffer;
		char * end = line_buffer;

		begin = strchr(begin, ':');
		end=strchr(end, ':');
		end=strchr(end, ',');
		begin++;
		memset(ipv4, 0,13);
		strncpy(ipv4,begin,end-begin);
		ipv4[end-begin]='\0';


		begin = strchr(begin, ':');
		end=strchr(end, ':');
		end=strchr(end, ',');
		begin++;
		memset(mask, 0,13);
		strncpy(mask,begin,end-begin);
		mask[end-begin]='\0';


		begin = strchr(begin, ':');
		end=strchr(end, ':');
		end=strchr(end, '\n');
		begin++;
		memset(gw, 0,13);
		strncpy(gw,begin,end-begin);
		gw[(end-begin)-1]='\0';

		return HAL_OK;
}

uint8_t read_line(UART_HandleTypeDef *huart){

	uint8_t m=0;
	char a= 0;

	memset(line_buffer, 0, sizeof(line_buffer));

	while(1){
		HAL_UART_Receive(huart,(uint8_t*)&a,1,300);

		line_buffer[m]=a;
		if(m>0 && a =='\n'){
			return HAL_OK;
		}
		m++;
		if(m>=line_size){
			return HAL_ERROR;
		}
}
}


/* USER CODE BEGIN 4 */

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
//static void MX_IWDG_Init(void)
//{
//
//  /* USER CODE BEGIN IWDG_Init 0 */
//
//  /* USER CODE END IWDG_Init 0 */
//
//  /* USER CODE BEGIN IWDG_Init 1 */
//
//  /* USER CODE END IWDG_Init 1 */
//  hiwdg.Instance = IWDG;
//  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
//  hiwdg.Init.Reload = 4095;
//  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN IWDG_Init 2 */
//
//  /* USER CODE END IWDG_Init 2 */
//
//}

void Uart_Transmit_Header_Init()
{
  memset(&uartResPackConf,        			0, sizeof(UartResPackConf_t));


  uartResPackConf.sop        		= ANT_UART_SOP;
  uartResPackConf.key        		= uartResPack_Conf;
  uartResPackConf.id[0]				= MMIO16(U_ID + 0x0);
  uartResPackConf.id[1]				= MMIO16(U_ID + 0x2);
  uartResPackConf.id[2]				= MMIO32(U_ID + 0x4);
  uartResPackConf.id[3]				= MMIO32(U_ID + 0x8);

}

void esp_reset()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);

	}

void RFchip_write_reg()
{
	uint8_t writeBuf[8] 	 = {0};
	uint8_t readBuf[8]  	 = {0};
	uint8_t transferSize     =  5;

//	memcpy((uint8_t*)&writeBuf,(uint8_t*)&uartReqPackConf.RF1_max, sizeof(UartReqPackConf_t)-5);
//
//	if(uartReqPackConf.id == spi_chip1 || uartReqPackConf.id == spi_chip2){
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
//	}else{
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
//	}
	HAL_SPI_TransmitReceive(&hspi2,(uint8_t*)writeBuf,(uint8_t*)readBuf,transferSize,1000);
//
//	if(uartReqPackConf.id == spi_chip1 || uartReqPackConf.id == spi_chip2){
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
//	}else{
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
//	}

}

void Gps_bridge_ON()
{
	uint8_t dummyBuff[500] = {0xFF};
	int pars_ok = 0;
	int poll_cnt = 0;
	uint16_t i = 0;

	SSD1306_GotoXY (10, 20);
	SSD1306_Puts ("GPS-Bridge", &Font_11x18, 1);
	SSD1306_UpdateScreen(); //display
	MX_SPI1_Init();

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_Delay(1); //wait until the device select.
	HAL_SPI_Transmit(&hspi1,(uint8_t*)dummyBuff,500,1000); // send to 0xFF for clean ublox buffer
	HAL_Delay(1); //wait until the device select.
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	HAL_Delay(1); //wait until the resend.


	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_Delay(1); //wait until the device select.
	HAL_SPI_Transmit(&hspi1,(uint8_t*)DST_Buffer,9,1000); // send to ubx poll messages
	HAL_Delay(1); //wait until the device select.
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	HAL_Delay(300); //wait until the ubx messages package is ready.

	while(pars_ok != 1 ){

		//SSD1306_GotoXY (10, 20);
						//SSD1306_Puts ("aliyom   ", &Font_11x18, 1);
						//SSD1306_UpdateScreen(); //display
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_Delay(1); //wait until the device select.
		HAL_SPI_Receive(&hspi1,(uint8_t*)gps_readBuf,BufferSize,1000); // receive to ubx packet
		HAL_Delay(1); //wait until the device select.
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_UART_Transmit(&huart2,(uint8_t*)gps_readBuf,BufferSize,1000);

		while(pars_ok == 0 && gps_readBuf[0] != 0xFF){


			if((gps_readBuf[i] == 0xB5) && (gps_readBuf[i+1] == 0x62) && (gps_readBuf[i+2] == DST_Buffer[2]) && (gps_readBuf[i+3] == DST_Buffer[3] )){
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				HAL_UART_Transmit(&huart1,(uint8_t*)&gps_readBuf[i],250,500);
				//SSD1306_GotoXY (10, 20);
				//SSD1306_Puts ("yolluyom    ", &Font_11x18, 1);
				//SSD1306_UpdateScreen(); //display
				pars_ok = 1;
				break;
			}else if(i >= BufferSize){
				HAL_UART_Transmit(&huart1,(uint8_t*)gps_readBuf,250,1000);
				pars_ok = -1;
				break;

			}
			//SSD1306_GotoXY (10, 20);
			//SSD1306_Puts ("while       ", &Font_11x18, 1);
			//SSD1306_UpdateScreen(); //display
			i++;

		}
		poll_cnt++;
		HAL_Delay(300);
		if(poll_cnt==3){
			break;
		}
	}

//	for(;;){
//
//
//		if((gps_readBuf[i] == 0xB5) && (gps_readBuf[i+1] == 0x62) && (gps_readBuf[i+2] == DST_Buffer[2]) && (gps_readBuf[i+3] == DST_Buffer[3] )){
//
	//			HAL_UART_Transmit(&huart1,(uint8_t*)&gps_readBuf[i],500,1000);
	//
	//			break;
	//		}else if(i == BufferSize){
	//			HAL_UART_Transmit(&huart1,(uint8_t*)gps_readBuf,500,1000);
	//			break;
	//
	//		}
	//		i++;
	//	}
	HAL_SPI_DeInit(&hspi1);
	  HAL_UART_DeInit(&huart1);
	  HAL_Delay(100);
	  MX_USART1_UART_Init();


}

void All_nemea_msg_OFF()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)spi_on, sizeof(spi_on), 1000);						// Send data byte
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)msg0_off, sizeof(msg0_off), 1000);						// Send data byte
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)msg1_off, sizeof(msg1_off), 1000);						// Send data byte
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)msg2_off, sizeof(msg2_off), 1000);						// Send data byte
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)msg3_off, sizeof(msg3_off), 1000);						// Send data byte
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)msg4_off, sizeof(msg4_off), 1000);						// Send data byte
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)msg5_off, sizeof(msg5_off), 1000);						// Send data byte
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)msg6_off, sizeof(msg6_off), 1000);						// Send data byte
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)msg7_off, sizeof(msg7_off), 1000);						// Send data byte
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)msg8_off, sizeof(msg8_off), 1000);						// Send data byte
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)msg9_off, sizeof(msg9_off), 1000);						// Send data byte
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)msg10_off, sizeof(msg10_off), 1000);						// Send data byte
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)msg11_off, sizeof(msg11_off), 1000);						// Send data byte
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)msg12_off, sizeof(msg12_off), 1000);						// Send data byte
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)msg13_off, sizeof(msg13_off), 1000);						// Send data byte
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)msg14_off, sizeof(msg14_off), 1000);						// Send data byte
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)msg15_off, sizeof(msg15_off), 1000);						// Send data byte
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)msg16_off, sizeof(msg16_off), 1000);						// Send data byte
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)msg17_off, sizeof(msg17_off), 1000);						// Send data byte
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)msg18_off, sizeof(msg18_off), 1000);						// Send data byte
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

}
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
//{
//
//	if(UartHandle->Instance == USART1)
//	{
//		uart1Mng.txReadyFlag = READY2SEND;
//
//
//	}
//
//	if(UartHandle->Instance == USART2)
//	{
//		uart2Mng.txReadyFlag = READY2SEND;
//	}
//
//}
//
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	//__HAL_UART_CLEAR_FLAG(&huart1,UART_IT_RXNE);
	//__HAL_UART_DISABLE_IT(&huart1,UART_IT_RXNE);

	//timeout_StartTime = HAL_GetTick();

	//		memcpy(&rxBuffer[number_SendBuf],DST_Buffer, sizeof(DST_Buffer));
	//		HAL_UART_Receive(&huart2,1+(uint8_t*)DST_Buffer,1,10);

	//int key = DST_Buffer[start_index + 1];
	__HAL_UART_FLUSH_DRREGISTER(&huart1);
	HAL_UART_Receive(&huart1,(uint8_t*)DST_Buffer,16,1000);


	//SSD1306_GotoXY (10, 20);
		//SSD1306_Puts ("bekliyom    ", &Font_11x18, 1);
		//SSD1306_UpdateScreen(); //display
	if(DST_Buffer[0] == 0xB5 && DST_Buffer[1] == 0x62){
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		Req_ubx_Flag = 1;
		Gps_bridge_ON();

	}else if(DST_Buffer[0] == ANT_UART_SOP){
		int key = DST_Buffer[start_index+1];
		size_t sz = sizeOfReqPack(key);
		//DST_Buffer = DST_Buffer >> 2;

		//TODO  try to get rid of time consuming operations away from ISR
		//			1) functions,  reqPack, crc_is_valid, memcpy
		uint8_t * req = reqPack(key);
		if (req != NULL) {
			size_t sz = sizeOfReqPack(key);
			memcpy(req, &(DST_Buffer[start_index]), sz);

			//if ( crc_is_valid(req, sz)) {
			switch (key) {
			case UartReqPack_Conf:
				Req_Conf_Flag = 1;
				break;
			default:
				break;
			} // switch
			//} // if crc
		} // if req*/
	}
	memset(DST_Buffer, 0   , UART_PACKET_SIZE);
	memset(gps_readBuf, 0   , 500);

	//__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);

}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
//  while(1)
//  {
//	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//
//  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
