/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#define buffersize 320
#define X_MAX 240
#define Y_MAX 239
#define adc_size 320
#include "main.h"
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"
#include "stdlib.h"
#include "stdio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
void DrawRaster(void);
void wave_freq_calculator(void);
int sum3(int k);
volatile int8_t conversion_ready;
uint16_t erase_1_buffer[buffersize]={0};
uint16_t erase_2_buffer[buffersize];
uint32_t adc_buffer[adc_size];
uint32_t trace_1_buffer[buffersize];
uint16_t trigger_ch =0;
volatile int8_t VRANGE;
volatile int8_t HRANGE;
volatile int16_t CHTRIG=100;
float sec;
int main(void)
{   uint16_t temp1;
   uint16_t i, j;
   uint16_t x1, y1, z1;
   uint16_t x2, y2, z2;
   uint16_t x3, y3, z3;
   uint16_t x4, y4, z4;
   uint16_t trigger_level = 100;                                             //middle of screen
   uint16_t previous_trigger_level = 120;
   uint16_t gnd=120;
   uint16_t trigger_point = 1;
   uint16_t previous_trigger_point = 1;
   uint16_t VMAX=0;
   uint16_t VMIN=0;
   float PVMAX=0;
   float PVMIN=0;
   float p2p=0;
   const char Buffer[30];


  /* USER CODE BEGIN 1 */
   LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
   LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
   NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
   LL_GPIO_AF_Remap_SWJ_NOJTAG();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */

  ILI9341_Init();
  ILI9341_SetRotation(SCREEN_HORIZONTAL_2);
 // DrawRaster();
  /* USER CODE END 2 */
  LL_ADC_Enable(ADC1);
LL_ADC_StartCalibration(ADC1);
while(LL_ADC_IsCalibrationOnGoing(ADC1));
LL_mDelay(10);
//default settings
   VRANGE = 2;

   HRANGE = 6;   //HRANGE on 10ms /div
     LL_TIM_SetPrescaler(TIM3, 0);
     LL_TIM_SetAutoReload(TIM3, 37499);
     conversion_ready=0;
   LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t) &ADC1->DR);
   LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t) &adc_buffer);
   LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, adc_size);
   LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);                           //Enable IRQ at transfer complete
   LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

   LL_ADC_REG_StartConversionExtTrig(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
   LL_TIM_EnableCounter(TIM3);
   //static char BufferText[30];
   //static char BufferText1[30];
     ILI9341_FillScreen(BLACK);

  while (1)
  {

     VMAX=0;
     VMIN=4096;
     //DrawRaster();
    /* USER CODE END WHILE */
     wave_freq_calculator();
     for(int i=0;i<X_MAX;i++)//FIND MAX VOLTAGE VALUE
     {
        if(adc_buffer[i]>VMAX)
        {
           VMAX=adc_buffer[i];
        }
        if(adc_buffer[i]<VMIN)
        {
           VMIN=adc_buffer[i];
        }
     }


     PVMAX=(3.3/4096*VMAX*212.9-313.3)/22.1;
           sprintf(Buffer, "VMAX %.2f  ", PVMAX);
           ILI9341_DrawText(Buffer, FONT2, 245, 90, WHITE, BLACK);
     PVMIN=(3.3/4096*VMIN*212.9-313.3)/22.1;
     sprintf(Buffer, "VMIN %.2f   ", PVMIN);
           ILI9341_DrawText(Buffer, FONT2, 245, 110, WHITE, BLACK);
     p2p=PVMAX-PVMIN;
     sprintf(Buffer, "P2P %.2f  ", p2p);
    ILI9341_DrawText(Buffer, FONT2, 245, 130,  YELLOW , BLACK);
     if(conversion_ready==1)
     {      trigger_level = CHTRIG;
        DrawRaster();
        switch(VRANGE){
               case 0:
                      ILI9341_DrawText("0.5V", FONT3, 250, 70, WHITE, BLACK);
                      for(i=0;i<buffersize;i++)
                         {
                            trace_1_buffer[i]=adc_buffer[i];
                            temp1=trace_1_buffer[i];
                            trace_1_buffer[i]=abs(temp1*0.565-914-240);//0.5v
                         }
                      break;

               case 1:
                       ILI9341_DrawText("1V   ", FONT3, 250, 70, WHITE, BLACK);
                      for(i=0;i<buffersize;i++)
                          {
                              trace_1_buffer[i]=adc_buffer[i];
                              temp1=trace_1_buffer[i];
                              trace_1_buffer[i]=abs(temp1 * 0.3 - 431-240);// 1v
                          }
                       break;

               case 2:
                      ILI9341_DrawText("2V   ", FONT3, 250, 70, WHITE, BLACK);
                       for(i=0;i<buffersize;i++)
                           {
                              trace_1_buffer[i]=adc_buffer[i];
                              temp1=trace_1_buffer[i];
                             trace_1_buffer[i]=abs(temp1 * 0.163 - 180-240);//2v
                           }
                       break;
               case 3:
                       ILI9341_DrawText("4V    ", FONT3, 250, 70, WHITE, BLACK);
                       for(i=0;i<buffersize;i++)
                           {
                              trace_1_buffer[i]=adc_buffer[i];
                              temp1=trace_1_buffer[i];
                              trace_1_buffer[i]=abs(temp1 * 0.065-240); //4V   if 0v 0.065
                           }
                       break;
           }
        switch(HRANGE){
                         case 0:
                                ILI9341_DrawText("200ms", FONT3, 250, 50, WHITE, BLACK);
                               LL_TIM_SetPrescaler(TIM3, 99);
                               LL_TIM_SetAutoReload(TIM3, 7499);// 200ms/div
                               sec=0.2;
                                break;

                         case 1:
                                ILI9341_DrawText("100ms", FONT3, 250, 50, WHITE, BLACK);
                               LL_TIM_SetPrescaler(TIM3, 99);
                               LL_TIM_SetAutoReload(TIM3, 3749);                        // 100ms/div
                                 sec=0.1;
                               break;

                         case 2:
                                ILI9341_DrawText("50ms", FONT3, 250, 50, WHITE, BLACK);
                                LL_TIM_SetPrescaler(TIM3, 99);
                                LL_TIM_SetAutoReload(TIM3, 1874);
                                sec=0.05;// 50ms/div
                                break;
                         case 3:
                                  LL_TIM_SetPrescaler(TIM3, 99);
                               LL_TIM_SetAutoReload(TIM3, 749);                        // 20ms/div
                               ILI9341_DrawText("20ms", FONT3, 250, 50, WHITE, BLACK);
                               sec=0.02;
                                  break;
                        case 4:
                              ILI9341_DrawText("10ms", FONT3, 250, 50, WHITE, BLACK);
                              LL_TIM_SetPrescaler(TIM3, 0);
                              LL_TIM_SetAutoReload(TIM3, 37499);// 10ms/div
                              sec=0.01;
                              break;
                        case 5:
                              ILI9341_DrawText("5ms", FONT3, 250, 50, WHITE, BLACK);
                              LL_TIM_SetPrescaler(TIM3, 0);
                              LL_TIM_SetAutoReload(TIM3, 18749);
                              sec=0.005;// 5ms/div
                              break;
                        case 6:
                              LL_TIM_SetPrescaler(TIM3, 0);
                              LL_TIM_SetAutoReload(TIM3, 7499);                        // 2ms/div
                              ILI9341_DrawText("2ms", FONT3, 250, 50, WHITE, BLACK);
                              sec=0.002;
                              break;
                        case 7:
                              ILI9341_DrawText("1ms", FONT3, 250, 50, WHITE, BLACK);
                              LL_TIM_SetPrescaler(TIM3, 0);
                              LL_TIM_SetAutoReload(TIM3, 3749);
                              sec=0.001;// 1ms/div
                              break;
                        case 8:
                              ILI9341_DrawText("500us", FONT3, 250, 50, WHITE, BLACK);
                              LL_TIM_SetPrescaler(TIM3, 0);
                              LL_TIM_SetAutoReload(TIM3, 1874);
                              sec=0.0005;// 500us/div
                                break;
                        case 9:
                              ILI9341_DrawText("200us", FONT3, 250, 50, WHITE, BLACK);
                              LL_TIM_SetPrescaler(TIM3, 0);
                              LL_TIM_SetAutoReload(TIM3, 749);
                              sec=0.0002;// 200us/div
                              break;
                        case 10:
                              ILI9341_DrawText("100us", FONT3, 250, 50, WHITE, BLACK);
                              LL_TIM_SetPrescaler(TIM3, 0);
                              LL_TIM_SetAutoReload(TIM3, 374);
                              sec=0.0001;// 100us/div
                              break;
                        case 11:
                              ILI9341_DrawText("50us", FONT3, 250, 50, WHITE, BLACK);
                              LL_TIM_SetPrescaler(TIM3, 0);
                              LL_TIM_SetAutoReload(TIM3, 187);
                              sec=0.00005;// 50us/div
                              break;
                        case 12:
                              ILI9341_DrawText("20us", FONT3, 250, 50, WHITE, BLACK);
                              LL_TIM_SetPrescaler(TIM3, 0);
                              LL_TIM_SetAutoReload(TIM3, 74);         //74               // 20us/div
                              sec=0.00004;
                              break;
                     }
        i = 1;                                                //find trigger_point
         if(trigger_ch==0)
        {
           while ((trace_1_buffer[i - 1] < trigger_level) || (trace_1_buffer[i] >= trigger_level)) i++;
        }
        if (i > 80) trigger_point = 1;                           //if the triggerpoint isn't found
                 else trigger_point = i;
        //DrawRaster();
        ILI9341_DrawHLine(1,previous_trigger_level,10,BLACK);
        ILI9341_DrawHLine(1,trigger_level,10,RED);

        previous_trigger_level = trigger_level;



        for (i = 1; i < X_MAX; i++)
                 {
                    j = i + 1;

                    x1 = i + previous_trigger_point;
                    x2 = x1 + 1;
                    x3 = i + trigger_point;
                    x4 = x3 + 1;
                    y1 = erase_1_buffer[x1];
                    y2 = erase_1_buffer[x2];
                    y3 = trace_1_buffer[x3];
                    y4 = trace_1_buffer[x4];
                    //ILI9341_DrawPixel(i,y3,GREEN);
                    //ILI9341_DrawPixel(i,y1,BLACK);

                    ILI9341_Line(i, y1, j, y2, BLACK);                  //remove old trace


                    ILI9341_Line(i, y3, j, y4, GREEN );                  //show new trace


                 }
        for (i = 0; i < buffersize; i++)                           //store values, used next time to erase the old trace
                 {
                    erase_1_buffer[i] = trace_1_buffer[i];
                 }
        previous_trigger_point = trigger_point;

                  conversion_ready=0;
                  LL_DMA_ClearFlag_TE1(DMA1);                                 //just in case...
              LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);                  //disable DMA so that it can be
              LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, buffersize);         //set to the number of transfers
              LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);                  //re-enable DMA
		LL_TIM_EnableCounter(TIM3);
     }


     if (trigger_ch == 0)  ILI9341_DrawText("T_CH1", FONT3, 250, 20, MAGENTA , BLACK);
           else                      ILI9341_DrawText("T_CH2", FONT3, 250, 20, MAGENTA , BLACK);




    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */

// External Interrupt ISR Handler CallBackFun
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

   if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET){
         VRANGE++;
         VRANGE %= 4;
         LL_mDelay(10);
        }


   if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_RESET){
          VRANGE--;
          VRANGE %= 4;
          if(VRANGE<0)
                         {
                            VRANGE=0;
                         }
          LL_mDelay(10);
         }


   if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_RESET){
               HRANGE ++;
               HRANGE %= 13;
               LL_mDelay(10);
            }
   if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET){
                  HRANGE --;
                  if(HRANGE<0)
                  {
                     HRANGE=0;
                  }
                  HRANGE %= 13;
                  LL_mDelay(10);
               }
   if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET){
      CHTRIG=CHTRIG+20;
                  if(CHTRIG>239)
                  {
                   CHTRIG=239;
                  }
                  LL_mDelay(10);
               }
   if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET){
      CHTRIG=CHTRIG-20;
      if(CHTRIG<1)
      {
         CHTRIG=1;
      }
                     LL_mDelay(10);
                  }

}
/**
  * @brief System Clock Configuration
  * @retval None
  */

void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(72000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_6);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration
  PA0-WKUP   ------> ADC1_IN0
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ADC1 DMA Init */

  /* ADC1 Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_VERYHIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_WORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_WORD);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM3_TRGO;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
  NVIC_EnableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 749;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_UPDATE);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  /*Configure GPIO pin : PB15 */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  /*Configure GPIO pins : PA8 PA9 PA10 PA11
                           PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   /* EXTI interrupt init*/
   HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);


   HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void DrawRaster(void){

   ILI9341_DrawVLine(0,0,240,LIGHTGREY);
   ILI9341_DrawVLine(40,0,240,LIGHTGREY);
   ILI9341_DrawVLine(80,0,240,LIGHTGREY);
   ILI9341_DrawVLine(120,0,240,LIGHTGREY);
   ILI9341_DrawVLine(160,0,240,LIGHTGREY);
   ILI9341_DrawVLine(200,0,240,LIGHTGREY);
   ILI9341_DrawVLine(240,0,240,LIGHTGREY);


   ILI9341_DrawHLine(0,0,240,LIGHTGREY);
   ILI9341_DrawHLine(0,40,240,LIGHTGREY);
   ILI9341_DrawHLine(0,80,240,LIGHTGREY);
   ILI9341_DrawHLine(0,120,240,LIGHTGREY);
   ILI9341_DrawHLine(0,160,240,LIGHTGREY);
   ILI9341_DrawHLine(0,200,240,LIGHTGREY);
   ILI9341_DrawHLine(0,239,240,LIGHTGREY);


}
void wave_freq_calculator(void)
{   const char buffer1[30];
   float freq=0;
   int first=0;
   int second=0;
   int center;
   int vmax1=0;
   int vmin1=4096;
   int firstd=0;
   int secondd=0;
   int gap=0;
   for(int i=0;i<buffersize;i++)
   {
      if(adc_buffer[i]>vmax1) vmax1=adc_buffer[i];

      if(adc_buffer[i]<vmin1) vmin1=adc_buffer[i];
   }
   center=(vmax1+vmin1)/2;


   for(int i=0;i<buffersize;i++)
   {
      if(firstd==0)
      {
    	  if(adc_buffer[i-1]<center && adc_buffer[i+1]>center)
    	  {
    		  first=i;
    		  firstd=1;
		  }
      }
      else if(firstd==1 && secondd ==0)
		  {
				  if(adc_buffer[i-1]>center && adc_buffer[i+1]<center)
				  {
								  second=i;
								  secondd=1;
				  }

		  }
      else break;

   }

   gap=second-first;

     freq=10.0/(gap*sec*1.04);
     ILI9341_DrawText("freq   ", FONT2, 250, 150, BLUE, BLACK);
     sprintf(buffer1,"%.2f",freq);
      ILI9341_DrawText(buffer1, FONT2, 250, 170, BLUE, BLACK);


}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
