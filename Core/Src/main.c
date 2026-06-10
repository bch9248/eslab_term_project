/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "arm_math.h"
#include "math.h"

#include "network.h"
#include "network_data.h"
#include "scaler_data.h"
/* USER CODE END Includes */

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define AUDIO_REC 1024

#define FFT_SIZE 512
#define FFT_BINS (FFT_SIZE / 2)
#define FS 16000

#define FRAME_QUEUE_LEN 16
#define MODEL_INPUT_SIZE (FFT_BINS * FRAME_QUEUE_LEN)

int32_t RecBuf[AUDIO_REC];
int32_t PlayBuf[AUDIO_REC];

uint8_t DmaRecHalfBuffCplt = 0;
uint8_t DmaRecBuffCplt = 0;

arm_rfft_fast_instance_f32 fft;

float32_t fft_input[FFT_SIZE];
float32_t fft_output[FFT_SIZE];
float32_t fft_mag[FFT_BINS];

float32_t fft_queue[FRAME_QUEUE_LEN][FFT_BINS];
float32_t model_input[MODEL_INPUT_SIZE];

uint8_t fft_queue_index = 0;
uint8_t fft_queue_count = 0;
uint32_t frame_counter = 0;

uint32_t led_off_tick = 0;
uint8_t led_active = 0;

int32_t TestBuf[FFT_SIZE];

/* Cube.AI variables */
AI_ALIGNED(4) static ai_u8 ai_activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

AI_ALIGNED(4) static ai_float ai_input_data[AI_NETWORK_IN_1_SIZE];
AI_ALIGNED(4) static ai_float ai_output_data[AI_NETWORK_OUT_1_SIZE];

static ai_handle ai_network_handle = AI_HANDLE_NULL;
static ai_buffer *ai_input_buf = NULL;
static ai_buffer *ai_output_buf = NULL;
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

int _close(int file)
{
    return -1;
}

int _fstat(int file, void *st)
{
    return 0;
}

int _isatty(int file)
{
    return 1;
}

int _lseek(int file, int ptr, int dir)
{
    return 0;
}

int _read(int file, char *ptr, int len)
{
    return 0;
}

void AI_Init_Custom(void)
{
    ai_error err;

    const ai_handle activations[] = {
        ai_activations
    };

    err = ai_network_create_and_init(
        &ai_network_handle,
        activations,
        NULL
    );

    if (err.type != AI_ERROR_NONE)
    {
        printf("AI create/init failed: type=%d code=%d\r\n",
               err.type,
               err.code);
        Error_Handler();
    }

    ai_input_buf = ai_network_inputs_get(ai_network_handle, NULL);
    ai_output_buf = ai_network_outputs_get(ai_network_handle, NULL);

    if (ai_input_buf == NULL || ai_output_buf == NULL)
    {
        printf("AI buffer get failed\r\n");
        Error_Handler();
    }

    ai_input_buf[0].data = AI_HANDLE_PTR(ai_input_data);
    ai_output_buf[0].data = AI_HANDLE_PTR(ai_output_data);

    printf("AI init OK\r\n");
}

void PushFFTFrame(float32_t *mag)
{
    for (int i = 0; i < FFT_BINS; i++)
    {
        fft_queue[fft_queue_index][i] = mag[i];
    }

    fft_queue_index = (fft_queue_index + 1) % FRAME_QUEUE_LEN;

    if (fft_queue_count < FRAME_QUEUE_LEN)
    {
        fft_queue_count++;
    }

    frame_counter++;
}

void MakeModelInput(void)
{
    int out_idx = 0;
    int start = fft_queue_index;

    /*
     * Python CSV shape:
     *   256 rows x 16 columns
     *
     * Python reshape(-1) row-major order:
     *   row0_col0, row0_col1, ..., row0_col15,
     *   row1_col0, row1_col1, ...
     *
     * Here:
     *   row = FFT bin / frequency index k
     *   col = time frame index t
     */
    for (int k = 0; k < FFT_BINS; k++)
    {
        for (int t = 0; t < FRAME_QUEUE_LEN; t++)
        {
            int frame_idx = (start + t) % FRAME_QUEUE_LEN;
            model_input[out_idx++] = fft_queue[frame_idx][k];
        }
    }
}

void LED_All_Off(void)
{
    HAL_GPIO_WritePin(LED2_GPIO_Port,
                      LED2_Pin,
                      GPIO_PIN_RESET);

    HAL_GPIO_WritePin(LED3_WIFI__LED4_BLE_GPIO_Port,
                      LED3_WIFI__LED4_BLE_Pin,
                      GPIO_PIN_RESET);
}

void LED_ShowPrediction(int pred)
{
    /*
     * LED is locked while active.
     * During the 3-second indication window,
     * new predictions will NOT interrupt the current LED.
     */
    if (led_active)
    {
        return;
    }

    LED_All_Off();

    if (pred == 1)
    {
        HAL_GPIO_WritePin(LED2_GPIO_Port,
                          LED2_Pin,
                          GPIO_PIN_SET);

        led_off_tick = HAL_GetTick();
        led_active = 1;
    }
    else if (pred == 2)
    {
        HAL_GPIO_WritePin(LED3_WIFI__LED4_BLE_GPIO_Port,
                          LED3_WIFI__LED4_BLE_Pin,
                          GPIO_PIN_SET);

        led_off_tick = HAL_GetTick();
        led_active = 1;
    }
    else
    {
        /*
         * pred == 0:
         * keep all LEDs off.
         * No lock needed.
         */
        LED_All_Off();
        led_active = 0;
    }
}

void LED_Update(void)
{
    if (led_active)
    {
        if ((HAL_GetTick() - led_off_tick) >= 3000)
        {
            LED_All_Off();
            led_active = 0;
        }
    }
}

void RunClassifier(float32_t *input_4096)
{
    for (int i = 0; i < MODEL_INPUT_SIZE; i++)
    {
        float scale = scaler_scale[i];

        if (scale == 0.0f)
        {
            scale = 1.0f;
        }

        ai_input_data[i] =
            (input_4096[i] - scaler_mean[i]) / scale;
    }

    ai_i32 batch = ai_network_run(
        ai_network_handle,
        ai_input_buf,
        ai_output_buf
    );

    if (batch != 1)
    {
        ai_error err = ai_network_get_error(ai_network_handle);

        printf("AI run failed: type=%d code=%d\r\n",
               err.type,
               err.code);
        return;
    }

    int pred = 0;
    float max_prob = ai_output_data[0];

    for (int i = 1; i < AI_NETWORK_OUT_1_SIZE; i++)
    {
        if (ai_output_data[i] > max_prob)
        {
            max_prob = ai_output_data[i];
            pred = i;
        }
    }

    int prob_int = (int)(max_prob * 1000.0f);

    printf("Prediction=%d prob=%d.%03d outputs:",
           pred,
           prob_int / 1000,
           prob_int % 1000);

    for (int i = 0; i < AI_NETWORK_OUT_1_SIZE; i++)
    {
        int out_int = (int)(ai_output_data[i] * 1000.0f);

        printf(" [%d]=%d.%03d",
               i,
               out_int / 1000,
               out_int % 1000);
    }

    printf(" frame=%lu\r\n",
           (unsigned long)frame_counter);

    LED_ShowPrediction(pred);
}

void ProcessFFT(int32_t *input)
{
    float32_t mean = 0.0f;

    for (int i = 0; i < FFT_SIZE; i++)
    {
        mean += input[i];
    }

    mean /= FFT_SIZE;

    for (int i = 0; i < FFT_SIZE; i++)
    {
        fft_input[i] = (float32_t)input[i] - mean;
    }

    for (int i = 0; i < FFT_SIZE; i++)
    {
        float32_t w =
            0.5f - 0.5f * cosf(2.0f * 3.1415926f * i / (FFT_SIZE - 1));

        fft_input[i] *= w;
    }

    arm_rfft_fast_f32(&fft, fft_input, fft_output, 0);
    arm_cmplx_mag_f32(fft_output, fft_mag, FFT_BINS);

    PushFFTFrame(fft_mag);

    if (fft_queue_count == FRAME_QUEUE_LEN)
    {
        MakeModelInput();
        RunClassifier(model_input);
    }
    else
    {
        printf("Collecting FFT frames: %d/%d\r\n",
               fft_queue_count,
               FRAME_QUEUE_LEN);
    }
}
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  uint16_t i = 0;
  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DFSDM1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  printf("System start\r\n");

  LED_All_Off();

  /*
   * LED boot test:
   * LED2 on 0.5 sec, then LED3 on 0.5 sec.
   * If these do not light up, the issue is GPIO/LED polarity, not AI.
   */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(LED3_WIFI__LED4_BLE_GPIO_Port,
                    LED3_WIFI__LED4_BLE_Pin,
                    GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(LED3_WIFI__LED4_BLE_GPIO_Port,
                    LED3_WIFI__LED4_BLE_Pin,
                    GPIO_PIN_RESET);

  arm_rfft_fast_init_f32(&fft, FFT_SIZE);

  AI_Init_Custom();

  if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0,
                                       RecBuf,
                                       AUDIO_REC) != HAL_OK)
  {
      printf("DFSDM DMA start failed\r\n");
      Error_Handler();
  }

  printf("DFSDM DMA started\r\n");
  /* USER CODE END 2 */

  while (1)
  {
    LED_Update();

    if (DmaRecHalfBuffCplt == 1)
    {
        for (i = 0; i < FFT_SIZE; i++)
        {
            PlayBuf[i] = RecBuf[i] >> 8;
        }

        DmaRecHalfBuffCplt = 0;
        ProcessFFT(&PlayBuf[0]);
    }

    if (DmaRecBuffCplt == 1)
    {
        for (i = 0; i < FFT_SIZE; i++)
        {
            PlayBuf[i] = RecBuf[i + FFT_SIZE] >> 8;
        }

        DmaRecBuffCplt = 0;
        ProcessFFT(&PlayBuf[0]);
    }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_DFSDM1_Init(void)
{
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 250;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;

  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }

  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 40;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x00;

  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0,
                                       DFSDM_CHANNEL_2,
                                       DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_UARTEx_SetTxFifoThreshold(&huart1,
                                    UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_UARTEx_SetRxFifoThreshold(&huart1,
                                    UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(LED2_GPIO_Port,
                    LED2_Pin,
                    GPIO_PIN_RESET);

  HAL_GPIO_WritePin(LED3_WIFI__LED4_BLE_GPIO_Port,
                    LED3_WIFI__LED4_BLE_Pin,
                    GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_WIFI__LED4_BLE_GPIO_Port, &GPIO_InitStruct);
}

void HAL_DFSDM_FilterRegConvHalfCpltCallback(
    DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    DmaRecHalfBuffCplt = 1;
}

void HAL_DFSDM_FilterRegConvCpltCallback(
    DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    DmaRecBuffCplt = 1;
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
