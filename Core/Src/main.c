/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ESC_ID      1                   /* 电调 C620 拨码 */
#define REDUCTION   19                  /* M3508 减速比 */
#define RX_ID       (0x200u + ESC_ID)   /* = 0x201 */
#define TX_ID_1_4   0x200u              /* ID 1~4 用 0x200 */

/* 控制模式 */
#define CTRL_OPEN   0           /* 开环 */
#define CTRL_SPEED  1           /* 速度环 */
#define CTRL_ANGLE  2           /* 角度环 */

/* 运行参数 */
#define CTRL_DT_MS        1U       /* 控制周期 1ms */
#define RX_TIMEOUT_MS     50U      /* 反馈超时 */
#define ANGLE_MAX_SPEED   400.0f   /* 角度环最大输出转速 */
#define OVERSPEED_RPM     3000.0f  /* 超速保护 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN PV */
volatile uint32_t g_tx_ok   = 0;
volatile uint32_t g_tx_fail = 0;
volatile uint32_t g_bo_cnt  = 0;
volatile uint32_t g_rx_cnt  = 0;

volatile uint8_t  g_fault      = 0;        /* 1 = 触发保护 */
volatile uint8_t  g_ctrl_mode  = CTRL_SPEED;
volatile float    g_target_rpm = 100.0f;   /* 速度环目标，转子 rpm */
volatile float    g_target_deg = 0.0f;     /* 角度环目标，输出轴度 */
volatile int16_t  g_out        = 0;        /* 最终电流指令 */

pid_type spd_pid;
pid_type pos_pid;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 电机反馈 */
typedef struct
{
    uint16_t angle;             /* 角度 0~8191 */
    int16_t  rpm;               /* 转子转速 */
    int16_t  cur;               /* 实际电流 */
    uint8_t  temp;              /* 温度 */
    int32_t  total;             /* 转子计数 */
    volatile uint32_t stamp;    /* 最近一帧的时刻，用于掉线判断 */
    uint32_t frames;            /* 累计帧数 */
} ESC_t;

static ESC_t esc;

/* 电调发送 */
static void C620_Send(int16_t cmd)
{
    uint8_t d[8] = {0};
    uint8_t i = (uint8_t)((ESC_ID - 1) * 2);   /* ID 各占 2 字节 */

    d[i]     = (uint8_t)((uint16_t)cmd >> 8);  /* 大端 */
    d[i + 1] = (uint8_t)((uint16_t)cmd & 0xFF);

    FDCAN_TxHeaderTypeDef h = {0};
    h.Identifier  = TX_ID_1_4;
    h.IdType      = FDCAN_STANDARD_ID;
    h.TxFrameType = FDCAN_DATA_FRAME;
    h.DataLength  = FDCAN_DLC_BYTES_8;
    h.FDFormat    = FDCAN_CLASSIC_CAN;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &h, d) == HAL_OK) g_tx_ok++;
    else                                                         g_tx_fail++;
}

/* bus-off 恢复 */
static void FDCAN_BusOffCheck(void)
{
    static uint32_t last = 0;

    if ((FDCAN1->PSR & FDCAN_PSR_BO) == 0U) return;
    if ((HAL_GetTick() - last) < 100U) return;      /* 恢复太频繁会一直无法退出 */

    last = HAL_GetTick();
    g_bo_cnt++;
    HAL_FDCAN_Stop(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan1);
}

/* 1kHz 控制 */
static void Control_1kHz(void)
{
    /* 反馈超时：闭环时电机掉线立即停止积分；开环跳过 */
    if (g_ctrl_mode != CTRL_OPEN &&
        (HAL_GetTick() - esc.stamp) > RX_TIMEOUT_MS)
    {
        g_out = 0;
        PID_clear(&spd_pid);
        PID_clear(&pos_pid);
        return;
    }

    /* 超速保护：防止极性接反导致发散 */
    if (esc.rpm > OVERSPEED_RPM || esc.rpm < -OVERSPEED_RPM)
    {
        g_fault = 1;
        g_out = 0;
        PID_clear(&spd_pid);
        PID_clear(&pos_pid);
        return;
    }

    float rpm_now = (float)esc.rpm;                                     /* 转子 rpm */
    float ang_now = (float)esc.total * 360.0f / (8192.0f * REDUCTION);  /* 输出轴 degree */

    if (g_ctrl_mode == CTRL_OPEN)
    {
        g_out = 1000;        /* 开环调试，约 1.2A */
    }
    else if (g_ctrl_mode == CTRL_SPEED)
    {
        g_out = (int16_t)Pid_cal(&spd_pid, g_target_rpm, rpm_now);
    }
    else
    {
        float tgt_rpm = Pid_cal(&pos_pid, g_target_deg, ang_now);
        if (tgt_rpm >  ANGLE_MAX_SPEED) tgt_rpm =  ANGLE_MAX_SPEED;
        if (tgt_rpm < -ANGLE_MAX_SPEED) tgt_rpm = -ANGLE_MAX_SPEED;
        g_out = (int16_t)Pid_cal(&spd_pid, tgt_rpm, rpm_now);
    }
}

/* 接收回调 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx;
    uint8_t d[8];

    (void)RxFifo0ITs;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, d) != HAL_OK) return;
    if (rx.Identifier != RX_ID) return;          /* 只认自己的电调 */

    g_rx_cnt++;

    uint16_t a = (uint16_t)((d[0] << 8) | d[1]);

    /* 判圈：与上一帧做差分，不足半圈（4096 计数）永远判对。1 kHz 下约合 30000 rpm，远高于 M3508 上限 */
    if (esc.frames == 0U)
    {
        esc.total = (int32_t)a;                    /* 首帧 */
    }
    else
    {
        int32_t delta = (int32_t)a - (int32_t)esc.angle;
        if (delta >  4096) delta -= 8192;          /* 反向跨过 0 */
        if (delta < -4096) delta += 8192;          /* 正向跨过 0 */
        esc.total += delta;
    }
    esc.angle = a;
    esc.frames++;

    esc.rpm   = (int16_t)((d[2] << 8) | d[3]);
    esc.cur   = (int16_t)((d[4] << 8) | d[5]);
    esc.temp  = d[6];
    esc.stamp = HAL_GetTick();
}

/* CubeMX NVIC 中断入口。CubeMX 勾选 FDCAN1 interrupt 0 可替代 */
void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  /* 滤波器 0：只放行自己的反馈 ID */
  FDCAN_FilterTypeDef fib = {0};
  fib.IdType       = FDCAN_STANDARD_ID;
  fib.FilterIndex  = 0;
  fib.FilterType   = FDCAN_FILTER_MASK;
  fib.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  fib.FilterID1    = RX_ID;
  fib.FilterID2    = 0x7FC;              /* 掩码：0x201~0x204 全部放行 */
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &fib) != HAL_OK) Error_Handler();

  /* 滤波器 1：StdFiltersNbr = 2 分配了两个元素 */
  FDCAN_FilterTypeDef fib_off = {0};
  fib_off.IdType       = FDCAN_STANDARD_ID;
  fib_off.FilterIndex  = 1;
  fib_off.FilterType   = FDCAN_FILTER_MASK;
  fib_off.FilterConfig = FDCAN_FILTER_DISABLE;
  fib_off.FilterID1    = 0x000;
  fib_off.FilterID2    = 0x7FF;
  (void)HAL_FDCAN_ConfigFilter(&hfdcan1, &fib_off);

  /* 丢弃不匹配的帧 */
  (void)HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) Error_Handler();
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
      Error_Handler();
  }

  /* FIFO0 有新报文时进入 FDCAN1_IT0_IRQHandler */
  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

  /* PID 初始化：先给 6000（约 7A），跑通后加到 16000 */
  const float spd_para[3] = {15.0f, 0.3f, 0.0f};
  const float pos_para[3] = {3.0f,  0.0f, 0.0f};
  PID_Parameter_Init(&spd_pid, PID_POSITION, spd_para, 6000.0f, 4000.0f, 0.0f);
  PID_Parameter_Init(&pos_pid, PID_POSITION, pos_para, ANGLE_MAX_SPEED, 0.0f, 0.0f);

  HAL_Delay(1000);     /* 等电调自检，避免上电瞬间 bus-off */
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t next = 0;
    if (next == 0U) next = HAL_GetTick();

    if ((int32_t)(HAL_GetTick() - next) >= 0)
    {
        /* 调试暂停后重新对齐 */
        if ((int32_t)(HAL_GetTick() - next) > 10) next = HAL_GetTick();
        next += CTRL_DT_MS;

        Control_1kHz();
        C620_Send(g_out);
    }

    FDCAN_BusOffCheck();
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 32;
  RCC_OscInitStruct.PLL.PLLN = 129;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 3;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 40;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 3;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 6;
  hfdcan1.Init.DataTimeSeg2 = 2;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 2;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 8;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 8;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
