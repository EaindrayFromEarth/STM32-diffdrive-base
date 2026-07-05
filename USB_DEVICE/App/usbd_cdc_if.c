/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : usbd_cdc_if.c
 * @version        : v1.0_Cube
 * @brief          : Usb device for Virtual Com Port.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "motor_control.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "mpu6050.h"
#define RX_BUF_SIZE 64
static uint8_t rx_buf[RX_BUF_SIZE];
static uint16_t rx_len = 0;
extern Motor motor1;
extern Motor motor2;
extern MPU6050_t imu1;

void send_encoder_values(void);
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
 * @brief Usb device library.
 * @{
 */

/** @addtogroup USBD_CDC_IF
 * @{
 */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
 * @brief Private types.
 * @{
 */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
 * @brief Private defines.
 * @{
 */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
 * @brief Private macros.
 * @{
 */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
 * @brief Private variables.
 * @{
 */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */
volatile uint8_t enc_request = 0;
volatile uint8_t i2c_scan_request = 0;
/* make these non-static so main.c can reset them too */
volatile int32_t enc_zero_m1 = 0;
volatile int32_t enc_zero_m2 = 0;

/* NEW: reset flag + token */
volatile uint8_t reset_request = 0;
volatile uint32_t reset_token = 0;

volatile uint8_t dbg_stream = 0;

/* USER CODE END PRIVATE_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
 * @brief Public variables.
 * @{
 */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
 * @brief Private functions declaration.
 * @{
 */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
 * @}
 */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
    {
        CDC_Init_FS,
        CDC_DeInit_FS,
        CDC_Control_FS,
        CDC_Receive_FS,
        CDC_TransmitCplt_FS};

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Initializes the CDC media low layer over the FS USB IP
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
 * @brief  DeInitializes the CDC media low layer
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
 * @brief  Manage the CDC class requests
 * @param  cmd: Command code
 * @param  pbuf: Buffer containing command data (request parameters)
 * @param  length: Number of data to be sent (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

  case CDC_SET_COMM_FEATURE:

    break;

  case CDC_GET_COMM_FEATURE:

    break;

  case CDC_CLEAR_COMM_FEATURE:

    break;

    /*******************************************************************************/
    /* Line Coding Structure                                                       */
    /*-----------------------------------------------------------------------------*/
    /* Offset | Field       | Size | Value  | Description                          */
    /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
    /* 4      | bCharFormat |   1  | Number | Stop bits                            */
    /*                                        0 - 1 Stop bit                       */
    /*                                        1 - 1.5 Stop bits                    */
    /*                                        2 - 2 Stop bits                      */
    /* 5      | bParityType |  1   | Number | Parity                               */
    /*                                        0 - None                             */
    /*                                        1 - Odd                              */
    /*                                        2 - Even                             */
    /*                                        3 - Mark                             */
    /*                                        4 - Space                            */
    /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
    /*******************************************************************************/
  case CDC_SET_LINE_CODING:

    break;

  case CDC_GET_LINE_CODING:

    break;

  case CDC_SET_CONTROL_LINE_STATE:

    break;

  case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
 * @brief  Data received over USB OUT endpoint are sent over CDC interface
 *         through this function.
 *
 *         @note
 *         This function will issue a NAK packet on any OUT packet received on
 *         USB endpoint until exiting this function. If you exit this function
 *         before transfer is complete on CDC interface (ie. using DMA controller)
 *         it will result in receiving more data while previous ones are still
 *         not sent.
 *
 * @param  Buf: Buffer of data to be received
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */

  for (uint32_t i = 0; i < *Len; i++)
  {
    uint8_t c = Buf[i];

    if (c == '\r' || c == '\n' || rx_len >= RX_BUF_SIZE - 1)
    {
      rx_buf[rx_len] = '\0';

      if (rx_len > 0 && rx_buf[rx_len - 1] == '\r')
      {
        rx_buf[--rx_len] = '\0';
      }

      if (strcmp((char *)rx_buf, "z") == 0 || strcmp((char *)rx_buf, "Z") == 0)
      {
        reset_request = 1;
      }

      else if (strcmp((char *)rx_buf, "e") == 0 || strcmp((char *)rx_buf, "E") == 0)
      {
        enc_request = 1;
      }
      else if (strncmp((char *)rx_buf, "d ", 2) == 0)
      {
        int on = 0;
        if (sscanf((char *)rx_buf + 2, "%d", &on) == 1)
        {
          dbg_stream = (on != 0);
          CDC_Printf("DBG %s\r\n", dbg_stream ? "ON" : "OFF");
        }
      }
      else if (Buf[0] == 'i')
      {
        i2c_scan_request = 1;
      }
      else if (strncmp((char *)rx_buf, "m ", 2) == 0)
      {
        float left_vel, right_vel;
        if (sscanf((char *)rx_buf + 2, "%f %f", &left_vel, &right_vel) == 2)
        {

          set_motor1_velocity(left_vel);
          set_motor2_velocity(right_vel);
        }
        else
        {
        }
      }
      else
      {
        char motor_id;
        float velocity;
        if (sscanf((char *)rx_buf, "M%c %f", &motor_id, &velocity) == 2)
        {

          if (motor_id == '1')
            set_motor1_velocity(velocity);
          else if (motor_id == '2')
            set_motor2_velocity(velocity);
        }
      }

      rx_len = 0;
    }
    else
    {
      rx_buf[rx_len++] = c;
    }
  }

  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return USBD_OK;

  /* USER CODE END 6 */
}

/**
 * @brief  CDC_Transmit_FS
 *         Data to send over USB IN endpoint are sent over CDC interface
 *         through this function.
 *         @note
 *
 *
 * @param  Buf: Buffer of data to be sent
 * @param  Len: Number of data to be sent (in bytes)
 * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
 */
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0)
  {
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/**
 * @brief  CDC_TransmitCplt_FS
 *         Data transmitted callback
 *
 *         @note
 *         This function is IN transfer complete callback used to inform user that
 *         the submitted Data is successfully sent over USB.
 *
 * @param  Buf: Buffer of data to be received
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

void send_encoder_values(void)
{
  read_encoder(&motor1);
  read_encoder(&motor2);

  char msg[160];
  int n = snprintf(msg, sizeof(msg),
                   "ENC %ld %ld %.6f %.6f\r\n",
                   (long)(motor1.encoder_count - enc_zero_m1),
                   (long)(motor2.encoder_count - enc_zero_m2),
                   imu1.gz_rad_s,
                   imu1.yaw_rad);

  if (n <= 0)
    return;

  uint8_t st = CDC_Write((const uint8_t *)msg, (uint16_t)n);
  if (st != USBD_OK)
    enc_request = 1;
}

#include <stdarg.h>

uint8_t CDC_Write(const uint8_t *data, uint16_t len)
{
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    return USBD_FAIL;

  USBD_CDC_HandleTypeDef *hcdc =
      (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if (hcdc == NULL)
    return USBD_FAIL;

  if (len > APP_TX_DATA_SIZE)
    len = APP_TX_DATA_SIZE;
  if (hcdc->TxState != 0)
    return USBD_BUSY;

  memcpy(UserTxBufferFS, data, len);
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, len);
  return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

int CDC_Printf(const char *fmt, ...)
{
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    return (int)USBD_FAIL;

  static char line[128];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);

  if (n < 0)
    return (int)USBD_FAIL;
  if (n > (int)sizeof(line))
    n = (int)sizeof(line);

  uint8_t st = CDC_Write((const uint8_t *)line, (uint16_t)n);
  return (st == USBD_OK) ? n : (int)st;
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
 * @}
 */

/**
 * @}
 */
