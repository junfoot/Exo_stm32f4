/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "can.h"

/* USER CODE BEGIN 0 */

CAN_TxHeaderTypeDef g_canx_txheader;    /* ���Ͳ������ */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

uint8_t can_send_msg(uint32_t id, uint8_t len, uint8_t *msg)
{
    uint32_t TxMailbox = CAN_TX_MAILBOX0;
    g_canx_txheader.StdId = id;         /* ��׼��ʶ�� */
    g_canx_txheader.ExtId = id;         /* ��չ��ʶ��(29λ) */
    g_canx_txheader.IDE = CAN_ID_STD;   /* ʹ�ñ�׼֡ */
    g_canx_txheader.RTR = CAN_RTR_DATA; /* ����֡ */
    g_canx_txheader.DLC = len;

//    //�ҵ��յķ������� �����ݷ��ͳ�ȥ
//    check1 = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
//    while (check1 == 0);      // ��������������䶼�����˾͵ȴ�ֱ������ĳ���������
    
//    if ((hcan.Instance->TSR & CAN_TSR_TME0) == RESET) // ���MAILBOX0������
//    {
//        HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX0);
//        TxMailbox = CAN_TX_MAILBOX1;
//        check2 = 0;
//    } 
//    else if ((hcan.Instance->TSR & CAN_TSR_TME1) == RESET) // ���MAILBOX1������
//    {
//        HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX1);
//        TxMailbox = CAN_TX_MAILBOX0;
//        check2 = 1;
//    } 
    
    
    if (HAL_CAN_AddTxMessage(&hcan1, &g_canx_txheader, msg, &TxMailbox) != HAL_OK) /* ������Ϣ */
    {
        return 1;
    }
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3); /* �ȴ��������,��������Ϊ�� */
    
    return 0;
}

/* USER CODE END 1 */
