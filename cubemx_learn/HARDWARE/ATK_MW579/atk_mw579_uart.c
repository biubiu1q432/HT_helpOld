/**
 ****************************************************************************************************
 * @file        atk_mw579_uart.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MW579ģ��UART�ӿ���������
 * @license     Copyright (c) 2020-2032, �������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� STM32F103������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "atk_mw579_uart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart4; /* ATK-MW579 UART */
 
struct rx_frame g_uart_rx_frame ={0};              

static uint8_t g_uart_tx_buf[ATK_MW579_UART_TX_BUF_SIZE];   /* ATK-MW579 UART���ͻ��� */

/**
 * @brief       ATK-MW579 UART printf
 * @param       fmt: ����ӡ������
 * @retval      ��
 */
void atk_mw579_uart_printf(char *fmt, ...)
{
    va_list ap;
    uint16_t len;
    
    va_start(ap, fmt);
    vsprintf((char *)g_uart_tx_buf, fmt, ap);
    va_end(ap);
    
    len = strlen((const char *)g_uart_tx_buf);
    HAL_UART_Transmit(&huart4, g_uart_tx_buf, len,HAL_MAX_DELAY);
}

/**	
 * @brief       ATK-MW579 UART���¿�ʼ��������
 * @param       ��
 * @retval      ��
 */
void atk_mw579_uart_rx_restart(void)
{
    g_uart_rx_frame.sta.len     = 0;
    g_uart_rx_frame.sta.finsh   = 0;
}

/**
 * @brief       ��ȡATK-MW579 UART���յ���һ֡����
 * @param       ��
 * @retval      NULL: δ���յ�һ֡����
 *              ����: ���յ���һ֡����
 */
uint8_t *atk_mw579_uart_rx_get_frame(void)
{
    if (g_uart_rx_frame.sta.finsh == 1)
    {
        g_uart_rx_frame.buf[g_uart_rx_frame.sta.len] = '\0';
        return g_uart_rx_frame.buf;
    }
    else
    {
       
			return NULL;
    }
}

/**
 * @brief       ��ȡATK-MW579 UART���յ���һ֡���ݵĳ���
 * @param       ��
 * @retval      0   : δ���յ�һ֡����
 *              ����: ���յ���һ֡���ݵĳ���
 */
uint16_t atk_mw579_uart_rx_get_frame_len(void)
{
    if (g_uart_rx_frame.sta.finsh == 1)
    {
        return g_uart_rx_frame.sta.len;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief       ATK-MW579 UART��ʼ��
 * @param       baudrate: UARTͨѶ������
 * @retval      ��
 */
//void atk_mw579_uart_init(uint32_t baudrate)
//{
//    g_uart_handle.Instance          = ATK_MW579_UART_INTERFACE;     /* ATK-MW579 UART */
//    g_uart_handle.Init.BaudRate     = baudrate;                     /* ������ */
//    g_uart_handle.Init.WordLength   = UART_WORDLENGTH_8B;           /* ����λ */
//    g_uart_handle.Init.StopBits     = UART_STOPBITS_1;              /* ֹͣλ */
//    g_uart_handle.Init.Parity       = UART_PARITY_NONE;             /* У��λ */
//    g_uart_handle.Init.Mode         = UART_MODE_TX_RX;              /* �շ�ģʽ */
//    g_uart_handle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;          /* ��Ӳ������ */
//    g_uart_handle.Init.OverSampling = UART_OVERSAMPLING_16;         /* ������ */
//    HAL_UART_Init(&g_uart_handle);                                  /* ʹ��ATK-MW579 UART
//                                                                     * HAL_UART_Init()����ú���HAL_UART_MspInit()
//                                                                     * �ú����������ļ�usart.c��
//                                                                     */
//}

///**
// * @brief       ATK-MW579 UART�жϻص�����,
// * @param       ��
// * @retval      ��
// */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    uint8_t tmp;
//    
//    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_ORE) != RESET)        /* UART���չ��ش����ж� */
//    {
//        __HAL_UART_CLEAR_OREFLAG(&huart4);                           /* ������չ��ش����жϱ�־ */
//        (void)huart4.Instance->SR;                                   /* �ȶ�SR�Ĵ������ٶ�DR�Ĵ��� */
//        (void)huart4.Instance->DR;
//    }
//    
//    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_RXNE) != RESET)       /* UART�����ж� ��	�յ�1���ֽڵ����� �� */
//    {
//        HAL_UART_Receive(&huart4, &tmp, 1, HAL_MAX_DELAY);           /* UART�������� */
//        
//        if (g_uart_rx_frame.sta.len < (ATK_MW579_UART_RX_BUF_SIZE - 1))     /* �ж�UART���ջ����Ƿ����
//																																						 * ����һλ��������'\0'
//                                                                             */
//        {
//            g_uart_rx_frame.buf[g_uart_rx_frame.sta.len] = tmp;             /* �����յ�������д�뻺�� */
//            g_uart_rx_frame.sta.len++;                                      /* ���½��յ������ݳ��� */
//        }
//        else                                                                /* UART���ջ������ */
//        {
//            g_uart_rx_frame.sta.len = 0;                                    /* ����֮ǰ�յ������� */
//            g_uart_rx_frame.buf[g_uart_rx_frame.sta.len] = tmp;             /* �����յ�������д�뻺�� */
//            g_uart_rx_frame.sta.len++;                                      /* ���½��յ������ݳ��� */
//        }
//    }
//    
//    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE) != RESET)       /* UART���߿����ж�(	һ��ʱ����û�н��յ�����	) */
//    {
//        g_uart_rx_frame.sta.finsh = 1;                                      /* ���֡������� */
//        
//        __HAL_UART_CLEAR_IDLEFLAG(&huart4);                          /* ���UART���߿����ж� */
//    }
//}