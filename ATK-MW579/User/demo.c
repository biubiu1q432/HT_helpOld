/**
 ****************************************************************************************************
 * @file        demo.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MW579ģ����豸͸������ʵ��
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
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

#include "demo.h"
#include "./BSP/ATK_MW579/atk_mw579.h"
#include "./BSP/LED/led.h"
#include "./BSP/KEY/key.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"

#define DEMO_BLE_NAME           "ATK-MW579"        /* �������� */
#define DEMO_BLE_HELLO          "HELLO ATK-MW579"  /* ������ӭ�� */
#define DEMO_BLE_ADPTIM         5                   /* �㲥�ٶ� */

/**
 * @brief       ������ʾ��ں���
 * @param       ��
 * @retval      ��
 */
void demo_run(void)
{
    uint8_t ret;
    uint8_t key;
    uint8_t *recv_dat;
    
		/* ATK-MW579��ʼ�� */
    ret = atk_mw579_init(ATK_MW579_UART_BAUDRATE_115200);
    if (ret != 0)
    {
        printf("ATK-MW579 init failed!\r\n");
        while (1)
        {
            LED0_TOGGLE();
            delay_ms(200);
        }
    }
    
    /* ����ATK-MW579 */
    atk_mw579_enter_config_mode();
    ret  = atk_mw579_set_name(DEMO_BLE_NAME);
    ret += atk_mw579_set_hello(DEMO_BLE_HELLO);
    ret += atk_mw579_set_tpl(ATK_MW579_TPL_P0DBM);
    ret += atk_mw579_set_uart(ATK_MW579_UART_BAUDRATE_115200, ATK_MW579_UART_DATA_8, ATK_MW579_UART_PARI_NONE, ATK_MW579_UART_STOP_1);
    ret += atk_mw579_set_adptim(DEMO_BLE_ADPTIM);
    ret += atk_mw579_set_linkpassen(ATK_MW579_LINKPASSEN_OFF);
    ret += atk_mw579_set_leden(ATK_MW579_LEDEN_ON);
    ret += atk_mw579_set_slavesleepen(ATK_MW579_SLAVESLEEPEN_OFF);
    ret += atk_mw579_set_maxput(ATK_MW579_MAXPUT_OFF);
    ret += atk_mw579_set_mode(ATK_MW579_MODE_S);
    if (ret != 0)
    {
        printf("ATK-MW579 config failed!\r\n");
        while (1)
        {
            LED0_TOGGLE();
            delay_ms(200);
        }
    }
    
    /* ���¿�ʼ�������� */
    atk_mw579_uart_rx_restart();
    
    
		//Ӧ�ò�
		while (1)
    {
        /* �ж��Ƿ����������豸 */
        if (atk_mw579_get_conn_sta() == ATK_MW579_CONNECTED)
        {
            key = key_scan(0);
            
            if (key == KEY0_PRES)
            {
                /* ͸���������������豸 */
                atk_mw579_uart_printf("From ATK-MW579\r\n");
            }
            
            /* ͸�������������豸������ */
            recv_dat = atk_mw579_uart_rx_get_frame();
            if (recv_dat != NULL)
            {
                printf("%s", recv_dat);
                atk_mw579_uart_rx_restart();
            }
            
            delay_ms(10);
        }
    }
}
