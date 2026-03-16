

/* ============================================================
 *  Wheel Driver CAN Protocol (per PDF)
 *   - TX to driver (host->driver):  ExtID 0x1801E600, 8 bytes, 100ms (or faster)
 *   - RX from driver (driver->host): ExtID 0x1801E001, 8 bytes, 100ms
 *
 *  Data0 bitfield:
 *    bit1..0: enable
 *      00: axis1&2 disabled
 *      01: axis2 enabled, axis1 disabled
 *      10: axis2 disabled, axis1 enabled
 *      11: axis1&2 enabled
 *    bit2: reset (optional) 0 disable, 1 reset enabled
 *    bit3: skid/slide (optional) 0 disable, 1 enable
 *    bit5..4: spare
 *    bit6: axis2 mode 0 torque, 1 speed
 *    bit7: axis1 mode 0 torque, 1 speed
 *
 *  Data1: axis2 high byte (int16)
 *  Data2: axis2 low  byte (int16)
 *  Data3: axis2 acc/dec (uint8)
 *  Data4: axis1 high byte (int16)
 *  Data5: axis1 low  byte (int16)
 *  Data6: axis1 acc/dec (uint8)
 *  Data7: reserve
 * ============================================================ */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#include "rtthread.h"
#include "CAN_AGMO.h"
#include "can.h"

/* ===================== IDs / CAN config ===================== */
#define WD_TX_EXTID 0x1801E600u /* host -> driver */
#define WD_RX_EXTID 0x1801E001u /* driver -> host */

// ===================== CAN TX MQ / CAN config  =====================
#define CAN_MQ_DEPTH 16

static struct rt_messagequeue can_tx_mq;
static struct rt_messagequeue can_rx_mq;

rt_mq_t g_can_tx_mq = RT_NULL; // tx mq handler
static rt_bool_t g_can_tx_mq_inited = RT_FALSE;
rt_mq_t g_can_rx_mq = RT_NULL; // rx mq handler
static rt_bool_t g_can_rx_mq_inited = RT_FALSE;

static uint8_t can_tx_mq_pool[CAN_MQ_DEPTH * sizeof(can_msg_t)];
static uint8_t can_rx_mq_pool[CAN_MQ_DEPTH * sizeof(can_msg_t)];

static CanRxMsg RxMessage;
static CanTxMsg TxMessage;
static uint8_t Mail_Box = 0;

/* can rx signleton*/
rt_mq_t can_rx_mq_get(void) {
  if (g_can_rx_mq_inited && g_can_rx_mq != RT_NULL) {
    return g_can_rx_mq;
  }
  return RT_NULL;
}

int can_mq_init(void) {
  if (rt_mq_init(&can_tx_mq, "can_tx", can_tx_mq_pool, sizeof(can_msg_t), sizeof(can_tx_mq_pool), RT_IPC_FLAG_FIFO) !=
      RT_EOK) {
    return -1;
  } else {
    g_can_tx_mq = (rt_mq_t)&can_tx_mq;
    g_can_tx_mq_inited = RT_TRUE;
  }

  if (rt_mq_init(&can_rx_mq, "can_rx", can_rx_mq_pool, sizeof(can_msg_t), sizeof(can_rx_mq_pool), RT_IPC_FLAG_FIFO) !=
      RT_EOK) {
    return -1;
  } else {
    g_can_rx_mq = (rt_mq_t)&can_rx_mq;
    g_can_rx_mq_inited = RT_TRUE;
  }

  return 0;
}

// StdPeriph send (Extended ID)
bool can_send_ext(uint32_t ext_id, const uint8_t data[8], uint8_t dlc) {
  int retry = 0;
  uint8_t tx_status = CAN_TxStatus_Failed;
  // SendCanDataPage(&TxMessage, data, WheelRTR, Motor1IDE, 0xfff, 0x18ff2100, WheelDLC); //

  memset(&TxMessage, 0, sizeof(TxMessage));

  TxMessage.ExtId = ext_id;
  TxMessage.IDE = CAN_Id_Extended;
  TxMessage.RTR = CAN_RTR_Data;
  TxMessage.DLC = dlc;

  for (uint8_t i = 0; i < dlc; i++)
    TxMessage.Data[i] = data[i];

  // TODO: choose CAN1/CAN2 and add retry/mailbox-full handling if needed
  Mail_Box = CAN_Transmit(CAN1, &TxMessage);

  if (Mail_Box == CAN_TxStatus_NoMailBox) {
    // rt_kprintf("no mail_box !\n");
    return false;
  }

  /* Keep wait short to avoid blocking RX handling for long periods. */
  for (retry = 0; retry < 3; retry++) {
    tx_status = CAN_TransmitStatus(CAN1, Mail_Box);
    if (tx_status == CAN_TxStatus_Ok) {
      Mail_Box = 0;
      return true;
    }
    if (tx_status == CAN_TxStatus_Failed) {
      Mail_Box = 0;
      return false;
    }
    rt_thread_delay(1);
  }

  Mail_Box = 0;
  return false;
}

/*
#define CAN_THREAD_STACK  2048
#define CAN_THREAD_PRIO   13
#define CAN_THREAD_TSLICE 10

static rt_thread_t Can_thread = RT_NULL;
static uint8_t g_cana_inited = 0;

// Test Task
static void Can_thread_entry(void* parameter) {
  (void)parameter;

  can_msg_t msg;
  while (1) {
    if (rt_mq_recv(&can_tx_mq, &msg, sizeof(msg), RT_WAITING_FOREVER) == RT_EOK) {
      can_send_ext(msg.ext_id, msg.data, msg.dlc);
    }
  }
}

int cana_init(void) {
  if (g_cana_inited)
    return 0;

  if (rt_mq_init(&can_tx_mq, "can_tx", can_mq_pool, sizeof(can_msg_t), sizeof(can_mq_pool), RT_IPC_FLAG_FIFO) !=
      RT_EOK) {
    return -1;
  }

  Can_thread = rt_thread_create("CAN", Can_thread_entry, RT_NULL, CAN_THREAD_STACK, CAN_THREAD_PRIO, CAN_THREAD_TSLICE);

  if (Can_thread == RT_NULL)
    return -2;

  rt_thread_startup(Can_thread);

  g_cana_inited = 1;
  return 0;
}

rt_err_t cana_send_async(const can_msg_t* m) {
  if (!m)
    return -RT_ERROR;
  return rt_mq_send(&can_tx_mq, (void*)m, sizeof(*m));
}
*/
/* ===================== CAN RX ISR hook ===================== */

void gateway_can_rx_push_isr(uint32_t ext_id, const uint8_t data[8], uint8_t dlc) {
  // tx mq handler
  if (g_can_rx_mq == RT_NULL)
    g_can_rx_mq = (rt_mq_t)&can_rx_mq;

  if (!g_can_rx_mq)
    return;

  can_msg_t fr;
  memset(&fr, 0, sizeof(fr));
  fr.ext_id = ext_id;
  fr.dlc = (dlc > 8) ? 8 : dlc;
  memcpy(fr.data, data, fr.dlc);

  /* ISR-safe send */
  rt_mq_send(g_can_rx_mq, &fr, sizeof(fr));
}

void gateway_can_rx_push_isr_from_rxmsg(const CanRxMsg* rx) {
  // rt_kprintf("rx CAN_msg\n");

  // tx mq handler
  if (g_can_rx_mq == RT_NULL)
    g_can_rx_mq = (rt_mq_t)&can_rx_mq;

  if (!g_can_rx_mq || !rx)
    return;

  // Extended frame? ??(??? StdId? ?? ??)
  if (rx->IDE != CAN_Id_Extended)
    return;

  can_msg_t fr;
  fr.ext_id = rx->ExtId;
  fr.dlc = (rx->DLC > 8) ? 8 : rx->DLC;
  memcpy(fr.data, rx->Data, fr.dlc);

  // ISR-safe: urgent ??(?? ?? ??)
  (void)rt_mq_urgent(g_can_rx_mq, &fr, sizeof(fr));
}

/* ===================== CAN1_RX0_IRQ: CAN1 Frame Collector ===================== */

void A_CAN1_RX0_IRQHandler(ext_func_t ext_func) {
  // RT-Thread ISR ??/??(??)
  rt_interrupt_enter();

  if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) == SET) {
    // FIFO0?? ? ??? ??
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

    // (??) ?? ??? ???? ??? ?? ??
    // CAN1_PrintRecvData(&RxMessage);

    // Gateway MQ? push (ID ??? can_thread?? decode? ????? ?? ? ??)
    ext_func(&RxMessage);

    // ???? pending clear (SPL?? ?? ??)
    CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
  }

  rt_interrupt_leave();
}
