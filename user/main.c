/**
 * @file main.c
 * @brief AGMO VCU application entry point.
 *
 * Keep this file focused on product boot flow. Optional legacy/test modules are
 * gated by MAIN_ENABLE_* switches so they can be enabled for board bring-up
 * without mixing test code into the normal VCU Gateway startup path.
 */

#include "rtthread.h"
#include "main.h"
#include "board.h"
#include "spi.h"
#include "vcu_gateway.h"

/* Optional board/module bring-up switches.
 * Default product boot should keep these disabled and start only VCU Gateway.
 */
#define MAIN_ENABLE_BATTERY_THREAD 0u /* AD/battery sampling test thread */
#define MAIN_ENABLE_MPU6050_THREAD 0u /* IMU attitude/gyro test thread */
#define MAIN_ENABLE_TH_THREAD      0u /* temperature/humidity test thread */
#define MAIN_ENABLE_MOTOR1_THREAD  0u /* legacy motor/CAN test thread */
#define MAIN_ENABLE_EC800_THREAD   0u /* EC800 cellular module test thread */
#define MAIN_ENABLE_SBUS_THREAD    0u /* legacy SBUS test thread */
#define MAIN_ENABLE_MODBUS_THREAD  0u /* Modbus test thread */
#define MAIN_ENABLE_W5500_THREAD   0u /* W5500 test hook; function declaration must be verified before enabling */
#define MAIN_ENABLE_EEPROM_THREAD  0u /* EEPROM thread writes fixed bytes; keep off for one-shot tests */
#define MAIN_ENABLE_EEPROM_ONESHOT 1u /* One-shot EEPROM write/read test for JTAG debug */
#define MAIN_ENABLE_LED_INIT       0u /* LED init is already performed in board init */

/* Legacy test parameter used by bsp_motor1_thread(). */
#define MAIN_MOTOR1_TEST_SPEED 111u

/* Legacy SBUS test selector used by bsp_Sbus_thread(). */
#define MAIN_SBUS_TEST_CHANNEL 4u

/* EEPROM one-shot test constants. Use a non-zero address to verify 16-bit
 * addressing on 24C256 without touching the legacy low-address test bytes.
 */
#define MAIN_EEPROM_TEST_ADDR 0x0100u
#define MAIN_EEPROM_TEST_SIZE 4u

/**
 * @brief Start optional legacy/test modules selected by MAIN_ENABLE_* flags.
 *
 * These modules are not required for normal VCU Gateway operation. Keep them
 * behind compile-time switches to avoid unexpected EEPROM writes, duplicate
 * peripheral use, or legacy test traffic during product boot.
 */

/* Legacy UART/RS485 interrupt handlers still share this receive scratch value.
*
*/
uint16_t RecData = 0u;
extern int8_t flag;
extern int8_t RS485_3_flag;
extern int8_t RS232_2_flag;
extern int8_t RS232_1_flag;

volatile uint8_t g_main_eeprom_present = 0u;
volatile uint8_t g_main_eeprom_write_ok = 0u;
volatile uint8_t g_main_eeprom_read_ok = 0u;
volatile uint8_t g_main_eeprom_match = 0u;
volatile uint8_t g_main_eeprom_tx[MAIN_EEPROM_TEST_SIZE] = {0xA5u, 0x5Au, 0x12u, 0x34u};
volatile uint8_t g_main_eeprom_rx[MAIN_EEPROM_TEST_SIZE] = {0u};

#if MAIN_ENABLE_EEPROM_ONESHOT
/**
 * @brief Run a single EEPROM write/read verification for board bring-up.
 *
 * This is intentionally a one-shot test, not a thread. JTAG can inspect
 * g_main_eeprom_* variables after boot to verify device presence, write/read
 * return values, and byte-for-byte compare status.
 */
static void main_run_eeprom_oneshot_test(void) {
  uint8_t tx[MAIN_EEPROM_TEST_SIZE];
  uint8_t rx[MAIN_EEPROM_TEST_SIZE];
  uint8_t i;
  uint8_t match = 1u;

  for (i = 0u; i < MAIN_EEPROM_TEST_SIZE; i++) {
    tx[i] = (uint8_t)g_main_eeprom_tx[i];
    rx[i] = 0u;
  }

  g_main_eeprom_present = ee_CheckOk();
  if (g_main_eeprom_present == 0u)
    return;

  g_main_eeprom_write_ok = ee_WriteBytes(tx, MAIN_EEPROM_TEST_ADDR, MAIN_EEPROM_TEST_SIZE);
  Delay_Ms(10);
  g_main_eeprom_read_ok = ee_ReadBytes(rx, MAIN_EEPROM_TEST_ADDR, MAIN_EEPROM_TEST_SIZE);

  for (i = 0u; i < MAIN_EEPROM_TEST_SIZE; i++) {
    g_main_eeprom_rx[i] = rx[i];
    if (rx[i] != tx[i])
      match = 0u;
  }

  g_main_eeprom_match = (g_main_eeprom_write_ok && g_main_eeprom_read_ok && match) ? 1u : 0u;
}
#endif

static void main_start_optional_modules(void) {
#if MAIN_ENABLE_BATTERY_THREAD
  (void)bsp_battery_thread();
#endif

#if MAIN_ENABLE_MPU6050_THREAD
  (void)bsp_MPU6050_thread();
#endif

#if MAIN_ENABLE_TH_THREAD
  (void)bsp_TH_thread();
#endif

#if MAIN_ENABLE_MOTOR1_THREAD
  (void)bsp_motor1_thread(MAIN_MOTOR1_TEST_SPEED);
#endif

#if MAIN_ENABLE_EC800_THREAD
  (void)bsp_EC800_USART_thread();
#endif

#if MAIN_ENABLE_SBUS_THREAD
  (void)bsp_Sbus_thread(MAIN_SBUS_TEST_CHANNEL);
#endif

#if MAIN_ENABLE_MODBUS_THREAD
  (void)bsp_Modbus_thread();
#endif

#if MAIN_ENABLE_W5500_THREAD
  /* TODO: Confirm the current W5500 thread function name before enabling. */
  (void)bsp_w5500_thread();
#endif

#if MAIN_ENABLE_EEPROM_THREAD
  /* EEPROM test thread writes fixed bytes to EEPROM; do not enable in product boot. */
  (void)bsp_Ee_thread();
#endif

#if MAIN_ENABLE_LED_INIT
  Init_LED();
#endif
}

/**
 * @brief Application main entry.
 *
 * Normal operation is intentionally simple: initialize the VCU Gateway. Board
 * and peripheral low-level initialization is handled by rt_hw_board_init().
 */
int main(void) {
  rt_kprintf("\r\nAGMO VCU START\r\n");

  (void)vcu_gateway_init();

#if MAIN_ENABLE_EEPROM_ONESHOT
  main_run_eeprom_oneshot_test();
#endif

  main_start_optional_modules();

  return 0;
}
