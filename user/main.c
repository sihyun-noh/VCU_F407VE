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
#define MAIN_ENABLE_EEPROM_THREAD  1u /* EEPROM bring-up test; writes fixed test bytes */
#define MAIN_ENABLE_LED_INIT       0u /* LED init is already performed in board init */

/* Legacy test parameter used by bsp_motor1_thread(). */
#define MAIN_MOTOR1_TEST_SPEED 111u

/* Legacy SBUS test selector used by bsp_Sbus_thread(). */
#define MAIN_SBUS_TEST_CHANNEL 4u

/**
 * @brief Start optional legacy/test modules selected by MAIN_ENABLE_* flags.
 *
 * These modules are not required for normal VCU Gateway operation. Keep them
 * behind compile-time switches to avoid unexpected EEPROM writes, duplicate
 * peripheral use, or legacy test traffic during product boot.
 */
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
  main_start_optional_modules();

  return 0;
}
