# Upper <-> Gateway CAN Interface Draft

## 1. Scope
- This draft documents current implementation in `user/vcu_gateway.c` and `user/vcu_gateway.h`.
- Focus:
  - `Upper -> Gateway` command RX
  - `Gateway -> Upper` status TX
- All frames use Extended ID and DLC 8 unless noted.

## 2. Communication Summary
- CAN bitrate: `500 kbps` (500k bps)
- TX periodic thread: every `100 ms` (`CAN_TX_PERIOD_MS`)
- FSM update period: every `10 ms` (`FSM_PERIOD_MS`)
- RX processing: event-driven from CAN RX queue
- Multi-byte integer byte order: **big-endian** (`MSB first`, `LSB second`)

Normalization scale (unified):
- Input scale (RC and Upper drive cmd): `RCM_MAX_RC_INPUT` (current `±500`)
- Driver output scale: `RCM_MAX_DRIVER_INPUT` (current `±664`, `5 km/h` max model)

## 3. CAN ID Table (Upper Interface)

| Direction | ExtID | Period | Purpose |
|---|---:|---:|---|
| Upper -> Gateway | `0x18FF0200` | Event | Drive command (throttle/steering) |
| Upper -> Gateway | `0x18FF0210` | Event | Config/aux command |
| Upper -> Gateway | `0x18FF0220` | Event | Auto command (linear speed/yaw rate) |
| Gateway -> Upper | `0x18FF0300` | 100 ms | Motor driver RPM feedback |
| Gateway -> Upper | `0x18FF0310` | 100 ms | Gateway/RC/FSM status |
| Gateway -> Upper | `0x18FF0320` | 100 ms | Vehicle motion status |
| Gateway -> Upper | `0x18FF0330` | 100 ms | Vehicle monitor/debug |
| Gateway -> Upper | `0x18FF0340` | 100 ms | Weed actuator status |
| Gateway -> Upper | `0x18FF0350` | 100 ms | Blade status |
| Weed actuator -> Gateway | `0x18FF00C8` | 100 ms | Weed actuator feedback |
| Blade -> Gateway | `0x18FF0032` | 100 ms | Blade left feedback |
| Blade -> Gateway | `0x18FF0030` | 100 ms | Blade right feedback |

## 4. Upper -> Gateway (CMD RX)

### 4.1 `0x18FF0200` Drive Command
- Decoder: `decode_upper_drive_cmd()`
- Type: signed `int16` big-endian per signal
- Clamp range in code: `-RCM_MAX_RC_INPUT..+RCM_MAX_RC_INPUT` (current: `-500..+500`)

| Byte | Signal | Type | Description |
|---|---|---|---|
| 0:1 | `throttle_cmd` | `int16` | Forward/backward command |
| 2:3 | `steering_cmd` | `int16` | Left/right command |
| 4:5 | `max_driver_input_cmd` | `uint16` | Runtime max driver input for upper mix |
| 6:7 | `max_speed_kmh_x100` | `uint16` | Runtime max speed (km/h * 100) for upper mix/yaw model |

Apply rule in current code:
- `throttle_cmd`, `steering_cmd` are clamped to `±RCM_MAX_RC_INPUT` (current `±500`).
- When `max_driver_input_cmd > 0`, upper mix uses that value as `vehicle_config.max_driver_input`.
- When `max_speed_kmh_x100 > 0`, upper mix uses `max_speed_kmh_x100 / 100.0` as `vehicle_config.max_speed_kmh`.
- If either runtime field is `0`, default compile-time config is kept (`RCM_MAX_DRIVER_INPUT`, `RCM_MAX_SPEED_KMH`).
- Encoding note: send speed as `km/h * 100`.
  - Example: `5 km/h` must be sent as `500`.

### 4.2 `0x18FF0210` Config/Aux Command
- Decoder: `decode_upper_cmd()`
- Note (current FSM policy):
  - Config timeout is **not** used as a hard stop gate.
  - Config validity for apply path is `upper.valid`.

| Byte | Signal | Type | Description |
|---|---|---|---|
| 0 | `automation` | `bool (bit0)` | Automation signal (used with RC remote automation for relay behavior) |
| 1 | `weed_target` | `uint8` | Weed target stage(0=UP,1=MID,2=DOWN) or direct mm (0..200) |
| 2 | `blade_cmd` | `uint8` | Blade stage(0=STOP,1=MID,2=HIGH) or direct rpm (0..max) |
| 3 | `upper_force_stop` | `bool (bit0)` | E-stop request |
| 4 | `upper_force_active` | `bool (bit0)` | Force-upper mode flag |
| 5 | `relay_mask` | `uint8` | Relay command mask |
| 6 | `left_accel_cmd` | `uint8` | Left accel command (0..255) |
| 7 | `right_accel_cmd` | `uint8` | Right accel command (0..255) |

Motor driver apply rule (current code):
- `enable_bit` is fixed to default:
  - `MOTOR_DRV_DEFAULT_ENABLE_BITS = 0xC3`
  - (`D0_EN_BOTH_ENABLE(0x03) | D0_AXIS1_SPEED_MODE(0x80) | D0_AXIS2_SPEED_MODE(0x40)`)
- Accel comes from `0x18FF0210` when `upper.valid == true`:
  - `data[6]` -> left axis1/axis2 accel
  - `data[7]` -> right axis1/axis2 accel
- If `upper.valid == false`, default accel is used:
  - `MOTOR_DRV_DEFAULT_AXIS1_ACC = 0x64`
  - `MOTOR_DRV_DEFAULT_AXIS2_ACC = 0x64`

## 5. Gateway -> Upper (STATUS TX)

### 5.1 `0x18FF0300` Motor Driver Feedback
- Packer: `pack_upper_status_rpm()`
- Source: motor status snapshot from `0x18FF0021` (left), `0x18FF0020` (right)

| Byte | Signal | Type | Range/Note |
|---|---|---|---|
| 0:1 | `driver_left_axis1_rpm` | `int16` | Clamped to `-664..664` (`±RCM_MAX_DRIVER_INPUT`) |
| 2:3 | `driver_left_axis2_rpm` | `int16` | Clamped to `-664..664` (`±RCM_MAX_DRIVER_INPUT`) |
| 4:5 | `driver_right_axis1_rpm` | `int16` | Clamped to `-664..664` (`±RCM_MAX_DRIVER_INPUT`) |
| 6:7 | `driver_right_axis2_rpm` | `int16` | Clamped to `-664..664` (`±RCM_MAX_DRIVER_INPUT`) |

### 5.2 `0x18FF0310` Gateway Status
- Packer: `pack_upper_status()`
- Layout matches requested status frame format.

| Byte | Signal | Type | Description |
|---|---|---|---|
| 0:1 | `power_supply_value` | `int16` | Currently from left motor `supply_volt`, clamped `-664..664` (`±RCM_MAX_DRIVER_INPUT`) |
| 2 | `md_left_fault_msg` | `uint8` | Driver 1 fault bits |
| 3 | `md_right_fault_msg` | `uint8` | Driver 2 fault bits |
| 4 | `rc_status_mask` | `uint8` | RC status bit mask |
| 5 | `fsm_status_mask` | `uint8` | FSM status bit mask |
| 6 | `relay_st` | `uint8` | Relay status mask |
| 7 | `timeout_detail_code` | `uint8` | Timeout source detail code (`TO_*`) |

RC status bit mask (`data[4]`):
- bit0: `RC_ST_ENABLE` (RC transmitter B button)
- bit1: `RC_ST_EMERGENCY_STOP` (RC transmitter A button, E-STOP)
- bit2: `RC_ST_FAILSAFE` (set when RC transmitter signal is lost/disconnected)
- bit3: `RC_ST_FRESH` (RC signal is being updated in real time; if not fresh, treated as timeout state)
- bit4: `RC_ST_CULTIVATOR_DOWN` (RC left toggle, implement lowered to ground)
- bit5: `RC_ST_CULTIVATOR_ON` (RC right toggle, weeding/cultivator drive ON)
- bit6: `RC_ST_REMOTE_AUTOMATION` (RC transmitter D button / remote automation active)
- bit7: `RC_ST_DRIVE_MODE` (RC transmitter C button / drive mode: `0=agile`, `1=stable`)

FSM status bit mask (`data[5]`):
- bit0: `FSM_ST_MODE_SAFE_STOP`
- bit1: `FSM_ST_MODE_MANUAL_RC`
- bit2: `FSM_ST_MODE_AUTO_ARMED`
- bit3: `FSM_ST_MODE_AUTO_ACTIVE`
- bit4: `FSM_ST_STOP_UPPER_FORCE`
- bit5: `FSM_ST_STOP_RC_EMG`
- bit6: `FSM_ST_STOP_MOTOR_FAULT`
- bit7: `FSM_ST_STOP_TIMEOUT`

FSM status bit set conditions (why each state occurs):
- Mode bits are one-hot (bit0~bit3).
- `bit0 SAFE_STOP`: STOP path selected or stop reason exists.
- `bit1 MANUAL_RC`: RC driving mode (`rc_ok && rc_enable`) without auto-active handover.
- `bit2 AUTO_ARMED`: RC automation requested but upper auto handover is not active yet.
- `bit3 AUTO_ACTIVE`: upper control path active (force-upper or auto handover active).
- `bit4 UPPER_FORCE`: set when stop reason is upper force stop.
- `bit5 RC_EMG`: set when stop reason is RC emergency stop.
- `bit6 MOTOR_FAULT`: set when stop reason is motor fault.
- `bit7 TIMEOUT`: set when stop reason is timeout.

Timeout detail code (`data[7]`) mapping:
- `0`: `TO_NONE` (no timeout detail)
- `1`: `TO_RC` (RC timeout)
- `2`: `TO_UPPER_CFG` (reserved; not used as hard timeout in current FSM path)
- `3`: `TO_UPPER_DRIVE` (upper drive timeout)
- `4`: `TO_MOTOR_LEFT` (left motor status timeout)
- `5`: `TO_MOTOR_RIGHT` (right motor status timeout)
- `6`: `TO_MULTIPLE` (multiple timeout conditions at the same time)

### 5.3 `0x18FF0320` Vehicle Motion Status
- Packer: `pack_upper_vehicle_status()`
- Purpose: motion snapshot

| Byte | Signal | Type | Scaling |
|---|---|---|---|
| 0:1 | `yaw_deg_0_360_x10` | `int16` | deg * 10 |
| 2:3 | `yaw_rate_deg_s_x10` | `int16` | deg/s * 10 |
| 4:5 | `left_speed_m_s_x100` | `int16` | m/s * 100 |
| 6:7 | `right_speed_m_s_x100` | `int16` | m/s * 100 |

### 5.4 `0x18FF0330` Vehicle Monitor/Debug
- Packer: `pack_upper_vehicle_monitor()`
- Purpose: test/monitoring aid

| Byte | Signal | Type | Scaling |
|---|---|---|---|
| 0 | `throttle_percent` | `int8` | -100..100 |
| 1 | `steering_percent` | `int8` | -100..100 |
| 2 | `left_cmd_percent` | `int8` | -100..100 |
| 3 | `right_cmd_percent` | `int8` | -100..100 |
| 4:5 | `yaw_rate_deg_s_x10` | `int16` | IMU gyro Z deg/s * 10 (fallback: command-based yaw rate) |
| 6:7 | `center_distance_m_x100` | `int16` | m * 100 (cm) |

### 5.5 `0x18FF0340` Weed Actuator Status
- Packer: `pack_upper_weed_status()`
- Purpose: forward weed actuator runtime state/error/position to upper

| Byte | Signal | Type | Description |
|---|---|---|---|
| 0 | `status_flags` | `uint8` | actuator status bit flags (from `0x18FF00C8 data[3]`) |
| 1 | `error_code` | `uint8` | actuator error code (`0x18FF00C8 data[4]`) |
| 2 | `current_raw` | `uint8` | current raw (`0.25A/bit`) |
| 3 | `input_state` | `uint8` | actuator input state |
| 4 | `meta_bits` | `uint8` | bit0 valid, bit1 timeout, bit2 pre_sent |
| 5 | `target_mm` | `uint8` | target position in mm (clamped 0..255) |
| 6 | `actual_pos_mm` | `uint8` | actual position in mm (clamped 0..255) |
| 7 | `speed_mm_s` | `uint8` | speed in mm/s (clamped 0..255) |


### 5.6 `0x18FF0350` Blade Status
- Packer: `pack_upper_blade_status()`

| Byte | Signal | Type | Description |
|---|---|---|---|
| 0 | `left_fault_bits` | `uint8` | blade left fault bits |
| 1 | `right_fault_bits` | `uint8` | blade right fault bits |
| 2 | `blade_cmd_rpm` | `uint8` | current blade command rpm (0..255 clamp) |
| 3 | `src_mask` | `uint8` | bit0 RC, bit1 UPPER_AUTO, bit2 STOP |
| 4 | `left_rpm_axis1_div10` | `int8` | left rpm axis1 / 10 |
| 5 | `right_rpm_axis1_div10` | `int8` | right rpm axis1 / 10 |
| 6 | `left_meta` | `uint8` | bit0 valid, bit1 fresh |
| 7 | `right_meta` | `uint8` | bit0 valid, bit1 fresh |

### 5.7 `0x18FF00C8` Weed Actuator Feedback RX
- Decoder: `decode_weed_actuator_status()`
- Byte order: little-endian for position/speed fields (device-native)

| Byte | Signal | Type | Description |
|---|---|---|---|
| 0:1 | `position_x10_mm` | `uint16 (LE)` | position in 0.1 mm |
| 2 | `current_raw` | `uint8` | current raw (`0.25A/bit`) |
| 3 | `status_flags` | `uint8` | status bit flags |
| 4 | `error_code` | `uint8` | actuator error code |
| 5:6 | `speed_x10_mm_s` | `uint16 (LE)` | speed in 0.1 mm/s |
| 7 | `input_state` | `uint8` | input state |

## 6. Control Arbitration (Current Code Behavior)
- STOP priority:
  1. `upper_force_stop`
  2. RC emergency stop
  3. Motor fault/timeout
- If not STOP:
  - `upper_force_active=true` -> Upper command active (force select)
  - else if `rc_ok && rc_enable && rc_remote_automation && upper.valid && upper.automation` -> Upper auto handover active
  - else if `rc_ok && rc_enable` -> RC command active (default priority)
  - else -> timeout stop
- Summary: default control source priority is RC first; Upper is entered only by force or auto handover gate.
- Weed actuator decision is evaluated in FSM (`weed_fsm_step()`), while `can_tx_thread` only transmits pre-built pending weed frames.
- Blade command path is handled in FSM by `blade_fsm_step()`:
  - `blade_cmd_step()` decides target rpm.
  - `blade_tx_plan_step()` creates 250ms periodic pending frames only when RC B button(`rc_enable`) is ON.
  - `can_tx_thread` only consumes/transmits pending blade frames.
- Upper drive timeout: `UPPER_DRIVE_TIMEOUT_MS = 1000 ms`
- Motor timeout: `MOTOR_TIMEOUT_MS = 500 ms`
- RC freshness timeout: `SBUS_TIMEOUT_MS = 1000 ms`

Driver OK condition (command execution prerequisite):
- `motor_left_ok = motor_left.valid && fresh(MOTOR_TIMEOUT_MS) && (fault_bits == 0)`
- `motor_right_ok = motor_right.valid && fresh(MOTOR_TIMEOUT_MS) && (fault_bits == 0)`
- If either driver is not OK, FSM goes to STOP path (`FSM_STOP_MOTOR_FAULT` or `FSM_STOP_TIMEOUT`) and normal drive command is not maintained.

### 6.1 Timeout Detection Note (Important)
- Timeout logic is freshness-based (`valid` + elapsed time from last received frame).
- If a CAN message is **not received periodically**, timeout-based behavior cannot be guaranteed as intended.
- In other words, periodic reception is a prerequisite; without periodic updates, system behavior may remain on stale state until timeout conditions are actually evaluated by updated timestamps/valid flags.

## 7. Motor Driver Config Rule (Current)
- Driver `enable_bit` is fixed to default:
  - `MOTOR_DRV_DEFAULT_ENABLE_BITS` (`0xC3`)
- Upper does not override `driver_config_bitmask` anymore.
- Accel is configurable from upper command `0x18FF0210`:
  - `data[6]`: left accel (`0..255`) -> applied to left axis1/axis2 accel
  - `data[7]`: right accel (`0..255`) -> applied to right axis1/axis2 accel
- If upper config is not valid, accel falls back to defaults:
  - `MOTOR_DRV_DEFAULT_AXIS1_ACC = 0x64`
  - `MOTOR_DRV_DEFAULT_AXIS2_ACC = 0x64`

## 8. Open Items / TODO
- `CANID_UPPER_CMD_RX (0x18FF0210)` comment says TODO; verify final ID assignment with upper controller.
- `CANID_MOTOR_CMD_DRIVER1_TX`, `CANID_MOTOR_CMD_DRIVER2_TX` comments still marked TODO; verify final production IDs.
- `power_supply_value` currently uses left motor supply only.
- Header comments at top of `vcu_gateway.c` still contain old temporary CAN examples and should be synced with current IDs.

## 9. Reference Files
- `user/vcu_gateway.h`
- `user/vcu_gateway.c`

## 10. Latest Update Note (Enum Integration)
- FSM/command status enums are now unified under `user/vcu_gateway.h` for centralized management.
- Unified base enums:
  - `vcu_control_src_t` (`SRC_NONE`, `SRC_RC`, `SRC_UPPER`)
  - `vcu_cmd_type_t` (`CMD_STOP`, `CMD_SETPOINT`)
  - `vcu_stop_reason_t` (`STOP_NONE`, `STOP_UPPER_FORCE`, `STOP_RC_EMG`, `STOP_MOTOR_FAULT`, `STOP_TIMEOUT`)
- Compatibility is preserved:
  - Existing names `cmd_src_t`, `cmd_type_t`, `fsm_control_src_t`, `fsm_stop_reason_t` remain as typedef aliases.
  - Existing labels `FSM_CTRL_SRC_*`, `FSM_STOP_*` remain as compatibility macros.
- Effect: status/bitmask-related semantics are managed in one header location, reducing mismatch risk between FSM logic and status reporting.

### 10.1 Unified Enum Quick Guide
- `vcu_control_src_t`: 현재 제어 명령의 주체를 나타냄 (`SRC_NONE`, `SRC_RC`, `SRC_UPPER`).
- `vcu_upper_cmd_t`: Upper 명령 데이터의 종류를 구분하기 위한 타입 (`UPPER_NONE`, `UPPER_RPM`, `UPPER_CONFIG`).
- `vcu_cmd_type_t`: 실제 출력 명령 모드 구분 (`CMD_STOP` = 정지, `CMD_SETPOINT` = 목표값 구동).
