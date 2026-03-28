# Upper <-> Gateway CAN Interface Draft

## 1. Scope
- This draft documents current implementation in `user/vcu_gateway.c` and `user/vcu_gateway.h`.
- Focus:
  - `Upper -> Gateway` command RX
  - `Gateway -> Upper` status TX
- All frames use Extended ID and DLC 8 unless noted.

## 2. Communication Summary
- TX periodic thread: every `100 ms` (`CAN_TX_PERIOD_MS`)
- FSM update period: every `10 ms` (`FSM_PERIOD_MS`)
- RX processing: event-driven from CAN RX queue

## 3. CAN ID Table (Upper Interface)

| Direction | ExtID | Period | Purpose |
|---|---:|---:|---|
| Upper -> Gateway | `0x18FF0200` | Event | Drive command (throttle/steering) |
| Upper -> Gateway | `0x18FF0210` | Event | Config/aux command |
| Gateway -> Upper | `0x18FF0300` | 100 ms | Motor driver RPM feedback |
| Gateway -> Upper | `0x18FF0310` | 100 ms | Gateway/RC/FSM status |
| Gateway -> Upper | `0x18FF0320` | 100 ms | Vehicle motion status |
| Gateway -> Upper | `0x18FF0330` | 100 ms | Vehicle monitor/debug |

## 4. Upper -> Gateway (CMD RX)

### 4.1 `0x18FF0200` Drive Command
- Decoder: `decode_upper_drive_cmd()`
- Type: signed `int16` big-endian per signal
- Clamp range in code: `CMD_MIN..CMD_MAX` (`-670..670`)

| Byte | Signal | Type | Description |
|---|---|---|---|
| 0:1 | `throttle_cmd` | `int16` | Forward/backward command |
| 2:3 | `steering_cmd` | `int16` | Left/right command |
| 4:7 | Reserved | - | Not used |

### 4.2 `0x18FF0210` Config/Aux Command
- Decoder: `decode_upper_cmd()`
- Note: `upper.automation` bit is currently used as `upper_ok` gate in FSM.

| Byte | Signal | Type | Description |
|---|---|---|---|
| 0 | `driver_config_bitmask` | `uint8` | Motor driver config bits |
| 1 | `cultivator_down` | `bool (bit0)` | Lower implement to the ground for field operation (RC left toggle) |
| 2 | `cultivator_on` | `bool (bit0)` | Turn weeding/cultivator motor ON (start) (RC right toggle) |
| 3 | `upper_force_stop` | `bool (bit0)` | E-stop request |
| 4 | `upper_force_active` | `bool (bit0)` | Force-upper mode flag |
| 5 | `relay_mask` | `uint8` | Relay command mask |
| 6 | `automation` | `bool (bit0)` | Upper mode enable gate |
| 7 | Reserved | - | Not used |

## 5. Gateway -> Upper (STATUS TX)

### 5.1 `0x18FF0300` Motor Driver Feedback
- Packer: `pack_upper_status_rpm()`
- Source: motor status snapshot from `0x18FF0021` (left), `0x18FF0020` (right)

| Byte | Signal | Type | Range/Note |
|---|---|---|---|
| 0:1 | `driver_left_axis1_rpm` | `int16` | Clamped to `-670..670` |
| 2:3 | `driver_left_axis2_rpm` | `int16` | Clamped to `-670..670` |
| 4:5 | `driver_right_axis1_rpm` | `int16` | Clamped to `-670..670` |
| 6:7 | `driver_right_axis2_rpm` | `int16` | Clamped to `-670..670` |

### 5.2 `0x18FF0310` Gateway Status
- Packer: `pack_upper_status()`
- Layout matches requested status frame format.

| Byte | Signal | Type | Description |
|---|---|---|---|
| 0:1 | `power_supply_value` | `int16` | Currently from left motor `supply_volt`, clamped `-670..670` |
| 2 | `md_left_fault_msg` | `uint8` | Driver 1 fault bits |
| 3 | `md_right_fault_msg` | `uint8` | Driver 2 fault bits |
| 4 | `rc_status_mask` | `uint8` | RC status bit mask |
| 5 | `vcu_fsm_status_mask` | `uint8` | VCU FSM bit mask |
| 6 | `relay_st` | `uint8` | Relay status mask |
| 7 | Reserved | `uint8` | `0x00` |

RC status bit mask (`data[4]`):
- bit0: `RC_ST_ENABLE` (RC transmitter B button)
- bit1: `RC_ST_EMERGENCY_STOP` (RC transmitter A button, E-STOP)
- bit2: `RC_ST_FAILSAFE` (set when RC transmitter signal is lost/disconnected)
- bit3: `RC_ST_FRESH` (RC signal is being updated in real time; if not fresh, treated as timeout state)
- bit4: `RC_ST_CULTIVATOR_DOWN` (RC left toggle, implement lowered to ground)
- bit5: `RC_ST_CULTIVATOR_ON` (RC right toggle, weeding/cultivator drive ON)
- bit6: `RC_ST_REMOTE_AUTOMATION` (RC transmitter D button / remote automation active)

VCU FSM status bit mask (`data[5]`):
- bit0: `VCU_ST_SRC_NONE`
- bit1: `VCU_ST_SRC_RC`
- bit2: `VCU_ST_SRC_UPPER`
- bit3: `VCU_ST_STOP_UPPER`
- bit4: `VCU_ST_STOP_RC_EMG`
- bit5: `VCU_ST_STOP_MOTOR_FAULT`
- bit6: `VCU_ST_STOP_TIMEOUT`
- bit7: `VCU_ST_RUNNING`

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
| 4:5 | `yaw_rate_deg_s_x10` | `int16` | deg/s * 10 |
| 6:7 | `center_distance_m_x100` | `int16` | m * 100 (cm) |

## 6. Control Arbitration (Current Code Behavior)
- STOP priority:
  1. `upper_force_stop`
  2. RC emergency stop
  3. Motor fault/timeout
- If not STOP:
  - RC valid + enabled -> RC command active
  - else Upper active (`automation=true`) -> Upper command active
  - else -> timeout stop
- Upper drive timeout: `UPPER_DRIVE_TIMEOUT_MS = 1000 ms`
- Motor timeout: `MOTOR_TIMEOUT_MS = 500 ms`
- RC freshness timeout: `SBUS_TIMEOUT_MS = 1000 ms`

### 6.1 Timeout Detection Note (Important)
- Timeout logic is freshness-based (`valid` + elapsed time from last received frame).
- If a CAN message is **not received periodically**, timeout-based behavior cannot be guaranteed as intended.
- In other words, periodic reception is a prerequisite; without periodic updates, system behavior may remain on stale state until timeout conditions are actually evaluated by updated timestamps/valid flags.

## 7. Motor Driver Config Rule (Current)
- Default config: `MOTOR_DRV_DEFAULT_ENABLE_BITS`
- Upper config override is accepted only when enable bits satisfy:
  - `(driver_config_bitmask & D0_ENABLE_MASK) == D0_EN_BOTH_ENABLE`
- Same config is applied to left/right driver command.

## 8. Open Items / TODO
- `CANID_UPPER_CMD_RX (0x18FF0210)` comment says TODO; verify final ID assignment with upper controller.
- `CANID_MOTOR_CMD_DRIVER1_TX`, `CANID_MOTOR_CMD_DRIVER2_TX` comments still marked TODO; verify final production IDs.
- `power_supply_value` currently uses left motor supply only.
- Header comments at top of `vcu_gateway.c` still contain old temporary CAN examples and should be synced with current IDs.

## 9. Reference Files
- `user/vcu_gateway.h`
- `user/vcu_gateway.c`
