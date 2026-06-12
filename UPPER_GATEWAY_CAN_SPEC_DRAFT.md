# Upper <-> Gateway CAN Interface Draft

> Release version: `AGMO-VCU-CAN-2026.05.22-DRAFT`
> Updated: 2026-05-22
> Reference code: `user/vcu_gateway.c`, `user/vcu_gateway.h`
> This document mirrors the current VCU Gateway implementation for Upper Controller integration.

## 1. Common Rules

| Item | Value |
| ---- | ----- |
| CAN bitrate | 500 kbps |
| CAN frame | Extended ID |
| DLC | 8 bytes unless otherwise noted |
| Multi-byte endian | Big-endian, MSB first |
| Signed integer | two's complement |
| Gateway status period | 200ms |
| Drive CMD period | 200ms recommended, timeout causes STOP |

Endian examples:

| Value | Type | Hex bytes |
| ----- | ---- | --------- |
| `+50` | int16 BE | `00 32` |
| `-50` | int16 BE | `FF CE` |
| `300` | uint16 BE | `01 2C` |
| `500` | uint16 BE | `01 F4` |
| `1500` | uint16 BE | `05 DC` |

## 2. CAN ID Summary

### 2.1 Upper -> Gateway RX

| CAN ID | Name | Period | DLC | Purpose |
| ------ | ---- | ------ | --- | ------- |
| `0x18FF0200` | Drive CMD | 200ms | 8 | Upper drive command, current Auto drive command source |
| `0x18FF0210` | Config CMD | On change | 8 | automation, force stop, force active, relay, accel, drive cmd select |
| `0x18FF0220` | Auto Direct Drive CMD | 200ms recommended when selected | 8 | left/right driver direct input, mixer bypass |
| `0x18FF0230` | Weed Actuator CMD | Event | 8 | actuator target/set/move/stop command |
| `0x18FF0240` | Weed Blade CMD | Event | 8 | blade RPM set/run/stop command |

### 2.2 Gateway -> Upper TX

| CAN ID | Name | Period | DLC | Purpose |
| ------ | ---- | ------ | --- | ------- |
| `0x18FF0300` | Motor Status RPM | 200ms | 8 | motor driver RPM feedback |
| `0x18FF0310` | Gateway Status | 200ms | 8 | VCU/RC/FSM/timeout status |
| `0x18FF0320` | Weed Actuator Status | 200ms | 8 | actuator state/position/error |
| `0x18FF0330` | Weed Blade Status | 200ms | 8 | blade state/fault/RPM |
| `0x18FF4000` | Vehicle Motion | 200ms | 8 | vehicle motion monitor, test/debug |
| `0x18FF4010` | Vehicle Monitor | 200ms | 8 | mixer/debug monitor, test/debug |

## 3. Upper -> Gateway RX Detail

### 3.1 `0x18FF0200` Drive CMD

Current Auto drive control uses this CAN ID. If this message times out during Upper/Auto control, VCU stops the motor command path.

| Byte | Signal | Type | Endian | Range/Unit | Description |
| ---- | ------ | ---- | ------ | ---------- | ----------- |
| data[0:1] | `throttle_cmd` | int16 | BE | -500..500 | forward/backward normalized command |
| data[2:3] | `steering_cmd` | int16 | BE | -500..500 | left/right normalized command |
| data[4:5] | `max_driver_input_cmd` | uint16 | BE | recommended 0..2000 | driver input value when normalized input is 500 |
| data[6:7] | `max_speed_kmh_x100` | uint16 | BE | km/h x 100 | reference max speed when normalized input is 500 |

Formula:

```text
driver_input = max_driver_input_cmd * (throttle_cmd / 500)
estimated_speed_kmh = (max_speed_kmh_x100 / 100.0) * (throttle_cmd / 500)
```

Important:

- Send 3.00km/h as `300`, not `3`.
- Send 5.00km/h as `500`.

Example, 3.00km/h reference, 50% forward, straight:

```text
00 FA 00 00 01 8C 01 2C
```

This means:

```text
throttle_cmd = 250
steering_cmd = 0
max_driver_input_cmd = 396
max_speed_kmh_x100 = 300
actual driver input = 396 * (250 / 500) = 198
estimated speed = 3.00km/h * (250 / 500) = 1.50km/h
```

### 3.2 `0x18FF0210` Config CMD

Common config command. It does not include actuator/blade direct commands.

| Byte | Signal | Type | Endian | Range/Unit | Description |
| ---- | ------ | ---- | ------ | ---------- | ----------- |
| data[0] | `automation` | uint8 | - | bit0 | Upper automation request |
| data[1] | `upper_force_stop` | uint8 | - | bit0 | global force stop |
| data[2] | `upper_force_active` | uint8 | - | bit0 | force Upper control request |
| data[3] | `relay_mask` | uint8 | - | bit mask | relay command |
| data[4] | `left_accel_cmd` | uint8 | - | 0..255 | left motor accel |
| data[5] | `right_accel_cmd` | uint8 | - | 0..255 | right motor accel |
| data[6] | `upper_drive_cmd_select` | uint8 | - | bit0 | `0`=use `0x18FF0200`, `1`=use `0x18FF0220` |
| data[7] | Reserved | - | - | - | send 0 |

Example, automation ON and accel 100:

```text
01 00 00 00 64 64 00 00
```

Current policy:

- Config timeout is not used as a hard stop gate.
- Auto handover requires RC remote automation ON and Upper automation ON.
- `data[6] bit0` selects the Upper drive command path. Default `0` preserves `0x18FF0200`; set `1` uses `0x18FF0220`.
- Actuator command uses `0x18FF0230`.
- Blade command uses `0x18FF0240`.

### 3.3 `0x18FF0220` Auto Direct Drive CMD

This frame is used only when `0x18FF0210 data[6] bit0` is set to `1`. It bypasses the VCU mixer and carries logical left/right driver inputs. VCU still applies the installation direction signs before sending motor driver commands.

| Byte | Signal | Type | Endian | Range/Unit | Description |
| ---- | ------ | ---- | ------ | ---------- | ----------- |
| data[0:1] | `left_driver_input_cmd` | int16 | BE | -2000..2000 recommended | logical left driver direct input |
| data[2:3] | `right_driver_input_cmd` | int16 | BE | -2000..2000 recommended | logical right driver direct input |
| data[4:5] | `max_driver_input_cmd` | uint16 | BE | 0..2000 recommended | clamp reference, `0` uses VCU default |
| data[6:7] | `max_speed_kmh_x100` | uint16 | BE | km/h x 100 | monitor/reference speed, e.g. 300 = 3.00km/h |

### 3.4 `0x18FF0230` Weed Actuator CMD

| Byte | Signal | Type | Endian | Range/Unit | Description |
| ---- | ------ | ---- | ------ | ---------- | ----------- |
| data[0] | `command_type` | uint8 | - | enum | `0=STOP`, `1=SET_TARGET`, `2=MOVE_TO_TARGET` |
| data[1] | `stage` | uint8 | - | enum | `0=UP`, `1=MID`, `2=DOWN` |
| data[2:3] | `target_position_mm` | uint16 | BE | 0..200mm | actuator target position |
| data[4] | `option` | uint8 | - | reserved | send 0 |
| data[5:7] | Reserved | - | - | - | send 0 |

Examples:

```text
02 02 00 B4 00 00 00 00  # move to 180mm down
01 01 00 5A 00 00 00 00  # set 90mm target only
```

### 3.5 `0x18FF0240` Weed Blade CMD

| Byte | Signal | Type | Endian | Range/Unit | Description |
| ---- | ------ | ---- | ------ | ---------- | ----------- |
| data[0] | `command_type` | uint8 | - | enum | `0=STOP`, `1=SET_RPM`, `2=RUN` |
| data[1] | `mode` | uint8 | - | enum | `0=SYNC`, reserved |
| data[2:3] | `left_blade_rpm` | uint16 | BE | 0..2000rpm | left blade target rpm |
| data[4:5] | `right_blade_rpm` | uint16 | BE | 0..2000rpm | right blade target rpm |
| data[6] | `blade_accel` | uint8 | - | 0 or 5..20 | `0`=VCU default, otherwise clamped to 5..20 |
| data[7] | Reserved | - | - | - | send 0 |

Examples:

```text
02 00 05 DC 05 DC 0A 00  # run left/right 1500rpm, accel=10
01 00 05 DC 05 DC 0A 00  # set left/right 1500rpm only, accel=10
00 00 00 00 00 00 00 00  # stop
```

## 4. Gateway -> Upper TX Detail

### 4.1 `0x18FF0300` Motor Status RPM

| Byte | Signal | Type | Endian | Unit | Description |
| ---- | ------ | ---- | ------ | ---- | ----------- |
| data[0:1] | `driver_left_axis1_rpm` | int16 | BE | rpm | left driver axis1 feedback |
| data[2:3] | `driver_left_axis2_rpm` | int16 | BE | rpm | left driver axis2 feedback |
| data[4:5] | `driver_right_axis1_rpm` | int16 | BE | rpm | right driver axis1 feedback |
| data[6:7] | `driver_right_axis2_rpm` | int16 | BE | rpm | right driver axis2 feedback |

### 4.2 `0x18FF0310` Gateway Status

| Byte | Signal | Type | Endian | Description |
| ---- | ------ | ---- | ------ | ----------- |
| data[0:1] | `power_supply_value` | int16 | BE | power supply value |
| data[2] | `md_left_fault_msg` | uint8 | - | left motor driver fault code |
| data[3] | `md_right_fault_msg` | uint8 | - | right motor driver fault code |
| data[4] | `rc_status_mask` | uint8 | - | RC status bit mask |
| data[5] | `fsm_status_mask` | uint8 | - | FSM status bit mask |
| data[6] | `relay_st` | uint8 | - | relay status bit mask |
| data[7] | `timeout_detail_code` | uint8 | - | timeout detail code |

RC status mask:

| Bit | Define | Meaning |
| --- | ------ | ------- |
| bit0 | `RC_ST_ENABLE` | RC B button enable |
| bit1 | `RC_ST_EMERGENCY_STOP` | RC A button E-stop |
| bit2 | `RC_ST_FAILSAFE` | RC receiver failsafe, disconnected signal |
| bit3 | `RC_ST_FRESH` | RC data freshness |
| bit4 | `RC_ST_CULTIVATOR_DOWN` | left toggle, implement down |
| bit5 | `RC_ST_CULTIVATOR_ON` | right toggle, weeding motor ON |
| bit6 | `RC_ST_REMOTE_AUTOMATION` | RC D button remote automation |
| bit7 | `RC_ST_DRIVE_MODE` | RC C button drive mode, `0=agile`, `1=stable` |

FSM status mask:

| Bit | Define | Meaning |
| --- | ------ | ------- |
| bit0 | `FSM_ST_MODE_SAFE_STOP` | safe stop state |
| bit1 | `FSM_ST_MODE_MANUAL_RC` | manual RC control state |
| bit2 | `FSM_ST_MODE_AUTO_ARMED` | auto mode armed/ready state |
| bit3 | `FSM_ST_MODE_AUTO_ACTIVE` | actual Upper command auto control state |
| bit4 | `FSM_ST_STOP_UPPER_FORCE` | Upper force stop occurred |
| bit5 | `FSM_ST_STOP_RC_EMG` | RC emergency stop occurred |
| bit6 | `FSM_ST_STOP_MOTOR_FAULT` | motor fault occurred |
| bit7 | `FSM_ST_STOP_TIMEOUT` | timeout stop occurred |

Timeout detail:

| Value | Define | Meaning |
| ----- | ------ | ------- |
| `0` | `TO_NONE` | no timeout |
| `1` | `TO_RC` | RC timeout |
| `2` | `TO_UPPER_CFG` | upper config timeout, not used as hard gate currently |
| `3` | `TO_UPPER_DRIVE` | upper drive command timeout |
| `4` | `TO_MOTOR_LEFT` | left motor feedback timeout |
| `5` | `TO_MOTOR_RIGHT` | right motor feedback timeout |
| `6` | `TO_MULTIPLE` | multiple timeouts |
| `7` | `TO_UPPER_AUTO` | selected `0x18FF0220` auto direct command timeout |

### 4.3 `0x18FF0320` Weed Actuator Status

| Byte | Signal | Type | Endian | Unit | Description |
| ---- | ------ | ---- | ------ | ---- | ----------- |
| data[0] | `actuator_state` | uint8 | - | enum | VCU interpreted actuator summary state |
| data[1] | `error_code` | uint8 | - | code | actuator raw error code |
| data[2:3] | `target_position_mm` | uint16 | BE | mm | VCU target position |
| data[4:5] | `actual_position_mm` | uint16 | BE | mm | actuator feedback position |
| data[6] | `status_flags` | uint8 | - | bit mask | actuator raw status flags |
| data[7] | `meta_bits` | uint8 | - | bit mask | VCU interpreted metadata |

Actuator state: `0=UNKNOWN`, `1=HOME`, `2=MOVING_DOWN`, `3=TARGET_REACHED`, `4=MOVING_UP`, `5=POSITION_MISMATCH`, `6=STOPPED`, `7=FAULT`, `8=TIMEOUT`.

Actuator meta bits: bit0 valid, bit1 fresh, bit2 timeout, bit3 moving, bit4 target reached, bit5 command active, bit6 fault.

### 4.4 `0x18FF0330` Weed Blade Status

| Byte | Signal | Type | Endian | Unit | Description |
| ---- | ------ | ---- | ------ | ---- | ----------- |
| data[0] | `blade_state` | uint8 | - | enum | VCU interpreted blade summary state |
| data[1] | `fault_summary` | uint8 | - | bit mask | left/right blade fault summary |
| data[2:3] | `left_feedback_rpm` | int16 | BE | rpm | left blade feedback rpm |
| data[4:5] | `right_feedback_rpm` | int16 | BE | rpm | right blade feedback rpm |
| data[6] | `cmd_rpm_scaled` | uint8 | - | rpm / 10 | current command rpm scaled |
| data[7] | `meta_bits` | uint8 | - | bit mask | VCU interpreted metadata |

Blade state: `0=UNKNOWN`, `1=STOPPED`, `2=RUNNING`, `3=SET_RPM_ONLY`, `4=FAULT`, `5=TIMEOUT`.

Fault summary: bit0 left fault, bit1 right fault, bit2 any fault.

Blade meta bits: bit0 left valid, bit1 left fresh, bit2 right valid, bit3 right fresh, bit4 running, bit5 command active, bit6 fault.

### 4.5 Test/Debug Vehicle Monitor IDs

`0x18FF4000` Vehicle Motion:

| Byte | Signal | Type | Endian | Scaling |
| ---- | ------ | ---- | ------ | ------- |
| data[0:1] | `yaw_deg_0_360_x10` | int16 | BE | deg x 10 |
| data[2:3] | `yaw_rate_deg_s_x10` | int16 | BE | deg/s x 10 |
| data[4:5] | `left_speed_m_s_x100` | int16 | BE | m/s x 100 |
| data[6:7] | `right_speed_m_s_x100` | int16 | BE | m/s x 100 |

`0x18FF4010` Vehicle Monitor:

| Byte | Signal | Type | Endian | Scaling |
| ---- | ------ | ---- | ------ | ------- |
| data[0] | `throttle_percent` | int8 | - | -100..100 |
| data[1] | `steering_percent` | int8 | - | -100..100 |
| data[2] | `left_cmd_percent` | int8 | - | -100..100 |
| data[3] | `right_cmd_percent` | int8 | - | -100..100 |
| data[4:5] | `yaw_rate_deg_s_x10` | int16 | BE | deg/s x 10 |
| data[6:7] | `center_distance_m_x100` | int16 | BE | m x 100 |

## 5. Control Policy

- Default control source is RC.
- Upper Auto handover requires RC fresh, RC enable ON, RC remote automation ON, Upper automation ON, and fresh Upper Drive CMD.
- Stop priority: Upper force stop, RC emergency stop, motor fault/timeout, Upper Drive timeout, RC timeout/no valid source.
- RC mode uses RC switch values for weed/blade.
- Auto mode starts from safe defaults and applies valid Upper weed/blade commands only.

## 6. Reference Files

- `user/vcu_gateway.h`
- `user/vcu_gateway.c`
- `user/rc_mixer.h`
- `user/rc_mixer.c`
