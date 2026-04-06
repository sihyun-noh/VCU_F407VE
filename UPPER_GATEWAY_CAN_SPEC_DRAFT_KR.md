# Upper <-> Gateway CAN 인터페이스 초안 (한글)

## 1. 범위
- 본 문서는 `user/vcu_gateway.c`, `user/vcu_gateway.h`의 현재 구현 기준입니다.
- 대상:
  - `Upper -> Gateway` 명령 수신
  - `Gateway -> Upper` 상태 송신
- 모든 프레임은 Extended ID, DLC 8 기준입니다.

## 2. 통신 개요
- CAN 통신 속도: `500 kbps` (500k bps)
- TX 주기 스레드: `100 ms` (`CAN_TX_PERIOD_MS`)
- FSM 갱신 주기: `10 ms` (`FSM_PERIOD_MS`)
- RX 처리: CAN RX 큐 이벤트 기반
- 멀티바이트 정수 바이트 순서: **big-endian** (`MSB 먼저`, `LSB 다음`)

정규화 스케일(통합):
- 입력 스케일(RC, Upper drive cmd): `RCM_MAX_RC_INPUT` (현재 `±500`)
- 드라이버 출력 스케일: `RCM_MAX_DRIVER_INPUT` (현재 `±664`, 최대속도 모델 `5 km/h` 기준)

## 3. CAN ID 요약

| 방향 | ExtID | 주기 | 용도 |
|---|---:|---:|---|
| Upper -> Gateway | `0x18FF0200` | Event | 주행 명령(throttle/steering + 런타임 제한값) |
| Upper -> Gateway | `0x18FF0210` | Event | 설정/보조 명령 |
| Gateway -> Upper | `0x18FF0300` | 100 ms | 모터 드라이버 RPM 피드백 |
| Gateway -> Upper | `0x18FF0310` | 100 ms | Gateway/RC/FSM 상태 |
| Gateway -> Upper | `0x18FF0320` | 100 ms | 차량 운동 상태 |
| Gateway -> Upper | `0x18FF0330` | 100 ms | 차량 모니터/디버그 |

## 4. Upper -> Gateway (CMD RX)

### 4.1 `0x18FF0200` Drive Command
- Decoder: `decode_upper_drive_cmd()`
- 타입: 신호별 signed `int16` / big-endian
- `throttle_cmd`, `steering_cmd`는 코드에서 `±RCM_MAX_RC_INPUT`(현재 `±500`)으로 clamp

| Byte | 신호 | 타입 | 설명 |
|---|---|---|---|
| 0:1 | `throttle_cmd` | `int16` | 전/후진 명령 |
| 2:3 | `steering_cmd` | `int16` | 좌/우 조향 명령 |
| 4:5 | `max_driver_input_cmd` | `uint16` | Upper 믹서 런타임 최대 출력 |
| 6:7 | `max_speed_kmh_x100` | `uint16` | Upper 믹서 런타임 최대 속도 (`km/h * 100`) |

적용 규칙:
- `max_driver_input_cmd > 0`이면 `vehicle_config.max_driver_input`으로 적용
- `max_speed_kmh_x100 > 0`이면 `vehicle_config.max_speed_kmh = max_speed_kmh_x100 / 100.0` 적용
- 값이 `0`이면 기본 컴파일 상수 유지(`RCM_MAX_DRIVER_INPUT`, `RCM_MAX_SPEED_KMH`)
- 인코딩 주의: 속도는 `km/h * 100`으로 전송
  - 예: `5 km/h`는 `500`으로 전송
- 적용 범위 주의:
  - 위 runtime limit(`data[4:7]`)은 **Upper 제어 경로에서만** 적용
  - RC 제어 경로는 기본값(`g_rcm_vehicle`) 사용

### 4.2 `0x18FF0210` Config/Aux Command
- Decoder: `decode_upper_cmd()`
- 참고 (현재 FSM 정책):
  - config timeout은 hard stop 조건으로 사용하지 않음
  - config 적용 유효성은 `upper.valid` 기준

| Byte | 신호 | 타입 | 설명 |
|---|---|---|---|
| 0 | `automation` | `bool (bit0)` | automation 신호 (RC remote automation과 함께 릴레이 동작에 사용) |
| 1 | `cultivator_down` | `bool (bit0)` | 작업기 하강 (좌 토글) |
| 2 | `cultivator_on` | `bool (bit0)` | 제초기/작업기 ON (우 토글) |
| 3 | `upper_force_stop` | `bool (bit0)` | E-stop 요청 |
| 4 | `upper_force_active` | `bool (bit0)` | Upper 강제 선택 플래그 |
| 5 | `relay_mask` | `uint8` | 릴레이 마스크 |
| 6 | `left_accel_cmd` | `uint8` | 좌측 accel (0..255) |
| 7 | `right_accel_cmd` | `uint8` | 우측 accel (0..255) |

모터 드라이버 설정 적용 규칙:
- `enable_bit`는 항상 기본값 고정
  - `MOTOR_DRV_DEFAULT_ENABLE_BITS = 0xC3`
- Upper는 `driver_config_bitmask`로 override 하지 않음
- accel은 `upper.valid`일 때 적용
  - 좌측: `data[6]` -> left axis1/axis2 accel
  - 우측: `data[7]` -> right axis1/axis2 accel
- `upper.valid`가 아니면 기본 accel 사용
  - `MOTOR_DRV_DEFAULT_AXIS1_ACC = 0x64`
  - `MOTOR_DRV_DEFAULT_AXIS2_ACC = 0x64`

## 5. Gateway -> Upper (STATUS TX)

### 5.1 `0x18FF0300` 모터 드라이버 피드백
- Packer: `pack_upper_status_rpm()`

| Byte | 신호 | 타입 | 범위/비고 |
|---|---|---|---|
| 0:1 | `driver_left_axis1_rpm` | `int16` | `±664` (`±RCM_MAX_DRIVER_INPUT`) clamp |
| 2:3 | `driver_left_axis2_rpm` | `int16` | `±664` clamp |
| 4:5 | `driver_right_axis1_rpm` | `int16` | `±664` clamp |
| 6:7 | `driver_right_axis2_rpm` | `int16` | `±664` clamp |

### 5.2 `0x18FF0310` Gateway 상태
- Packer: `pack_upper_status()`

| Byte | 신호 | 타입 | 설명 |
|---|---|---|---|
| 0:1 | `power_supply_value` | `int16` | 현재 left motor `supply_volt`, `±664` clamp |
| 2 | `md_left_fault_msg` | `uint8` | Driver1 fault |
| 3 | `md_right_fault_msg` | `uint8` | Driver2 fault |
| 4 | `rc_status_mask` | `uint8` | RC 상태 비트마스크 |
| 5 | `fsm_status_mask` | `uint8` | FSM 상태 비트마스크 |
| 6 | `relay_st` | `uint8` | 릴레이 상태 |
| 7 | `timeout_detail_code` | `uint8` | timeout 원인 상세 코드 |

RC status bitmask (`data[4]`):
- bit0: `RC_ST_ENABLE` (조종기 B 버튼)
- bit1: `RC_ST_EMERGENCY_STOP` (조종기 A 버튼, E-STOP)
- bit2: `RC_ST_FAILSAFE` (조종기 신호 끊김)
- bit3: `RC_ST_FRESH` (실시간 수신 상태, 실패 시 timeout 상태)
- bit4: `RC_ST_CULTIVATOR_DOWN` (좌 토글, 작업기 하강)
- bit5: `RC_ST_CULTIVATOR_ON` (우 토글, 제초기 ON)
- bit6: `RC_ST_REMOTE_AUTOMATION` (조종기 D 버튼, remote automation)
- bit7: `RC_ST_DRIVE_MODE` (조종기 C 버튼, 주행 모드: `0=agile`, `1=stable`)

주의 (`rc.valid` vs `rc.failsafe`):
- `rc.valid`와 `rc.failsafe`는 서로 다른 의미입니다.
- `rc.failsafe`는 SBUS 수신 데이터가 전달하는 RC 연결 상태(disconnect/connect) 신호입니다.
- 따라서 `rc.valid`만으로 failsafe를 판단하면 안 됩니다.

FSM status bitmask (`data[5]`):
- bit0: `FSM_ST_MODE_SAFE_STOP`
- bit1: `FSM_ST_MODE_MANUAL_RC`
- bit2: `FSM_ST_MODE_AUTO_ARMED`
- bit3: `FSM_ST_MODE_AUTO_ACTIVE`
- bit4: `FSM_ST_STOP_UPPER_FORCE`
- bit5: `FSM_ST_STOP_RC_EMG`
- bit6: `FSM_ST_STOP_MOTOR_FAULT`
- bit7: `FSM_ST_STOP_TIMEOUT`

비트 발생 조건:
- 모드 비트(bit0~bit3)는 one-hot으로 사용
- `bit0 SAFE_STOP`: STOP 경로이거나 stop reason 존재
- `bit1 MANUAL_RC`: RC 주행 모드(`rc_ok && rc_enable`), 자동전환 비활성
- `bit2 AUTO_ARMED`: RC 자동화 요청은 있으나 Upper 자동전환 미성립
- `bit3 AUTO_ACTIVE`: Upper 제어 경로 활성(강제 Upper 또는 자동전환 성립)
- `bit4 UPPER_FORCE`: stop reason이 upper force stop
- `bit5 RC_EMG`: stop reason이 RC 비상정지
- `bit6 MOTOR_FAULT`: stop reason이 모터 fault
- `bit7 TIMEOUT`: stop reason이 timeout

timeout detail code (`data[7]`):
- `0`: `TO_NONE`
- `1`: `TO_RC`
- `2`: `TO_UPPER_CFG` (예약값, 현재 FSM hard timeout 경로에서는 미사용)
- `3`: `TO_UPPER_DRIVE`
- `4`: `TO_MOTOR_LEFT`
- `5`: `TO_MOTOR_RIGHT`
- `6`: `TO_MULTIPLE`

### 5.3 `0x18FF0320` 차량 운동 상태
- Packer: `pack_upper_vehicle_status()`

| Byte | 신호 | 타입 | 스케일 |
|---|---|---|---|
| 0:1 | `yaw_deg_0_360_x10` | `int16` | deg * 10 |
| 2:3 | `yaw_rate_deg_s_x10` | `int16` | deg/s * 10 |
| 4:5 | `left_speed_m_s_x100` | `int16` | m/s * 100 |
| 6:7 | `right_speed_m_s_x100` | `int16` | m/s * 100 |

### 5.4 `0x18FF0330` 차량 모니터/디버그
- Packer: `pack_upper_vehicle_monitor()`

| Byte | 신호 | 타입 | 스케일 |
|---|---|---|---|
| 0 | `throttle_percent` | `int8` | -100..100 |
| 1 | `steering_percent` | `int8` | -100..100 |
| 2 | `left_cmd_percent` | `int8` | -100..100 |
| 3 | `right_cmd_percent` | `int8` | -100..100 |
| 4:5 | `yaw_rate_deg_s_x10` | `int16` | deg/s * 10 |
| 6:7 | `center_distance_m_x100` | `int16` | m * 100 |

## 6. 제어 우선순위 (현재 코드)
- STOP 우선순위:
  1. `upper_force_stop`
  2. RC emergency stop
  3. motor fault/timeout
- STOP이 아니면:
  - `upper_force_active=true`면 Upper 강제 선택
  - 아니면 `rc_ok && rc_enable && rc_remote_automation && upper.valid && upper.automation`이면 Upper 자동전환
  - 아니면 RC 유효+enable이면 RC 선택(기본 우선순위)
  - 아니면 timeout stop
- 타임아웃 상수:
  - `UPPER_DRIVE_TIMEOUT_MS = 1000`
  - `MOTOR_TIMEOUT_MS = 500`
  - `SBUS_TIMEOUT_MS = 1000`

FSM 동작 전 신뢰성 확인(핵심):
- FSM은 수신 데이터의 `valid + freshness`를 먼저 확인한 뒤 제어 로직을 수행합니다.
- RC(SBUS), Upper(CAN), Motor status(CAN) 중 필요한 신뢰성 조건이 깨지면 제어를 유지하지 않고 STOP 경로로 전환합니다.
- 즉, 이 모듈은 \"값이 들어왔다\"만으로 동작하지 않고, \"최신/유효한 값\"인지 확인한 뒤 동작합니다.

체크 기준 요약:
- RC: `rc.valid && fresh(SBUS_TIMEOUT_MS)`
- Upper config: `upper.valid` (freshness timeout은 hard stop gate로 미사용)
- Upper drive: `upper_drive.valid && fresh(UPPER_DRIVE_TIMEOUT_MS)`
- Motor left/right: `valid && fresh(MOTOR_TIMEOUT_MS) && fault_bits==0`

Driver OK 조건(명령 유지 전제):
- `motor_left_ok = motor_left.valid && fresh(MOTOR_TIMEOUT_MS) && (fault_bits == 0)`
- `motor_right_ok = motor_right.valid && fresh(MOTOR_TIMEOUT_MS) && (fault_bits == 0)`
- 좌/우 중 하나라도 Driver OK가 아니면 FSM은 STOP 경로로 전환되며(`FSM_STOP_MOTOR_FAULT` 또는 `FSM_STOP_TIMEOUT`), 정상 주행 명령을 유지하지 않습니다.

### 6.1 Timeout 관련 주의
- Timeout 판정은 freshness(`valid` + 마지막 수신시각) 기반입니다.
- CAN 메시지가 주기적으로 수신되지 않으면 timeout 동작이 의도대로 보장되지 않습니다.

## 7. 참고 파일
- `user/vcu_gateway.h`
- `user/vcu_gateway.c`
- `user/rc_mixer.h`
- `user/rc_mixer.c`
