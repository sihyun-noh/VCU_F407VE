# upper_vcu_st 의미 설명서

이 문서는 `upper_vcu_st`(VCU 내부 상태)의 각 멤버가 어떤 의미인지, 상위 제어기에서 어떻게 해석하면 되는지 예시 중심으로 설명합니다.

## 1) `upper_vcu_st`란?
- VCU가 현재 판단한 운전 상태를 담는 구조체입니다.
- 이 값은 CAN `0x18FF0310` 프레임으로 상위 제어기에 전달됩니다.
- 멀티바이트 정수(`int16/uint16`)는 **big-endian**(`MSB 먼저`)으로 전송됩니다.

## 2) 멤버별 의미

### `control_src`
- 현재 제어 권한 소스
- 내부 상태이며 `fsm_status_mask(data[5])` 구성에 사용
- 값:
  - `0`: 정지/무제어(`SRC_NONE`)
  - `1`: RC 제어(`SRC_RC`)
  - `2`: Upper 제어(`SRC_UPPER`)
  - `3`: Upper AUTO 제어(`SRC_UPPER_AUTO`)

### `stop_reason`
- 정지 원인 코드
- 내부 상태이며 `fsm_status_mask(data[5])` 해석 보조용
- 값:
  - `0`: 정지 원인 없음(`FSM_STOP_REASON_NONE`)
  - `1`: upper 강제 정지(`FSM_STOP_UPPER_FORCE`)
  - `2`: RC 비상정지(`FSM_STOP_RC_EMG`)
  - `3`: 모터 드라이버 fault(`FSM_STOP_MOTOR_FAULT`)
  - `4`: timeout(`FSM_STOP_TIMEOUT`)

### `power_supply_value`
- 전원 관련 값(현재 구현은 left driver 전압값 기반)
- `±RCM_MAX_DRIVER_INPUT`(현재 `±664`)로 clamp
- CAN `0x18FF0310 data[0:1]`

### `md_left_fault_msg`
- left 모터 드라이버 fault 코드
- CAN `data[2]`

### `md_right_fault_msg`
- right 모터 드라이버 fault 코드
- CAN `data[3]`

### `rc_status_mask`
- RC 상태 비트마스크
- CAN `data[4]`
- 비트 정의:
  - bit0: `RC_ST_ENABLE` (조종기 B 버튼)
  - bit1: `RC_ST_EMERGENCY_STOP` (조종기 A 버튼, E-STOP)
  - bit2: `RC_ST_FAILSAFE` (조종기 신호 끊김)
  - bit3: `RC_ST_FRESH` (실시간 수신 상태)
  - bit4: `RC_ST_CULTIVATOR_DOWN` (좌 토글)
  - bit5: `RC_ST_CULTIVATOR_ON` (우 토글)
  - bit6: `RC_ST_REMOTE_AUTOMATION` (조종기 D 버튼)
  - bit7: `RC_ST_DRIVE_MODE` (조종기 C 버튼, 주행 모드: `0=agile`, `1=stable`)
- 중요 구분:
  - `rc.valid`와 `rc.failsafe`는 동일 개념이 아님
  - `rc.failsafe`는 SBUS 수신 데이터가 전달하는 RC connect/disconnect 상태 신호
  - 따라서 `rc.valid`만으로 failsafe 상태를 대체 해석하면 안 됨

### `fsm_status_mask`
- FSM 상태 비트마스크
- CAN `data[5]`
- 비트 정의:
  - bit0: `FSM_ST_MODE_SAFE_STOP`
  - bit1: `FSM_ST_MODE_MANUAL_RC`
  - bit2: `FSM_ST_MODE_AUTO_ARMED`
  - bit3: `FSM_ST_MODE_AUTO_ACTIVE`
  - bit4: `FSM_ST_STOP_UPPER_FORCE`
  - bit5: `FSM_ST_STOP_RC_EMG`
  - bit6: `FSM_ST_STOP_MOTOR_FAULT`
  - bit7: `FSM_ST_STOP_TIMEOUT`

### `relay_st`
- 릴레이 동작 상태(bit mask)
- CAN `data[6]`

### `timeout_detail_code`
- timeout 세부 원인 코드
- CAN `data[7]`
- 코드:
  - `0`: `TO_NONE`
  - `1`: `TO_RC`
  - `2`: `TO_UPPER_CFG`
  - `3`: `TO_UPPER_DRIVE`
  - `4`: `TO_MOTOR_LEFT`
  - `5`: `TO_MOTOR_RIGHT`
  - `6`: `TO_MULTIPLE`

## 3) CAN 프레임 매핑 (`0x18FF0310`)
- `data[0:1]` = `power_supply_value` (`int16`, big-endian)
- `data[2]` = `md_left_fault_msg`
- `data[3]` = `md_right_fault_msg`
- `data[4]` = `rc_status_mask`
- `data[5]` = `fsm_status_mask`
- `data[6]` = `relay_st`
- `data[7]` = `timeout_detail_code`

## 4) Driver OK 조건
정상 주행 명령 유지 전제:
- `motor_left_ok = motor_left.valid && freshness(MOTOR_TIMEOUT_MS) && fault_bits==0`
- `motor_right_ok = motor_right.valid && freshness(MOTOR_TIMEOUT_MS) && fault_bits==0`

좌/우 중 하나라도 `driver_ok`가 아니면 FSM은 STOP 경로로 전환됩니다.

## 5) 빠른 해석 예시

### 예시 A: RC 정상 주행
- `control_src=1`
- `stop_reason=0`
- `rc_status_mask=0x09` (enable + fresh)
- `fsm_status_mask=0x02` (`MANUAL_RC`)

### 예시 B: Upper 강제 정지
- `control_src=0`
- `stop_reason=1`
- `fsm_status_mask`에 `SAFE_STOP + STOP_UPPER_FORCE`

### 예시 C: timeout 정지 + 원인 추적
- `stop_reason=4`
- `fsm_status_mask`에 `SAFE_STOP + STOP_TIMEOUT`
- `timeout_detail_code=3` 이면 upper drive timeout

## 6) 관련 프레임 참고
- `0x18FF0300`: 모터 RPM 피드백 (`±664` clamp)
- `0x18FF0320`: 차량 운동 상태(yaw/yaw rate/speed)
- `0x18FF0330`: 테스트/디버그 모니터
  - `data[4:5]`는 IMU gyro Z 기반 yaw rate(deg/s*10)를 우선 사용
  - IMU 값이 유효하지 않으면 명령 기반 yaw rate로 fallback
- `0x18FF0340`: weed actuator 상태(status/error/current/target/actual/speed)
- `0x18FF0350`: blade 상태(fault/cmd/source/rpm/meta)
- `0x18FF00C8`: weed actuator 피드백 RX(position/status/error/speed/input)
- `0x18FF0200`: Upper drive cmd (`throttle/steering + runtime max`)
- `0x18FF0210`: Upper config (`automation + relay + accel`)
