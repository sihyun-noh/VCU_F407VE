# TEST SHEET - VCU Gateway (Current Logic)

## 1. 목적
- VCU Gateway의 현재 구현 로직(RC/Upper/FSM/CAN TX-RX/Timeout/Fault)을 실차 또는 HIL 환경에서 검증한다.
- 판정 기준은 CAN 송신 프레임(`0x18FF0300`, `0x18FF0310`, `0x18FF0320`, `0x18FF0330`, `0x18FF0340`)과 FSM 비트/timeout detail 일치 여부로 한다.

## 2. 테스트 환경
- CAN bitrate: `500 kbps`
- Endian: 멀티바이트 정수 `big-endian`(MSB first)
- 주요 주기:
  - `fsm_thread`: 10ms
  - `can_tx_thread`: 100ms
- 주요 ID:
  - Upper->Gateway: `0x18FF0200`, `0x18FF0210`, `0x18FF0220`
  - Actuator->Gateway: `0x18FF00C8`
  - Gateway->Upper: `0x18FF0300`, `0x18FF0310`, `0x18FF0320`, `0x18FF0330`, `0x18FF0340`

## 3. 스케일/인코딩 기준
- 입력 정규화: `±RCM_MAX_RC_INPUT = ±500`
- 출력 정규화: `±RCM_MAX_DRIVER_INPUT = ±664`
- `0x18FF0200 data[6:7]` 속도값: `max_speed_kmh_x100`
  - 예: `5 km/h -> 500`

## 4. 공통 판정 포인트
- 정상 제어 시:
  - `0x18FF0310 data[5]`: `VCU_ST_RUNNING=1`
  - STOP 비트(`STOP_UPPER/RC_EMG/MOTOR_FAULT/TIMEOUT`)는 0
  - `0x18FF0310 data[7]`(`timeout_detail_code`) = `0(TO_NONE)`
- timeout 정지 시:
  - `VCU_ST_STOP_TIMEOUT=1`
  - `data[7]` 코드가 원인과 일치

## 5. 테스트 케이스

| TC ID | 항목 | 입력 조건 | 절차 | 기대 결과 (핵심) | 판정 |
|---|---|---|---|---|---|
| TC-001 | RC 정상 주행 | RC enable ON, estop OFF, failsafe OFF, fresh 유지 | throttle/steering 입력 | `0310:data[5]` SRC_RC+RUNNING, STOP 비트 0, `data[7]=0` | ☐ |
| TC-002 | Upper 정상 주행 | `0200` 주기 입력 + `0210` fresh | RC 비활성 상태에서 upper 명령 인가 | `0310:data[5]` SRC_UPPER+RUNNING, `data[7]=0` | ☐ |
| TC-002A | Upper AUTO 정상 주행 | `0220` 주기 입력 + `0210 automation=1` + RC remote automation | AUTO handover 성립 후 주행 | `0310:data[5]` AUTO_ACTIVE+RUNNING, `data[7]=0` | ☐ |
| TC-003 | Upper 강제 선택 | RC 활성 + `0210 upper_force_active=1` | RC 입력 유지 중 upper cmd 인가 | RC 활성이어도 SRC_UPPER 선택 | ☐ |
| TC-004 | Upper force stop | `0210 upper_force_stop=1` | force stop bit set | 즉시 STOP, `STOP_UPPER=1` | ☐ |
| TC-005 | RC E-STOP | RC estop(A 버튼) ON | RC 주행 중 estop ON | 즉시 STOP, `STOP_RC_EMG=1` | ☐ |
| TC-006 | RC timeout | RC 수신 중단(`SBUS_TIMEOUT_MS` 초과) | RC 입력 끊기 | `STOP_TIMEOUT=1`, `data[7]=TO_RC(1)` 또는 복합코드 | ☐ |
| TC-007 | Upper cfg timeout | `0210` 중단(`UPPER_TIMEOUT_MS` 초과) | Upper config 끊기 | Upper 경로 조건 미충족 시 timeout 정지 및 detail 반영 | ☐ |
| TC-008 | Upper drive timeout | `0200` 중단(`UPPER_DRIVE_TIMEOUT_MS` 초과) | Upper 주행 중 `0200` 중단 | `STOP_TIMEOUT=1`, `data[7]=TO_UPPER_DRIVE(3)` | ☐ |
| TC-009 | Motor left timeout | left motor status 중단 | left status 수신 중단 | STOP, `data[7]=TO_MOTOR_LEFT(4)` | ☐ |
| TC-010 | Motor right timeout | right motor status 중단 | right status 수신 중단 | STOP, `data[7]=TO_MOTOR_RIGHT(5)` | ☐ |
| TC-011 | Motor fault | fault_bits != 0 (좌/우) | fault 프레임 인가 | `STOP_MOTOR_FAULT=1`, `0310:data[2/3]` fault 반영 | ☐ |
| TC-012 | 0200 입력 clamp(+상한) | throttle/steering > +500 | 0200로 상한 초과 전송 | 내부 clamp 후 동작 (`+500` 기준) | ☐ |
| TC-013 | 0200 입력 clamp(-하한) | throttle/steering < -500 | 0200로 하한 초과 전송 | 내부 clamp 후 동작 (`-500` 기준) | ☐ |
| TC-014 | driver max runtime 적용 | `0200 data[4:5]=500` | upper 주행 | upper 믹서 출력 상한이 500 기준으로 제한 | ☐ |
| TC-015 | speed max runtime 적용 | `0200 data[6:7]=500` | upper 주행 | yaw/speed 모델이 5.00km/h 기준 반영 | ☐ |
| TC-016 | speed 인코딩 검증 | `0200 data[6:7]=500` vs `5` | 두 값 각각 입력 | `500`일 때가 5km/h, `5`는 0.05km/h로 해석됨 | ☐ |
| TC-017 | 0210 accel 적용(좌/우) | `0210 data[6]=L`, `data[7]=R` | upper_ok 상태로 적용 | motor cmd accel에 L/R 반영 | ☐ |
| TC-018 | 0210 accel fallback | upper_ok false | 0210 끊김 상태 | accel 기본값 `0x64` 복귀 | ☐ |
| TC-019 | 0300 피드백 맵 검증 | motor status 정상 입력 | 수신->송신 연계 확인 | `0300`의 axis1/2 값 매핑/부호 일치 | ☐ |
| TC-020 | 0310 timeout detail 복합 | 복수 timeout 유도 | RC+Upper 동시 끊김 등 | `data[7]=TO_MULTIPLE(6)` | ☐ |
| TC-021 | MPU yaw 반영(0330) | MPU thread 동작 + gyro z 유효 | steering 입력/회전 후 `0330` 확인 | `0330:data[4:5]`가 IMU yaw rate(deg/s x10) 우선 반영 | ☐ |
| TC-022 | MPU fallback 검증 | MPU 미동작/무효 | 동일 조건에서 `0330` 확인 | `0330:data[4:5]`가 명령기반 yaw rate fallback 반영 | ☐ |
| TC-023 | Actuator RX 파싱(00C8) | `00C8` 주기 수신 | position/status/error/speed 값을 변경 전송 | 내부 status가 `0340`으로 반영됨 | ☐ |
| TC-024 | 위치도달 기반 pre 재무장 | target 변경 + 실제 position 도달 | target 전환 2회 반복 | 도달 이후 다음 target에서 pre 명령이 선행 송신됨 | ☐ |
| TC-025 | Actuator RX timeout 보고 | `00C8` 중단(`WEED_ACTUATOR_TIMEOUT_MS` 초과) | actuator 피드백 중단 | `0340:data[4]` timeout bit set | ☐ |

## 6. bitmask 빠른 체크표

### RC status (`0x18FF0310 data[4]`)
- bit0: ENABLE (B)
- bit1: EMG STOP (A)
- bit2: FAILSAFE
- bit3: FRESH
- bit4: CULTIVATOR_DOWN (left toggle)
- bit5: CULTIVATOR_ON (right toggle)
- bit6: REMOTE_AUTOMATION (D)

### VCU FSM status (`0x18FF0310 data[5]`)
- bit0: SRC_NONE
- bit1: SRC_RC
- bit2: SRC_UPPER
- bit3: STOP_UPPER
- bit4: STOP_RC_EMG
- bit5: STOP_MOTOR_FAULT
- bit6: STOP_TIMEOUT
- bit7: RUNNING

### Timeout detail (`0x18FF0310 data[7]`)
- 0: TO_NONE
- 1: TO_RC
- 2: TO_UPPER_CFG
- 3: TO_UPPER_DRIVE
- 4: TO_MOTOR_LEFT
- 5: TO_MOTOR_RIGHT
- 6: TO_MULTIPLE

## 7. 실행 로그 템플릿
- 일시:
- 시험자:
- SW Commit:
- 환경(실차/HIL):
- 비고:

| TC ID | 결과(P/F) | 관측 CAN 로그 요약 | 이슈/메모 |
|---|---|---|---|
| TC-001 |  |  |  |
| TC-002 |  |  |  |
| ... |  |  |  |
