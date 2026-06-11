# VCU Gateway Reference

이 문서는 VCU Gateway 프로젝트의 현재 구조/인터페이스/튜닝 포인트를 빠르게 참조하기 위한 레퍼런스입니다.

## 1) 적용 범위
- `user/vcu_gateway.c`
- `user/vcu_gateway.h`
- `user/rc_mixer.c`
- `user/rc_mixer.h`

## 2) 런타임 구조 (Thread)
- `sbus_thread` (입력 전처리 스레드)
  - 역할: SBUS 원시 입력을 \"제어 가능한 RC 데이터\"로 변환
  - 입력: SBUS 25B 프레임
  - 처리:
    - 채널 디코딩
    - 정규화 변환(`axis1~axis4`)
    - 필터링(이동평균)
    - RC 메타데이터 생성(`valid`, `failsafe`, 스위치 상태)
    - RC 기반 좌/우 명령 계산(`mix_rc_to_tracks`)
  - 출력: `g_latest.rc` 갱신

- `fsm_thread` (제어 결정 스레드, 주기 10ms)
  - 역할: 현재 최신 입력(RC/Upper/Motor)을 검증하고 최종 구동 명령 결정
  - 입력: `g_latest.rc`, `g_latest.upper_cmd_config`, `g_latest.upper_cmd_drive`, `g_latest.motor_*`
  - 처리:
    - valid/freshness/fault 기반 신뢰성 확인
    - STOP 우선순위 적용
    - RC 제어 / Upper 제어 경로 선택
    - weed FSM(`weed_fsm_step`)에서 actuator pre/periodic 판단 및 pending frame 생성
    - blade FSM(`blade_fsm_step`)에서 목표 rpm 결정 + pending frame 생성(RC B enable gate)
    - 모터 cmd, 상위 보고 status, timeout detail 계산
  - 출력:
    - `g_latest.motor_cmd_left/right`
    - `g_latest.upper_vcu_st`, `g_latest.upper_rpm_st`
    - `g_latest.motion_monitor`

- `can_rx_thread` (CAN 수신 파싱 스레드)
  - 역할: 수신 CAN 프레임을 의미 있는 내부 구조체로 변환
  - 입력: CAN RX 큐 프레임
  - 처리:
    - `0x18FF0200` drive 명령 파싱
    - `0x18FF0210` config 명령 파싱
    - `0x18FF0220` auto direct 명령 파싱(`0x18FF0210 data[6] bit0=1`일 때 사용)
    - `0x18FF0230` weed actuator 명령 파싱
    - `0x18FF0240` weed blade 명령 파싱
    - `0x18FF00C8` weed actuator feedback 파싱
    - `0x18FF0032` blade left status 파싱
    - `0x18FF0030` blade right status 파싱
    - `0x18FF0021/0020` 모터 상태 파싱
  - 출력: `g_latest.upper_*`, `g_latest.motor_*` 갱신

- `can_tx_thread` (CAN 송신 스레드, control/pending TX base 100ms, Upper status 200ms)
  - 역할: FSM에서 결정된 최신 상태/명령을 CAN으로 주기 송신 (판단 로직 없음, 송신 전담)
  - 입력: `g_latest.motor_cmd_*`, `g_latest.upper_*`, `g_latest.motion_monitor`, `g_latest.rc`
  - 송신:
    - 모터 명령(`0x18FF2100`, `0x18FF2000`)
    - 상위 상태(`0x18FF0310`, `0x18FF0300`)
    - weed actuator 상태(`0x18FF0320`)
    - blade 상태(`0x18FF0330`)
    - 차량 모니터링(`0x18FF4000`, `0x18FF4010`)
    - weed actuator 제어(`0x18EFC800`, pending frame만 전송)
    - blade 제어(`0x18FF3200`, `0x18FF3000`, 250ms periodic)

## 2-1) 모듈 책임 경계 (권장 구조)

### A. SBUS 입력/가공 계층 (`sbus_thread`)
- 목적: 차량 제어에 필요한 입력 데이터를 \"raw -> 가공\" 형태로 정리
- 처리 항목:
  - SBUS raw frame 수신/디코딩
  - 채널 정규화 변환(입력 스케일 `±RCM_MAX_RC_INPUT`)
  - 필터 적용(예: moving average)
  - 메타데이터 생성:
    - `rc.valid`
    - `rc.failsafe` (RC connect/disconnect 상태)
    - freshness/스위치 상태(enable, e-stop, cultivator, remote automation)
- 결과: FSM/믹서가 바로 사용할 수 있는 `rc_intent` 최신값 갱신

### B. 제어 계산 계층 (`rc_mixer`)
- 목적: throttle/steering -> 좌/우 구동 명령 계산
- 처리 항목:
  - 차동 믹싱
  - 고속 조향 제한
  - 급선회(inner/outer) 보정
  - 출력 saturation (`±RCM_MAX_DRIVER_INPUT`)
- 결과: `left_input/right_input` 계산 결과 반환

### C. 제어 결정 계층 (`fsm_thread`)
- 목적: 신뢰성 검증 후 제어 소스 최종 결정
- 처리 항목:
  - RC/Upper/Motor의 valid/freshness/fault 기반 게이트
  - STOP 우선순위 처리
  - 최종 motor cmd, upper status, timeout detail 결정

### D. CAN I/O 계층 (`can_rx_thread`, `can_tx_thread`)
- 목적: 인터페이스 입출력 분리 및 주기 송신 보장
- 처리 항목:
  - RX 파싱 및 최신 상태 저장
  - TX 주기 송신(명령/상태/모니터링)

## 3) 통신 기본
- CAN bitrate: `500 kbps`
- 멀티바이트 정수: **big-endian** (`MSB first`)
- 스케일 통합:
  - 입력: `±RCM_MAX_RC_INPUT` (현재 `±500`)
  - 출력: `±RCM_MAX_DRIVER_INPUT` (현재 `±664`)

## 4) CAN 인터페이스 (Gateway 기준)

### RX
- `0x18FF0200` (Upper -> Gateway drive)
  - `data[0:1]` `throttle_cmd` (`int16`)
  - `data[2:3]` `steering_cmd` (`int16`)
  - `data[4:5]` `max_driver_input_cmd` (`uint16`)
  - `data[6:7]` `max_speed_kmh_x100` (`uint16`)
- `0x18FF0210` (Upper -> Gateway config)
  - `data[0]` `automation`
  - `data[1]` `upper_force_stop`
  - `data[2]` `upper_force_active`
  - `data[3]` `relay_mask`
  - `data[4]` `left_accel_cmd`
  - `data[5]` `right_accel_cmd`
  - `data[6]` `upper_drive_cmd_select` (`bit0=0`: `0x18FF0200`, `bit0=1`: `0x18FF0220`)
  - `data[7]` reserved
- `0x18FF0220` (Upper -> Gateway auto direct drive, 선택 시 사용)
  - `data[0:1]` `left_driver_input_cmd` (`int16`)
  - `data[2:3]` `right_driver_input_cmd` (`int16`)
  - `data[4:5]` `max_driver_input_cmd` (`uint16`)
  - `data[6:7]` `max_speed_kmh_x100` (`uint16`)
- `0x18FF0230` (Upper -> Gateway weed actuator cmd)
  - `data[0]` `command_type` (`0 STOP`, `1 SET_TARGET`, `2 MOVE_TO_TARGET`)
  - `data[1]` `stage` (`0 UP`, `1 MID`, `2 DOWN`)
  - `data[2:3]` `target_position_mm` (`uint16`)
- `0x18FF0240` (Upper -> Gateway weed blade cmd)
  - `data[0]` `command_type` (`0 STOP`, `1 SET_RPM`, `2 RUN`)
  - `data[1]` `mode` (`0 SYNC`)
  - `data[2:3]` `left_blade_rpm` (`uint16`)
  - `data[4:5]` `right_blade_rpm` (`uint16`)
- `0x18FF0021` left motor status
- `0x18FF0020` right motor status
- `0x18FF00C8` weed actuator feedback
- `0x18FF0032` blade left status
- `0x18FF0030` blade right status

### TX
- `0x18FF2100` Gateway -> Driver1(left) cmd
- `0x18FF2000` Gateway -> Driver2(right) cmd
- `0x18FF0300` motor feedback rpm/status
- `0x18FF0310` gateway status (`upper_vcu_st`)
- `0x18FF0320` weed actuator status
- `0x18FF0330` weed blade status
- `0x18FF4000` vehicle motion status(test/debug)
- `0x18FF4010` vehicle monitor/debug status(test/debug)
- `0x18EFC800` weed actuator command
- `0x18FF3200` blade left command
- `0x18FF3000` blade right command

## 5) FSM 제어 우선순위
- STOP 우선순위:
  1. `upper_force_stop`
  2. `rc_emergency_stop`
  3. motor fault/timeout
- STOP이 아니면:
  - `upper_force_active=true` 또는 auto handover 성립 -> Upper drive(`0x18FF0200`) 선택
  - else `RC valid + rc_enable` -> RC 선택
  - else -> STOP(timeout)

### FSM 타임아웃/신뢰성 게이트 설명
- FSM은 "수신된 최신 데이터의 신뢰성"을 먼저 확인한 뒤 제어 경로를 선택합니다.
- 즉, 인터페이스 입력이 유효(`valid`)하고 신선도(`freshness`)를 만족해야만 정상 제어를 지속합니다.

신뢰성 체크 항목:
- RC(SBUS):
  - `rc.valid == true` 이어야 함
  - `SBUS_TIMEOUT_MS` 이내 최신 프레임이어야 함 (`RC_ST_FRESH`)
  - `rc.failsafe`는 `rc.valid`와 별개 개념:
    - failsafe는 수신된 SBUS 데이터가 \"RC disconnect/ connect 상태\"를 나타내는 신호
    - 따라서 `rc.valid`만으로 failsafe 상태를 대체 해석하면 안 됨
- Upper(CAN):
  - config 경로: `upper.valid` (config freshness timeout은 hard stop gate로 미사용)
  - drive 경로: `upper_drive.valid && UPPER_DRIVE_TIMEOUT_MS 이내`
- Motor status(CAN):
  - 좌/우 각각 `valid && MOTOR_TIMEOUT_MS 이내 && fault_bits==0`

결론:
- 위 조건 중 제어에 필요한 입력 신뢰성이 깨지면 FSM은 STOP 경로로 전환합니다.
- STOP 원인은 `vcu_fsm_status_mask(data[5])`와 `timeout_detail_code(data[7])`로 상위에 보고됩니다.
- 참고: `upper.automation`은 `upper_ok` 게이트가 아니라, 현재 릴레이 자동화 동작 판단(`upper.automation && rc.rc_remote_automation`)에 사용됩니다.

### Driver OK 조건
- `motor_left_ok = valid && fresh(MOTOR_TIMEOUT_MS) && fault_bits==0`
- `motor_right_ok = valid && fresh(MOTOR_TIMEOUT_MS) && fault_bits==0`
- 하나라도 실패 시 STOP 경로

## 6) 모터 설정 정책
- 본 시스템은 모터를 직접 제어하지 않고 **motor driver**를 통해 제어합니다.
- 각 driver는 **2개 축(axis1, axis2) 출력**을 가집니다.
- 현재 명령 형태는 **speed 모드 기반 cmd**이며, 핵심 제어값은 다음 2가지입니다.
  - `speed` (`rpm_axis1`, `rpm_axis2`): 목표 회전 속도 명령
  - `accel` (`axis1_accel_bit`, `axis2_accel_bit`): 가속도 관련 명령

- `enable_bit`는 항상 기본값 사용:
  - `MOTOR_DRV_DEFAULT_ENABLE_BITS = 0xC3`
- accel:
  - `upper.valid`일 때 `0x18FF0210 data[4:5]` 적용
  - 아니면 기본 accel `0x64`

### Runtime limit 적용 정책 (`0x18FF0200 data[4:7]`)
- `max_driver_input_cmd`, `max_speed_kmh_x100`는 **Upper 제어 경로에서만** 적용됩니다.
- RC 제어 경로는 `g_rcm_vehicle` 기본값(`RCM_MAX_DRIVER_INPUT`, `RCM_MAX_SPEED_KMH`)을 사용합니다.
- 의도:
  - RC 수동 운전 체감/안전 일관성 유지
  - Upper 자동/시험 제어에서만 런타임 튜닝 허용
- 따라서 RC와 Upper의 런타임 제한값을 항상 동기화하지는 않습니다(정책적으로 분리).

관련 define 의미 (`vcu_gateway.h`):
- `D0_AXIS1_SPEED_MODE`, `D0_AXIS2_SPEED_MODE`
  - 각 축을 speed 모드(`1`)로 동작시키는 비트
- `D0_EN_BOTH_ENABLE`
  - axis1/axis2 모두 enable
- `MOTOR_DRV_DEFAULT_ENABLE_BITS`
  - 기본 동작 비트 조합
  - 현재값 `0xC3 = D0_EN_BOTH_ENABLE | D0_AXIS1_SPEED_MODE | D0_AXIS2_SPEED_MODE`
- `MOTOR_DRV_DEFAULT_AXIS1_ACC`, `MOTOR_DRV_DEFAULT_AXIS2_ACC`
  - upper accel 명령이 없거나 유효하지 않을 때 사용하는 기본 accel 값(현재 `0x64`)

## 7) 모터 상태값 수신/상위 전달
- 모터 드라이버 상태는 CAN RX(`0x18FF0021`, `0x18FF0020`)로 수신됩니다.
- 현장 설정 기준으로 모터 드라이버는 보통 `100ms` 간격으로 상태를 송신합니다.
- 수신 항목(현재 decode 기준):
  - `fault_bits` (`data[0]`)
  - `temperature` (`data[1]`)
  - `rpm_axis2` (`data[2:3]`)
  - `rpm_axis1` (`data[4:5]`)
  - `supply_volt` (`data[6:7]`)

상위 제어기로 전달되는 항목:
- `0x18FF0300`:
  - 좌/우 driver의 axis1/axis2 RPM 피드백 전달
- `0x18FF0310`:
  - 좌/우 driver fault(`data[2]`, `data[3]`)
  - power_supply_value(`data[0:1]`, 현재 left supply 기반)

참고:
- `temperature`는 현재 내부에서 수신/보관되지만, 상위 송신 payload에는 직접 매핑되어 있지 않습니다.

### Fault code 의미 (사용자 제공 스크린샷 기준)
- `0`: normal
- `1~4`: Drive failures
- `5`: overcurrent
- `6`: overvoltage
- `7`: undervoltage
- `8`: Overweight
- `11`: Motor 2 overspeed
- `12`: Motor 1 overspeed
- `13`: Motor 2 overload
- `14`: Motor 1 overload
- `15`: Motor 2 is out of phase
- `16`: Motor 1 is out of phase
- `17`: Motor 2 trip
- `18`: Motor 1 trip
- `19`: Motor 2 encoder
- `20`: Motor 1 encoder
- `21`: Motor 2 overheating
- `22`: Motor 1 overheating
- `23`: Motor 2 Hall fault
- `24`: Motor 1 Hall fault
- `25`: Motor 2 stall
- `26`: Motor 1 stall
- `27`: UART communication failure
- `28`: RS485 communication failure
- `29`: CAN communication failure
- `30`: Rakebar failure
- `31`: Switching faults

## 8) timeout 상세 코드 (`0x18FF0310 data[7]`)
- `0`: `TO_NONE`
- `1`: `TO_RC`
- `2`: `TO_UPPER_CFG`
- `3`: `TO_UPPER_DRIVE`
- `4`: `TO_MOTOR_LEFT`
- `5`: `TO_MOTOR_RIGHT`
- `6`: `TO_MULTIPLE`
- `7`: `TO_UPPER_AUTO` (`0x18FF0220` selected command timeout)

## 9) RC 상태 비트 (`0x18FF0310 data[4]`)
- bit0 `RC_ST_ENABLE` (B 버튼)
- bit1 `RC_ST_EMERGENCY_STOP` (A 버튼)
- bit2 `RC_ST_FAILSAFE` (신호 끊김)
- bit3 `RC_ST_FRESH` (실시간 수신)
- bit4 `RC_ST_CULTIVATOR_DOWN` (좌 토글)
- bit5 `RC_ST_CULTIVATOR_ON` (우 토글)
- bit6 `RC_ST_REMOTE_AUTOMATION` (D 버튼)
- bit7 `RC_ST_DRIVE_MODE` (C 버튼, `0=민첩형`, `1=안정형`)

## 10) VCU FSM 비트 (`0x18FF0310 data[5]`)
- bit0 `FSM_ST_MODE_SAFE_STOP`
- bit1 `FSM_ST_MODE_MANUAL_RC`
- bit2 `FSM_ST_MODE_AUTO_ARMED`
- bit3 `FSM_ST_MODE_AUTO_ACTIVE`
- bit4 `FSM_ST_STOP_UPPER_FORCE`
- bit5 `FSM_ST_STOP_RC_EMG`
- bit6 `FSM_ST_STOP_MOTOR_FAULT`
- bit7 `FSM_ST_STOP_TIMEOUT`

비트 set 조건(VCU Gateway 상태 기준):
- `bit0 FSM_ST_MODE_SAFE_STOP`
  - STOP 경로이거나 stop reason 존재 시 set
- `bit1 FSM_ST_MODE_MANUAL_RC`
  - RC 경로가 선택된 정상 주행 구간에서 set
- `bit2 FSM_ST_MODE_AUTO_ARMED`
  - RC remote automation 요청은 있으나 upper auto handover 미성립 시 set
- `bit3 FSM_ST_MODE_AUTO_ACTIVE`
  - Upper AUTO 제어 경로 활성 시 set
- `bit4 FSM_ST_STOP_UPPER_FORCE`
  - `upper_force_stop=1` 정지 시 set
- `bit5 FSM_ST_STOP_RC_EMG`
  - RC 비상정지(`rc_emergency_stop=1`) 시 set
- `bit6 FSM_ST_STOP_MOTOR_FAULT`
  - 모터 fault로 정지 시 set
- `bit7 FSM_ST_STOP_TIMEOUT`
  - freshness timeout 계열 정지 시 set
  - 원인 상세는 `data[7] timeout_detail_code`(`TO_RC`, `TO_UPPER_*`, `TO_MOTOR_*`)로 구분

## 11) 모니터링 계산 기준
- 차량 모니터링은 `0x18FF4000`, `0x18FF4010`으로 이동됨
- `0x18FF0320`, `0x18FF0330`은 각각 weed actuator/blade 상위 상태 보고에 사용
- 차량 motion monitor 값은 현재 명령값(out_cmd) 기반 적분값이며 실측 RPM 기반이 아님

## 12) 관련 문서
- `UPPER_GATEWAY_CAN_SPEC_DRAFT.md`
- `UPPER_GATEWAY_CAN_SPEC_DRAFT_KR.md`
- `UPPER_VCU_STATUS_GUIDE.md`
- `RC_MIXER_FLOW.md`
