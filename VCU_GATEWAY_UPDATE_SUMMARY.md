# VCU Gateway Update Summary

## 1) 대상 파일
- `user/vcu_gateway.c`
- `user/vcu_gateway.h`

## 2) 제어 구조 변경 (Forward/Reverse -> Left/Right)
- 변수/구조체 이름을 물리 위치 기준으로 정리
  - `*_f_*`, `*_r_*` 계열을 `left/right`로 변경
- 좌/우 모터 상태/명령 흐름 가독성 개선

## 3) SBUS 차동구동 반영
- 조작 입력 기준
  - `CH3`: throttle (전/후진)
  - `CH1`: steering (좌/우)
- 차동구동 믹싱
  - `left = throttle + steering`
  - `right = throttle - steering`
- `rc_intent_t`에 추가
  - `left_rpm_value`
  - `right_rpm_value`
- RC 제어 시 각 측 바퀴 2개 동일 명령 적용
  - left driver axis1/axis2 = `left_rpm_value`
  - right driver axis1/axis2 = `right_rpm_value`

## 4) Upper Feedback CAN 프레임 구성

### 4.1 ExtID `0x18FF0300` (모터 드라이버 상태 피드백)
- 전송 주기: 100ms
- 포맷
  - `data[0:1]`: Driver1 Axis1
  - `data[2:3]`: Driver1 Axis2
  - `data[4:5]`: Driver2 Axis1
  - `data[6:7]`: Driver2 Axis2
- 소스 데이터
  - left/right motor status의 axis rpm 값을 사용
  - `-500 ~ 500` 범위로 클램프

### 4.2 ExtID `0x18FF0310` (VCU gateway 상태)
- 전송 주기: 100ms
- 포맷
  - `data[0:1]`: power supply (현재 left supply 기준, `-500 ~ 500` 클램프)
  - `data[2]`: fault message (Driver1/left)
  - `data[3]`: fault message (Driver2/right)
  - `data[4]`: RC status bit mask
  - `data[5]`: VCU FSM status bit mask
  - `data[6]`: relay status bit mask
  - `data[7]`: reserved

## 5) Bit Mask 정의

### 5.1 RC Status (`data[4]`)
- `RC_ST_ENABLE` `(1u << 0)`
- `RC_ST_EMERGENCY_STOP` `(1u << 1)`
- `RC_ST_FAILSAFE` `(1u << 2)`
- `RC_ST_FRESH` `(1u << 3)`
- `RC_ST_CULTIVATOR_DOWN` `(1u << 4)`
- `RC_ST_CULTIVATOR_ON` `(1u << 5)`

### 5.2 VCU FSM Status (`data[5]`)
- `VCU_ST_SRC_NONE` `(1u << 0)`
- `VCU_ST_SRC_RC` `(1u << 1)`
- `VCU_ST_SRC_UPPER` `(1u << 2)`
- `VCU_ST_STOP_UPPER` `(1u << 3)`
- `VCU_ST_STOP_RC_EMG` `(1u << 4)`
- `VCU_ST_STOP_MOTOR_FAULT` `(1u << 5)`
- `VCU_ST_STOP_TIMEOUT` `(1u << 6)`
- `VCU_ST_RUNNING` `(1u << 7)`

## 6) Upper -> Gateway CMD 파싱

### 6.1 SPEED 채널 (`0x18FF0200`)
- `upper_intent_rpm_t` 사용
- `data[0:1]`: Driver1 Axis1
- `data[2:3]`: Driver1 Axis2
- `data[4:5]`: Driver2 Axis1
- `data[6:7]`: Driver2 Axis2

### 6.2 설정 채널 (`0x18FF0210`)
- `upper_intent_t` 반영
  - `data[0]`: `driver_config_bitmask`
  - `data[1]`: `cultivator_down`
  - `data[2]`: `cultivator_on`
  - `data[3]`: `upper_force_stop`
  - `data[4]`: `upper_force_active`
  - `data[5]`: `relay_mask`
  - `data[6]`: `automation`
  - `data[7]`: reserved

## 7) 모터 드라이버 설정값 처리
- 좌/우 드라이버에 동일 설정 적용
- 기본 설정 define 추가 (`vcu_gateway.h`)
  - `MOTOR_DRV_DEFAULT_ENABLE_BITS`
  - `MOTOR_DRV_DEFAULT_AXIS1_ACC`
  - `MOTOR_DRV_DEFAULT_AXIS2_ACC`
- Upper에서 전달한 `driver_config_bitmask`는 아래 조건일 때만 적용
  - `(driver_config_bitmask & D0_ENABLE_MASK) == D0_EN_BOTH_ENABLE`
- 조건 미충족 시 기본 설정 유지

## 8) 좌/우 모터 명령 전송 분리
- 기존 단일 `motor_cmd` 대신
  - `motor_cmd_left`
  - `motor_cmd_right`
- CAN TX 시 각각 별도 패킹 후 송신
  - Driver1 TX <- `motor_cmd_left`
  - Driver2 TX <- `motor_cmd_right`

## 9) 기타 반영
- `can_send_ext()` 성공/실패 반환 로직 정상화(성공 시 `true`, 실패 시 `false`)
- CAN RX/TX 처리 성능 개선
  - `canrx`/`cantx` 스레드 분리 (RX 우선 처리)
  - `can_send_ext()` 블로킹 대기 축소(짧은 재시도)
  - CAN TX/RX MQ 메모리 풀 분리
- 테스트용 차동 믹서 API 공개
  - `vcu_diff_drive_mix(throttle, steering, &left, &right)`
  - 주행 중 스티어링 제한(`|steering| <= |throttle|`) 로직 포함
- `.gitignore` 추가
  - `Objects/`
  - `JLinkLog.txt`
  - `Listings/`
  - `project.uvguix.User`
  - `project.uvoptx`

## 10) 현재 확인 필요 포인트
- `power supply`를 left 기준으로 보낼지, left/right 평균/최소값 정책으로 보낼지 확정 필요
- `automation(data[6])` 사용처(FSM/상태 비트 반영 여부) 정책 확정 필요
- CAN TX 실패 시 재전송 정책(현재는 짧은 재시도 후 실패 반환) 확정 필요
