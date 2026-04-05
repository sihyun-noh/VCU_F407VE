# VCU Gateway 최근 변경 요약 (최신)

## 0. 2026-04-05 업데이트

### 0.1 RC Mixer 민첩형(Dex) 확장 추가
- 대상 파일:
  - `user/rc_mixer.c`
  - `user/rc_mixer.h`
- 변경 내용:
  - 기존 `apply_turn_shaping()`를 base 로직으로 정리
  - 선택형 민첩형 보정 함수 `apply_turn_shaping_dex()` 추가
  - 동작 조건:
    - `throttle != 0`
    - `|steering| > |throttle|`
  - 목표:
    - inner 비율은 최대 `RCM_DEX_INNER_MIN`(기본 `-0.4`)까지 허용
    - outer 비율은 `1.0 ~ 1.4` 범위까지 확장 가능

### 0.2 Dex 튜닝 파라미터 추가
- `RCM_DEX_ENABLE` (`0`=OFF, `1`=ON)
- `RCM_DEX_INNER_MIN` (기본 `-0.4`)
- `RCM_DEX_OUTER_MAX` (기본 `1.3`, 적용 범위 `1.0~1.4`)
- `RCM_DEX_BLEND_GAIN` (blend 감도)

### 0.3 디버깅 상태값 확장
- `calc_state_t`에 아래 필드 추가:
  - `dex_over`
  - `dex_applied`
- 목적:
  - Dex 조건 진입 여부와 과조향(over) 정도를 로그/모니터링으로 확인

### 0.4 기본 동작 호환성
- `RCM_DEX_ENABLE=0` 기본값이므로 현재 운영 기본 동작은 기존 base mixer와 동일 유지
- Dex는 필요 시에만 활성화하여 테스트 가능

## 1. 적용 파일
- `user/vcu_gateway.h`
- `user/vcu_gateway.c`
- `user/rc_mixer.h`
- `user/rc_mixer.c`
- `UPPER_GATEWAY_CAN_SPEC_DRAFT.md`
- `UPPER_GATEWAY_CAN_SPEC_DRAFT_KR.md`
- `UPPER_VCU_STATUS_GUIDE.md`

## 2. 핵심 변경

### 2.1 CAN 스케일 통합
- 기존 `CMD_MIN/CMD_MAX` 사용 제거
- 입력 정규화 기준 통합:
  - `RCM_MAX_RC_INPUT = ±500`
- 출력/드라이버 기준 통합:
  - `RCM_MAX_DRIVER_INPUT = ±664`

### 2.2 `0x18FF0200` Drive CMD 확장
- 기존:
  - `data[0:1] throttle_cmd`
  - `data[2:3] steering_cmd`
  - `data[4:7] reserved`
- 현재:
  - `data[0:1] throttle_cmd (int16)`
  - `data[2:3] steering_cmd (int16)`
  - `data[4:5] max_driver_input_cmd (uint16)`
  - `data[6:7] max_speed_kmh_x100 (uint16)`
- 적용 규칙:
  - `max_driver_input_cmd > 0`이면 upper 믹서 `max_driver_input` 런타임 적용
  - `max_speed_kmh_x100 > 0`이면 upper 믹서 `max_speed_kmh` 런타임 적용
  - speed 인코딩: `km/h * 100` (예: `5 km/h -> 500`)

### 2.3 `0x18FF0210` Config CMD 정리
- `data[0] automation` 복구
- `data[6] left_accel_cmd`, `data[7] right_accel_cmd`
- `driver_config_bitmask` override 제거
- 드라이버 enable 비트는 항상 기본값 고정:
  - `MOTOR_DRV_DEFAULT_ENABLE_BITS = 0xC3`
- accel은 `upper_ok`일 때만 upper 값 적용, 아니면 기본값(`0x64`) 사용

### 2.4 FSM 우선순위 보강
- STOP 우선 처리 유지
- STOP이 아닐 때:
  - `upper_force_active=1`이면 RC 활성이어도 Upper 경로 강제 선택
  - 아니면 RC 우선
  - 아니면 Upper

### 2.5 timeout 상세 원인 코드 추가 (`0x18FF0310 data[7]`)
- `timeout_detail_code` 추가
- 코드:
  - `0 TO_NONE`
  - `1 TO_RC`
  - `2 TO_UPPER_CFG`
  - `3 TO_UPPER_DRIVE`
  - `4 TO_MOTOR_LEFT`
  - `5 TO_MOTOR_RIGHT`
  - `6 TO_MULTIPLE`

### 2.6 right motor timeout 처리 반영
- `motor_right_ok`도 STOP 판정 경로에 반영
- right timeout 발생 시 `TO_MOTOR_RIGHT` 설정

### 2.7 RC status/의미 보강
- `RC_ST_REMOTE_AUTOMATION(bit6)` 반영
- 의미 명시:
  - `ENABLE = B 버튼`
  - `E-STOP = A 버튼`
  - `FAILSAFE = 조종기 신호 끊김`
  - `CULTIVATOR_DOWN = 좌 토글`
  - `CULTIVATOR_ON = 우 토글`
  - `REMOTE_AUTOMATION = D 버튼`

### 2.8 문서 최신화
- 영문/한글 CAN 스펙 초안 동기화
- Endian 규칙 명시:
  - 멀티바이트 정수는 big-endian (`MSB first`)
- Driver OK 조건 및 timeout 동작 설명 추가
- CAN 속도 명시:
  - `500 kbps`

## 3. 비고
- 모니터링(`0x18FF0320/0330`)은 현재 명령 기반 적분값임
- 실측 RPM 기반으로 바꾸려면 `update_motion_monitor()` 입력 소스 변경 필요
