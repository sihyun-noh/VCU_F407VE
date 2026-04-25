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

### 0.5 RC Mixer Gamma Shaping 추가 (옵션)
- 대상 파일:
  - `user/rc_mixer.c`
  - `user/rc_mixer.h`
- 변경 내용:
  - 선형 조향 응답 보정을 위한 gamma shaping 경로 추가
  - 계산식:
    - `beta_shaped = pow(beta_abs, gamma)` (옵션 ON일 때)
    - `inner = 1 - beta_shaped`
    - `outer = 1 - k * beta_shaped`
  - 기존 turn direction(좌/우 할당) 로직은 변경 없음
- 설정/제약:
  - feature flag: `RCM_MIXER_FEAT_GAMMA`
  - 기본 플래그: `RCM_MIXER_FEATURE_FLAGS = 0` (OFF)
  - gamma 범위: `0.5 ~ 3.0`
  - 기본값: `1.0`
- 호환성 보장:
  - gamma OFF: 기존 base 결과와 동일
  - gamma ON + `gamma=1.0`: 기존 선형 결과와 동일
- 디버깅 확장:
  - `calc_state_t`에 `beta_raw`, `beta_shaped`, `gamma`, `gamma_enabled` 추가
  - `rcm_mixer_debug_t` 구조체 추가

### 0.6 RC C버튼 기반 주행 모드 분리 가독성 개선
- 대상 파일:
  - `user/vcu_gateway.c`
  - `user/rc_mixer.c`
  - `user/rc_mixer.h`
- 변경 내용:
  - `CH10`(RC C 버튼)으로 전달되는 `rc_drive_mode` 의미를 코드 주석으로 명확화
  - mixer 내부 모드 분기 가독성 개선:
    - `stable_mode=true`  : 고속 조향 제한 ON, DEX OFF
    - `agile_mode=false` : 고속 조향 제한 OFF, DEX ON
  - 함수 파라미터 오타 수정:
    - `deive_mode` -> `drive_mode`
- 안정성 보강:
  - stable 모드 경로에서도 `dex_over_dbg`, `dex_applied_dbg`가 항상 초기화되도록 보정

### 0.7 FSM 제어 우선순위/자동전환 게이트 정책 갱신
- 대상 파일:
  - `user/vcu_gateway.c`
  - `UPPER_GATEWAY_CAN_SPEC_DRAFT.md`
  - `UPPER_GATEWAY_CAN_SPEC_DRAFT_KR.md`
- 변경 내용:
  - 기본 우선순위를 RC 중심으로 명확화
    - STOP 조건(upper_force_stop / RC E-stop / motor fault·timeout) 유지
    - STOP이 아닐 때: RC 우선, Upper는 강제 선택 또는 자동전환 게이트 성립 시 진입
  - 자동전환(hand-over) 게이트 추가
    - 조건: `rc_ok && rc_enable && rc_remote_automation && upper.valid && upper.automation`
    - 조건 성립 시에만 Upper 자동 제어 진입 허용
  - `upper config timeout`은 hard stop 판정에서 제외
    - config 적용 유효성은 `upper.valid` 기준
    - timeout detail 생성 시 `TO_UPPER_CFG`는 현재 hard timeout 경로에서 미사용(예약)
  - relay automation flag도 동일 게이트(`upper_auto_ready`) 기준으로 동작하도록 정합화

### 0.8 FSM 상태 비트마스크(`0x18FF0310 data[5]`) 재정의
- 대상 파일:
  - `user/vcu_gateway.h`
  - `user/vcu_gateway.c`
  - `UPPER_GATEWAY_CAN_SPEC_DRAFT.md`
  - `UPPER_GATEWAY_CAN_SPEC_DRAFT_KR.md`
  - `UPPER_VCU_STATUS_GUIDE.md`
- 변경 내용:
  - 기존 `VCU_ST_*` 중심 비트 정의를 `FSM_ST_*` 중심으로 재구성
  - mode 비트(one-hot) 추가:
    - `FSM_ST_MODE_SAFE_STOP`(bit0)
    - `FSM_ST_MODE_MANUAL_RC`(bit1)
    - `FSM_ST_MODE_AUTO_ARMED`(bit2)
    - `FSM_ST_MODE_AUTO_ACTIVE`(bit3)
  - stop reason 비트:
    - `FSM_ST_STOP_UPPER_FORCE`(bit4)
    - `FSM_ST_STOP_RC_EMG`(bit5)
    - `FSM_ST_STOP_MOTOR_FAULT`(bit6)
    - `FSM_ST_STOP_TIMEOUT`(bit7)
  - `decide_fsm_mode()` + `pack_fsm_status_mask()`로 data[5] 생성 로직 명확화
  - 기존 `VCU_ST_*` 매크로는 하위 호환 alias로 유지

### 0.9 Upper AUTO CMD + MPU 모니터링 연동
- 대상 파일:
  - `user/vcu_gateway.h`
  - `user/vcu_gateway.c`
  - `user/main.c`
  - `UPPER_GATEWAY_CAN_SPEC_DRAFT.md`
  - `UPPER_GATEWAY_CAN_SPEC_DRAFT_KR.md`
  - `UPPER_VCU_STATUS_GUIDE.md`
- 변경 내용:
  - 신규 Upper 자동주행 명령 ID 추가:
    - `0x18FF0220` (`linear_mps_x1000`, `yaw_rate_deg_s_x10`)
  - Upper 제어 경로 통일:
    - `force_upper_active` / auto handover 모두 `0x18FF0220` 기반 경로 사용
  - AUTO 혼합 방식 선택 매크로 추가 (`vcu_gateway.h`):
    - `UPPER_AUTO_MIX_MODE_KINEMATIC`
    - `UPPER_AUTO_MIX_MODE_RC_MIXER`
    - `UPPER_AUTO_MIX_MODE`로 선택
  - MPU6050 모니터링 연결:
    - `main`에서 `bsp_MPU6050_thread()` 활성화
    - `vcu_motion_monitor_t`에 `imu_yaw_rate_deg_s`, `imu_yaw_rate_valid` 추가
    - `0x18FF0330 data[4:5]`는 IMU gyro Z 기반 yaw rate 우선, 미유효 시 명령 기반 fallback

### 1.0 Weed actuator RX/Status 연동 + FSM/TX 역할 분리 (2026-04-26)
- 대상 파일:
  - `user/vcu_gateway.h`
  - `user/vcu_gateway.c`
  - `hardware/CAN.c`
  - `UPPER_GATEWAY_CAN_SPEC_DRAFT.md`
  - `UPPER_GATEWAY_CAN_SPEC_DRAFT_KR.md`
  - `UPPER_VCU_STATUS_GUIDE.md`
  - `VCU_GATEWAY_REFERENCE.md`
  - `TEST_SHEET_VCU_GATEWAY.md`
- 변경 내용:
  - 신규 weed actuator RX ID 추가:
    - `0x18FF00C8` (`CYL -> VCU`)
    - 필드 파싱: position/current/status flags/error/speed/input
  - 신규 upper weed status TX ID 추가:
    - `0x18FF0340` (`Gateway -> Upper`)
    - status/error/current/input/meta/target/actual/speed 보고
  - actuator pre-command 재무장 정책 보강:
    - 실제 position이 target 근접(`WEED_ACTUATOR_POS_TOL_MM`) 시 `pre_sent` 재무장
  - 구조 개선:
    - weed 판단은 `weed_fsm_step()`에서 수행
    - `can_tx_thread`는 pending frame 송신 전용으로 정리
  - CAN1 필터 반영:
    - `hardware/CAN.c` filter #4를 `0x18FF00C8` 수신용으로 설정

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
