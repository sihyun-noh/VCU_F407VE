# VCU Gateway 최근 변경 요약

## 1. 대상 파일
- `user/vcu_gateway.h`
- `user/vcu_gateway.c`
- `user/rc_mixer.h`
- `user/rc_mixer.c`
- `RC_MIXER_FLOW.md`
- `UPPER_VCU_STATUS_GUIDE.md`

## 2. 핵심 변경 사항

주의:
- 아래 2.6 항목(Upper drive cmd)은 현재 테스트 중인 임시 사양이며 최종 확정 전 변경될 수 있음.

### 2.1 차동 구동/믹싱 관련
- `sbus_thread_entry()`에서
  - `CH3`(throttle), `CH1`(steering) 기반으로 차동 입력 계산.
  - 기존 `vcu_diff_drive_mix()` 호출 유지.
  - 추가 mixer(`mix_rc_to_tracks`)를 통해 좌/우 명령값(`left_rpm_value`, `right_rpm_value`) 최종 반영.
- RC mixer 코드를 `vcu_gateway.c`에서 분리:
  - `user/rc_mixer.c`, `user/rc_mixer.h`로 이동
  - 네이밍을 프로젝트 스타일(snake_case)로 정리
  - 주요 타입: `rc_input_t`, `vehicle_config_t`, `tune_config_t`, `calc_state_t`, `motor_output_t`

### 2.2 급선회/제자리 회전 보정(최신)
- 제자리 회전(`throttle==0`) 속도는 고정 비율 스케일 사용:
  - `RCM_INPLACE_TURN_SCALE_PERCENT` (기본 50%)
- 일반 주행(`throttle!=0`)은 `apply_turn_shaping()`으로 inner/outer 동시 보정:
  - `RCM_TURN_SHARPNESS`
  - `RCM_MIN_INNER_RATIO`
  - `RCM_MIN_OUTER_RATIO`
- `calc_state_t`에 디버그 필드 확장:
  - `beta_abs`, `left_ratio`, `right_ratio`, `inner_ratio`, `outer_ratio`

### 2.3 FSM 상태/명령 경로
- `fsm_thread_entry()`에서
  - RC/Upper/Motor 상태를 보고 최종 모터 명령 선택.
  - Left/Right 드라이버 설정 비트 공통 적용.
  - Upper status(0x18FF0310), Upper rpm status(0x18FF0300)용 데이터 갱신.

### 2.4 모니터링 구조체 추가
- `vcu_gateway.h`에 `vcu_motion_monitor_t` 추가:
  - `ts_tick`, `valid`
  - `left_driver_input`, `right_driver_input`
  - `yaw_deg_0_360`, `yaw_rate_deg_s`
  - `left_speed_m_s`, `right_speed_m_s`, `center_speed_m_s`
  - `left_distance_m`, `right_distance_m`, `center_distance_m`

- 조회 API 추가:
  - `int vcu_gateway_get_motion_monitor(vcu_motion_monitor_t* out);`

### 2.5 모니터링 실시간 계산 로직 추가
- `vcu_gateway.c`에 `update_motion_monitor()` 추가.
- FSM 주기(기본 10ms)마다 명령값 기준으로 갱신.
- 우측 설치 부호 반전을 고려해 운동학 계산 입력은 물리 방향으로 변환:
  - `update_motion_monitor(..., left_cmd, -right_cmd)`
- 계산 기준:
  - `driver input ~= rpm * 10` (즉 `rpm = input * 0.1`)
  - 속도: `m/s = rpm * (pi * wheel_diameter) / 60`
  - 중심속도: `(left + right)/2`
  - 요레이트(rad/s): `(v_right - v_left) / track_width`
  - 요레이트(deg/s): `rad/s * 57.2957795`
  - 거리 누적: `distance += speed * dt`
  - Yaw 누적 후 `[0, 360)` 정규화

### 2.6 Upper 제어 입력 경로 변경 (테스트 중)
- `0x18FF0200` 수신 처리 네이밍 정리:
  - `upper_intent_rpm_t` -> `upper_intent_drive_t`
  - `decode_upper_rpm_cmd()` -> `decode_upper_drive_cmd()`
  - `CANID_UPPER_CMD_RPM_RX` -> `CANID_UPPER_CMD_DRIVE_RX`
- 현재 payload 사용값:
  - `data[0:1]`: `throttle_cmd` (`int16`)
  - `data[2:3]`: `steering_cmd` (`int16`)
  - `data[4:7]`: reserved
- 상위에서 정규화 상수는 받지 않고 코드 기본값(`g_rcm_vehicle`, `g_rcm_tune`) 사용
- Upper drive timeout 추가:
  - `UPPER_DRIVE_TIMEOUT_MS = 1000ms`
  - timeout 시 Upper 경로는 STOP 처리

## 3. CAN 송신 확장

### 3.1 기존 유지
- `0x18FF0300`: Motor driver feedback rpm/status payload (기존 유지)
- `0x18FF0310`: VCU gateway status payload (기존 유지)

### 3.2 신규 추가 (기존 payload 비변경)
- 신규 ID: `0x18FF0320` (`CANID_UPPER_VEHICLE_STATUS_TX`)
- 위치: `can_tx_thread_entry()`에서 100ms 주기 송신
- 패킹 함수: `pack_upper_vehicle_status()`
- 데이터 포맷:
  - `data[0:1]`: `yaw_deg_0_360 * 10`
  - `data[2:3]`: `yaw_rate_deg_s * 10`
  - `data[4:5]`: `left_speed_m_s * 100`
  - `data[6:7]`: `right_speed_m_s * 100`

### 3.3 신규 추가: 테스트/디버그 모니터링
- 신규 ID: `0x18FF0330` (`CANID_UPPER_VEHICLE_MON_TX`)
- 위치: `can_tx_thread_entry()`에서 100ms 주기 송신
- 패킹 함수: `pack_upper_vehicle_monitor()`
- 데이터 포맷:
  - `data[0]`: `throttle_percent` (`s8`, -100~100)
  - `data[1]`: `steering_percent` (`s8`, -100~100)
  - `data[2]`: `left_cmd_percent` (`s8`, -100~100)
  - `data[3]`: `right_cmd_percent` (`s8`, -100~100)
  - `data[4:5]`: `yaw_rate_deg_s * 10` (`s16`)
  - `data[6:7]`: `center_distance_m * 100` (`s16`, cm)

## 4. 참고 사항
- 기존 `0x18FF0300`, `0x18FF0310` 포맷은 수정하지 않고 유지.
- 모니터링 값(`0x18FF0320`, `0x18FF0330`)은 현재 "명령 기반(command-based)" 계산값.
- 실측 피드백 기반으로 바꾸려면 `update_motion_monitor()` 입력 소스를 `motor_status` RPM으로 교체하면 됨.
- 문서 동기화:
  - `RC_MIXER_FLOW.md`: 믹서 흐름/급선회 보정/튜닝 파라미터 최신화
  - `UPPER_VCU_STATUS_GUIDE.md`: `0x18FF0320`, `0x18FF0330` 설명 및 해석 예시 추가
