# VCU Gateway 최근 변경 요약

## 1. 대상 파일
- `user/vcu_gateway.h`
- `user/vcu_gateway.c`

## 2. 핵심 변경 사항

### 2.1 차동 구동/믹싱 관련
- `sbus_thread_entry()`에서
  - `CH3`(throttle), `CH1`(steering) 기반으로 차동 입력 계산.
  - 기존 `vcu_diff_drive_mix()` 호출 유지.
  - 추가 mixer(`mix_rc_to_tracks`)를 통해 좌/우 명령값(`left_rpm_value`, `right_rpm_value`) 최종 반영.

### 2.2 FSM 상태/명령 경로
- `fsm_thread_entry()`에서
  - RC/Upper/Motor 상태를 보고 최종 모터 명령 선택.
  - Left/Right 드라이버 설정 비트 공통 적용.
  - Upper status(0x18FF0310), Upper rpm status(0x18FF0300)용 데이터 갱신.

### 2.3 모니터링 구조체 추가
- `vcu_gateway.h`에 `vcu_motion_monitor_t` 추가:
  - `ts_tick`, `valid`
  - `left_driver_input`, `right_driver_input`
  - `yaw_deg_0_360`, `yaw_rate_deg_s`
  - `left_speed_m_s`, `right_speed_m_s`, `center_speed_m_s`
  - `left_distance_m`, `right_distance_m`, `center_distance_m`

- 조회 API 추가:
  - `int vcu_gateway_get_motion_monitor(vcu_motion_monitor_t* out);`

### 2.4 모니터링 실시간 계산 로직 추가
- `vcu_gateway.c`에 `update_motion_monitor()` 추가.
- FSM 주기(기본 10ms)마다 `out_cmd_left.rpm_axis1`, `out_cmd_right.rpm_axis1` 기준으로 갱신.
- 계산 기준:
  - `driver input ~= rpm * 10` (즉 `rpm = input * 0.1`)
  - 속도: `m/s = rpm * (pi * wheel_diameter) / 60`
  - 중심속도: `(left + right)/2`
  - 요레이트(rad/s): `(v_right - v_left) / track_width`
  - 요레이트(deg/s): `rad/s * 57.2957795`
  - 거리 누적: `distance += speed * dt`
  - Yaw 누적 후 `[0, 360)` 정규화

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

## 4. 참고 사항
- 기존 `0x18FF0300`, `0x18FF0310` 포맷은 수정하지 않고 유지.
- 모니터링 값은 현재 "명령 기반(command-based)" 계산값이며, 실측 피드백 기반으로 바꾸려면 소스만 `motor_status` 값으로 교체하면 됨.
