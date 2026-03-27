# VCU Gateway Reference

이 문서는 VCU Gateway 프로젝트의 현재 구조/인터페이스/튜닝 포인트를 빠르게 참조하기 위한 레퍼런스 문서입니다.

## 1) 적용 범위
- `user/vcu_gateway.c`
- `user/vcu_gateway.h`
- `user/rc_mixer.c`
- `user/rc_mixer.h`

## 2) 런타임 구조 (Thread)
- `sbus_thread`:
  - SBUS 수신/디코딩
  - RC 입력(`axis1~axis4`) 생성
  - `vcu_diff_drive_mix()` + `mix_rc_to_tracks()`로 좌/우 명령 계산
- `fsm_thread`:
  - RC/Upper/Motor 상태를 기반으로 최종 좌/우 모터 명령 결정
  - STOP 우선순위 처리(Upper stop, RC e-stop, fault/timeout)
  - `upper_vcu_st`, `upper_rpm_st`, `motion_monitor` 갱신
- `can_rx_thread`:
  - 상위 CMD(0200/0210) 수신/파싱
  - 모터 상태 수신(좌/우)
- `can_tx_thread` (100ms):
  - 모터 명령 송신
  - 상위 상태/모니터링 프레임 송신

## 3) CAN 인터페이스 (Gateway 기준)

### RX
- `0x18FF0200`:
  - Upper -> Gateway RPM 명령
  - `upper_intent_rpm_t`
- `0x18FF0210`:
  - Upper -> Gateway 설정/제어 명령
  - `upper_intent_t`
- `0x18FF0021`:
  - Left motor status
- `0x18FF0020`:
  - Right motor status

### TX
- `0x18FF2100`:
  - Gateway -> Driver1(left) 모터 명령
- `0x18FF2000`:
  - Gateway -> Driver2(right) 모터 명령
- `0x18FF0300`:
  - 모터 드라이버 상태 피드백 (좌/우 axis rpm)
- `0x18FF0310`:
  - VCU 상태 (`upper_vcu_st`)
- `0x18FF0320`:
  - vehicle motion status (`yaw/speed`)
- `0x18FF0330`:
  - vehicle monitor/debug (`throttle/steering %, cmd %, yaw_rate, center_distance`)

## 4) 제어 우선순위 (FSM)
- 정지 우선순위:
  1. `upper_force_stop`
  2. `rc_emergency_stop`
  3. 모터 fault/timeout
- 정지 조건 없을 때:
  - `RC valid + rc_enable` 이면 RC 제어 우선
  - 아니면 `upper valid`이면 Upper 제어
  - 둘 다 아니면 STOP

## 5) RC Mixer 개요
- 입력:
  - `CH3 = throttle`, `CH1 = steering`
- 제자리 회전:
  - `throttle == 0`일 때 `RCM_INPLACE_TURN_SCALE_PERCENT` 적용 (기본 50%)
- 일반 주행 급선회 보정:
  - `apply_turn_shaping()`에서 inner/outer 동시 보정
  - 튜닝:
    - `RCM_TURN_SHARPNESS`
    - `RCM_MIN_INNER_RATIO`
    - `RCM_MIN_OUTER_RATIO`
- 디버깅:
  - `calc_state_t`로 `beta`, `left/right_ratio`, `inner/outer_ratio`, `yaw_rate` 확인 가능

자세한 믹서 흐름:
- `RC_MIXER_FLOW.md` 참고

## 6) 모니터링 계산 기준 (중요)
- `vcu_motion_monitor_t` 값은 현재 **명령값(out_cmd) 기반** 적분.
- yaw/거리 계산 시 우측 설치 부호를 물리 방향으로 변환해 적용:
  - `update_motion_monitor(..., left_cmd, -right_cmd)`
- 실측 기반으로 변경하려면 입력 소스를 motor feedback RPM으로 변경 필요.

## 7) 비트마스크 참조
- RC 상태 (`0x18FF0310 data[4]`):
  - `RC_ST_ENABLE`, `RC_ST_EMERGENCY_STOP`, `RC_ST_FAILSAFE`, `RC_ST_FRESH`, `RC_ST_CULTIVATOR_DOWN`, `RC_ST_CULTIVATOR_ON`
- VCU FSM 상태 (`0x18FF0310 data[5]`):
  - `VCU_ST_SRC_NONE`, `VCU_ST_SRC_RC`, `VCU_ST_SRC_UPPER`
  - `VCU_ST_STOP_UPPER`, `VCU_ST_STOP_RC_EMG`, `VCU_ST_STOP_MOTOR_FAULT`, `VCU_ST_STOP_TIMEOUT`, `VCU_ST_RUNNING`

자세한 upper 상태 해석:
- `UPPER_VCU_STATUS_GUIDE.md` 참고

## 8) 현재 기본 정책
- 좌/우 드라이버 설정값은 동일 적용.
- Upper에서 전달된 driver config는 `D0_EN_BOTH_ENABLE` 조건을 만족할 때만 반영.
- power supply는 현재 left driver 기준을 사용.

## 9) 유지보수 체크포인트
- CAN ID 변경 시 `vcu_gateway.h` 정의와 문서 동시 업데이트
- RC mixer 튜닝값 변경 시:
  - `rc_mixer.h`
  - `RC_MIXER_FLOW.md`
  - 필요 시 `0x18FF0330` 로그 해석 기준 동기화
