# Differential Drive Control (SBUS -> Left/Right RPM)

## 1. 개요
- 구동 구조: 탱크형 차동구동 (Differential Drive / Skid Steering)
- 물리 구성:
  - 좌측 바퀴 2개 = `LEFT MOTOR` (동일 명령)
  - 우측 바퀴 2개 = `RIGHT MOTOR` (동일 명령)
- 따라서 제어 목표는 `left_rpm_value`, `right_rpm_value` 두 값 생성이다.

## 2. SBUS 입력 정의
- `CH3`: Throttle (전/후진 속도)
- `CH1`: Steering (좌/우 회전)

현재 코드에서는 각 채널을 `sbus_to_cmd()`로 변환해 `CMD_MIN ~ CMD_MAX` 범위(기본 `-500 ~ 500`)의 정규화 명령값으로 사용한다.

## 3. 차동 믹싱 공식
- `left = throttle + steering`
- `right = throttle - steering`

계산 후 안전하게 아래 범위로 클램프한다.
- `left  in [CMD_MIN, CMD_MAX]`
- `right in [CMD_MIN, CMD_MAX]`

## 4. 코드 반영 위치
- 파일: `user/vcu_gateway.c`

### (1) `rc_intent_t` 확장
- `int16_t left_rpm_value`
- `int16_t right_rpm_value`

### (2) 믹싱 함수 추가
- `make_diff_drive_rpm(int16_t throttle, int16_t steering, int16_t* left, int16_t* right)`

### (3) `sbus_thread_entry()`에서 값 생성
- `throttle = rc.axis3` (`CH3`)
- `steering = rc.axis1` (`CH1`)
- `make_diff_drive_rpm(throttle, steering, &rc.left_rpm_value, &rc.right_rpm_value)`

### (4) FSM RC 분기에서 모터 명령 적용
- 좌측 드라이버:
  - `out_cmd_left.rpm_axis1 = rc.left_rpm_value`
  - `out_cmd_left.rpm_axis2 = rc.left_rpm_value`
- 우측 드라이버:
  - `out_cmd_right.rpm_axis1 = rc.right_rpm_value`
  - `out_cmd_right.rpm_axis2 = rc.right_rpm_value`

즉, "좌측 2개 바퀴 동일값 / 우측 2개 바퀴 동일값" 원칙을 그대로 반영한다.

## 5. 동작 예시
- 직진:
  - `steering = 0`, `throttle > 0`
  - `left = right > 0`
- 제자리 좌회전:
  - `throttle = 0`, `steering > 0`
  - `left > 0`, `right < 0`
- 우회전하며 전진:
  - `throttle > 0`, `steering < 0`
  - 좌/우 값 차이를 통해 곡률 생성

## 6. 튜닝 포인트
- 조향 감도:
  - 필요하면 `steering`에 가중치 적용 (예: `steering * k`)
- 최대 속도:
  - `CMD_MIN/CMD_MAX` 조정
- 데드밴드:
  - `sbus_to_cmd()`의 데드밴드(`DEADBAND`) 조정

## 7. 확인 체크리스트
- 조종기 중립에서 좌/우 0 출력
- CH3만 조작 시 좌/우 같은 방향/같은 크기
- CH1만 조작 시 좌/우 반대 방향
- 최대 입력에서 클램프 정상 동작
