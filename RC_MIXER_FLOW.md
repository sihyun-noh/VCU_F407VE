# RC Mixer 코드 흐름 정리

대상 파일:
- `user/rc_mixer.h`
- `user/rc_mixer.c`

핵심 함수:
- `mix_rc_to_tracks(const rc_input_t* in, const vehicle_config_t* cfg, const tune_config_t* tune, calc_state_t* st)`

---

## 1) 입력/설정 구성

### 입력 (`rc_input_t`)
- `throttle` : 전진/후진 입력 (`-500 ~ +500`)
- `steering` : 조향 입력 (`-500 ~ +500`)

### 고정값 (`vehicle_config_t`)
- `track_width_m`, `wheelbase_m`, `wheel_diameter_m`
- `max_rc_input` (기본 `500`)
- `max_driver_input` (기본 `664`)

### 튜닝값 (`tune_config_t`)
- deadband (`deadband_throttle`, `deadband_steering`)
- gain (`steering_gain`, `left_gain`, `right_gain`)
- 고속 조향 제한 (`high_speed_throttle_threshold`, `max_steering_at_high_speed`)

---

## 2) 처리 순서

### Step A. 입력 클램프
- `throttle`, `steering`를 `[-max_rc_input, +max_rc_input]`로 제한.

### Step B. deadband 적용
- deadband 범위 내 입력은 `0`으로 처리.

### Step C. 고속 조향 제한
- `|throttle|`이 임계값 이상이면 `steering`을 제한값으로 클램프.

### Step D. steering gain 적용
- `steering = steering * steering_gain` 후 다시 입력 범위로 제한.

### Step E. 중심 속도/비율 계산
- `center_input = max_driver_input * (throttle / max_rc_input)`
- `beta = steering / max_rc_input`

의미:
- `center_input`은 직진 기준값
- `beta`는 조향 정규화 값 (`-1 ~ +1`)

### Step F. 좌/우 raw 계산

일반 주행:
- `left_raw  = center_input * (1 + beta)`
- `right_raw = center_input * (1 - beta)`

제자리 회전(`throttle == 0 && steering != 0`):
- `spin = max_driver_input * (steering / max_rc_input)`
- `left_raw = spin`
- `right_raw = -spin`

좌우 비율 핵심:
- `left : right = (1 + beta) : (1 - beta)`

### Step G. 좌/우 gain 적용
- `left_logical = left_raw * left_gain`
- `right_logical = right_raw * right_gain`

### Step H. 우측 모터 부호 매핑
- 현재 구현은 대칭 설치 기준으로 우측 명령 부호 반전:
  - `left_final = left_logical`
  - `right_final = -right_logical`

### Step I. 출력 제한(saturation)
- `scale_to_limit()`에서 한쪽이라도 최대치 초과 시,
  좌/우를 같은 비율로 축소해 **좌우 비율 유지**.
- 최종적으로 각 축을 `[-max_driver_input, +max_driver_input]`로 클램프.

### Step J. 최종 출력
- `motor_output_t.left_input`, `motor_output_t.right_input` 반환.

---

## 3) 디버그/모니터링 계산값 (`calc_state_t`)

`st != NULL`이면 아래 값을 채움:
- `center_input`, `beta`
- `radius_m` (beta 기반 회전 반경 추정)
- `yaw_rate_rad_s`, `yaw_rate_deg_s` (모델 기반)
- `left_input_raw`, `right_input_raw`
- `left_input_final`, `right_input_final`

---

## 4) vcu_gateway 연동 지점

`user/vcu_gateway.c`의 `sbus_thread_entry()`에서:
1. `vcu_diff_drive_mix(...)` 호출
2. `mix_rc_to_tracks(...)` 호출 결과로 `rc.left_rpm_value`, `rc.right_rpm_value` 최종 반영

현재 최종값은 `rc_mixer` 결과가 사용됨.
