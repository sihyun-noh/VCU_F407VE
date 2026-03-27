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
- `left_dir_sign`, `right_dir_sign` (모터 설치 방향 부호, 기본 `+1`, `-1`)

### 튜닝값 (`tune_config_t`)
- deadband (`deadband_throttle`, `deadband_steering`)
- gain (`steering_gain`, `left_gain`, `right_gain`)
- 고속 조향 제한 (`high_speed_throttle_threshold`, `max_steering_at_high_speed`)
- 제자리 회전 스케일 (`RCM_INPLACE_TURN_SCALE_PERCENT`, 기본 50%)
- 급선회 보정 파라미터
  - `RCM_TURN_SHARPNESS`
  - `RCM_MIN_INNER_RATIO`
  - `RCM_MIN_OUTER_RATIO`

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
- `beta = steering_for_mix / max_rc_input`

의미:
- `center_input`은 직진 기준값
- `beta`는 조향 정규화 값 (`-1 ~ +1`)
- `steering_for_mix`는 아래 조건을 반영한 steering
  - 제자리 회전이면 `RCM_INPLACE_TURN_SCALE_PERCENT` 적용
  - 일반 주행이면 원래 steering 사용

### Step F. 제자리 회전 로직 (`throttle == 0 && steering != 0`)
- `steering_for_mix = steering * (RCM_INPLACE_TURN_SCALE_PERCENT / 100)`
- `spin = max_driver_input * (steering_for_mix / max_rc_input)`
- `left_raw = spin`
- `right_raw = -spin`

목적:
- 제자리 회전 속도를 고정 비율로 줄여 테스트 안정성 확보

### Step G. 일반 주행 급선회 보정 (`apply_turn_shaping`)
일반 주행(`throttle != 0`)에서는 아래 함수로 비율 계산:
- `inner_ratio = max(1 - |beta|, RCM_MIN_INNER_RATIO)`
- `outer_ratio = clamp(1 - RCM_TURN_SHARPNESS * |beta|, RCM_MIN_OUTER_RATIO, 1.0)`

방향 적용:
- `beta > 0`: left=outer, right=inner
- `beta < 0`: left=inner, right=outer
- `beta = 0`: left=1, right=1

raw 계산:
- `left_raw = center_input * left_ratio`
- `right_raw = center_input * right_ratio`

목적:
- inner가 너무 빠르게 0으로 떨어지는 현상 완화
- outer도 약간 감쇠해 급선회 응답을 부드럽게 만듦

### Step H. 좌/우 gain 적용
- `left_logical = left_raw * left_gain`
- `right_logical = right_raw * right_gain`

### Step I. 모터 설치 방향 매핑(Geometry sign)
- 하드코딩 반전 대신 `vehicle_config_t`의 방향 부호 사용:
  - `left_final = left_logical * left_dir_sign`
  - `right_final = right_logical * right_dir_sign`

기본값:
- `left_dir_sign = +1`
- `right_dir_sign = -1`

의미:
- 플랫폼 설치 방향이 바뀌어도 수식 변경 없이 설정값만 바꿔 대응 가능

### Step J. 출력 제한(saturation)
- `scale_to_limit()`에서 한쪽이라도 최대치 초과 시,
  좌/우를 같은 비율로 축소해 **좌우 비율 유지**.
- 최종적으로 각 축을 `[-max_driver_input, +max_driver_input]`로 클램프.

### Step K. 최종 출력
- `motor_output_t.left_input`, `motor_output_t.right_input` 반환.

---

## 3) 디버그/모니터링 계산값 (`calc_state_t`)

`st != NULL`이면 아래 값을 채움:
- `center_input`, `beta`, `beta_abs`
- `radius_m` (beta 기반 회전 반경 추정)
- `yaw_rate_rad_s`, `yaw_rate_deg_s` (모델 기반)
- `left_input_raw`, `right_input_raw`
- `left_ratio`, `right_ratio`
- `inner_ratio`, `outer_ratio`
- `left_input_final`, `right_input_final`

이 값들로 "왜 inner/outer가 해당 속도로 나왔는지"를 추적 가능.

---

## 4) vcu_gateway 연동 지점

`user/vcu_gateway.c`의 `sbus_thread_entry()`에서:
1. `vcu_diff_drive_mix(...)` 호출
2. `mix_rc_to_tracks(...)` 호출 결과로 `rc.left_rpm_value`, `rc.right_rpm_value` 최종 반영

현재 최종값은 `rc_mixer` 결과가 사용됨.

---

## 5) 최근 수정 핵심 요약

1. 제자리 회전은 단계 양자화가 아니라 `RCM_INPLACE_TURN_SCALE_PERCENT` 고정 비율 방식으로 변경.
2. 일반 주행은 `apply_turn_shaping()` 함수로 분리해 가독성 개선.
3. 급선회 시 inner/outer를 동시에 조정하는 파라미터(`TURN_SHARPNESS`, `MIN_INNER_RATIO`, `MIN_OUTER_RATIO`) 추가.
4. `calc_state_t`에 ratio 관련 디버그 필드를 추가해 튜닝 근거 확인 가능.
