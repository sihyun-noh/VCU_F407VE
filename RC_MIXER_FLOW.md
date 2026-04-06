# RC Mixer Flow (Base + Gamma + DEX)

대상 파일:
- `user/rc_mixer.h`
- `user/rc_mixer.c`

핵심 함수:
- `mix_rc_to_tracks(const rc_input_t* in, bool drive_mode, const vehicle_config_t* cfg, const tune_config_t* tune, calc_state_t* st)`

---

## 1) 모드 개념 (`drive_mode`)

- `drive_mode = true`  : **Stable 모드**
  - 고속 조향 제한 ON
  - DEX OFF
- `drive_mode = false` : **Agile 모드**
  - 고속 조향 제한 OFF
  - DEX ON

SBUS 기준으로는 `CH10(C 버튼)`에서 이 값을 만듭니다.

---

## 2) 처리 순서

1. 입력 클램프
- `throttle`, `steering`를 `[-max_rc_input, +max_rc_input]`로 제한

2. Deadband 적용
- deadband 내부 입력은 `0` 처리

3. Stable 모드 고속 조향 제한
- `|throttle| >= high_speed_throttle_threshold`이면
- `steering`을 `[-max_steering_at_high_speed, +max_steering_at_high_speed]`로 제한

4. Steering gain 적용
- `steering = steering * steering_gain`

5. 제자리 회전 처리
- 조건: `throttle == 0 && steering != 0`
- `RCM_INPLACE_TURN_SCALE_PERCENT`를 적용해 회전 강도 축소
- 좌/우 반대 부호로 spin 출력 생성

6. 일반 주행 처리
- `center_input = max_driver_input * (throttle / max_rc_input)`
- `beta = steering_for_mix / max_rc_input`
- `apply_turn_shaping()`으로 Base 비율 계산
- Agile 모드면 `apply_turn_shaping_dex()` 추가 적용
- `left_raw = center_input * left_ratio`
- `right_raw = center_input * right_ratio`

7. 좌/우 gain + 설치 방향 부호 적용
- `left_gain`, `right_gain` 적용
- `left_dir_sign`, `right_dir_sign` 적용

8. Saturation
- 한쪽 초과 시 양쪽 동일 비율 축소(`scale_to_limit`)
- 최종적으로 `[-max_driver_input, +max_driver_input]` 클램프

---

## 3) Base Turn Shaping

함수:
- `apply_turn_shaping(beta, gamma, ...)`

의도:
- 일반 주행에서 조향 입력이 커질 때 inner 바퀴가 너무 빨리 0으로 떨어지는 현상을 완화
- outer 바퀴도 완만하게 감쇠시켜 선회 시 거동을 급격하지 않게 유지
- 농기계/저속 작업 환경에서 "안정적이고 예측 가능한 회전"을 기본 정책으로 제공

기본 구조:
- `beta_abs = |beta|`
- `beta_shaped = rcm_apply_gamma(beta_abs, gamma)`  (Gamma 옵션)
- `inner = 1 - beta_shaped`
- `outer = 1 - RCM_TURN_SHARPNESS * beta_shaped`
- `inner`는 `RCM_MIN_INNER_RATIO` 하한 보장
- `outer`는 `[RCM_MIN_OUTER_RATIO, 1.0]`로 클램프

방향 할당:
- `beta > 0` (우회전): left=outer, right=inner
- `beta < 0` (좌회전): left=inner, right=outer

회전 정책(요약):
- Base는 "안정형 정책"이다.
- 조향을 키워도 한쪽을 공격적으로 밀어붙이기보다, 좌우 비율을 점진적으로 벌린다.
- 결과적으로 급격한 스핀보다는 추종성/직진 안정성을 우선한다.

---

## 4) Gamma 기능

관련 매크로:
- `RCM_MIXER_FEAT_GAMMA`
- `RCM_MIXER_FEATURE_FLAGS`
- `RCM_GAMMA_MIN`, `RCM_GAMMA_MAX`, `RCM_GAMMA_DEFAULT`

동작:
- Gamma OFF: `beta_shaped = beta_abs` (기존 선형과 동일)
- Gamma ON:
  - `beta_shaped = powf(beta_abs, gamma)`
  - `gamma`는 `0.5 ~ 3.0` 클램프

의도:
- Base 비율식을 유지한 채, 조향 입력의 "초반/중반/후반 감도 곡선"만 조정
- 기구/하중/노면 차이로 생기는 "센터 부근 과민 또는 둔감" 문제를 미세 튜닝

감도 의미:
- `gamma = 1.0`: 기존과 동일
- `gamma > 1.0`: 센터 구간 둔감(초반 조향 부드러움)
- `gamma < 1.0`: 센터 구간 민감(초반 조향 빠름)

방향성:
- Gamma는 좌/우 회전 방향을 바꾸지 않는다.
- `beta`의 부호(좌/우)는 그대로 유지하고, `|beta|`의 크기 응답만 재매핑한다.
- 즉, "어느 쪽으로 도는가"가 아니라 "얼마나 빨리 반응하는가"를 조절한다.

---

## 5) DEX 기능 (Agile 확장)

함수:
- `apply_turn_shaping_dex(throttle, steering, beta, ...)`

적용 조건:
- `RCM_DEX_ENABLE == 1`
- 일반 주행 경로(`throttle != 0`)
- `|steering| > |throttle|` 인 과조향 상황
- `drive_mode == false` (Agile 모드)

핵심 의도:
- inner를 추가로 낮추고(필요 시 음수 허용)
- outer를 1.0 이상으로 확장
- 매우 공격적인 회전 응답 확보

세부 의도:
- `|steering| > |throttle|` 구간은 운전자가 의도적으로 강한 선회를 요구한 상황으로 해석
- 이 구간에서만 Base보다 적극적으로 좌우 비율 차를 키워 회전 민첩성 확보
- 제자리 회전과 달리, 진행 성분을 남긴 비대칭 회전을 만들기 쉬움

주요 매크로:
- `RCM_DEX_INNER_MIN` (예: `-0.4`)
- `RCM_DEX_OUTER_MAX` (예: `1.3`, 내부적으로 `1.0~1.4` 제한)
- `RCM_DEX_BLEND_GAIN`

방향성:
- DEX도 좌/우 방향 결정 자체는 `beta` 부호를 그대로 따른다.
- 차이는 강도이다. 같은 좌회전이라도 DEX는 inner를 더 낮추고 outer를 더 밀어
  yaw 반응을 빠르게 만든다.

회전 정책(요약):
- DEX는 "민첩형 정책"이다.
- 급선회/회피/테스트 상황에서 즉응성을 높이되, 과도하면 슬립/진동 가능성이 커질 수 있다.

---

## 6) 튜닝 가이드

안정형(농기계/저진동) 추천:
1. `RCM_MIXER_FEATURE_FLAGS = 0u` 또는 `gamma=1.2~1.8`
2. `drive_mode=true` 사용
3. `RCM_TURN_SHARPNESS` 소폭 증가(outer 감쇠)
4. `RCM_MIN_INNER_RATIO`를 높여 inner 0 추락 방지

민첩형(테스트/빠른 회전) 추천:
1. `drive_mode=false`
2. `RCM_DEX_ENABLE=1`
3. `RCM_DEX_OUTER_MAX`를 1.2~1.4 범위에서 조정
4. `RCM_DEX_INNER_MIN`을 0~-0.4 범위에서 조정
5. `gamma`는 0.8~1.0부터 시작

튜닝 순서 권장:
1. Base만 맞추기 (`DEX OFF`, `gamma=1.0`)
2. Gamma로 센터 감도 조절
3. 마지막에 DEX로 과조향 영역만 보강

---

## 7) 디버그 확인 포인트 (`calc_state_t`)

Gamma/DEX 확인 시 아래 필드를 우선 봅니다.
- `beta_raw`, `beta_abs`, `beta_shaped`, `gamma`, `gamma_enabled`
- `inner_ratio`, `outer_ratio`
- `dex_over`, `dex_applied`
- `left_input_raw`, `right_input_raw`
- `left_input_final`, `right_input_final`

---

## 8) vcu_gateway 연동

`sbus_thread_entry()`에서:
1. SBUS -> `rc.axis*` 변환/필터
2. `rc.rc_drive_mode` (`CH10`) 결정
3. `mix_rc_to_tracks(..., rc.rc_drive_mode, ...)` 호출
4. `rc.left_rpm_value`, `rc.right_rpm_value`에 최종 반영

즉, RC C버튼으로 Stable/Agile 정책을 실시간 전환할 수 있습니다.
