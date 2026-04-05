#ifndef _RC_MIXER_H_
#define _RC_MIXER_H_

#include <stdint.h>

/* [CONST] Vehicle geometry and input/output limits */
#define RCM_TRACK_WIDTH_M    (1.0f)   /* +: 같은 좌우 속도차에서 yaw 감소(덜 꺾임), -: 더 민감하게 회전 */
#define RCM_WHEELBASE_M      (1.5f)   /* 현재 믹서 계산에는 직접 미사용(향후 모델 확장용) */
#define RCM_WHEEL_DIAMETER_M (0.36f)  /* +: 같은 rpm에서 속도/거리 추정 증가, -: 감소 바퀴지름 0.36m */
#define RCM_MAX_RC_INPUT     (500.0f) /* RC 입력 정규화 기준. 실제 송신기 스케일과 일치 필요 */
#define RCM_MAX_DRIVER_INPUT (664.0f) /* +: 최대 출력/선회 여유 증가, -: 전체 출력 제한 강화 */
#define RCM_MAX_SPEED_KMH    (5.0f)   /* throttle=500 기준 속도 모델. +: yaw_rate 추정 증가, -: 감소 */

/* [TUNE] Default tuning values */
#define RCM_DEADBAND_THROTTLE             (10.0f)  /* +: 미세 스로틀 무시 증가(둔감), -: 미세 조작 민감 */
#define RCM_DEADBAND_STEERING             (10.0f)  /* +: 센터 근처 조향 무시 증가, -: 센터 근처 민감 */
#define RCM_STEERING_GAIN                 (1.0f)   /* +: 더 급하게 선회, -: 더 완만한 선회 */
#define RCM_LEFT_GAIN                     (1.0f)   /* +: 좌측 출력 상대 증가, -: 좌측 출력 상대 감소 */
#define RCM_RIGHT_GAIN                    (1.0f)   /* +: 우측 출력 상대 증가, -: 우측 출력 상대 감소 */
#define RCM_HIGH_SPEED_THROTTLE_THRESHOLD (350.0f) /* +: 고속 조향제한이 늦게 시작, -: 더 빨리 시작 */
#define RCM_MAX_STEERING_AT_HIGH_SPEED    (250.0f) /* +: 고속에서도 조향 허용 증가, -: 고속 안정성 우선 */
#define RCM_INPLACE_TURN_SCALE_PERCENT    (50.0f)  /* throttle==0일 때 steering 비율(100/75/50/30/0) */
#define RCM_TURN_SHARPNESS                (0.25f)  /* +: 선회 시 outer 감쇠 증가, -: outer 유지 */
#define RCM_MIN_INNER_RATIO               (0.0f)  /* +: inner 최소 속도 증가(0 방지), -: inner 더 줄어듦 */
#define RCM_MIN_OUTER_RATIO               (0.90f)  /* +: outer 감쇠 하한 증가, -: outer 더 줄어듦 */

/* [FEATURE] Mixer optional features */
#define RCM_MIXER_FEAT_GAMMA  (1u << 0) /* bit0: steering gamma shaping ON/OFF */
#define RCM_MIXER_FEATURE_FLAGS (0u) /* 기능 비트마스크:
                                      * 0u = 모든 옵션 OFF(기본, 기존 동작)
                                      * RCM_MIXER_FEAT_GAMMA = steering gamma ON
                                      * (가정) throttle gamma 기능 추가 시:
                                      *   #define RCM_MIXER_FEAT_THROTTLE_GAMMA (1u << 1)
                                      *   #define RCM_MIXER_FEATURE_FLAGS (RCM_MIXER_FEAT_GAMMA | RCM_MIXER_FEAT_THROTTLE_GAMMA)
                                      */

/* [TUNE-GAMMA] Steering gamma shaping */
#define RCM_GAMMA_MIN     (0.5f) /* 최소값: 작을수록(1.0 미만) 센터 구간 조향이 더 민감 */
#define RCM_GAMMA_MAX     (3.0f) /* 최대값: 클수록(1.0 초과) 센터 구간 조향이 더 둔감 */
#define RCM_GAMMA_DEFAULT (1.0f) /* 적용 범위 0.5~3.0, 1.0=선형(기존 동일), <1 민감, >1 둔감 */
/* 현재 gamma는 steering(beta) 경로에만 적용됨.
 * (참고) throttle에도 동일 개념 적용 시 예:
 *  - throttle_gamma < 1.0f : 저속 구간 응답 민감(초반 가속 빠름)
 *  - throttle_gamma = 1.0f : 선형(기존 동일)
 *  - throttle_gamma > 1.0f : 저속 구간 둔감(초반 가속 완만)
 */

/* [TUNE-DEX] Agile mixer extension (optional) */
#define RCM_DEX_ENABLE        (0u)    /* 0: base only, 1: base + dex shaping */
#define RCM_DEX_INNER_MIN     (-0.4f) /* when |steering|>|throttle|, inner can go down to this value */
#define RCM_DEX_OUTER_MAX     (1.3f)  /* when |steering|>|throttle|, outer can rise up to this value */
#define RCM_DEX_BLEND_GAIN    (1.0f)  /* dex blend sensitivity */

/* [INPUT] RC command input set */
typedef struct {
  /* Forward/backward command (-max_rc_input ~ +max_rc_input) */
  float throttle;
  /* Left/right steering command (-max_rc_input ~ +max_rc_input) */
  float steering;
} rc_input_t;

/* [CONST] Vehicle physical/setup constants */
typedef struct {
  /* Left-right wheel center distance */
  float track_width_m;
  /* Front-rear axle distance (reserved for future use) */
  float wheelbase_m;
  /* Wheel diameter */
  float wheel_diameter_m;
  /* RC normalized max input (e.g. 500) */
  float max_rc_input;
  /* Driver command max magnitude (e.g. 664) */
  float max_driver_input;
  /* Speed model max (km/h) used for yaw-rate estimation */
  float max_speed_kmh;
  /* Drive direction sign per side (+1 or -1) */
  int8_t left_dir_sign;
  int8_t right_dir_sign;
} vehicle_config_t;

/* [TUNE] Runtime tuning parameters */
typedef struct {
  /* Zero zone for throttle */
  float deadband_throttle;
  /* Zero zone for steering */
  float deadband_steering;
  /* Global steering sensitivity */
  float steering_gain;
  /* Left track gain (asymmetry compensation) */
  float left_gain;
  /* Right track gain (asymmetry compensation) */
  float right_gain;
  /* Apply steering cap above this |throttle| */
  float high_speed_throttle_threshold;
  /* Max steering magnitude at high speed */
  float max_steering_at_high_speed;
  /* Steering gamma for optional nonlinear shaping (0.5~3.0) */
  float steering_gamma;
} tune_config_t;

typedef struct {
  float beta_raw;
  float beta_abs;
  float beta_shaped;
  float gamma;
  float inner_ratio;
  float outer_ratio;
  uint8_t gamma_enabled;
} rcm_mixer_debug_t;

/* [CALC] Optional debug/monitor output of intermediate values */
typedef struct {
  /* Center speed command before steering split */
  float center_input;
  /* Normalized steering ratio */
  float beta;
  /* Raw beta before shaping */
  float beta_raw;
  /* Estimated turn radius from beta */
  float radius_m;
  /* Estimated yaw rate [rad/s] */
  float yaw_rate_rad_s;
  /* Estimated yaw rate [deg/s] */
  float yaw_rate_deg_s;
  /* Pre-gain/raw left command */
  float left_input_raw;
  /* Pre-gain/raw right command (command-side mapped) */
  float right_input_raw;
  /* Turn shaping-applied left/right ratio vs center_input */
  float left_ratio;
  float right_ratio;
  /* Turn shaping debug values */
  float beta_abs;
  float beta_shaped;
  float gamma;
  uint8_t gamma_enabled;
  float inner_ratio;
  float outer_ratio;
  /* Dex shaping debug values */
  float dex_over;
  float dex_applied;
  /* Final left command after gain/saturation */
  float left_input_final;
  /* Final right command after gain/saturation */
  float right_input_final;
} calc_state_t;

/* [OUTPUT] Final left/right driver command */
typedef struct {
  /* Left driver command */
  int16_t left_input;
  /* Right driver command */
  int16_t right_input;
} motor_output_t;

/* Default const configuration/tuning instance */
extern const vehicle_config_t g_rcm_vehicle;
extern const tune_config_t g_rcm_tune;

/**
 * @brief Mix RC throttle/steering to left/right track commands.
 * @param in    RC input (throttle, steering)
 * @param cfg   Vehicle constants/limits
 * @param tune  Tuning parameters
 * @param st    Optional calculation state output (NULL allowed)
 * @return Final left/right driver command pair
 */
motor_output_t mix_rc_to_tracks(const rc_input_t* in, const vehicle_config_t* cfg, const tune_config_t* tune,
                                calc_state_t* st);

#endif /* _RC_MIXER_H_ */
