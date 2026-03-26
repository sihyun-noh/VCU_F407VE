#ifndef _RC_MIXER_H_
#define _RC_MIXER_H_

#include <stdint.h>

/* [CONST] */
#define RCM_TRACK_WIDTH_M      (1.0f)
#define RCM_WHEELBASE_M        (1.5f)
#define RCM_WHEEL_DIAMETER_M   (0.36f)
#define RCM_MAX_RC_INPUT       (500.0f)
#define RCM_MAX_DRIVER_INPUT   (664.0f)
#define RCM_MAX_SPEED_KMH      (5.0f) /* throttle=500 -> 5 km/h */

/* [TUNE] */
#define RCM_DEADBAND_THROTTLE             (10.0f)
#define RCM_DEADBAND_STEERING             (10.0f)
#define RCM_STEERING_GAIN                 (1.0f)
#define RCM_LEFT_GAIN                     (1.0f)
#define RCM_RIGHT_GAIN                    (1.0f)
#define RCM_HIGH_SPEED_THROTTLE_THRESHOLD (350.0f)
#define RCM_MAX_STEERING_AT_HIGH_SPEED    (250.0f)

typedef struct {
  /* [INPUT] */
  float throttle;
  float steering;
} rc_input_t;

typedef struct {
  /* [CONST] */
  float track_width_m;
  float wheelbase_m;
  float wheel_diameter_m;
  float max_rc_input;
  float max_driver_input;
} vehicle_config_t;

typedef struct {
  /* [TUNE] */
  float deadband_throttle;
  float deadband_steering;
  float steering_gain;
  float left_gain;
  float right_gain;
  float high_speed_throttle_threshold;
  float max_steering_at_high_speed;
} tune_config_t;

typedef struct {
  /* [CALC] */
  float center_input;
  float beta;
  float radius_m;
  float yaw_rate_rad_s;
  float yaw_rate_deg_s;
  float left_input_raw;
  float right_input_raw;
  float left_input_final;
  float right_input_final;
} calc_state_t;

typedef struct {
  /* [OUTPUT] */
  int16_t left_input;
  int16_t right_input;
} motor_output_t;

extern const vehicle_config_t g_rcm_vehicle;
extern const tune_config_t g_rcm_tune;

motor_output_t mix_rc_to_tracks(const rc_input_t* in, const vehicle_config_t* cfg, const tune_config_t* tune,
                                calc_state_t* st);

#endif /* _RC_MIXER_H_ */
