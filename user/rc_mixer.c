#include "rc_mixer.h"

const vehicle_config_t g_rcm_vehicle = {
  RCM_TRACK_WIDTH_M,
  RCM_WHEELBASE_M,
  RCM_WHEEL_DIAMETER_M,
  RCM_MAX_RC_INPUT,
  RCM_MAX_DRIVER_INPUT,
  RCM_MAX_SPEED_KMH,
  1,  /* left_dir_sign */
  -1, /* right_dir_sign */
};

const tune_config_t g_rcm_tune = { RCM_DEADBAND_THROTTLE,
                                   RCM_DEADBAND_STEERING,
                                   RCM_STEERING_GAIN,
                                   RCM_LEFT_GAIN,
                                   RCM_RIGHT_GAIN,
                                   RCM_HIGH_SPEED_THROTTLE_THRESHOLD,
                                   RCM_MAX_STEERING_AT_HIGH_SPEED };

static float clamp_f32(float v, float lo, float hi) {
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

static float apply_deadband_f32(float v, float deadband) {
  if (v > -deadband && v < deadband)
    return 0.0f;
  return v;
}

/* [CALC] turn shaping for non in-place motion
 * Goal:
 * - prevent inner track from collapsing too early to zero
 * - reduce outer track slightly on sharp turns
 */
static void apply_turn_shaping(float steering_raw, float throttle_raw,float beta, float* left_ratio, float* right_ratio, float* inner_ratio_out,
                               float* outer_ratio_out) {
  float beta_abs, inner_ratio, outer_ratio;

  if (!left_ratio || !right_ratio)
    return;

  beta_abs = (beta >= 0.0f) ? beta : -beta;
  inner_ratio = 1.0f - beta_abs;
  if (inner_ratio < RCM_MIN_INNER_RATIO)
    inner_ratio = RCM_MIN_INNER_RATIO;
	if(-steering_raw > throttle_raw)
		inner_ratio = -0.40;
	

  outer_ratio = 1.0f - (RCM_TURN_SHARPNESS * beta_abs);
  outer_ratio = clamp_f32(outer_ratio, RCM_MIN_OUTER_RATIO, 1.0f);

  if (beta > 0.0f) {
    /* steering > 0: left is outer, right is inner */
    *left_ratio = outer_ratio;
    *right_ratio = inner_ratio;
  } else if (beta < 0.0f) {
    /* steering < 0: right is outer, left is inner */
    *left_ratio = inner_ratio;
    *right_ratio = outer_ratio;
  } else {
    *left_ratio = 1.0f;
    *right_ratio = 1.0f;
  }

  if (inner_ratio_out)
    *inner_ratio_out = inner_ratio;
  if (outer_ratio_out)
    *outer_ratio_out = outer_ratio;
}

/* [CALC] saturation by common ratio scaling */
static void scale_to_limit(float* left, float* right, float limit_abs) {
  float l_abs, r_abs, max_abs, scale;
  if (!left || !right || limit_abs <= 0.0f)
    return;

  l_abs = (*left >= 0.0f) ? *left : -(*left);
  r_abs = (*right >= 0.0f) ? *right : -(*right);
  max_abs = (l_abs > r_abs) ? l_abs : r_abs;
  if (max_abs <= limit_abs || max_abs <= 0.0f)
    return;

  scale = limit_abs / max_abs;
  *left *= scale;
  *right *= scale;
}

/* [CALC] core RC mix function */
motor_output_t mix_rc_to_tracks(const rc_input_t* in, const vehicle_config_t* cfg, const tune_config_t* tune,
                                calc_state_t* st) {
  float throttle, steering, throttle_abs;
  float steering_for_mix, beta_abs;
  float center_input, beta, left_ratio, right_ratio, inner_ratio_dbg, outer_ratio_dbg;
  float left_raw, right_raw;
  float left_logical, right_logical;
  float left_final, right_final;
  float speed_kmh, vc_m_s;
  motor_output_t out = { 0, 0 };

  if (!in || !cfg || !tune)
    return out;

  /* [INPUT] */
  throttle = clamp_f32(in->throttle, -cfg->max_rc_input, cfg->max_rc_input);
  steering = clamp_f32(in->steering, -cfg->max_rc_input, cfg->max_rc_input);

  /* [TUNE] */
  throttle = apply_deadband_f32(throttle, tune->deadband_throttle);
  steering = apply_deadband_f32(steering, tune->deadband_steering);

  throttle_abs = (throttle >= 0.0f) ? throttle : -throttle;
  if (throttle_abs >= tune->high_speed_throttle_threshold) {
    steering = clamp_f32(steering, -tune->max_steering_at_high_speed, tune->max_steering_at_high_speed);
  }

  steering = clamp_f32(steering * tune->steering_gain, -cfg->max_rc_input, cfg->max_rc_input);
  steering_for_mix = steering;

  if (throttle == 0.0f && steering != 0.0f) {
    /* In-place turn scale (default 50%):
     * e.g. 100/75/50/30/0 by changing RCM_INPLACE_TURN_SCALE_PERCENT.
     */
    float inplace_scale = clamp_f32(RCM_INPLACE_TURN_SCALE_PERCENT / 100.0f, 0.0f, 1.0f);
    steering_for_mix = steering * inplace_scale;
  }

  /* [CALC] base references
   *
   * center_input:
   *   - throttle only speed reference (straight speed)
   *
   * beta:
   *   - normalized steering in [-1.0, +1.0]
   */
  center_input = cfg->max_driver_input * (throttle / cfg->max_rc_input);
  beta = steering_for_mix / cfg->max_rc_input;
  beta_abs = (beta >= 0.0f) ? beta : -beta;

  if (throttle == 0.0f && steering != 0.0f) {
    /* In-place rotation: logical left/right are opposite. */
    float spin = cfg->max_driver_input * (steering_for_mix / cfg->max_rc_input);
    left_raw = spin;
    right_raw = -spin;
    left_ratio = 1.0f;
    right_ratio = -1.0f;
    inner_ratio_dbg = 1.0f;
    outer_ratio_dbg = 1.0f;
  } else {
    /* Normal steering path:
     * shape both outer/inner ratios to avoid abrupt inner zero.
     */
    apply_turn_shaping(steering_for_mix, throttle_abs, beta, &left_ratio, &right_ratio, &inner_ratio_dbg, &outer_ratio_dbg);
    left_raw = center_input * left_ratio;
    right_raw = center_input * right_ratio;
  }

  left_logical = left_raw * tune->left_gain;
  right_logical = right_raw * tune->right_gain;

  /* Installation mapping by geometry config (+1/-1 per side).
   * Default: left normal, right inverted.
   */
  left_final = left_logical * (float)((cfg->left_dir_sign >= 0) ? 1 : -1);
  right_final = right_logical * (float)((cfg->right_dir_sign >= 0) ? 1 : -1);

  /* [OUTPUT] saturation
   * If either side exceeds max_driver_input, both sides are scaled
   * by the same ratio so the left:right proportion is preserved.
   */
  scale_to_limit(&left_final, &right_final, cfg->max_driver_input);
  left_final = clamp_f32(left_final, -cfg->max_driver_input, cfg->max_driver_input);
  right_final = clamp_f32(right_final, -cfg->max_driver_input, cfg->max_driver_input);

  out.left_input = (int16_t)left_final;
  out.right_input = (int16_t)right_final;

  if (st) {
    st->center_input = center_input;
    st->beta = beta;
    st->beta_abs = beta_abs;
    st->left_ratio = left_ratio;
    st->right_ratio = right_ratio;
    st->inner_ratio = inner_ratio_dbg;
    st->outer_ratio = outer_ratio_dbg;

    if (beta == 0.0f) {
      st->radius_m = 0.0f; /* straight */
    } else {
      st->radius_m = (cfg->track_width_m * 0.5f) / beta_abs;
    }

    speed_kmh = cfg->max_speed_kmh * (throttle / cfg->max_rc_input);
    vc_m_s = speed_kmh / 3.6f;
    st->yaw_rate_rad_s = (2.0f * vc_m_s * beta) / cfg->track_width_m;
    st->yaw_rate_deg_s = st->yaw_rate_rad_s * 57.2957795f;
    st->left_input_raw = left_raw;
    st->right_input_raw = -right_raw; /* mapped command-side raw */
    st->left_input_final = left_final;
    st->right_input_final = right_final;
  }

  return out;
}
