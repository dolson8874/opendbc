#pragma once

#include "opendbc/safety/declarations.h"

// Host LKAS_OP_TO_FLEXRAY.ReqAngleTorque calibration: 0.085 degrees per count (17/200).
#define FLEXRAY_ANGLE_SCALE_NUM 17
#define FLEXRAY_ANGLE_SCALE_DEN 200
#define FLEXRAY_DEG_TO_CAN ((float)FLEXRAY_ANGLE_SCALE_DEN / (float)FLEXRAY_ANGLE_SCALE_NUM)
#define FLEXRAY_ANGLE_NEUTRAL 9000
#define FLEXRAY_MAX_ANGLE (90 * FLEXRAY_ANGLE_SCALE_DEN / FLEXRAY_ANGLE_SCALE_NUM)
//#define FLEXRAY_USE_PSCM_OUT 1

static bool landrover_flexray_harness = true;
static bool landrover_cruise_hud_bench = false;
enum { LANDROVER_HUD_UNKNOWN, LANDROVER_HUD_OFF, LANDROVER_HUD_ON };
static bool landrover_rr2017 = false;
static bool landrover_rr_mads = false;
static bool landrover_rr_acc_off_seen = false;
static bool landrover_rr_heartbeat_seen = false;
static uint32_t landrover_rr_heartbeat_ts = 0U;
// The host normally sends a heartbeat every 100 ms.
static const uint32_t LANDROVER_RR_HEARTBEAT_TIMEOUT_US = 500000U;
// Match the RR2017 host button parser. Hold the first CANCEL in the ACC domain.
static const uint32_t LANDROVER_RR_ACC_FRESH_US = 100000U;
static const uint32_t LANDROVER_RR_ACC_OFF_CONFIRM_US = 100000U;
static const uint32_t LANDROVER_RR_CANCEL_TIMEOUT_US = 400000U;
static bool landrover_rr_acc_fresh_seen = false;
static bool landrover_rr_acc_off = false;
static uint32_t landrover_rr_acc_off_since_ts = 0U;
static uint32_t landrover_rr_acc_ts = 0U;
static bool landrover_rr_cancel_armed = false;
static uint32_t landrover_rr_cancel_ts = 0U;
static bool landrover_rr_camera_seen = false;
static bool landrover_rr_camera_progress = false;
static uint8_t landrover_rr_camera_counter = 0U;
static uint32_t landrover_rr_camera_words[2] = {0U, 0U};
static uint32_t landrover_rr_camera_progress_ts = 0U;
static const uint32_t LANDROVER_RR_CAMERA_TIMEOUT_US = 200000U;

static RxCheck landrover_rr_rx_checks[] = {
  {.msg = {{0x172, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}}},
  {.msg = {{0x1CB, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}}},
  {.msg = {{0x158, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}}},
  {.msg = {{0x156, 0, 8, 25U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}}},
  {.msg = {{0x28F, 2, 8, 25U, .ignore_counter = true, .ignore_quality_flag = true}}},
  // The driver CANCEL input is required only in the RR MADS profile.
  {.msg = {{0x2D0, 0, 8, 14U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}}},
};

static void landrover_rr_disarm_mads(void) {
  if (landrover_rr_mads) {
    mads_exit_controls(MADS_DISENGAGE_REASON_STEERING_DISENGAGE);
    m_mads_state.controls_requested_lateral = false;
  }
}

static void landrover_rr_expire_heartbeat(uint32_t now) {
  if (landrover_rr_mads && landrover_rr_heartbeat_seen &&
      (safety_get_ts_elapsed(now, landrover_rr_heartbeat_ts) >= LANDROVER_RR_HEARTBEAT_TIMEOUT_US)) {
    landrover_rr_heartbeat_seen = false;
    heartbeat_engaged_mads = false;
    controls_allowed = false;
    landrover_rr_disarm_mads();
  }
}

static inline void landrover_rr_receive_heartbeat(bool engaged) {
  const uint32_t now = microsecond_timer_get();
  landrover_rr_expire_heartbeat(now);
  if (heartbeat_engaged_mads && !engaged) {
    controls_allowed = false;
    landrover_rr_disarm_mads();
  }
  // Repeated OFF messages leave a new stock ACC engagement pending for the host's ON.
  heartbeat_engaged_mads = engaged;
  landrover_rr_heartbeat_seen = true;
  landrover_rr_heartbeat_ts = now;
}

static void landrover_rr_camera_rx(const CANPacket_t *msg) {
  const uint32_t now = microsecond_timer_get();
  const uint8_t counter = (msg->data[3] >> 3) & 0xFU;
  const uint32_t low = GET_BYTES_LE(msg, 0, 4);
  const uint32_t high = GET_BYTES_LE(msg, 4, 4);
  const bool expired = safety_get_ts_elapsed(now, landrover_rr_camera_progress_ts) > LANDROVER_RR_CAMERA_TIMEOUT_US;

  // The stock camera repeats each word. Repeats never refresh counter liveness.
  if (landrover_rr_camera_seen && (counter == landrover_rr_camera_counter)) {
    if (expired || (low != landrover_rr_camera_words[0]) || (high != landrover_rr_camera_words[1])) {
      landrover_rr_camera_progress = false;
      controls_allowed = false;
      landrover_rr_disarm_mads();
    }
    return;
  }

  const uint8_t expected = (landrover_rr_camera_counter + 1U) & 0xFU;
  if (landrover_rr_camera_seen && (expired || (counter != expected))) {
    controls_allowed = false;
    landrover_rr_disarm_mads();
  }
  landrover_rr_camera_progress = landrover_rr_camera_seen && (counter == expected);
  landrover_rr_camera_seen = true;
  landrover_rr_camera_counter = counter;
  landrover_rr_camera_words[0] = low;
  landrover_rr_camera_words[1] = high;
  landrover_rr_camera_progress_ts = now;
}

static void landrover_rr_update_cancel(const CANPacket_t *msg, uint32_t now) {
  if (safety_get_ts_elapsed(now, landrover_rr_cancel_ts) >= LANDROVER_RR_CANCEL_TIMEOUT_US) {
    landrover_rr_cancel_armed = false;
  }
  landrover_rr_cancel_ts = now;
  if (GET_BIT(msg, 17U) == 0U) {
    landrover_rr_cancel_armed = true;
  } else if (landrover_rr_cancel_armed) {
    landrover_rr_cancel_armed = false;
    if (controls_allowed_lateral && landrover_rr_acc_fresh_seen && landrover_rr_acc_off &&
        (safety_get_ts_elapsed(now, landrover_rr_acc_ts) <= LANDROVER_RR_ACC_FRESH_US) &&
        (safety_get_ts_elapsed(now, landrover_rr_acc_off_since_ts) >= LANDROVER_RR_ACC_OFF_CONFIRM_US)) {
      controls_allowed = false;
      landrover_rr_disarm_mads();
    }
  }
}

static void landrover_rr_rx_hook(const CANPacket_t *msg) {
  const uint32_t now = microsecond_timer_get();
  landrover_rr_expire_heartbeat(now);
  if (msg->bus == 0U) {
    if (msg->addr == 0x172U) {
      int raw_torque = ((msg->data[2] & 0x3U) << 8) | msg->data[3];
      update_sample(&torque_driver, raw_torque * 4 - 2032);
    }
    if (msg->addr == 0x1CBU) {
      int raw_speed = ((msg->data[1] & 0x7FU) << 8) | msg->data[2];
      vehicle_moving = raw_speed != 0;
      UPDATE_VEHICLE_SPEED(raw_speed * 0.0101 * KPH_TO_MS);
    }
    if (msg->addr == 0x158U) {
      gas_pressed = msg->data[0] != 0U;
    }
    if (landrover_rr_mads && (msg->addr == 0x2D0U)) {
      landrover_rr_update_cancel(msg, now);
    }
    if (msg->addr == 0x156U) {
      brake_pressed = GET_BIT(msg, 36U) != 0U;
      const bool acc_on = GET_BIT(msg, 38U) != 0U;
      if (!acc_on) {
        landrover_rr_acc_off_seen = true;
      }
      if (landrover_rr_mads) {
        if (!landrover_rr_acc_fresh_seen || !landrover_rr_acc_off ||
            (safety_get_ts_elapsed(now, landrover_rr_acc_ts) > LANDROVER_RR_ACC_FRESH_US)) {
          landrover_rr_acc_off_since_ts = now;
        }
        landrover_rr_acc_fresh_seen = true;
        landrover_rr_acc_off = !acc_on;
        landrover_rr_acc_ts = now;
      }
      pcm_cruise_check(acc_on);
      if (landrover_rr_mads && (!landrover_rr_acc_off_seen || !landrover_rr_camera_progress)) {
        controls_allowed = false;
      }
    }
    if (brake_pressed || gas_pressed) {
      controls_allowed = false;
    }
    if (landrover_rr_mads && gas_pressed) {
      controls_allowed = false;
      landrover_rr_disarm_mads();
    }
  } else if (msg->bus == 2U && msg->addr == 0x28FU) {
    landrover_rr_camera_rx(msg);
  }
}

static uint8_t landrover_rr_lkas_checksum(uint16_t word) {
  static const uint16_t parity_masks[] = {0x0D23, 0x1447, 0x2FAD, 0x5878, 0x3FD2, 0x75A4, 0x6748, 0x4091};
  uint8_t result = 0U;
  for (uint8_t i = 0U; i < 8U; i++) {
    uint32_t parity = word & parity_masks[i];
    parity ^= parity >> 8;
    parity ^= parity >> 4;
    parity ^= parity >> 2;
    parity ^= parity >> 1;
    result |= (parity & 1U) << i;
  }
  return result;
}

static uint32_t landrover_get_checksum(const CANPacket_t *msg) {
  return msg->data[0];
}

static uint32_t landrover_compute_checksum(const CANPacket_t *msg) {
  uint16_t word = ((uint16_t)msg->data[3] << 8) | msg->data[4];
  return landrover_rr_lkas_checksum(word);
}

static bool landrover_rr_tx_hook(const CANPacket_t *msg) {
  if (msg->addr == 0x1D8U) {
    return (GET_BYTES_LE(msg, 0, 4) == 0x05059546U) && (GET_BYTES_LE(msg, 4, 4) == 0xBC080000U);
  }
  if (msg->addr == 0x3D4U) {
    return (GET_BYTES_LE(msg, 0, 4) == 0x41010D80U) && ((GET_BYTES_LE(msg, 4, 4) & 0xFFFFC3FFU) == 0x00ED02E7U);
  }

  const TorqueSteeringLimits limits = {
    .max_torque = 250,
    // NEOKII limits torque at 100 Hz and transmits every fourth tick.
    .max_rate_up = 12,
    .max_rate_down = 24,
    .max_rt_delta = 84,
    .type = TorqueDriverLimited,
    .driver_torque_allowance = 300,
    .driver_torque_multiplier = 1,
  };
  uint16_t word = ((uint16_t)msg->data[3] << 8) | msg->data[4];
  bool valid = ((msg->data[3] & 0x80U) != 0U) && (msg->data[1] == 0xFFU) && (msg->data[2] == 0xFFU) &&
               (GET_BYTES_LE(msg, 5, 3) == 0x70U) && (msg->data[0] == landrover_rr_lkas_checksum(word));
  if (!valid) {
    return false;
  }

  int torque = 1024 - (int)(word & 0x7FFU);
  uint32_t now = microsecond_timer_get();
  landrover_rr_expire_heartbeat(now);
  bool fresh = landrover_rr_camera_progress &&
               (safety_get_ts_elapsed(now, landrover_rr_camera_progress_ts) <= LANDROVER_RR_CAMERA_TIMEOUT_US);
  const uint8_t check_count = (uint8_t)(sizeof(landrover_rr_rx_checks) / sizeof(landrover_rr_rx_checks[0])) -
                              (landrover_rr_mads ? 0U : 1U);
  for (uint8_t i = 0U; i < check_count; i++) {
    const RxCheck *check = &landrover_rr_rx_checks[i];
    uint32_t deadline = 5000000U / check->msg[0].frequency;
    fresh &= check->status.msg_seen && check->status.valid_checksum && check->status.valid_quality_flag &&
             (check->status.wrong_counters < MAX_WRONG_COUNTERS) && (safety_get_ts_elapsed(now, check->status.last_timestamp) <= deadline);
  }
  if (!fresh || gas_pressed) {
    controls_allowed = false;
    landrover_rr_disarm_mads();
  }
  if (brake_pressed) {
    controls_allowed = false;
  }
  const bool lateral_allowed = landrover_rr_mads ?
                               (fresh && vehicle_moving && heartbeat_engaged_mads && !gas_pressed &&
                                m_mads_state.system_enabled && controls_allowed_lateral) :
                               controls_allowed;
  if (torque == 0) {
    // Neutral torque must be able to take effect immediately on disengagement.
    desired_torque_last = 0;
    if (!lateral_allowed) {
      rt_torque_last = 0;
      ts_torque_check_last = now;
    }
    return true;
  }
  return lateral_allowed && !steer_torque_cmd_checks(torque, 1, limits);
}

static uint8_t landrover_hud_state;
static uint8_t landrover_hud_counter;
static bool landrover_hud_counter_valid;
static bool landrover_hud_transport_valid;
static uint32_t landrover_hud_received;
static bool landrover_lkas_button_armed;
static uint32_t landrover_lkas_button_received;

static void landrover_reset_lkas_button(void) {
  landrover_lkas_button_armed = false;
  mads_button_press = MADS_BUTTON_UNAVAILABLE;
  m_mads_state.controls_requested_lateral = false;
}

static void landrover_hud_input_fault(void) {
  landrover_hud_transport_valid = false;
  landrover_hud_state = LANDROVER_HUD_UNKNOWN;
  pcm_cruise_check(false);
  landrover_reset_lkas_button();
  mads_exit_controls(MADS_DISENGAGE_REASON_LAG);
}

static void landrover_check_input_timeouts(uint32_t now) {
  const uint32_t LANDROVER_HUD_TIMEOUT_US = 200000U;
  const uint32_t LANDROVER_LKAS_BUTTON_TIMEOUT_US = 300000U;  // 0x24 is expected at 15 Hz.
  if (landrover_hud_counter_valid && (safety_get_ts_elapsed(now, landrover_hud_received) >= LANDROVER_HUD_TIMEOUT_US)) {
    landrover_hud_counter_valid = false;
    landrover_hud_input_fault();
  }
  if (landrover_lkas_button_armed &&
      (safety_get_ts_elapsed(now, landrover_lkas_button_received) >= LANDROVER_LKAS_BUTTON_TIMEOUT_US)) {
    landrover_reset_lkas_button();
    mads_exit_controls(MADS_DISENGAGE_REASON_LAG);
  }
}

static void landrover_cruise_hud_rx(const CANPacket_t *msg) {
  const uint32_t now = microsecond_timer_get();
  landrover_check_input_timeouts(now);
  if ((msg->bus == 1U) && (msg->addr == 0x1FBU)) {
    uint8_t crc = 0U;
    for (uint8_t i = 1U; i <= 4U; i++) {
      crc ^= msg->data[i];
      for (uint8_t bit = 0U; bit < 8U; bit++) {
        crc = (uint8_t)((crc << 1U) ^ (((crc & 0x80U) != 0U) ? 0x1DU : 0U));
      }
    }
    const bool valid = !msg->extended && ((crc ^ 0xCCU) == msg->data[0]) && ((msg->data[2] & 0xE0U) == 0U) &&
                       ((msg->data[4] | msg->data[5] | msg->data[6] | msg->data[7]) == 0U);
    const uint8_t counter = msg->data[1] & 0xFU;
    const uint8_t delta = (uint8_t)((counter + 16U - landrover_hud_counter) & 0xFU);
    if (!valid) {
      landrover_hud_input_fault();
    } else if (!landrover_hud_counter_valid || ((delta != 0U) && (delta <= 7U))) {
      landrover_hud_counter_valid = true;
      landrover_hud_transport_valid = true;
      landrover_hud_counter = counter;
      landrover_hud_received = now;
      const uint8_t flags = msg->data[1] & 0xF0U;
      const uint8_t mode = msg->data[2] & 7U;
      if ((flags == 0x30U) && (mode == 3U)) {
        if (landrover_hud_state == LANDROVER_HUD_UNKNOWN) {
          // Recovery already in ON is not evidence of a new enable edge.
          cruise_engaged_prev = true;
        }
        landrover_hud_state = LANDROVER_HUD_ON;
        pcm_cruise_check(true);
      } else if (((flags == 0x80U) && (mode == 0U)) || ((flags == 0xC0U) && (mode == 6U))) {
        // Known inactive cruise leaves the independent LKAS button state intact.
        landrover_hud_state = LANDROVER_HUD_OFF;
        pcm_cruise_check(false);
      } else {
        // A fresh, valid report with unknown cruise semantics only clears PCM.
        landrover_hud_state = LANDROVER_HUD_UNKNOWN;
        pcm_cruise_check(false);
      }
    } else {
      // A duplicate or out-of-order counter cannot refresh the state.
    }
  }
}

static void landrover_rx_hook(const CANPacket_t *msg) {
  if (landrover_rr2017) {
    landrover_rr_rx_hook(msg);
    return;
  }
  if (landrover_cruise_hud_bench) {
    landrover_cruise_hud_rx(msg);
  }
  if (landrover_flexray_harness) {
    if (msg->bus == 0U)  {
      #ifndef FLEXRAY_USE_PSCM_OUT
      // Steering angle: (0.1 * val) - 780 in deg.
      if (msg->addr == 0x56U) {
        int angle_raw = (((msg->data[3] & 0x3FU) << 8) | msg->data[4]);

        // Convert SWM's 0.1 degree counts to request counts.
        int angle_meas_new = ((angle_raw - 7800) * FLEXRAY_ANGLE_SCALE_DEN) / (10 * FLEXRAY_ANGLE_SCALE_NUM);
        update_sample(&angle_meas, angle_meas_new);
      }
      #else

      // The optional PSCM_Out path retains its legacy 0.09 degree interpretation.
      if (msg->addr == 0x32U) {
        unsigned int raw_val = (((msg->data[2] & 0x3FU) << 8) | msg->data[3]);
        int angle_meas_new = (((int)raw_val - 9000) * 9 * FLEXRAY_ANGLE_SCALE_DEN) / (100 * FLEXRAY_ANGLE_SCALE_NUM);
        update_sample(&angle_meas, angle_meas_new);
      }
      #endif

      // Vehicle speed (info02)
      if (msg->addr == 0x11U) {
        int speed = (msg->data[4] << 8) | msg->data[5];

        vehicle_moving = speed > 0.0;
        UPDATE_VEHICLE_SPEED(speed * 0.01 * KPH_TO_MS);
      }

      // Gas pressed
      if (msg->addr == 0x189U) {
        gas_pressed = (GET_BIT(msg, 58U) == 1);
      }

      // Brake pressed
      if (msg->addr == 0x84U) {
        brake_pressed = (GET_BIT(msg, 22U) == 1);
      }

      // Cruise state
      if ((msg->addr == 258U) && !landrover_cruise_hud_bench) {
        pcm_cruise_check((GET_BIT(msg, 34U) == 1));
      }

      // lkas btn
      if (msg->addr == 0x24U) {
        const bool on = GET_BIT(msg, 61U);
        if (!landrover_cruise_hud_bench) {
          mads_button_press = on ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
        } else if (!msg->extended && landrover_hud_transport_valid) {
          landrover_lkas_button_received = microsecond_timer_get();
          if (!on) {
            if (mads_button_press == MADS_BUTTON_PRESSED) {
              mads_exit_controls(MADS_DISENGAGE_REASON_BUTTON);
            }
            m_mads_state.controls_requested_lateral = false;
            landrover_lkas_button_armed = true;
            mads_button_press = MADS_BUTTON_NOT_PRESSED;
          } else if (landrover_lkas_button_armed) {
            mads_button_press = MADS_BUTTON_PRESSED;
          } else {
            // Require a fresh OFF-to-ON request after startup or input loss.
          }
        } else {
          // Invalid or unavailable input cannot refresh the button.
        }
      }

    }

  } else {

  }

}


static bool landrover_tx_hook(const CANPacket_t *msg) {
  if (landrover_rr2017) {
    return landrover_rr_tx_hook(msg);
  }
  bool tx = true;

  const AngleSteeringLimits LANDROVER_STEERING_LIMITS = {
    .max_angle = FLEXRAY_MAX_ANGLE,  // angle * deg_to_can
    .angle_deg_to_can = FLEXRAY_DEG_TO_CAN,
    .frequency = 50U,
    #if 0
    .angle_rate_up_lookup = {
      {0., 5., 25.},
      {2.5, 1.5, 0.2}
    },
    .angle_rate_down_lookup = {
      {0., 5., 25.},
      {5., 2.0, 0.3}
    },
    #endif
  };

  const AngleSteeringParams LANDROVER_STEERING_PARAMS = {
    .slip_factor = -0.000580374383851451,  // calc_slip_factor(VM)
    .steer_ratio = 19.,
    .wheelbase = 3.022,
  };



  // TODO find long params
  const LongitudinalLimits LANDROVER_LONG_LIMITS = {
    .max_accel = 425,       // 2 m/s^2
    .min_accel = 288,       // -3.48 m/s^2
    .inactive_accel = 375,  // 0. m/s^2
  };

  SAFETY_UNUSED(LANDROVER_LONG_LIMITS);

  if ((msg->bus == 1U) && (msg->addr == 0x1F0U)) {
    if (landrover_cruise_hud_bench) {
      landrover_check_input_timeouts(microsecond_timer_get());
    }
    if (!landrover_flexray_harness) {
      tx = false;
    } else {
      unsigned int raw_angle_can = ((msg->data[3] & 0x3FU) << 8) | msg->data[4];
      int desired_angle = (int)raw_angle_can - FLEXRAY_ANGLE_NEUTRAL;
      bool steer_control_enabled = GET_BIT(msg, 31U) == 1;
      if (steer_control_enabled) {
        bool steering_input_valid = true;
        if (landrover_cruise_hud_bench) {
          steering_input_valid = landrover_hud_state != LANDROVER_HUD_UNKNOWN;
          if (m_mads_state.system_enabled) {
            steering_input_valid = landrover_hud_transport_valid && (mads_button_press == MADS_BUTTON_PRESSED) && controls_allowed_lateral;
          }
        }
        tx = false;
        if (steering_input_valid) {
          tx = !steer_angle_cmd_checks_vm(desired_angle, true, LANDROVER_STEERING_LIMITS, LANDROVER_STEERING_PARAMS);
        }
      } else {
        // The FlexRay gateway receives a neutral request while LKAS is inactive.
        // Prime the next active jerk check with the measured angle.
        desired_angle_last = SAFETY_CLAMP(angle_meas.values[0], -FLEXRAY_MAX_ANGLE, FLEXRAY_MAX_ANGLE);
        tx = desired_angle == 0;
      }
    }
  }

  return tx;
}


static safety_config landrover_init(uint16_t param) {
  const uint16_t LANDROVER_PARAM_FLEXRAY_HARNESS = 1;
  const uint16_t LANDROVER_PARAM_CRUISE_HUD_BENCH = 2;
  const uint16_t rr_param = 1U << 15;
  const uint16_t rr_mads_param = rr_param | (1U << 14);

  // CAN messages for RANGE ROVER 2017 camera
  static const CanMsg LANDROVER_TX_MSGS[] = {
     {0x28F, 0, 8, .check_relay = true},
     {0x3D4, 0, 8, .check_relay = true},
     {0x1D8, 0, 8, .check_relay = true},
  };


  // CAN messages for OP to Flexray board
  // 0x1F0 = LkasCmd, 0x1F1 = ACC
  static const CanMsg LANDROVER_FLEXRAY_TX_MSGS[] = {
     {0x1F0, 1, 8, .check_relay = false},
     {0x1F9, 1, 8, .check_relay = false},
     {0x1BE, 0, 8, .check_relay = true, .disable_static_blocking = true}, // check for relay
  };


  static RxCheck landrover_flexray_rx_checks[] = {
    {.msg = {{0x24, 0, 8, 15U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // LKAS_btn 
#ifndef FLEXRAY_USE_PSCM_OUT
    {.msg = {{0x56, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
#else
    {.msg = {{0x32, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // PSCM_Out (angleTorque)
#endif
    {.msg = {{0x11, 0, 8, 25U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},    // Speed Info02 
    {.msg = {{0x189, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // GasPedal (gas pedal)
    {.msg = {{0x84, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},    // StopAndGo (brakes)
    {.msg = {{258, 0, 8, 25U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},     // CruiseInfo (cruise state)
    {.msg = {{0x1FB, 1, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // Bench HUD; checked in RX hook
  };


  landrover_flexray_harness = GET_FLAG(param, LANDROVER_PARAM_FLEXRAY_HARNESS);
  landrover_cruise_hud_bench = landrover_flexray_harness && GET_FLAG(param, LANDROVER_PARAM_CRUISE_HUD_BENCH);
  landrover_hud_state = LANDROVER_HUD_UNKNOWN;
  landrover_hud_received = 0U;
  landrover_hud_counter = 0U;
  landrover_hud_counter_valid = false;
  landrover_hud_transport_valid = false;
  landrover_lkas_button_received = 0U;
  landrover_reset_lkas_button();

  landrover_rr_mads = param == rr_mads_param;
  landrover_rr2017 = (param == rr_param) || landrover_rr_mads;
  landrover_rr_acc_off_seen = false;
  landrover_rr_heartbeat_seen = false;
  landrover_rr_acc_fresh_seen = false;
  landrover_rr_acc_off = false;
  landrover_rr_cancel_armed = false;
  landrover_rr_camera_seen = false;
  landrover_rr_camera_progress = false;

  if (landrover_rr2017) {
    safety_config config = BUILD_SAFETY_CFG(landrover_rr_rx_checks, LANDROVER_TX_MSGS);
    if (!landrover_rr_mads) {
      config.rx_checks_len -= 1;  // The final CANCEL check belongs to the RR MADS profile.
    }
    mads_set_alternative_experience(&alternative_experience);
    if (landrover_rr_mads) {
      // Require a new host heartbeat after entering this safety profile.
      heartbeat_engaged_mads = false;
      heartbeat_engaged_mads_mismatches = 0U;
    }
    return config;
  }
  if (!landrover_flexray_harness) {
    return (safety_config){0};
  }

  safety_config ret;
  ret = BUILD_SAFETY_CFG(landrover_flexray_rx_checks, LANDROVER_FLEXRAY_TX_MSGS);
  ret.mads_button_only = landrover_cruise_hud_bench;
  if (!landrover_cruise_hud_bench) {
    ret.rx_checks_len--;  // The final HUD check belongs only to the bench mode.
  }
  return ret;
}

const safety_hooks landrover_hooks = {
  .init = landrover_init,
  .rx = landrover_rx_hook,
  .tx = landrover_tx_hook,
  .get_checksum = landrover_get_checksum,
  .compute_checksum = landrover_compute_checksum,
};
