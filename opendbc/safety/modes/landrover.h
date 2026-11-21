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

#ifdef _RR_2017_
  // CAN messages for RANGE ROVER 2017 camera
  static const CanMsg LANDROVER_TX_MSGS[] = {
     {0x28F, 0, 8, .check_relay = true},
     {0x3D4, 0, 8, .check_relay = true},
     {0x1D8, 0, 8, .check_relay = true},
  };
#endif

  // CAN messages for OP to Flexray board
  // 0x1F0 = LkasCmd, 0x1F1 = ACC
  static const CanMsg LANDROVER_FLEXRAY_TX_MSGS[] = {
     {0x1F0, 1, 8, .check_relay = false},
     {0x1F9, 1, 8, .check_relay = false},
     {0x1BE, 0, 8, .check_relay = true, .disable_static_blocking = true}, // check for relay
  };

#ifdef _RR_2017_
  static RxCheck landrover_rr_rx_checks[] = {
    {.msg = {{0xf2, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // EPS_01 (STEER_ANGLE01)
   {.msg = {{0x1CB, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},    // SPEED_02  SPEED02
    {.msg = {{0x158, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // ACCELATOR_DRIVER (ACCELATOR_DRIVER)
    {.msg = {{0x156, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},    // CRUISE_CONTROL (DRIVER_BRAKE, CRUISE_ON)
    {.msg = {{0x28F, 2, 8, 25U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // CAM msg LKAS_RUN
  };
#endif

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

  safety_config ret;

  //if (landrover_flexray_harness) {
    ret = BUILD_SAFETY_CFG(landrover_flexray_rx_checks, LANDROVER_FLEXRAY_TX_MSGS);
    ret.mads_button_only = landrover_cruise_hud_bench;
    if (!landrover_cruise_hud_bench) {
      ret.rx_checks_len--;  // The final HUD check belongs only to the bench mode.
    }
  //} else {
  //  ret = BUILD_SAFETY_CFG(landrover_rr_rx_checks, LANDROVER_TX_MSGS);
  //}
  return ret;
}

const safety_hooks landrover_hooks = {
  .init = landrover_init,
  .rx = landrover_rx_hook,
  .tx = landrover_tx_hook,
};
