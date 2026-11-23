#pragma once

#include "opendbc/safety/safety_declarations.h"

#define FLEXRAY_MAX_ANGLE   1170  // angle * deg_to_can
#define FLEXRAY_DEG_TO_CAN 13.009 //  1/factor, 1/0.07687
#define FLEXRAY_USE_PSCM_OUT 1

static bool landrover_flexray_harness = true;
static bool landrover_rr2017 = false;
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
};

static void landrover_rr_camera_rx(const CANPacket_t *msg) {
  const uint32_t now = microsecond_timer_get();
  const uint8_t counter = (msg->data[3] >> 3) & 0xFU;
  const uint32_t low = GET_BYTES(msg, 0, 4);
  const uint32_t high = GET_BYTES(msg, 4, 4);
  const bool expired = get_ts_elapsed(now, landrover_rr_camera_progress_ts) > LANDROVER_RR_CAMERA_TIMEOUT_US;

  // The stock camera repeats each word. Repeats never refresh counter liveness.
  if (landrover_rr_camera_seen && (counter == landrover_rr_camera_counter)) {
    if (expired || (low != landrover_rr_camera_words[0]) || (high != landrover_rr_camera_words[1])) {
      landrover_rr_camera_progress = false;
      controls_allowed = false;
    }
    return;
  }

  const uint8_t expected = (landrover_rr_camera_counter + 1U) & 0xFU;
  if (landrover_rr_camera_seen && (expired || (counter != expected))) {
    controls_allowed = false;
  }
  landrover_rr_camera_progress = landrover_rr_camera_seen && (counter == expected);
  landrover_rr_camera_seen = true;
  landrover_rr_camera_counter = counter;
  landrover_rr_camera_words[0] = low;
  landrover_rr_camera_words[1] = high;
  landrover_rr_camera_progress_ts = now;
}

static void landrover_rr_rx_hook(const CANPacket_t *msg) {
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
    if (msg->addr == 0x156U) {
      brake_pressed = GET_BIT(msg, 36U) != 0U;
      pcm_cruise_check(GET_BIT(msg, 38U) != 0U);
    }
    if (brake_pressed || gas_pressed) {
      controls_allowed = false;
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
    return (GET_BYTES(msg, 0, 4) == 0x05059546U) && (GET_BYTES(msg, 4, 4) == 0xBC080000U);
  }
  if (msg->addr == 0x3D4U) {
    return (GET_BYTES(msg, 0, 4) == 0x41010D80U) && ((GET_BYTES(msg, 4, 4) & 0xFFFFC3FFU) == 0x00ED02E7U);
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
               (GET_BYTES(msg, 5, 3) == 0x70U) && (msg->data[0] == landrover_rr_lkas_checksum(word));
  if (!valid) {
    return false;
  }

  int torque = 1024 - (int)(word & 0x7FFU);
  uint32_t now = microsecond_timer_get();
  bool fresh = landrover_rr_camera_progress &&
               (get_ts_elapsed(now, landrover_rr_camera_progress_ts) <= LANDROVER_RR_CAMERA_TIMEOUT_US);
  for (uint8_t i = 0U; i < (sizeof(landrover_rr_rx_checks) / sizeof(landrover_rr_rx_checks[0])); i++) {
    const RxCheck *check = &landrover_rr_rx_checks[i];
    uint32_t deadline = 5000000U / check->msg[0].frequency;
    fresh &= check->status.msg_seen && check->status.valid_checksum && check->status.valid_quality_flag &&
             (check->status.wrong_counters < MAX_WRONG_COUNTERS) && (get_ts_elapsed(now, check->status.last_timestamp) <= deadline);
  }
  if (!fresh || brake_pressed || gas_pressed) {
    controls_allowed = false;
  }
  if (torque == 0) {
    // Neutral torque must be able to take effect immediately on disengagement.
    desired_torque_last = 0;
    if (!controls_allowed) {
      rt_torque_last = 0;
      ts_torque_check_last = now;
    }
    return true;
  }
  return controls_allowed && !steer_torque_cmd_checks(torque, 1, limits);
}

static void landrover_rx_hook(const CANPacket_t *msg) {
  if (landrover_rr2017) {
    landrover_rr_rx_hook(msg);
  } else if (landrover_flexray_harness) {
    if (msg->bus == 0U)  {
      #ifndef FLEXRAY_USE_PSCM_OUT
      // Steering angle: (0.1 * val) - 780 in deg.
      if (msg->addr == 0x56) {
        // Store it 1/10 deg to match steering request
        int angle_raw = (((msg->data[3] & 0x3FU) << 8) | msg->data[4]);

        int angle_meas_new = (angle_raw - 7800U ) * 0.1 * FLEXRAY_DEG_TO_CAN;
        update_sample(&angle_meas, angle_meas_new);
      }
      #else

      // PSCM_Out Steering angleTorque: (0.07687 * val) - 691.83 in deg.
      if (msg->addr == 0x32U) {
        // Store it 1/10 deg to match steering request
        unsigned int raw_val = (((msg->data[2] & 0x3FU) << 8) | msg->data[3]);
        int angle_meas_new = (int)raw_val - 9000;
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
      if (msg->addr == 258U) {
        pcm_cruise_check((GET_BIT(msg, 34U) == 1));
      }

      // lkas btn
      if (msg->addr == 0x24U) {
        mads_button_press = GET_BIT(msg, 61U) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
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
    .angle_deg_to_can = FLEXRAY_DEG_TO_CAN, //  1/factor, 1/0.076
    .frequency = 50U,
    .angle_rate_up_lookup = {
      {0., 5., 25.},
      {2.5, 1.5, 0.2}
    },
    .angle_rate_down_lookup = {
      {0., 5., 25.},
      {5., 2.0, 0.3}
    },
  };


  // TODO find long params
  const LongitudinalLimits LANDROVER_LONG_LIMITS = {
    .max_accel = 425,       // 2 m/s^2
    .min_accel = 288,       // -3.48 m/s^2
    .inactive_accel = 375,  // 0. m/s^2
  };

  UNUSED(LANDROVER_LONG_LIMITS);

  if (landrover_flexray_harness) {

    if (msg->bus == 1U) {

      // Steering control 
      // (0.07687 * val) - 691.83 in deg.
      // deg_to_can = 1/0.07687 , max_angle = angle * deg_to_can
      if (msg->addr == 0x1F0U) {
        unsigned int raw_angle_can = ((msg->data[3] & 0x3FU) << 8) | msg->data[4];
        int desired_angle = (int)raw_angle_can - 9000;

        bool steer_control_enabled = GET_BIT(msg, 31U) == 1;
        //bool steer_control_enabled = GET_BIT(msg, 55U) == 1;

        if (steer_angle_cmd_checks(desired_angle, steer_control_enabled, LANDROVER_STEERING_LIMITS)) {
          tx = false;
        }
      }
    }
  } else {

  }

  return tx;
}


static safety_config landrover_init(uint16_t param) {
  const uint16_t flexray_param = 1U;
  const uint16_t rr_param = 1U << 15;

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
  };


  landrover_flexray_harness = param == flexray_param;
  landrover_rr2017 = param == rr_param;
  landrover_rr_camera_seen = false;
  landrover_rr_camera_progress = false;

  if (landrover_rr2017) {
    return BUILD_SAFETY_CFG(landrover_rr_rx_checks, LANDROVER_TX_MSGS);
  }
  if (landrover_flexray_harness) {
    return BUILD_SAFETY_CFG(landrover_flexray_rx_checks, LANDROVER_FLEXRAY_TX_MSGS);
  }
  return (safety_config){0};
}

const safety_hooks landrover_hooks = {
  .init = landrover_init,
  .rx = landrover_rx_hook,
  .tx = landrover_tx_hook,
  .get_checksum = landrover_get_checksum,
  .compute_checksum = landrover_compute_checksum,
};
