/**
 * @author Aaron Berk
 *
 * @section LICENSE
 *
 * Copyright (c) 2010 ARM Limited
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "doogie_drivers/quadrature_encoder_driver.hpp"
#include "wiringPi.h"
#include <cstddef>
#include <math.h>

namespace doogie_drivers {

int QuadratureEncoder::channel_pins_[4] = {14, 15, 16, 20};
volatile int QuadratureEncoder::pulses_[2] = {0, 0};
uint8_t QuadratureEncoder::prev_state_[2] = {0, 0};
uint8_t QuadratureEncoder::curr_state_[2] = {0, 0};

QuadratureEncoder::Encoding QuadratureEncoder::encoding_ = QuadratureEncoder::Encoding::X2_ENCODING;

QuadratureEncoder::QuadratureEncoder(unsigned int pulses_per_rev, double wheel_radius, unsigned int gear_ratio) {
  this->pulses_per_rev_ = pulses_per_rev * gear_ratio;
  this->pulses_per_meters_ = (2.0f * M_PI * wheel_radius) / this->pulses_per_rev_;
}

void QuadratureEncoder::init() {
  if (wiringPiSetupGpio() == -1) throw("Error on wiringPi setup");

  size_t i = 0;
  uint8_t channels_state[4];
  for (i = 0; i < 4; i++) {
    pinMode(channel_pins_[i], INPUT);
    channels_state[i] = digitalRead(channel_pins_[i]);  // Workout what the current state is.
  }

  // 2-bit state.
  curr_state_[LEFT_ENC] = (channels_state[LEFT_ENC_CHA] << 1) | (channels_state[LEFT_ENC_CHB]);
  curr_state_[RIGHT_ENC] = (channels_state[RIGHT_ENC_CHA] << 1) | (channels_state[RIGHT_ENC_CHB]);
  prev_state_[LEFT_ENC] = curr_state_[LEFT_ENC];
  prev_state_[RIGHT_ENC] = curr_state_[RIGHT_ENC];

  // X2 encoding uses interrupts on only channel A.
  // X4 encoding uses interrupts on channel A and on channel B.
  wiringPiISR(channel_pins_[LEFT_ENC_CHA], INT_EDGE_BOTH, &QuadratureEncoder::encodeLeft);
  wiringPiISR(channel_pins_[RIGHT_ENC_CHA], INT_EDGE_BOTH, &QuadratureEncoder::encodeRight);

  // If we're using X4 encoding, then attach interrupts to channel B too.
  if (encoding_ == X4_ENCODING) {
    wiringPiISR(channel_pins_[LEFT_ENC_CHB], INT_EDGE_BOTH, &QuadratureEncoder::encodeLeft);
    wiringPiISR(channel_pins_[RIGHT_ENC_CHB], INT_EDGE_BOTH, &QuadratureEncoder::encodeRight);
  }
}

void QuadratureEncoder::reset(EncoderSide enc_side) {
  pulses_[enc_side] = 0;
}

int QuadratureEncoder::getCurrentState(EncoderSide enc_side) {
  return curr_state_[enc_side];
}

int QuadratureEncoder::getPulses(EncoderSide enc_side) {
  return pulses_[enc_side];
}

double QuadratureEncoder::getAngularPosition(EncoderSide enc_side) {
  double count_factor;
  encoding_ == X2_ENCODING ? count_factor = 2.0 : count_factor = 4.0;
  double ang_position = static_cast<double>(pulses_[enc_side]) / (count_factor * static_cast<double>(pulses_per_rev_));
  ang_position *= 2 * M_PI;
  return ang_position;
}

double QuadratureEncoder::getLinearPosition(EncoderSide enc_side) {
  double lin_poisition = static_cast<double>(pulses_[enc_side]) / static_cast<double>(encoding_ * pulses_per_rev_);
  lin_poisition *= (1.0f / pulses_per_meters_);
  return lin_poisition;
}

/** +-------------+
 * | X2 Encoding |
 * +-------------+
 *
 * When observing states two patterns will appear:
 *
 * Counter clockwise rotation:
 *
 * 10 -> 01 -> 10 -> 01 -> ...
 *
 * Clockwise rotation:
 *
 * 11 -> 00 -> 11 -> 00 -> ...
 *
 * We consider counter clockwise rotation to be "forward" and
 * counter clockwise to be "backward". Therefore pulse count will increase
 * during counter clockwise rotation and decrease during clockwise rotation.
 *
 * +-------------+
 * | X4 Encoding |
 * +-------------+
 *
 * There are four possible states for a quadrature encoder which correspond to
 * 2-bit gray code.
 *
 * A state change is only valid if of only one bit has changed.
 * A state change is invalid if both bits have changed.
 *
 * Clockwise Rotation ->
 *
 *    00 01 11 10 00
 *
 * <- Counter Clockwise Rotation
 *
 * If we observe any valid state changes going from left to right, we have
 * moved one pulse clockwise [we will consider this "backward" or "negative"].
 *
 * If we observe any valid state changes going from right to left we have
 * moved one pulse counter clockwise [we will consider this "forward" or
 * "positive"].
 *
 * We might enter an invalid state for a number of reasons which are hard to
 * predict - if this is the case, it is generally safe to ignore it, update
 * the state and carry on, with the error correcting itself shortly after.
 * 
 * Note that these concepts apply to the left engine. The right engine is
 * mirrored relative to the left. Thus, the direction of rotation is reverse
 * and the encoder reading code of both motors differs slightly. 
 */
void QuadratureEncoder::encodeLeft() {
  int change = 0;
  uint8_t channel_a_state = digitalRead(channel_pins_[LEFT_ENC_CHA]);
  uint8_t channel_b_state = digitalRead(channel_pins_[LEFT_ENC_CHB]);

  curr_state_[LEFT_ENC] = (channel_a_state << 1) | (channel_b_state);  // 2-bit state.
  if (encoding_ == X2_ENCODING) {
    // 11->00->11->00 is counter clockwise rotation or "forward" for left motor.
    if ((prev_state_[LEFT_ENC] == 0x3 && curr_state_[LEFT_ENC] == 0x0) ||
       (prev_state_[LEFT_ENC] == 0x0 && curr_state_[LEFT_ENC] == 0x3)) {
      pulses_[LEFT_ENC]++;
    } else {
      // 10->01->10->01 is clockwise rotation or "backward" for left motor.
      if ((prev_state_[LEFT_ENC] == 0x2 && curr_state_[LEFT_ENC] == 0x1) ||
          (prev_state_[LEFT_ENC] == 0x1 && curr_state_[LEFT_ENC] == 0x2)) {
        pulses_[LEFT_ENC]--;
      }
    }
  } else {
    if (encoding_ == X4_ENCODING) {
      // Entered a new valid state.
      if (((curr_state_[LEFT_ENC] ^ prev_state_[LEFT_ENC]) != INVALID) &&
          (curr_state_[LEFT_ENC] != prev_state_[LEFT_ENC])) {
        // 2 bit state. Right hand bit of prev XOR left hand bit of current
        // gives 0 if clockwise rotation and 1 if counter clockwise rotation.
        change = (prev_state_[LEFT_ENC] & PREV_MASK) ^ ((curr_state_[LEFT_ENC] & CURR_MASK) >> 1);
        if (change == 0) change = -1;
        pulses_[LEFT_ENC] -= change;
      }
    }
  }
  prev_state_[LEFT_ENC] = curr_state_[LEFT_ENC];
}

void QuadratureEncoder::encodeRight() {
  int change = 0;
  uint8_t channel_a_state = digitalRead(channel_pins_[RIGHT_ENC_CHA]);
  uint8_t channel_b_state = digitalRead(channel_pins_[RIGHT_ENC_CHB]);

  curr_state_[RIGHT_ENC] = (channel_a_state << 1) | (channel_b_state);  // 2-bit state.
  if (encoding_ == X2_ENCODING) {
    // 11->00->11->00 is counter clockwise rotation or "backward" for right motor.
    if ((prev_state_[RIGHT_ENC] == 0x3 && curr_state_[RIGHT_ENC] == 0x0) ||
       (prev_state_[RIGHT_ENC] == 0x0 && curr_state_[RIGHT_ENC] == 0x3)) {
      pulses_[RIGHT_ENC]--;
    } else {
      // 10->01->10->01 is clockwise rotation or "forward" for right motor.
      if ((prev_state_[RIGHT_ENC] == 0x2 && curr_state_[RIGHT_ENC] == 0x1) ||
          (prev_state_[RIGHT_ENC] == 0x1 && curr_state_[RIGHT_ENC] == 0x2)) {
        pulses_[RIGHT_ENC]++;
      }
    }
  } else {
    if (encoding_ == X4_ENCODING) {
      // Entered a new valid state.
      if (((curr_state_[RIGHT_ENC] ^ prev_state_[RIGHT_ENC]) != INVALID) &&
          (curr_state_[RIGHT_ENC] != prev_state_[RIGHT_ENC])) {
        // 2 bit state. Right hand bit of prev XOR left hand bit of current
        // gives 1 if clockwise rotation and 0 if counter clockwise rotation.
        change = (prev_state_[RIGHT_ENC] & PREV_MASK) ^ ((curr_state_[RIGHT_ENC] & CURR_MASK) >> 1);
        if (change == 0) change = -1;
        pulses_[RIGHT_ENC] += change;
      }
    }
  }
  prev_state_[RIGHT_ENC] = curr_state_[RIGHT_ENC];
}

}  // namespace doogie_drivers
