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

namespace doogie_drivers {

int QuadratureEncoder::channel_a_pin_ = 14;
int QuadratureEncoder::channel_b_pin_ = 15;

volatile int QuadratureEncoder::pulses_ = 0;

uint8_t QuadratureEncoder::prev_state_ = 0;
uint8_t QuadratureEncoder::curr_state_ = 0;

int QuadratureEncoder::pulses_per_rev_ = 360;
QuadratureEncoder::Encoding QuadratureEncoder::encoding_ = QuadratureEncoder::Encoding::X4_ENCODING;

QuadratureEncoder::QuadratureEncoder() {
  if (wiringPiSetupGpio() == -1) throw("Error on wiringPi setup");

  pinMode(channel_a_pin_, INPUT);
  pinMode(channel_b_pin_, INPUT);

  // Workout what the current state is.
  uint8_t channel_a_state = digitalRead(channel_a_pin_);
  uint8_t channel_b_state = digitalRead(channel_b_pin_);

  // 2-bit state.
  curr_state_ = (channel_a_state << 1) | (channel_b_state);
  prev_state_ = curr_state_;

  // X2 encoding uses interrupts on only channel A.
  // X4 encoding uses interrupts on channel A and on channel B.
  wiringPiISR(channel_a_pin_, INT_EDGE_BOTH, &QuadratureEncoder::encode);

  // If we're using X4 encoding, then attach interrupts to channel B too.
  if (encoding_ == X4_ENCODING) {
    wiringPiISR(channel_b_pin_, INT_EDGE_BOTH, &QuadratureEncoder::encode);
  }
}

void QuadratureEncoder::reset(void) {
  pulses_ = 0;
}

int QuadratureEncoder::getCurrentState(void) {
  return curr_state_;
}

int QuadratureEncoder::getPulses(void) {
  return pulses_;
}

// +-------------+
// | X2 Encoding |
// +-------------+
//
// When observing states two patterns will appear:
//
// Counter clockwise rotation:
//
// 10 -> 01 -> 10 -> 01 -> ...
//
// Clockwise rotation:
//
// 11 -> 00 -> 11 -> 00 -> ...
//
// We consider counter clockwise rotation to be "forward" and
// counter clockwise to be "backward". Therefore pulse count will increase
// during counter clockwise rotation and decrease during clockwise rotation.
//
// +-------------+
// | X4 Encoding |
// +-------------+
//
// There are four possible states for a quadrature encoder which correspond to
// 2-bit gray code.
//
// A state change is only valid if of only one bit has changed.
// A state change is invalid if both bits have changed.
//
// Clockwise Rotation ->
//
//    00 01 11 10 00
//
// <- Counter Clockwise Rotation
//
// If we observe any valid state changes going from left to right, we have
// moved one pulse clockwise [we will consider this "backward" or "negative"].
//
// If we observe any valid state changes going from right to left we have
// moved one pulse counter clockwise [we will consider this "forward" or
// "positive"].
//
// We might enter an invalid state for a number of reasons which are hard to
// predict - if this is the case, it is generally safe to ignore it, update
// the state and carry on, with the error correcting itself shortly after.
void QuadratureEncoder::encode() {
  int change = 0;
  uint8_t channel_a_state = digitalRead(channel_a_pin_);
  uint8_t channel_b_state = digitalRead(channel_b_pin_);

  // 2-bit state.
  curr_state_ = (channel_a_state << 1) | (channel_b_state);
  if (encoding_ == X2_ENCODING) {
    // 11->00->11->00 is counter clockwise rotation or "forward".
    if ((prev_state_ == 0x3 && curr_state_ == 0x0) || (prev_state_ == 0x0 && curr_state_ == 0x3)) {
      pulses_++;
    } else {
      // 10->01->10->01 is clockwise rotation or "backward".
      if ((prev_state_ == 0x2 && curr_state_ == 0x1) || (prev_state_ == 0x1 && curr_state_ == 0x2)) {
        pulses_--;
      }
    }
  } else {
    if (encoding_ == X4_ENCODING) {
      // Entered a new valid state.
      if (((curr_state_ ^ prev_state_) != INVALID) && (curr_state_ != prev_state_)) {
        // 2 bit state. Right hand bit of prev XOR left hand bit of current
        // gives 0 if clockwise rotation and 1 if counter clockwise rotation.
        change = (prev_state_ & PREV_MASK) ^ ((curr_state_ & CURR_MASK) >> 1);
        if (change == 0) change = -1;
        pulses_ -= change;
      }
    }
  }
  prev_state_ = curr_state_;
}

}  // namespace doogie_drivers
