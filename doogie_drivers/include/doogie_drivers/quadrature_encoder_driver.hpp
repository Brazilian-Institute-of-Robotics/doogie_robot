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
 *
 * @section DESCRIPTION
 *
 * Quadrature Encoder Interface.
 *
 * A quadrature encoder consists of two code tracks on a disc which are 90
 * degrees out of phase. It can be used to determine how far a wheel has
 * rotated, relative to a known starting position.
 *
 * Only one code track changes at a time leading to a more robust system than
 * a single track, because any jitter around any edge won't cause a state
 * change as the other track will remain constant.
 *
 * Encoders can be a homebrew affair, consisting of infrared emitters/receivers
 * and paper code tracks consisting of alternating black and white sections;
 * alternatively, complete disk and PCB emitter/receiver encoder systems can
 * be bought, but the interface, regardless of implementation is the same.
 *
 *               +-----+     +-----+     +-----+
 * Channel A     |  ^  |     |     |     |     |
 *            ---+  ^  +-----+     +-----+     +-----
 *               ^  ^
 *               ^  +-----+     +-----+     +-----+
 * Channel B     ^  |     |     |     |     |     |
 *            ------+     +-----+     +-----+     +-----
 *               ^  ^
 *               ^  ^
 *               90deg
 *
 * The interface uses X2 encoding by default which calculates the pulse count
 * based on reading the current state after each rising and falling edge of
 * channel A.
 *
 *               +-----+     +-----+     +-----+
 * Channel A     |     |     |     |     |     |
 *            ---+     +-----+     +-----+     +-----
 *               ^     ^     ^     ^     ^
 *               ^  +-----+  ^  +-----+  ^  +-----+
 * Channel B     ^  |  ^  |  ^  |  ^  |  ^  |     |
 *            ------+  ^  +-----+  ^  +-----+     +--
 *               ^     ^     ^     ^     ^
 *               ^     ^     ^     ^     ^
 * Pulse count 0 1     2     3     4     5  ...
 *
 * This interface can also use X4 encoding which calculates the pulse count
 * based on reading the current state after each rising and falling edge of
 * either channel.
 *
 *               +-----+     +-----+     +-----+
 * Channel A     |     |     |     |     |     |
 *            ---+     +-----+     +-----+     +-----
 *               ^     ^     ^     ^     ^
 *               ^  +-----+  ^  +-----+  ^  +-----+
 * Channel B     ^  |  ^  |  ^  |  ^  |  ^  |     |
 *            ------+  ^  +-----+  ^  +-----+     +--
 *               ^  ^  ^  ^  ^  ^  ^  ^  ^  ^
 *               ^  ^  ^  ^  ^  ^  ^  ^  ^  ^
 * Pulse count 0 1  2  3  4  5  6  7  8  9  ...
 *
 * It defaults
 *
 * An optional index channel can be used which determines when a full
 * revolution has occured.
 *
 * If a 4 pulses per revolution encoder was used, with X4 encoding,
 * the following would be observed.
 *
 *               +-----+     +-----+     +-----+
 * Channel A     |     |     |     |     |     |
 *            ---+     +-----+     +-----+     +-----
 *               ^     ^     ^     ^     ^
 *               ^  +-----+  ^  +-----+  ^  +-----+
 * Channel B     ^  |  ^  |  ^  |  ^  |  ^  |     |
 *            ------+  ^  +-----+  ^  +-----+     +--
 *               ^  ^  ^  ^  ^  ^  ^  ^  ^  ^
 *               ^  ^  ^  ^  ^  ^  ^  ^  ^  ^
 *               ^  ^  ^  +--+  ^  ^  +--+  ^
 *               ^  ^  ^  |  |  ^  ^  |  |  ^
 * Index      ------------+  +--------+  +-----------
 *               ^  ^  ^  ^  ^  ^  ^  ^  ^  ^
 * Pulse count 0 1  2  3  4  5  6  7  8  9  ...
 * Rev.  count 0          1           2
 *
 * Rotational position in degrees can be calculated by:
 *
 * (pulse count / X * N) * 360
 *
 * Where X is the encoding type [e.g. X4 encoding => X=4], and N is the number
 * of pulses per revolution (PPR). Note that PPR is not the same Counter Per
 * Revolution (CPR). These stands are related by the follow equation:
 * 
 * PPR = CPR / 4
 *
 * Linear position can be calculated by:
 *
 * (pulse count / X * N) * (1 / PPI)
 *
 * Where X is encoding type [e.g. X4 encoding => X=4], N is the number of
 * pulses per revolution, and PPI is pulses per inch, or the equivalent for
 * any other unit of displacement. PPI can be calculated by taking the
 * circumference of the wheel or encoder disk and dividing it by the number
 * of pulses per revolution.
 */

#ifndef DOOGIE_DRIVERS_QUADRATURE_ENCODER_DRIVER_H_
#define DOOGIE_DRIVERS_QUADRATURE_ENCODER_DRIVER_H_

#include <cstdint>
#include <cstddef>

#define PREV_MASK 0x1  // Mask for the previous state in determining direction f rotation.
#define CURR_MASK 0x2  // Mask for the current state in determining direction of rotation.
#define INVALID   0x3  // XORing two states where both bits have changed.

namespace doogie_drivers {

enum EncoderPinsIndex {
  LEFT_ENC_CHA,
  LEFT_ENC_CHB,
  RIGHT_ENC_CHA,
  RIGHT_ENC_CHB
};

enum EncoderSide {
  LEFT_ENC,
  RIGHT_ENC
};

class QuadratureEncoder {
 public:
  enum Encoding {
    X2_ENCODING = 2,
    X4_ENCODING = 4
  };

  /**
   * @brief Construct a new Quadrature Encoder object
   * 
   * @param pulses_per_rev 
   * @param wheel_radius 
   * @param gear_ratio 
   * @param velocity_rolling_window_size 
   */
  QuadratureEncoder(unsigned int counts_per_rev, double wheel_radius, unsigned int gear_ratio = 1);

  /**
   * @brief Setup pins and interrupts
   * 
   */
  void init();

  /**
   * @brief Reset the encoder.
   *
   * Sets the pulses and revolutions count to zero.
   */
  void reset(EncoderSide enc_side);

  /**
   * @brief Read the number of pulses recorded by the encoder.
   *
   * @return Number of pulses which have occured.
   */
  int getPulses(EncoderSide enc_side);

  /**
   * @brief Get the Angular Position of the wheel
   * 
   * Rotational position in rad can be calculated by:
   *
   * (pulse count / X * N) * 2*pi
   *
   * Where X is the encoding type [e.g. X4 encoding => X=4], and N is the number
   * of pulses per revolution.
   * 
   * @param enc_side Robot side where encoder is assembled 
   * @return double Angular poisiton in radians
   */
  double getAngularPosition(EncoderSide enc_side);

  /**
   * @brief Update actuator velocity according to pulses count.
   * 
   * This method should be called periodically to better velocity estimation.
   * 
   */
  void updateVelocity(double dt);
  /**
   * @brief Get the actuator velocity
   * 
   * @param enc_side 
   * @return double 
   */
  double getVelocity(EncoderSide enc_side);

 private:
  /**
   * @brief Update the pulse count.
   *
   * Called on every rising/falling edge of channels A/B.
   *
   * Reads the state of the channels and determines whether a pulse forward
   * or backward has occured, updating the count appropriately.
   */
  static void encodeLeft(void);
  static void encodeRight(void);

  unsigned int counts_per_rev_;
  double pulses_per_meters_;

  static int channel_pins_[4];
  static uint8_t prev_state_[2];
  static uint8_t curr_state_[2];
  static volatile int pulses_[2];

  double old_pulse_cnt_[2];
  double vel_[2];

  static Encoding encoding_;
};

}  // namespace doogie_drivers

#endif  // DOOGIE_DRIVERS_QUADRATURE_ENCODER_DRIVER_HPP_
