/*
  romi-rover

  Copyright (C) 2019 Sony Computer Science Laboratories
  Author(s) Peter Hanappe

  romi-rover is collection of applications for the Romi Rover.

  romi-rover is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see
  <http://www.gnu.org/licenses/>.

 */

#include <Arduino.h>
#include "Controller.h"
#include "Block.h"
#include "Timer.h"
#include "Stepper.h"

namespace romi {

        volatile int32_t stepper_position_[3];
        volatile int32_t accumulation_error_[3];
        volatile int32_t delta_[3];
        volatile int32_t dt_;
        volatile int16_t step_dir_[3];
        volatile block_t *current_block_;
        volatile int32_t interrupts_ = 0;
        volatile int16_t milliseconds_ = 0;
        volatile int8_t stepper_reset_ = 0;
        static int32_t interrupts_per_millisecond;

        void run_steppers();

        void stepper_zero()
        {
                for (int i = 0; i < 3; i++) {
                        stepper_position_[i] = 0;
                        accumulation_error_[i] = 0;
                }
        }

        void stepper_reset()
        {
                stepper_reset_ = 1;
        }

        void stepper_enable()
        {
                timer_enable();
        }
        
        void stepper_disable()
        {
                timer_disable();
        }
        
        void stepper_init(TimerMode mode)
        {
                stepper_zero();
                
                // Serial.print("stepper_init: Mode ");
                // Serial.println(romi::k10kHz);
                
                switch (mode) {
                case k10kHz:
                        timer_init(k10kHz, run_steppers, controller_reset_step_pins);
                        interrupts_per_millisecond = 10;
                        break;
                case k25kHz:
                        timer_init(k25kHz, run_steppers, controller_reset_step_pins);
                        interrupts_per_millisecond = 25;
                        break;
                default:
                        Serial.println("stepper_init: invalid mode");
                        while (true) ; // FIXME
                }
        }

        void stepper_get_position(int32_t *pos)
        {
                pos[0] = stepper_position_[0];
                pos[1] = stepper_position_[1];
                pos[2] = stepper_position_[2];
        }

        bool stepper_is_idle()
        {
                return ((current_block_ == 0)
                        && (block_buffer_available() == 0));
        }

        /**
         * \brief The interrupt service routine for the stepper timer.
         *
         */
        void run_steppers()
        {
                /* If a reset is requested, set the current block and other
                 * state variables to zero and return. */
                if (stepper_reset_) {
                        current_block_ = 0;
                        interrupts_ = 0;
                        milliseconds_ = 0;
                        stepper_reset_ = 0;
                        return;
                }

                /* If there is no block active then pop the next block from
                 * the buffer and do some initialization.  */
                if (current_block_ == 0) {
                
                        current_block_ = block_buffer_get_next();

                        if (current_block_ == 0) {
                                return;
                        }
                
                        /* Do the necessary initializations for the new
                         * block. */
                
                        interrupts_ = 0;
                        milliseconds_ = 0;
                
                        if (current_block_->type == BLOCK_MOVE
                            || current_block_->type == BLOCK_MOVETO
                            || current_block_->type == BLOCK_MOVEAT) {

                                /* Sanity check */
                                if (current_block_->type == BLOCK_MOVE
                                    && current_block_->data[DT] <= 0) {
                                        current_block_ = 0;
                                        return;
                                }
                        
                                /* Check the direction and set DIR pins. */
                                uint8_t dir = 0;
                                step_dir_[0] = 1;
                                step_dir_[1] = 1;
                                step_dir_[2] = 1;
                        
                                delta_[0] = (int32_t) current_block_->data[DX];
                                delta_[1] = (int32_t) current_block_->data[DY];
                                delta_[2] = (int32_t) current_block_->data[DZ];

                                /* For moveto events, substract the current
                                 * position of the CNC. */
                                if (current_block_->type == BLOCK_MOVETO) {
                                        // if (delta_[0] < 0)
                                        //         delta_[0] = 0;
                                        // else
                                        delta_[0] -= stepper_position_[0];
                                
                                        // if (delta_[1] < 0)
                                        //         delta_[1] = 0;
                                        // else
                                        delta_[1] -= stepper_position_[1];
                                
                                        // if (delta_[2] < 0)
                                        //         delta_[2] = 0;
                                        // else
                                        delta_[2] -= stepper_position_[2];
                                }

                                /* Check the directions */
                                if (delta_[0] < 0) {
                                        dir = controller_toggle_x_dir(dir);
                                        delta_[0] = -delta_[0];
                                        step_dir_[0] = -1;
                                }
                                if (delta_[1] < 0) {
                                        dir = controller_toggle_y_dir(dir);
                                        delta_[1] = -delta_[1];
                                        step_dir_[1] = -1;
                                }
                                if (delta_[2] < 0) {
                                        dir = controller_toggle_z_dir(dir);
                                        delta_[2] = -delta_[2];
                                        step_dir_[2] = -1;
                                }

                                /* Set the direction output pins */
                                controller_set_dir_pins(dir);

                                /* For a moveto, compute the duration of the
                                 * movement (in ms), based on the requested
                                 * speed given in current_block_->data[DT] (in
                                 * steps/s).
                                 * 
                                 * T = 1000 ms/s x n steps / V steps/s
                                 */
                                if (current_block_->type == BLOCK_MOVETO) {
                                        int32_t n = delta_[0];
                                        if (delta_[1] > n)
                                                n = delta_[1];
                                        if (delta_[2] > n)
                                                n = delta_[2];
                                        int32_t T = 1000 * n / (int32_t) current_block_->data[DT];
                                
                                        // Replace the speed value in the
                                        // current block by the computed
                                        // duration.
                                        current_block_->data[DT] = (int16_t) T;
                                }
                        
                                /* The number of interrupts during which the
                                 * stepper positions are updated is equal to
                                 * the length of the segment in milliseconds
                                 * times the number of interrupts per
                                 * millisecond. */
                                dt_ = interrupts_per_millisecond * (int32_t) current_block_->data[DT];
                        
                                /* Initialize the accumulation error for the
                                 * Bresenham algorithm. */
                                accumulation_error_[0] = delta_[0] - dt_ / 2;
                                accumulation_error_[1] = delta_[1] - dt_ / 2;
                                accumulation_error_[2] = delta_[2] - dt_ / 2;
                        } 
                
                }

                /* Update the interrupt and millisecond counter. */
                if (++interrupts_ == interrupts_per_millisecond) {
                        interrupts_ = 0;
                        milliseconds_++;
                }
        
                /* Move and moveat block: 99.9% of the time, we will end up
                 * here. The block below handles the movement of the
                 * steppers. It must be executed as quickly as possible. Just
                 * like Grbl, the handler uses Bresenham's algorithm to
                 * determine when a STEP pin should be raised. The
                 * implementation here is based on the pseudo code found in
                 * Wikipedia
                 * (https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm).
                 */
                if (current_block_->type == BLOCK_MOVE
                    || current_block_->type == BLOCK_MOVETO
                    || current_block_->type == BLOCK_MOVEAT) {
                
                        uint8_t pins = 0;

                        // X-axis
                        if (accumulation_error_[0] > 0) {
                                pins = controller_toggle_x_step(pins);
                                stepper_position_[0] += step_dir_[0];
                                accumulation_error_[0] -= dt_;
                        }
                        accumulation_error_[0] += delta_[0];

                        // Y-axis
                        if (accumulation_error_[1] > 0) {
                                pins = controller_toggle_y_step(pins);
                                stepper_position_[1] += step_dir_[1];
                                accumulation_error_[1] -= dt_;
                        }
                        accumulation_error_[1] += delta_[1];

                        // Z-axis
                        if (accumulation_error_[2] > 0) {
                                pins = controller_toggle_z_step(pins);
                                stepper_position_[2] += step_dir_[2];
                                accumulation_error_[2] -= dt_;
                        }
                        accumulation_error_[2] += delta_[2];

                        // Raise the STEP pins, if needed, and schedule a
                        // reset pins event.
                        if (pins) {
                                controller_set_step_pins(pins);
                                timer_schedule_reset();
                        }

                        // Check whether we have to move to the next block
                        if ((current_block_->type == BLOCK_MOVE 
                             && milliseconds_ >= current_block_->data[DT])
                            || (current_block_->type == BLOCK_MOVETO 
                                && milliseconds_ >= current_block_->data[DT])
                            || (current_block_->type == BLOCK_MOVEAT
                                && block_buffer_available() > 0)) {
                                current_block_ = 0;
                        }
                
                        return;
                }
        }
}
