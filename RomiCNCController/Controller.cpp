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

#include "Block.h"
#include "Controller.h"

namespace romi {

        Controller *controller_ = nullptr;
        void run_steppers_();
        void reset_step_pins_();                

        Controller::Controller(IStepperController& steppers, ITimer& timer)
                : steppers_(steppers),
                  timer_(timer),
                  dt_(0),
                  current_block_(nullptr),
                  interrupts_(0),
                  milliseconds_(0),
                  reset_(false),
                  interrupts_per_millisecond_(1)
        {
                zero();
        }
        
        void Controller::zero()
        {
                for (int i = 0; i < 3; i++) {
                        position_[i] = 0;
                        accumulation_error_[i] = 0;
                        delta_[i] = 0;
                        step_dir_[i] = 0;
                }
        }
        
        void Controller::init(TimerMode mode)
        {
                controller_ = this;
                zero();
                
                // Serial.print("stepper_init: Mode ");
                // Serial.println(romi::k10kHz);
                
                switch (mode) {
                case k10kHz:
                        timer_.init(k10kHz, run_steppers_, reset_step_pins_);
                        interrupts_per_millisecond_ = 10;
                        break;
                case k25kHz:
                        timer_.init(k25kHz, run_steppers_, reset_step_pins_);
                        interrupts_per_millisecond_ = 25;
                        break;
                default:
                        //Serial.println("stepper_init: invalid mode");
                        while (true) ; // FIXME
                }
        }

        void Controller::reset()
        {
                reset_ = true;
        }

        void Controller::enable()
        {
                steppers_.enable();
                timer_.enable();
        }
        
        void Controller::disable()
        {
                steppers_.disable();
                timer_.disable();
        }
        
        void Controller::get_position(int32_t *pos)
        {
                pos[0] = position_[0];
                pos[1] = position_[1];
                pos[2] = position_[2];
        }

        bool Controller::is_idle()
        {
                return ((current_block_ == nullptr)
                        && (block_buffer_available() == 0));
        }

        void run_steppers_()
        {
                if (controller_)
                        controller_->run_steppers();
        }
        
        void reset_step_pins_()
        {
                if (controller_)
                        controller_->reset_step_pins();
        }
        
        void Controller::reset_step_pins()
        {
                steppers_.reset_step_pins();
        }

        void Controller::run_steppers()
        {
                /* If a reset is requested, set the current block and other
                 * state variables to zero and return. */
                if (reset_) {
                        current_block_ = nullptr;
                        interrupts_ = 0;
                        milliseconds_ = 0;
                        reset_ = false;
                        return;
                }

                /* If there is no block active then pop the next block from
                 * the buffer and do some initialization.  */
                if (current_block_ == nullptr) {
                
                        current_block_ = block_buffer_get_next();

                        if (current_block_ == nullptr) {
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
                                        current_block_ = nullptr;
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
                                        delta_[0] = delta_[0] - position_[0];
                                
                                        // if (delta_[1] < 0)
                                        //         delta_[1] = 0;
                                        // else
                                        delta_[1] = delta_[1] - position_[1];
                                
                                        // if (delta_[2] < 0)
                                        //         delta_[2] = 0;
                                        // else
                                        delta_[2] = delta_[2] - position_[2];
                                }

                                /* Check the directions */
                                if (delta_[0] < 0) {
                                        dir = steppers_.toggle_x_dir(dir);
                                        delta_[0] = -delta_[0];
                                        step_dir_[0] = -1;
                                }
                                if (delta_[1] < 0) {
                                        dir = steppers_.toggle_y_dir(dir);
                                        delta_[1] = -delta_[1];
                                        step_dir_[1] = -1;
                                }
                                if (delta_[2] < 0) {
                                        dir = steppers_.toggle_z_dir(dir);
                                        delta_[2] = -delta_[2];
                                        step_dir_[2] = -1;
                                }

                                /* Set the direction output pins */
                                steppers_.set_dir_pins(dir);

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
                                dt_ = interrupts_per_millisecond_ * (int32_t) current_block_->data[DT];
                        
                                /* Initialize the accumulation error for the
                                 * Bresenham algorithm. */
                                accumulation_error_[0] = delta_[0] - dt_ / 2;
                                accumulation_error_[1] = delta_[1] - dt_ / 2;
                                accumulation_error_[2] = delta_[2] - dt_ / 2;
                        } 
                
                }

                /* Update the interrupt and millisecond counter. */
                interrupts_ = interrupts_ + 1;
                if (interrupts_ == interrupts_per_millisecond_) {
                        interrupts_ = 0;
                        milliseconds_ = milliseconds_ + 1;
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
                                pins = steppers_.toggle_x_step(pins);
                                position_[0] = position_[0]+ step_dir_[0];
                                accumulation_error_[0] = accumulation_error_[0] - dt_;
                        }
                        accumulation_error_[0] = accumulation_error_[0] + delta_[0];

                        // Y-axis
                        if (accumulation_error_[1] > 0) {
                                pins = steppers_.toggle_y_step(pins);
                                position_[1] = position_[1] + step_dir_[1];
                                accumulation_error_[1] = accumulation_error_[1] - dt_;
                        }
                        accumulation_error_[1] = accumulation_error_[1] + delta_[1];

                        // Z-axis
                        if (accumulation_error_[2] > 0) {
                                pins = steppers_.toggle_z_step(pins);
                                position_[2] = position_[2] + step_dir_[2];
                                accumulation_error_[2] = accumulation_error_[2] - dt_;
                        }
                        accumulation_error_[2] = accumulation_error_[2] + delta_[2];

                        // Raise the STEP pins, if needed, and schedule a
                        // reset pins event.
                        if (pins) {
                                steppers_.set_step_pins(pins);
                                timer_.schedule_reset();
                        }

                        // Check whether we have to move to the next block
                        if ((current_block_->type == BLOCK_MOVE 
                             && milliseconds_ >= current_block_->data[DT])
                            || (current_block_->type == BLOCK_MOVETO 
                                && milliseconds_ >= current_block_->data[DT])
                            || (current_block_->type == BLOCK_MOVEAT
                                && block_buffer_available() > 0)) {
                                current_block_ = nullptr;
                        }
                
                        return;
                }
        }
}
