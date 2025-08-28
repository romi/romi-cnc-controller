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
#include "CNC.h"

namespace romi {

        CNC::CNC(romiserial::RomiSerial& serial,
                 IController& controller,
                 IStepperController& steppers)
                : serial_(serial),
                  controller_(controller),
                  steppers_(steppers),
                  state_(kStateRunning),
                  homing_mode_(kHomingDefault),
                  error_code_(0),
                  error_message_(kNoMessage)
        {
                clear_error();
                for (int i = 0; i < 3; i++) {
                        homing_axes_[i] = -1;
                        homing_speeds_[i] = 1000;
                        limit_switches_[i] = false;
                }
        }

        void CNC::init(TimerMode mode)
        {
                init_block_buffer();
                steppers_.init();
                controller_.init(mode);
                controller_.disable();
        }
        
        void CNC::power_up()
        {
                clear_error();
                controller_.enable();
        }
        
        void CNC::power_down()
        {
                clear_error();
                controller_.disable();
        }
        
        int8_t CNC::pause()
        {
                clear_error();
                if (state_ == kStateRunning) {
                        controller_.disable();
                        state_ = kStatePaused;
                } else if (state_ == kStatePaused) {
                        // all good
                } else {
                        set_error(kInvalidState, kInvalidStateMessage);
                }
                return error_code_;
        }

        int8_t CNC::continue_()
        {
                clear_error();
                if (state_ == kStatePaused) {
                        state_ = kStateRunning;
                        controller_.enable();
                } else if (state_ == kStateRunning) {
                        // all good
                } else {
                        set_error(kInvalidState, kInvalidStateMessage);
                }
                return error_code_;
        }
        
        int8_t CNC::reset()
        {
                clear_error();
                if (state_ == kStatePaused) {
                        state_ = kStateRunning;
                        block_buffer_clear();
                        controller_.reset();
                        controller_.enable();
                } else if (state_ == kStateHoming) {
                        block_buffer_clear();
                        controller_.reset();
                } else {
                        set_error(kInvalidState, kInvalidStateMessage);
                }
                return error_code_;
        }

        int8_t CNC::zero()
        {
                clear_error();
                if (state_ == kStatePaused || is_idle()) {
                        controller_.zero();
                } else {
                        set_error(kInvalidState, kInvalidStateMessage);
                }
                return error_code_;
        }

        bool CNC::is_idle()
        {
                return ((state_ == kStateRunning)
                        && controller_.is_idle());
        }

        char CNC::get_state()
        {
                return state_;
        }

        bool CNC::is_enabled()
        {
                return steppers_.is_enabled();
        }

        void CNC::set_spindle(bool state)
        {
                steppers_.set_relay(0, state);
        }

        int8_t CNC::schedule_moveto(int16_t dt, int16_t dx, int16_t dy, int16_t dz)
        {
                clear_error();
                if (state_ == kStateRunning || state_ == kStatePaused) {
                        if (dt > 0) {
                                block_t *block = block_buffer_get_empty();
                                if (block) {
                                        block->type = BLOCK_MOVETO;
                                        block->data[DT] = dt;
                                        block->data[DX] = dx;
                                        block->data[DY] = dy;
                                        block->data[DZ] = dz;
                                        block_buffer_ready();
                                } else {
                                        set_error(kAgain, kAgainMessage);
                                }
                        } else {
                                set_error(kInvalidDT, kInvalidDTMessage);
                        }
                } else {
                        set_error(kInvalidState, kInvalidStateMessage);
                }
                return error_code_;
        }

        int8_t CNC::schedule_moveat(int16_t dx, int16_t dy, int16_t dz)
        {
                //serial_.log("schedule_moveat");
                clear_error();
                if (state_ == kStateRunning || state_ == kStatePaused) {
                        moveat(dx, dy, dz);
                } else {
                        set_error(kInvalidState, kInvalidStateMessage);
                }
                return error_code_;
        }

        int8_t CNC::moveat(int16_t dx, int16_t dy, int16_t dz)
        {
                //serial_.log("moveat");
                clear_error();
                block_t *block = block_buffer_get_empty();
                if (block) {
                        block->type = BLOCK_MOVEAT;
                        block->data[DT] = 1000;
                        block->data[DX] = dx;
                        block->data[DY] = dy;
                        block->data[DZ] = dz;
                        block_buffer_ready();
                } else {
                        set_error(kAgain, kAgainMessage);
                }
                return error_code_;
        }

        int8_t CNC::schedule_move(int16_t dt, int16_t dx, int16_t dy, int16_t dz)
        {
                clear_error();
                if (state_ == kStateRunning || state_ == kStatePaused) {
                        if (dt > 0) {
                                block_t *block = block_buffer_get_empty();
                                if (block) {
                                        block->type = BLOCK_MOVE;
                                        block->data[DT] = dt;
                                        block->data[DX] = dx;
                                        block->data[DY] = dy;
                                        block->data[DZ] = dz;
                                        block_buffer_ready();
                                } else {
                                        set_error(kAgain, kAgainMessage);
                                }
                        } else {
                                set_error(kInvalidDT, kInvalidDTMessage);
                        }
                } else {
                        set_error(kInvalidState, kInvalidStateMessage);
                }
                        
                return error_code_;
        }

        int8_t CNC::homing_mode(uint8_t mode)
        {
                clear_error();
                if (mode == kHomingDefault
                    || mode == kHomingWithContact) {
                        homing_mode_ = mode;
                } else {
                        set_error(kInvalidMode, kInvalidModeMessage);
                }
                return error_code_;
        }
        
        int8_t CNC::homing_axes(int8_t a0, int8_t a1, int8_t a2)
        {
                clear_error();
                if (a0 >= -1 && a0 <= 2) {
                        homing_axes_[0] = a0;
                } else {
                        set_error(kInvalidAxis, kInvalidAxisMessage); 
                }
                if (a1 >= -1 && a1 <= 2) {
                        homing_axes_[1] = a1;
                } else {
                        set_error(kInvalidAxis, kInvalidAxisMessage); 
                }
                if (a2 >= -1 && a2 <= 2) {
                        homing_axes_[2] = a2;
                } else {
                        set_error(kInvalidAxis, kInvalidAxisMessage); 
                }
                return error_code_;
        }
        
        int8_t CNC::homing_speeds(int16_t v0, int16_t v1, int16_t v2)
        {
                clear_error();
                if ((homing_axes_[0] >= 0) && (v0 <= 0 || v0 > 2000)) {
                        set_error(kInvalidSpeed, kInvalidSpeedMessage); 
                } else {
                        homing_speeds_[0] = v0;
                }
                if ((homing_axes_[1] >= 0) && (v1 <= 0 || v1 > 2000)) {
                        set_error(kInvalidSpeed, kInvalidSpeedMessage); 
                } else {
                        homing_speeds_[1] = v1;
                }
                if ((homing_axes_[2] >= 0) && (v2 <= 0 || v2 > 2000)) {
                        set_error(kInvalidSpeed, kInvalidSpeedMessage); 
                } else {
                        homing_speeds_[2] = v2;
                }
                return error_code_;
        }

        int8_t CNC::homing()
        {
                //serial_.log("homing");
                
                clear_error();
                
                // Stop whatever is ongoing
                reset();

                // Make sure the timer is running
                controller_.enable();

                state_ = kStateHoming;
        
                if (do_homing()) {
                        reset();
                        controller_.zero();
                        state_ = kStateRunning;
                } else {
                        state_ = kStateError;
                }

                return error_code_;
        }

        bool CNC::do_homing()
        {
                //serial_.log("do_homing");
                bool success = true;
                for (uint8_t i = 0; i < 3; i++) {
                        int8_t axis = homing_axes_[i];
                        if (axis >= 0 && axis < 3) {
                                success = do_homing_axis(axis);
                                if (!success)
                                        break;
                        }
                }
                //serial_.log("do_homing DONE");
                return success;
        }

        bool CNC::do_homing_axis(int8_t axis)
        {
                //serial_.log("do_homing_axis");
                bool success = false; 
                if (homing_mode_ == kHomingDefault) {
                        success = do_homing_axis_default(axis);
                } else if (homing_mode_ == kHomingWithContact) {
                        success = do_homing_axis_contact(axis);
                }
                //serial_.log("do_homing_axis DONE");
                return success;
        }

        bool CNC::do_homing_axis_default(int8_t axis)
        {
                bool success = false; 
                if (homing_moveto_switch_pressed(axis) == 0 
                    && homing_moveto_switch_released(axis) == 0
                    && homing_move(100, homing_speeds_[axis]/5, axis) == 0) {
                        success = true;
                }
                return success;
        }

        int8_t CNC::homing_moveto_switch_pressed(int8_t axis)
        {
                //serial_.log("homing_moveto_switch_pressed");
                return homing_wait_switch(-homing_speeds_[axis], axis, false);
        }

        int8_t CNC::homing_moveto_switch_released(int8_t axis)
        {
                return homing_wait_switch(homing_speeds_[axis], axis, true);
        }

        int8_t CNC::homing_wait_switch(int16_t speed, int8_t axis, bool state)
        {
                // if (state) 
                        //serial_.log("homing_wait_switch: till state true");
                // else
                        //serial_.log("homing_wait_switch: till state false");
                
                int8_t err = 0;
        
                err = homing_moveat(speed, axis);
                if (err != 0)
                        return err;
        
                while (1) {
                        update_limit_switches();

                        // if (limit_switches_[axis]) 
                                //serial_.log("Switch true");
                        // else
                                //serial_.log("Switch false");

                        if (limit_switches_[axis] == state) {
                                //serial_.log("Homing axis done!");
                                // This will stop the moveat
                                err = moveat(0, 0, 0);
                                err = reset();
                                break;
                        }
                
                        serial_.handle_input();
                        //delay(1);
                }
                //serial_.log("homing_wait_switch done");
                return err;
        }

        bool CNC::do_homing_axis_contact(int8_t axis)
        {
                //serial_.log("do_homing_axis_contact");
                bool success = false; 
                if (homing_moveto_switch_pressed(axis) == 0) {
                        success = true;
                }
                //serial_.log("do_homing_axis_contact DONE");
                return success;
        }

        void CNC::wait()
        {
                while (!is_idle()) {
                        //romiSerial.handle_input();
                        //delay(1);
                        ;
                }
        }

        void CNC::update_limit_switches()
        {
                limit_switches_[0] = steppers_.x_limit_switch();
                limit_switches_[1] = steppers_.y_limit_switch();
                limit_switches_[2] = steppers_.z_limit_switch();
        }

        int8_t CNC::homing_move(int16_t dt, int16_t delta, int8_t axis)
        {
                int8_t r;
                if (axis == 0)
                        r = schedule_move(dt, delta, 0, 0);
                else if (axis == 1)
                        r = schedule_move(dt, 0, delta, 0);
                else if (axis == 2)
                        r = schedule_move(dt, 0, 0, delta);
                return r;
        }

        int8_t CNC::homing_moveat(int16_t v, int8_t axis)
        {
                //serial_.log("homing_moveat");
                int8_t r;
                if (axis == 0)
                        r = moveat(v, 0, 0);
                else if (axis == 1)
                        r = moveat(0, v, 0);
                else if (axis == 2)
                        r = moveat(0, 0, v);
                return r;
        }

        void CNC::get_position(int32_t *pos)
        {
                controller_.get_position(pos);
        }
        
        void CNC::set_error(int8_t code, const char *message)
        {
                error_code_ = code;
                error_message_ = message;
        }

        int8_t CNC::error_code()
        {
                return error_code_;
        }

        const char *CNC::error_message()
        {
                return error_message_;
        }
}
