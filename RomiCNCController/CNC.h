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
#ifndef _CNC_LOWLEVELCNC_H_
#define _CNC_LOWLEVELCNC_H_

#include "Timer.h"

namespace romi {

        class CNC
        {
        public:
                enum {
                        kStateRunning = 'r',
                        kStatePaused = 'p',
                        kStateHoming = 'h',
                        kStateError = 'e'
                };

                enum {
                        kHomingDefault = 0,
                        kHomingWithContact = 1
                };

                enum {
                        kNoError = 0,
                        kAgain = 1,
                        kInvalidDT = 100,
                        kInvalidState = 101,
                        kInvalidMode = 102,
                        kInvalidAxis = 103,
                        kInvalidSpeed = 104
                };

                static constexpr const char *kNoMessage = "";
                static constexpr const char *kAgainMessage = "Again";
                static constexpr const char *kInvalidDTMessage = "Bad DT";
                static constexpr const char *kInvalidStateMessage = "Bad state";
                static constexpr const char *kInvalidModeMessage = "Bad mode";
                static constexpr const char *kInvalidAxisMessage = "Bad axis";
                static constexpr const char *kInvalidSpeedMessage = "Bad speed";
                
        protected:
                char state_;
                int8_t homing_axes_[3]; // =  {-1, -1, -1};
                int16_t homing_speeds_[3]; // =  {1000, 1000, 400};
                uint8_t homing_mode_; // = kHomingDefault;
                bool limit_switches_[3]; // = {LOW, LOW, LOW};
                int8_t error_code_;
                const char *error_message_;
                
        public:
                CNC();
                ~CNC() {}

                void init();

                void power_up();
                void power_down();
                uint8_t pause();
                uint8_t continue_();
                
                uint8_t reset();
                uint8_t zero();
                
                bool is_idle();
                char get_state();
                bool is_enabled();
                
                uint8_t homing();
                uint8_t homing_mode(uint8_t mode);
                uint8_t homing_axes(int8_t a0, int8_t a1, int8_t a2);
                uint8_t homing_speeds(int16_t vx, int16_t vy, int16_t vz);
                
                uint8_t schedule_moveto(int16_t dt, int16_t dx, int16_t dy, int16_t dz);
                uint8_t schedule_move(int16_t dt, int16_t dx, int16_t dy, int16_t dz);
                uint8_t schedule_moveat(int16_t dx, int16_t dy, int16_t dz);

                void get_position(int32_t *pos);
                void set_spindle(bool state);
                
                uint8_t error_code();
                const char *error_message();
                
        protected:
                bool do_homing();
                bool do_homing_axis(int8_t axis);
                bool do_homing_axis_default(int8_t axis);
                uint8_t homing_moveto_switch_pressed(int8_t axis);
                uint8_t homing_moveto_switch_released(int8_t axis);
                uint8_t homing_wait_switch(int speed, int8_t axis, bool state);
                bool do_homing_axis_contact(int8_t axis);
                void update_limit_switches();
                uint8_t homing_move(int dt, int delta, int8_t axis);
                uint8_t homing_moveat(int v, int8_t axis);                
                void wait();

                void set_error(uint8_t code, const char *message);
                
                void clear_error() {
                        error_code_ = 0;
                }
        };
}

#endif // _CNC_LOWLEVELCNC_H_




