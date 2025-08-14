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

#if defined(ARDUINO_ARCH_ESP32)

#include <stdexcept>
#include <Arduino.h>
#include "Timer.h"

namespace romi {

        hw_timer_t *step_timer_ = nullptr;
        hw_timer_t *reset_timer_ = nullptr;
        
        TimerCallback step_callback_ = nullptr;
        TimerCallback reset_callback_ = nullptr;

        void init_step_timer(TimerMode mode);
        void init_reset_timer();

        void timer_init(TimerMode mode,
                        TimerCallback step_callback,
                        TimerCallback reset_callback)
        {
                step_callback_ = step_callback;
                reset_callback_ = reset_callback;
                
                init_step_timer(mode);
                init_reset_timer();
        }
        
        void IRAM_ATTR reset_isr()
        {
                reset_callback_();
        }
        
        void init_reset_timer()
        {
                reset_timer_ = timerBegin(1000000);
                if (reset_timer_ == nullptr) {
                        throw std::runtime_error("init_reset_timer: timerBegin failed");
                }
                timerAttachInterrupt(reset_timer_, &reset_isr);
        }
        
        void IRAM_ATTR step_isr()
        {
                step_callback_();
        }

        void init_step_timer(TimerMode mode)
        {
                step_timer_ = timerBegin(1000000);
                if (step_timer_ == nullptr) {
                        throw std::runtime_error("init_step_timer: timerBegin failed");
                }
                
                timerAttachInterrupt(step_timer_, &step_isr);
                switch (mode) {
                case k10kHz:
                        timerAlarm(step_timer_, 100, true, 0);
                        break;
                case k25kHz:
                        timerAlarm(step_timer_, 25, true, 0);
                        break;
                default:
                        throw std::runtime_error("init_step_timer: bad mode");
                }
        }

        void timer_schedule_reset()
        {
                timerAlarm(reset_timer_, 10, false, 0);
        }

        void timer_enable()
        {
                timerStart(step_timer_);
        }

        void timer_disable()
        {
                timerStop(step_timer_);
        }
}

#endif // defined(ARDUINO_ARCH_ESP32)
