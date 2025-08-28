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
#ifndef _CNC_CONTROLLER_H_
#define _CNC_CONTROLLER_H_

#include "IStepperController.h"
#include "ITimer.h"
#include "Block.h"
#include "IController.h"

namespace romi {

        class Controller : public IController
        {
        protected:
                IStepperController& steppers_;
                ITimer& timer_;
                volatile int32_t position_[3];
                volatile int32_t accumulation_error_[3];
                volatile int32_t delta_[3];
                volatile int32_t dt_;
                volatile int16_t step_dir_[3];
                volatile block_t *current_block_;
                volatile int32_t interrupts_ = 0;
                volatile int16_t milliseconds_ = 0;
                volatile bool reset_ = 0;
                int32_t interrupts_per_millisecond_;

                friend void run_steppers_();                
                void run_steppers();
                
                friend void reset_step_pins_();                
                void reset_step_pins();
                
        public:
                Controller(IStepperController& steppers, ITimer& timer);
                ~Controller() override = default;
                
                Controller(const Controller&) = delete;
                Controller(Controller&&) = delete;
                Controller& operator=(const Controller&) = delete;
                Controller& operator=(Controller&&) = delete;
                
                void init(TimerMode mode) override;
                void zero() override;
                void reset() override;
                void enable() override;
                void disable() override;
                bool is_idle() override;
                void get_position(int32_t *pos) override;
        };
}

#endif // _CNC_CONTROLLER_H_
