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
#ifndef _CNC_TESTCONTROLLER_H_
#define _CNC_TESTCONTROLLER_H_

#include "IStepperController.h"
#include "ITimer.h"
#include "IController.h"

namespace romi {

        class TestController : public IController
        {
        protected:
                IStepperController& steppers_;
                ITimer& timer_;
                bool pull_ = false;
                
                
                friend void test_run_steppers_();
                
                void run_steppers() {
                        if (pull_) {
                                pull_ = false;
                                timer_.schedule_reset();
                        } else {
                                pull_ = true;
                        }
                }
                
                friend void test_reset_step_pins_();                
                void reset_step_pins() {
                        steppers_.reset_step_pins();
                }
                
        public:
                TestController(IStepperController& steppers, ITimer& timer)
                        : steppers_(steppers), timer_(timer) {
                }
                
                ~TestController() override = default;
                
                TestController(const TestController&) = delete;
                TestController(TestController&&) = delete;
                TestController& operator=(const TestController&) = delete;
                TestController& operator=(TestController&&) = delete;
                
                void init(TimerMode mode) override;
                void zero() override {}
                void reset() override {}
                void enable() override {}
                void disable() override {}
                bool is_idle() override { return true; }
                
                void get_position(int32_t *pos) override {
                        for (int i = 0; i < 3; i++)
                                pos[i] = 0;
                }
        };
}

#endif // _CNC_TESTCONTROLLER_H_
