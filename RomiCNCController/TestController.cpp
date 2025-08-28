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

#include "TestController.h"

namespace romi {

        TestController *test_controller_ = nullptr;

        void test_run_steppers_()
        {
                if (test_controller_)
                        test_controller_->run_steppers();
        }
        
        void test_reset_step_pins_()
        {
                if (test_controller_)
                        test_controller_->reset_step_pins();
        }

        void TestController::init(TimerMode mode)
        {
                test_controller_ = this;
                
                switch (mode) {
                case k10kHz:
                        timer_.init(k10kHz, test_run_steppers_,
                                    test_reset_step_pins_);
                        break;
                case k25kHz:
                        timer_.init(k25kHz, test_run_steppers_,
                                    test_reset_step_pins_);
                        break;
                default:
                        while (true) ; // FIXME
                }
        }

}
