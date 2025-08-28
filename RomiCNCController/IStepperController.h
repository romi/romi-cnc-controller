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
#ifndef _CNC_ISTEPPERCONTROLLER_H_
#define _CNC_ISTEPPERCONTROLLER_H_

#include <stdint.h>

namespace romi {

        class IStepperController {
        public:
                virtual ~IStepperController() = default;
                
                virtual void init() = 0;

                virtual void enable() = 0;
                virtual void disable() = 0;
                virtual bool is_enabled() = 0;

                virtual uint8_t toggle_x_dir(uint8_t dir) = 0;
                virtual uint8_t toggle_y_dir(uint8_t dir) = 0;
                virtual uint8_t toggle_z_dir(uint8_t dir) = 0;
                virtual void set_dir_pins(uint8_t dir) = 0;

                virtual uint8_t toggle_x_step(uint8_t step) = 0;
                virtual uint8_t toggle_y_step(uint8_t step) = 0;
                virtual uint8_t toggle_z_step(uint8_t step) = 0;
                virtual void set_step_pins(uint8_t step) = 0;
                virtual void reset_step_pins() = 0;

                virtual bool x_limit_switch() = 0;
                virtual bool y_limit_switch() = 0;
                virtual bool z_limit_switch() = 0;

                virtual void set_relay(uint8_t index, bool state) = 0;
        };        
}

#endif // _CNC_ISTEPPERCONTROLLER_H_
