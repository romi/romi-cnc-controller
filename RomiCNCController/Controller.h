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

#include <stdint.h>

namespace romi {
        void controller_init();

        void controller_enable();
        void controller_disable();
        bool controller_is_enabled();

        uint8_t controller_toggle_x_dir(uint8_t dir);
        uint8_t controller_toggle_y_dir(uint8_t dir);
        uint8_t controller_toggle_z_dir(uint8_t dir);
        void controller_set_dir_pins(uint8_t dir);

        uint8_t controller_toggle_x_step(uint8_t step);
        uint8_t controller_toggle_y_step(uint8_t step);
        uint8_t controller_toggle_z_step(uint8_t step);
        void controller_set_step_pins(uint8_t step);
        void controller_reset_step_pins();

        bool controller_x_limit_switch();
        bool controller_y_limit_switch();
        bool controller_z_limit_switch();

        void controller_set_spindle(bool state);
}

#endif // _CNC_CONFIG_H_
