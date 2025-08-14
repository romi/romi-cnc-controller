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
#ifndef _CNC_STEPPER_H_
#define _CNC_STEPPER_H_

#include "Timer.h"

namespace romi {

        /**
         * \brief: Configure the stepper. Must be called with interrupts
         * disabled (see cli()).
         *
         */
        void stepper_init(TimerMode mode);


        /**
         * \brief: Sets the current position as the origin.
         *
         */
        void stepper_zero();

        void stepper_reset();
        void stepper_enable();
        void stepper_disable();
        bool stepper_is_idle();
        void stepper_get_position(int32_t *pos);
}

#endif // _CNC_STEPPER_H_
