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

#include <Arduino.h>
#include "XXXController.h"

namespace romi {

        static const uint8_t kXMask = 1;
        static const uint8_t kYMask = 2;
        static const uint8_t kZMask = 4;
        
        /* Configuration of the pins: */
        static const uint8_t kXStepPin = 2;
        static const uint8_t kYStepPin = 3;
        static const uint8_t kZStepPin = 4;
        static const uint8_t kXDirPin = 5;
        static const uint8_t kYDirPin = 6;
        static const uint8_t kZDirPin = 7;
        static const uint8_t kEnablePin = 8;
        static const uint8_t kXLimitSwitch = 9;
        static const uint8_t kYLimitSwitch = 10;
        static const uint8_t kZLimitSwitch = 11;
        static const uint8_t kSpindlePin = 12;

#define ENABLE_PIN_HIGH 1   

        static bool is_enabled_ = false;
        
        /**
         * \brief Configure the step and dir pins as output.
         */
        void controller_init() 
        {
                // Set as output pins
                pinMode(kXStepPin, OUTPUT);  
                pinMode(kYStepPin, OUTPUT);  
                pinMode(kZStepPin, OUTPUT);  
                pinMode(kXDirPin, OUTPUT);  
                pinMode(kYDirPin, OUTPUT);  
                pinMode(kZDirPin, OUTPUT);  
                pinMode(kEnablePin, OUTPUT);  
                pinMode(kSpindlePin, OUTPUT);  
        
                // Set as input pins and enable internal pull-up
                // resistors. Normal high operation
                pinMode(kXLimitSwitch, INPUT_PULLUP);  
                pinMode(kYLimitSwitch, INPUT_PULLUP);  
                pinMode(kZLimitSwitch, INPUT_PULLUP);  

                digitalWrite(kSpindlePin, LOW);
                controller_disable();                
        }

        void set_enable_pin_high()
        {
                digitalWrite(kEnablePin, HIGH);                
        }

        void set_enable_pin_low()
        {
                digitalWrite(kEnablePin, LOW);                
        }

        void controller_enable()
        {
#if ENABLE_PIN_HIGH
                set_enable_pin_high();
#else
                set_enable_pin_low();
#endif
                is_enabled_ = true;
        }

        void controller_disable()
        {
#if ENABLE_PIN_HIGH
                set_enable_pin_low();
#else
                set_enable_pin_high();
#endif
                is_enabled_ = false;
        }

        bool controller_is_enabled()
        {
                return is_enabled_;
        }

        uint8_t controller_toggle_x_dir(uint8_t mask)
        {
                mask |= kXMask;
                return mask;
        }
                
        uint8_t controller_toggle_y_dir(uint8_t mask)
        {
                mask |= kYMask;
                return mask;
        }
        
        uint8_t controller_toggle_z_dir(uint8_t mask)
        {
                mask |= kZMask;
                return mask;
        }

        void controller_set_dir_pins(uint8_t pins)                            
        {
                int value;

                value = (pins & kXMask)? HIGH : LOW;
                digitalWrite(kXDirPin, value);

                value = (pins & kYMask)? HIGH : LOW;
                digitalWrite(kYDirPin, value);

                value = (pins & kZMask)? HIGH : LOW;
                digitalWrite(kZDirPin, value);
        }

        /**
         * \brief Toggle a step bit in the pins mask.
         */

        uint8_t controller_toggle_x_step(uint8_t mask)
        {
                mask |= kXMask;
                return mask;
        }
        
        uint8_t controller_toggle_y_step(uint8_t mask)
        {
                mask |= kYMask;
                return mask;
        }
                
        uint8_t controller_toggle_z_step(uint8_t mask)
        {
                mask |= kZMask;
                return mask;
        }

        /**
         * \brief Enable the step pins according to mask.
         */
        void controller_set_step_pins(uint8_t pins) 
        {
                int value;

                value = (pins & kXMask)? HIGH : LOW;
                digitalWrite(kXStepPin, value);

                value = (pins & kYMask)? HIGH : LOW;
                digitalWrite(kYStepPin, value);

                value = (pins & kZMask)? HIGH : LOW;
                digitalWrite(kZStepPin, value);
        }
        
        /**
         * \brief Reset the step pins to zero.
         */
        void controller_reset_step_pins()
        {
                digitalWrite(kXStepPin, LOW);
                digitalWrite(kYStepPin, LOW);
                digitalWrite(kZStepPin, LOW);
        }

        bool controller_x_limit_switch()
        {
                return digitalRead(kXLimitSwitch) == HIGH;
        }
        
        bool controller_y_limit_switch()
        {
                return digitalRead(kYLimitSwitch) == HIGH;
        }
        
        bool controller_z_limit_switch()
        {
                return digitalRead(kZLimitSwitch) == HIGH;
        }
        
        void controller_set_spindle(bool state)
        {
                int value = state? HIGH : LOW;
                digitalWrite(kSpindlePin, value);
        }

}

#endif // defined(ARDUINO_AVR_UNO)
