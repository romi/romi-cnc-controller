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

#if defined(ARDUINO_AVR_UNO)

#include <avr/io.h>
#include <Arduino.h>
#include "Controller.h"

namespace romi {

// The list of stepper controllers that we handle  
#define GSHIELD_CONTROLLER 1
#define XCARVE_CONTROLLER 2 
#define CABLEBOT_CONTROLLER 3 

#define CONTROLLER XCARVE_CONTROLLER
        
// Romi Rover 
#if CONTROLLER == GSHIELD_CONTROLLER
#define ENABLE_PIN_HIGH      1   
#define PIN_LIMIT_SWITCH_Z   11
#define PIN_SPINLDE          12

// XCarve
#elif CONTROLLER == XCARVE_CONTROLLER
#define ENABLE_PIN_HIGH      0
#define PIN_LIMIT_SWITCH_Z   12
#define PIN_SPINLDE          11

// Cablebot - Microstep Driver ST-4045-A1
#elif CONTROLLER == CABLEBOT_CONTROLLER
#define ENABLE_PIN_HIGH      1
#define PIN_LIMIT_SWITCH_Z   11
#define PIN_SPINLDE          12
#endif
        
/* Configuration of the pins: */

#define PIN_LIMIT_SWITCH_X   9
#define PIN_LIMIT_SWITCH_Y   10

/* 
 * The STEP_ and DIRECTION_ defines below are taken from Grbl.
 */

/** 
 * Define step pulse output pins. NOTE: All step bit pins must be on
 * the same port. 
 */
#define STEP_DDR          DDRD
#define STEP_PORT         PORTD
#define X_STEP_BIT        2  // Uno Digital Pin 2
#define Y_STEP_BIT        3  // Uno Digital Pin 3
#define Z_STEP_BIT        4  // Uno Digital Pin 4
#define STEP_MASK         ((1 << X_STEP_BIT) | (1 << Y_STEP_BIT) | (1 << Z_STEP_BIT))

/** 
 * Define step direction output pins. NOTE: All direction pins must be
 * on the same port.
 */
#define DIRECTION_DDR     DDRD
#define DIRECTION_PORT    PORTD
#define X_DIRECTION_BIT   5  // Uno Digital Pin 5
#define Y_DIRECTION_BIT   6  // Uno Digital Pin 6
#define Z_DIRECTION_BIT   7  // Uno Digital Pin 7
#define DIRECTION_MASK    ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT))

/**
 * Define stepper driver enable/disable output pin.
 */
#define STEPPERS_DISABLE_DDR    DDRB
#define STEPPERS_DISABLE_PORT   PORTB
#define STEPPERS_DISABLE_BIT    0  // Uno Digital Pin 8
#define STEPPERS_DISABLE_MASK   (1 << STEPPERS_DISABLE_BIT)

        static bool is_enabled_ = false;
        
        /**
         * \brief Configure the step and dir pins as output.
         */
        void controller_init() 
        {
                controller_disable();
                
                /* Enable output on the selected pins */
                STEP_DDR |= STEP_MASK;
                DIRECTION_DDR |= DIRECTION_MASK;
                STEPPERS_DISABLE_DDR |= STEPPERS_DISABLE_MASK;
        
                // Set as input pins and enable internal pull-up
                // resistors. Normal high operation
                pinMode(PIN_LIMIT_SWITCH_X, INPUT_PULLUP);  
                pinMode(PIN_LIMIT_SWITCH_Y, INPUT_PULLUP);  
                pinMode(PIN_LIMIT_SWITCH_Z, INPUT_PULLUP);  

                // Set as output pins
                pinMode(PIN_SPINLDE, OUTPUT);  
                digitalWrite(PIN_SPINLDE, LOW);
        }

        /**
         * \brief Raise the enable pin.
         */
#define set_enable_pin_high()                                           \
        {                                                               \
                STEPPERS_DISABLE_PORT |= (1 << STEPPERS_DISABLE_BIT);   \
        }

        /**
         * \brief Reset the enable pin.
         */
#define set_enable_pin_low()                                            \
        {                                                               \
                STEPPERS_DISABLE_PORT &= ~(1 << STEPPERS_DISABLE_BIT);  \
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
        
        /**
         * \brief Toggle a direction bit in the DIR mask.
         */

        uint8_t controller_toggle_x_dir(uint8_t mask)
        {
                mask |= (1 << X_DIRECTION_BIT);
                return mask;
        }
                
        uint8_t controller_toggle_y_dir(uint8_t mask)
        {
                mask |= (1 << Y_DIRECTION_BIT);
                return mask;
        }
        
        uint8_t controller_toggle_z_dir(uint8_t mask)
        {
                mask |= (1 << Z_DIRECTION_BIT);
                return mask;
        }

        /**
         * \brief Enable the DIR pins according to mask.
         */
        void controller_set_dir_pins(uint8_t __pins)                            
        {                                               
                DIRECTION_PORT &= ~DIRECTION_MASK;      
                DIRECTION_PORT |= __pins;               
        }

        /**
         * \brief Toggle a step bit in the pins mask.
         */

        uint8_t controller_toggle_x_step(uint8_t mask)
        {
                mask |= (1 << X_STEP_BIT);
                return mask;
        }
        
        uint8_t controller_toggle_y_step(uint8_t mask)
        {
                mask |= (1 << Y_STEP_BIT);
                return mask;
        }
                
        uint8_t controller_toggle_z_step(uint8_t mask)
        {
                mask |= (1 << Z_STEP_BIT);
                return mask;
        }

        /**
         * \brief Enable the step pins according to mask.
         */
        void controller_set_step_pins(uint8_t mask) 
        {                                                      
                STEP_PORT |= mask;                   
        }
        
        /**
         * \brief Reset the step pins to zero.
         */
        void controller_reset_step_pins()
        {
                STEP_PORT &= ~STEP_MASK;
        }

        bool controller_x_limit_switch()
        {
                return digitalRead(PIN_LIMIT_SWITCH_X) == HIGH;
        }
        
        bool controller_y_limit_switch()
        {
                return digitalRead(PIN_LIMIT_SWITCH_Y) == HIGH;
        }
        
        bool controller_z_limit_switch()
        {
                return digitalRead(PIN_LIMIT_SWITCH_Z) == HIGH;
        }
        
        void controller_set_spindle(bool state)
        {
                int value = state? HIGH : LOW;
                digitalWrite(PIN_SPINLDE, value);
        }

}

#endif // defined(ARDUINO_AVR_UNO)
