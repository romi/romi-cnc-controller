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
#include <avr/interrupt.h>
#include <Arduino.h>
#include "TimerUno.h"

volatile uint32_t count_timer_calls_ = 0;
volatile uint32_t count_reset_calls_ = 0;

namespace romi {

        TimerCallback timer_callback_ = nullptr;
        TimerCallback reset_callback_ = nullptr;

        void init_stepper_timer(TimerMode mode);
        void init_reset_timer();

        TimerUno::TimerUno()
        {
        }
        
        void TimerUno::init(TimerMode mode,
                            TimerCallback timer_callback,
                            TimerCallback reset_callback)
        {
                timer_callback_ = timer_callback;
                reset_callback_ = reset_callback;
                
                cli();
                init_stepper_timer(mode);
                init_reset_timer();
                sei(); 
        }

        /*
          https://fr.wikiversity.org/wiki/Micro_contr%C3%B4leurs_AVR/Le_Timer_1
          http://maxembedded.com/2011/07/avr-timers-ctc-mode/
        */
        /**
         * \brief: Initialize Timer2 to reset the STEP pins back to zero 10 µs
         * after a pulse.
         *
         *  Timer2 is used as the "reset step timer", i.e. this timer will
         *  pulse the set the STEP pins of the stepper drivers to zero 10 µs
         *  after they have been raised to 1 by the stepper timer.
         */
        void init_reset_timer()
        {
                /* Don't enable the timer, yet. */
                TIMSK2 &= ~(1 << OCIE2A);

                // Use the waveform generation mode, or Clear Timer on Compare
                // (CTC) mode:
                //
                // Register  WGM22 WGM21 WGM20
                // TCCR2A          1     0
                // TCCR2B    0                 
                TCCR2A &= ~(1 << WGM20);
                TCCR2A |=  (1 << WGM21);
                TCCR2B &= ~(1 << WGM22); 

                // Disconnect OC1 output: Don't send the PWM to an output
                // pin.
                TCCR2A &= ~((1 << COM2A1) | (1 << COM2A0)
                            | (1 << COM2B1) | (1 << COM2B0));

                // Set the prescaling
                //   CS22  CS21  CS20
                //   0     0     0     Disabled
                //   0     0     1     1
                //   0     1     0     8
                //   0     1     1     32
                //   1     0     0     64
                //   1     0     1     128
                //   1     1     0     256
                //   1     1     1     1024

                // Prescaling: 8
                TCCR2B &= ~(1 << CS20);
                TCCR2B |=  (1 << CS21);
                TCCR2B &= ~(1 << CS22);
                // TCCR2B &= ~(1 << CS20);
                // TCCR2B &= ~(1 << CS21);
                // TCCR2B &= ~(1 << CS22);

                /* Set the compare value:

                   Timer delay = T = 10 µs
                   F_CPU = 16 MHz
                   Prescaling = 8
                   F_CLOCK = 2 MHz
                   P_CLOCK = 1/2 µs
                   N = 10 µs / (1/2 µs) = 20
                   N-1 = 19
           
                   int n = T / (1 / F_CPU / prescaling) - 1
                   = (T * F_CPU / prescaling) - 1
                   = 19
                */
                OCR2A = 19;
        
                /* Initialize counter */
                TCNT2 = 0;
        }

        /**
         * \brief: Configure Timer1 to drive the stepper's STEP pulse train.
         *
         *  Timer1 is used as the "stepper timer", i.e. this timer will pulse
         *  the STEP pins of the stepper drivers.
         */
        void init_stepper_timer(TimerMode mode)
        {
                //current_block_ = 0;   FIXME

                /* Don't enable the timer, yet */
                TIMSK1 &= ~(1 << OCIE1A);
        
                // Use the waveform generation mode, or Clear Timer on Compare
                // (CTC) mode.
                //
                // Register  WGM13 WGM12 WGM11 WGM10  
                // TCCR1A                0     0    
                // TCCR1B    0     1                
                TCCR1A &= ~(1 << WGM10);
                TCCR1A &= ~(1 << WGM11);
                TCCR1B |=  (1 << WGM12);
                TCCR1B &= ~(1 << WGM13); 

                // Disconnect OC1 output: Don't send the PWM to an output
                // pin.
                TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0)
                            | (1 << COM1B1) | (1 << COM1B0));


                /* Set the compare value: 
           
                   F_STEPPER = 25000 (25 kHz)
                   P_STEPPER = 1/25000 = 40 µs
                   F_CPU = 16 MHz
                   Prescaling = 1
                   F_CLOCK = 16 MHz / 1
                   P_CLOCK = 1/16 µs
                   N = 40 µs / (1/16 µs) = 640
                   N-1 = 639

                   F_STEPPER = 10000 (10 kHz)
                   P_STEPPER = 1/10000 = 100 µs
                   F_CPU = 16 MHz
                   Prescaling = 8
                   F_CLOCK = 16 MHz / 8 = 2 MHz
                   P_CLOCK = 1/2 µs
                   N = 100 µs / (1/2 µs) = 200
                   N-1 = 199

                   int n = (1 / F_STEPPER) / (1 / F_CPU / prescaling) - 1
                   = F_CPU / (F_STEPPER * prescaling) - 1
                   = 639
                */
                
                uint16_t prescaling;
                uint16_t frequency;
                
                // Set the prescaling
                //   CS12  CS11  CS10
                //   0     0     0     Disabled
                //   0     0     1     1
                //   0     1     0     8
                //   0     1     1     64
                //   1     0     0     256
                //   1     0     1     1024
                //   1     1     0     Use external clock, falling edge
                //   1     1     1     Use external clock, rising edge

                switch (mode) {
                case k25kHz:
                        frequency = 25000;
                        prescaling = 1;
                        TCCR1B |=  (1 << CS10);
                        TCCR1B &= ~(1 << CS11);
                        TCCR1B &= ~(1 << CS12);
                        break;
                case k10kHz:
                        frequency = 10000;
                        prescaling = 8;
                        TCCR1B &=  ~(1 << CS10);
                        TCCR1B |= (1 << CS11);
                        TCCR1B &= ~(1 << CS12);
                        break;
                default:
                        Serial.println("TimerUno.cpp: invalid mode");
                        while (true) ; // FIXME
                }

                uint16_t compare_value = F_CPU / prescaling / frequency - 1;
                OCR1A = compare_value;
                
                // Serial.print("TimerUno.cpp: compare_value=");
                // Serial.println(compare_value);
        }

        /**
         * \brief The interrupt service handler for the reset-step-pins timer
         */
        ISR(TIMER2_COMPA_vect)
        {
                count_reset_calls_++;
                /* Reset the step pins to zero */
                reset_callback_();
                /* Disable Timer2 interrupt */
                TIMSK2 &= ~(1 << OCIE2A);
        }

        /**
         * \brief The interrupt service routine for the stepper timer.
         *
         */
        ISR(TIMER1_COMPA_vect)
        {
                count_timer_calls_++;
                timer_callback_();
        }

        void TimerUno::schedule_reset()
        {
                // debug_reset_init++;
                
                /* Initialize counter */
                TCNT2 = 0;
                /* Enable Timer2 */
                TIMSK2 |= (1 << OCIE2A);
        }

        void TimerUno::enable()
        {
                /* Initialize counter */
                TCNT1 = 0;
                /* Set the status of the stepper thread */
                /* thread_state = STATE_THREAD_EXECUTING; */
                /* Enable Timer1 */
                TIMSK1 |= (1 << OCIE1A);
        }

        void TimerUno::disable()
        {
                /* Set the status of the stepper thread */
                /*thread_state = STATE_THREAD_IDLE;*/
                /* Disable Timer1 interrupt */
                TIMSK1 &= ~(1 << OCIE1A);
        }

        uint32_t TimerUno::get_count_timer_calls()
        {
                return count_timer_calls_;
        }
        
        uint32_t TimerUno::get_count_reset_calls()
        {
                return count_reset_calls_;
        }
}

#endif // defined(ARDUINO_AVR_UNO)
