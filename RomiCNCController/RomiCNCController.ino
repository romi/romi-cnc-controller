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
#include <ArduinoSerial.h>
#include <RomiSerial.h>
//#include "Timer.h"
#include "CNC.h"

using namespace romiserial;

void handle_moveto(IRomiSerial *romiSerial, int16_t *args, const char *string_arg);
void handle_move(IRomiSerial *romiSerial, int16_t *args, const char *string_arg);
void handle_moveat(IRomiSerial *romiSerial, int16_t *args, const char *string_arg);
void handle_pause(IRomiSerial *romiSerial, int16_t *args, const char *string_arg);
void handle_continue(IRomiSerial *romiSerial, int16_t *args, const char *string_arg);
void handle_reset(IRomiSerial *romiSerial, int16_t *args, const char *string_arg);
void handle_zero(IRomiSerial *romiSerial, int16_t *args, const char *string_arg);
void send_position(IRomiSerial *romiSerial, int16_t *args, const char *string_arg);
void send_idle(IRomiSerial *romiSerial, int16_t *args, const char *string_arg);
void handle_homing(IRomiSerial *romiSerial, int16_t *args, const char *string_arg);
void handle_set_homing_axes(IRomiSerial *romiSerial, int16_t *args, const char *string_arg);
void handle_set_homing_speeds(IRomiSerial *romiSerial, int16_t *args, const char *string_arg);
void handle_set_homing_mode(IRomiSerial *romiSerial, int16_t *args, const char *string_arg);
void handle_enable(IRomiSerial *romiSerial, int16_t *args, const char *string_arg);
void handle_is_enabled(IRomiSerial *romiSerial, int16_t *args, const char *string_arg);
void handle_spindle(IRomiSerial *romiSerial, int16_t *args, const char *string_arg);
void send_info(IRomiSerial *romiSerial, int16_t *args, const char *string_arg);
// void handle_test(IRomiSerial *romiSerial, int16_t *args, const char *string_arg);

const static MessageHandler handlers[] = {
        { 'm', 4, false, handle_moveto },
        { 'M', 4, false, handle_move },
        { 'V', 3, false, handle_moveat },
        { 'p', 0, false, handle_pause },
        { 'c', 0, false, handle_continue },
        { 'r', 0, false, handle_reset },
        { 'z', 0, false, handle_zero },
        { 'P', 0, false, send_position },
        { 'I', 0, false, send_idle },
        { 'H', 0, false, handle_homing },
        { 'h', 3, false, handle_set_homing_axes },
        { 's', 3, false, handle_set_homing_speeds },
        { 'o', 1, false, handle_set_homing_mode },
        { 'E', 1, false, handle_enable },
        { 'e', 0, false, handle_is_enabled },
        { 'S', 1, false, handle_spindle },
        { '?', 0, false, send_info },
        // { 'T', 1, false, handle_test },
};

romi::CNC cnc;
ArduinoSerial serial(Serial);
RomiSerial romiSerial(serial, serial, handlers, sizeof(handlers) / sizeof(MessageHandler));
static char reply_string[80];

// volatile uint16_t debug_step_calls = 0;
// volatile uint16_t debug_reset_calls = 0;

// void step_callback()
// {
//         debug_step_calls++;
// }

// void reset_callback()
// {
//         debug_reset_calls++;
// }

void setup()
{
        Serial.begin(115200);
        // while (!Serial)
        //         ;

        cnc.init();
        //romi::timer_init(romi::k10kHz, step_callback, reset_callback);
}

void loop()
{
        romiSerial.handle_input();
        delay(1);

        // Serial.print("Step calls: ");
        // Serial.println(debug_step_calls);
        // Serial.print("Reset calls: ");
        // Serial.println(debug_reset_calls);
        // romi::timer_schedule_reset();
        // delay(100);
}

void handle_moveto(IRomiSerial *romiSerial, int16_t *args, const char *string_arg)
{
        if (cnc.schedule_moveto(args[0], args[1], args[2], args[3]) == 0) {
                romiSerial->send_ok();  
        } else {
                romiSerial->send_error(cnc.error_code(), cnc.error_message());  
        }
}

void handle_move(IRomiSerial *romiSerial, int16_t *args, const char *string_arg)
{
        if (cnc.schedule_move(args[0], args[1], args[2], args[3]) == 0) {
                romiSerial->send_ok();  
        } else {
                romiSerial->send_error(cnc.error_code(), cnc.error_message());  
        }
}

void handle_moveat(IRomiSerial *romiSerial, int16_t *args, const char *string_arg)
{
        if (cnc.schedule_moveat(args[0], args[1], args[2]) == 0) {
                romiSerial->send_ok();  
        } else {
                romiSerial->send_error(cnc.error_code(), cnc.error_message());  
        }
}

void handle_pause(IRomiSerial *romiSerial, int16_t *args, const char *string_arg)
{
        if (cnc.pause() == 0) {
                romiSerial->send_ok();  
        } else {
                romiSerial->send_error(cnc.error_code(), cnc.error_message());  
        }
}

void handle_continue(IRomiSerial *romiSerial, int16_t *args, const char *string_arg)
{
        if (cnc.continue_() == 0) {
                romiSerial->send_ok();  
        } else {
                romiSerial->send_error(cnc.error_code(), cnc.error_message());  
        }
}

void handle_reset(IRomiSerial *romiSerial, int16_t *args, const char *string_arg)
{
        if (cnc.reset() == 0) {
                romiSerial->send_ok();  
        } else {
                romiSerial->send_error(cnc.error_code(), cnc.error_message());  
        }
}

void handle_zero(IRomiSerial *romiSerial, int16_t *args, const char *string_arg)
{
        if (cnc.zero() == 0) {
                romiSerial->send_ok();  
        } else {
                romiSerial->send_error(cnc.error_code(), cnc.error_message());  
        }
}

void send_position(IRomiSerial *romiSerial, int16_t *args, const char *string_arg)
{
        int32_t pos[3];
        cnc.get_position(pos);

        snprintf(reply_string, sizeof(reply_string),
                 "[0,%ld,%ld,%ld]", pos[0], pos[1], pos[2]);
        
        romiSerial->send(reply_string); 
}

void send_idle(IRomiSerial *romiSerial, int16_t *args, const char *string_arg)
{
        snprintf(reply_string, sizeof(reply_string), "[0,%d,\"%c\"]",
                 (int) cnc.is_idle(), cnc.get_state());
        romiSerial->send(reply_string); 
}

void handle_homing(IRomiSerial *romiSerial, int16_t *args, const char *string_arg)
{
        if (cnc.homing() == 0) {
                romiSerial->send_ok();  
        } else {
                romiSerial->send_error(cnc.error_code(), cnc.error_message());  
        }
}

void handle_set_homing_axes(IRomiSerial *romiSerial, int16_t *args, const char *string_arg)
{
        if (cnc.homing_axes(args[0], args[1], args[2]) == 0) {
                romiSerial->send_ok();  
        } else {
                romiSerial->send_error(cnc.error_code(), cnc.error_message());  
        }
}

void handle_set_homing_speeds(IRomiSerial *romiSerial, int16_t *args, const char *string_arg)
{
        if (cnc.homing_speeds(args[0], args[1], args[2]) == 0) {
                romiSerial->send_ok();  
        } else {
                romiSerial->send_error(cnc.error_code(), cnc.error_message());  
        }
}

void handle_set_homing_mode(IRomiSerial *romiSerial, int16_t *args, const char *string_arg)
{
        if (cnc.homing_mode(args[0]) == 0) {
                romiSerial->send_ok();  
        } else {
                romiSerial->send_error(cnc.error_code(), cnc.error_message());  
        }
}

void handle_enable(IRomiSerial *romiSerial, int16_t *args, const char *string_arg)
{
        if (args[0] == 0) {
                cnc.power_down();
        } else {
                cnc.power_up();
        }
        romiSerial->send_ok();
}

void handle_is_enabled(IRomiSerial *romiSerial, int16_t *args, const char *string_arg)
{
        snprintf(reply_string, sizeof(reply_string), "[0,%d]",
                 (int) cnc.is_enabled());
        romiSerial->send(reply_string);
}

void handle_spindle(IRomiSerial *romiSerial, int16_t *args, const char *string_arg)
{
        if (args[0] == 0) {
                cnc.set_spindle(false);
        } else {
                cnc.set_spindle(true);
        }
        romiSerial->send_ok();
}

void send_info(IRomiSerial *romiSerial, int16_t *args, const char *string_arg)
{
        romiSerial->send("[0,\"RomiCNCController\",\"0.1\",\"" __DATE__ " " __TIME__ "\"]");
}

// static bool quit_testing;

// void start_test()
// {
//         quit_testing = false;
//         while (!quit_testing) {
                
//                 // cnc.schedule_moveat(1000, 1000, 100);
//                 // delay(500);
                
//                 // cnc.schedule_moveat(-1000, -1000, -100);
//                 // delay(500);
                
//                 // update_limit_switches();
//                 // Serial.print("#![");
//                 // Serial.print(limit_switches[0]);
//                 // Serial.print(',');
//                 // Serial.print(limit_switches[1]);
//                 // Serial.print(',');
//                 // Serial.print(limit_switches[2]);
//                 // Serial.print("]:xxxx\r\n");

//                 romiSerial.handle_input();
//         }
// }

// void stop_test()
// {
//         quit_testing = true;
//         cnc.schedule_moveat(0, 0, 0);
// }

// void handle_test(IRomiSerial *romiSerial, int16_t *args, const char *string_arg)
// {
//         romiSerial->send_ok();
//         if (args[0] == 0) {
//                 stop_test();
//         } else {
//                 start_test();
//         }
// }
