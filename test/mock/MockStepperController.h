#include "gmock/gmock.h"
#include "IStepperController.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Weffc++"

class MockStepperController : public IStepperController
{
public:
        MOCK_METHOD(void, init, (TimerMode mode), (override));
        MOCK_METHOD(void, enable, (), (override));
        MOCK_METHOD(void, disable, (), (override));
        MOCK_METHOD(bool, is_enabled, (), (override));
        MOCK_METHOD(uint8_t, toggle_x_dir, (uint8_t), (override));
        MOCK_METHOD(uint8_t, toggle_y_dir, (uint8_t), (override));
        MOCK_METHOD(uint8_t, toggle_z_dir, (uint8_t), (override));
        MOCK_METHOD(void, set_dir_pins, (uint8_t), (override));
        MOCK_METHOD(uint8_t, toggle_x_step, (uint8_t), (override));
        MOCK_METHOD(uint8_t, toggle_y_step, (uint8_t), (override));
        MOCK_METHOD(uint8_t, toggle_z_step, (uint8_t), (override));
        MOCK_METHOD(void, set_step_pins, (uint8_t), (override));
        MOCK_METHOD(void, reset_step_pins, (), (override));
        MOCK_METHOD(bool, x_limit_switch, (), (override));
        MOCK_METHOD(bool, y_limit_switch, (), (override));
        MOCK_METHOD(bool, z_limit_switch, (), (override));
        MOCK_METHOD(void, set_relay, (uint8_t, bool), (override));
};
#pragma GCC diagnostic pop
