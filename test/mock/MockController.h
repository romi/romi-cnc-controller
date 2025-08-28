#include "gmock/gmock.h"
#include "IController.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Weffc++"

class MockController : public IController
{
public:
        MOCK_METHOD(void, init, (TimerMode mode), (override));
        MOCK_METHOD(void, zero, (), (override));
        MOCK_METHOD(void, reset, (), (override));
        MOCK_METHOD(void, enable, (), (override));
        MOCK_METHOD(void, disable, (), (override));
        MOCK_METHOD(bool, is_idle, (), (override));
        MOCK_METHOD(void, get_position, (int32_t *pos), (override));
};
#pragma GCC diagnostic pop
