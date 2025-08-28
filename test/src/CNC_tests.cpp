#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "CNC.h"

class CNC_tests : public ::testing::Test
{
protected:
        
	CNC_tests() {
        }

	~CNC_tests() override = default;

	void SetUp() override {
        }

	void TearDown() override {}
};

TEST_F(CNC_tests, cnc_constructor)
{
        romi::init_block_buffer();
        ASSERT_EQ(romi::kCNCBufferSize - 1, romi::block_buffer_space());
        ASSERT_EQ(0, romi::block_buffer_available());
}

