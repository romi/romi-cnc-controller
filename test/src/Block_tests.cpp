#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "Block.h"

class Block_tests : public ::testing::Test
{
protected:
        
	Block_tests() {
        }

	~Block_tests() override = default;

	void SetUp() override {
        }

	void TearDown() override {}
};

TEST_F(Block_tests, block_initialization)
{
        romi::init_block_buffer();
        ASSERT_EQ(romi::kBlockBufferSize - 1, romi::block_buffer_space());
        ASSERT_EQ(0, romi::block_buffer_available());
}

