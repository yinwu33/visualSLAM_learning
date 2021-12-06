#include <gtest/gtest.h>
#include <myAdd.h>


TEST(AddTest, handleAdd) {
    ASSERT_EQ(add(1, 1), 2);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}