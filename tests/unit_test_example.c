#include "unity.h"

// Add your modules here

void setUp(void)
{

}

void tearDown(void)
{
}

void test_example_1(void)
{
    TEST_ASSERT_EQUAL_HEX8(0, 0);
}

void test_example_2(void)
{

    TEST_ASSERT_EQUAL_HEX8(0, 0);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_example_1);
    RUN_TEST(test_example_2);
    return UNITY_END();
}
