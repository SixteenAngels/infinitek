#include <unity.h>

static void test_truth(void) {
  TEST_ASSERT_TRUE(1);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_truth);
  return UNITY_END();
}
