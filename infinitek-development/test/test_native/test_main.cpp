#include <unity.h>

void test_true(void) {
  TEST_ASSERT_TRUE(true);
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_true);
  return UNITY_END();
}
