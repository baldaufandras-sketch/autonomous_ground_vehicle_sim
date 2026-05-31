#include <gtest/gtest.h>

TEST(ControllerTest, theBasic) {
  double result = 4.0 + 3.0;

  EXPECT_DOUBLE_EQ(result, 7.0);
}