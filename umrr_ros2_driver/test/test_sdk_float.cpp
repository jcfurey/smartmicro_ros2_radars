// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <Instruction.h>
#include <array>
#include <cstring>

TEST(SdkFloat, PreservesIeeeBitsWithoutOverreadingOrSignExtension)
{
  // Positive/negative normal, signed zero, subnormals, infinities and quiet NaN payload.
  const std::array<uint32_t, 10> patterns = {
    0x3fc00000, 0xc0200000, 0x00000000, 0x80000000, 0x00000001,
    0x80000001, 0x7f7fffff, 0x7f800000, 0xff800000, 0x7fc12345};
  for (const auto bits : patterns) {
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    com::master::SetParamRequest<float> request("test", "float", value);
    uint64_t actual = UINT64_MAX;
    ASSERT_TRUE(request.GetConvertValue(actual));
    EXPECT_EQ(actual, static_cast<uint64_t>(bits));
    EXPECT_EQ(request.GetDataType(), com::master::DATA_TYPE_F32);
  }
}

TEST(SdkFloat, ReadRequestsAndIntegerRequestsRemainValid)
{
  com::master::GetParamRequest<float> read("test", "float");
  uint64_t value = UINT64_MAX;
  ASSERT_TRUE(read.GetConvertValue(value));
  EXPECT_EQ(value, 0U);
  com::master::SetParamRequest<uint8_t> small("test", "byte", 255);
  ASSERT_TRUE(small.GetConvertValue(value));
  EXPECT_EQ(value, 255U);
  com::master::SetParamRequest<int32_t> negative("test", "signed", -2);
  ASSERT_TRUE(negative.GetConvertValue(value));
  EXPECT_EQ(value, static_cast<uint64_t>(int64_t{-2}));
}
