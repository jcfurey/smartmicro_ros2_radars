// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <umrr_ros2_driver/stream_codecs.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

namespace
{
// The parts of an SDK CAN target list that can_target_timestamp_us reads.
struct Header
{
  uint32_t seconds{};
  float fraction{};
  uint32_t GetTimeStamp() const {return seconds;}
  float GetAcqTimeStampFraction() const {return fraction;}
};
struct HeaderWithoutFraction
{
  uint32_t seconds{};
  uint32_t GetTimeStamp() const {return seconds;}
};
template<typename H>
struct List
{
  std::shared_ptr<H> header;
  std::shared_ptr<H> GetTargetListHeader() const {return header;}
};

template<typename H>
uint64_t stamp(H header)
{
  return smartmicro::drivers::radar::codec::can_target_timestamp_us(
    List<H>{std::make_shared<H>(header)});
}
}  // namespace

TEST(StreamCodecs, CanTargetTimestampCombinesSecondsAndFraction)
{
  EXPECT_EQ(stamp(Header{1000, .25F}), 1000250000U);
  EXPECT_EQ(stamp(Header{0, 0.F}), 0U);
  EXPECT_EQ(stamp(Header{4294967295U, .5F}), 4294967295500000U);
  // float32 has about 60 ns resolution near 1 s; rounding up carries into the seconds.
  EXPECT_EQ(stamp(Header{7, std::nextafter(1.F, 0.F)}), 8000000U);
  // Outside [0, 1) the fraction is not a fraction: the seconds alone are used.
  for (const float invalid : {-.25F, 1.F, 2.5F, std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity()})
  {
    EXPECT_EQ(stamp(Header{3, invalid}), 3000000U) << invalid;
  }
  EXPECT_EQ(stamp(HeaderWithoutFraction{3}), 3000000U);
}
