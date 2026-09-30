// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <umrr_ros2_driver/instruction_reply.hpp>

#include <memory>
#include <string>
#include <vector>

using com::master::InstructionBatch;
using com::master::ResponseBatch;
using smartmicro::drivers::radar::decode_instruction_reply;
using smartmicro::drivers::radar::InstructionBatchLease;
using smartmicro::drivers::radar::InstructionItem;
using smartmicro::drivers::radar::InstructionValueType;

namespace
{
struct TestBatch : InstructionBatch
{
  TestBatch()
  : InstructionBatch(1, 2, nullptr, 4) {}
};

class CountingInstructionService : public com::master::InstructionServiceIface
{
public:
  uint16_t GetMajorVer() const override {return 0;}
  uint16_t GetMinorVer() const override {return 0;}
  uint16_t GetPatchVer() const override {return 0;}
  bool Init() override {return true;}
  bool AllocateInstructionBatch(
    com::types::ClientId, std::shared_ptr<InstructionBatch> &) override {return false;}
  com::types::ErrorCode ReleaseInstructionBatch(std::shared_ptr<InstructionBatch> batch) override
  {
    released.push_back(batch);
    return com::types::ERROR_CODE_OK;
  }
  com::types::ErrorCode SendInstructionBatch(
    std::shared_ptr<InstructionBatch>, com::master::ResponseCallback) override
  {
    return com::types::ERROR_CODE_OK;
  }

  std::vector<std::shared_ptr<InstructionBatch>> released;
};

template<typename T>
void add(
  ResponseBatch & batch, const std::string & section, const std::string & name, T value,
  com::master::ResponseType type = com::master::RESPONSE_SUCCESS)
{
  ASSERT_TRUE(batch.AddResponse(std::make_shared<com::master::Response<T>>(
      com::master::REQUEST_TYPE_GET_PARAM, type, section, name, value)));
}
}  // namespace

TEST(InstructionReply, DecodesEveryTypedValue)
{
  auto batch = std::make_shared<ResponseBatch>();
  add<float>(*batch, "s", "f", 1.5F);
  add<uint32_t>(*batch, "s", "u32", 4000000000U);
  add<uint16_t>(*batch, "s", "u16", 65535);
  add<uint8_t>(*batch, "s", "u8", 7);
  add<int32_t>(*batch, "s", "i32", -3);
  const auto result = decode_instruction_reply(
    batch, 200, "s", {{"s", "f", InstructionValueType::kF32},
      {"s", "u32", InstructionValueType::kU32}, {"s", "u16", InstructionValueType::kU16},
      {"s", "u8", InstructionValueType::kU8}, {"s", "i32", InstructionValueType::kI32}});
  EXPECT_TRUE(result["success"].get<bool>());
  EXPECT_EQ(result["sensor_id"], 200);
  EXPECT_EQ(result["section"], "s");
  EXPECT_FLOAT_EQ(result["values"]["f"]["value"].get<float>(), 1.5F);
  EXPECT_EQ(result["values"]["u32"]["value"], 4000000000U);
  EXPECT_EQ(result["values"]["u16"]["value"], 65535);
  EXPECT_EQ(result["values"]["u8"]["value"], 7);  // A number, not a character.
  EXPECT_EQ(result["values"]["i32"]["value"], -3);
  EXPECT_EQ(result["values"]["i32"]["response_type"], com::master::RESPONSE_SUCCESS);
}

TEST(InstructionReply, RejectionMissingValueAndWrongTypeFailTheReply)
{
  auto batch = std::make_shared<ResponseBatch>();
  add<uint8_t>(*batch, "s", "rejected", 0, com::master::RESPONSE_ERROR_MAX);
  add<uint8_t>(*batch, "s", "narrow", 1);
  add<uint8_t>(*batch, "t", "other_section", 1);
  const auto result = decode_instruction_reply(
    batch, 1, "s", {{"s", "rejected", InstructionValueType::kU8},
      {"s", "narrow", InstructionValueType::kU32},
      {"s", "absent", InstructionValueType::kU8},
      {"t", "other_section", InstructionValueType::kU8}});
  EXPECT_FALSE(result["success"].get<bool>());
  const auto & values = result["values"];
  EXPECT_EQ(values["rejected"]["response_type"], com::master::RESPONSE_ERROR_MAX);
  EXPECT_EQ(
    values["rejected"]["error"], "Sensor rejected the instruction: value above maximum");
  EXPECT_FALSE(values["rejected"].contains("value"));
  EXPECT_EQ(values["narrow"]["response_type"], 0);
  EXPECT_EQ(values["absent"]["error"], "Missing or unexpected sensor response");
  EXPECT_EQ(values["other_section"]["value"], 1);

  const auto no_reply = decode_instruction_reply(
    nullptr, 1, "s", {{"s", "x", InstructionValueType::kU8}});
  EXPECT_FALSE(no_reply["success"].get<bool>());
}

TEST(InstructionReply, LeaseReleasesUnlessHandedOn)
{
  const auto service = std::make_shared<CountingInstructionService>();
  const std::shared_ptr<InstructionBatch> kept = std::make_shared<TestBatch>();
  const std::shared_ptr<InstructionBatch> dropped = std::make_shared<TestBatch>();
  {
    InstructionBatchLease lease{service, dropped};
    EXPECT_EQ(lease.get(), dropped);
  }
  {
    InstructionBatchLease lease{service, kept};
    EXPECT_EQ(lease.release(), kept);
    EXPECT_EQ(lease.get(), nullptr);
  }
  {InstructionBatchLease empty{nullptr, nullptr};}
  ASSERT_EQ(service->released.size(), 1U);
  EXPECT_EQ(service->released.front(), dropped);
}
