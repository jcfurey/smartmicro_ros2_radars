// SPDX-License-Identifier: Apache-2.0
#ifndef UMRR_ROS2_DRIVER__INSTRUCTION_REPLY_HPP_
#define UMRR_ROS2_DRIVER__INSTRUCTION_REPLY_HPP_

// Smart Access instruction batches as used by the control services of the data
// and readback nodes: batch ownership and decoding of the sensor's reply into
// the JSON text returned in the service response.

#include <InstructionBatch.h>
#include <InstructionServiceIface.h>
#include <nlohmann/json.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace smartmicro::drivers::radar
{
enum class InstructionValueType { kF32, kU32, kU16, kU8, kI32 };

// One instruction of a batch whose reply value is reported.
struct InstructionItem
{
  std::string section;
  std::string name;
  InstructionValueType type;
};

// The SDK keeps every allocated batch until ReleaseInstructionBatch, including
// batches that were rejected or never answered. Releases on scope exit unless
// release() handed the batch on.
class InstructionBatchLease
{
public:
  InstructionBatchLease(
    std::shared_ptr<com::master::InstructionServiceIface> service,
    std::shared_ptr<com::master::InstructionBatch> batch)
  : service_(std::move(service)), batch_(std::move(batch)) {}
  ~InstructionBatchLease()
  {
    if (service_ && batch_) {
      service_->ReleaseInstructionBatch(batch_);
    }
  }
  InstructionBatchLease(const InstructionBatchLease &) = delete;
  InstructionBatchLease & operator=(const InstructionBatchLease &) = delete;

  const std::shared_ptr<com::master::InstructionBatch> & get() const {return batch_;}
  std::shared_ptr<com::master::InstructionBatch> release() {return std::exchange(batch_, nullptr);}

private:
  std::shared_ptr<com::master::InstructionServiceIface> service_;
  std::shared_ptr<com::master::InstructionBatch> batch_;
};

inline const char * response_type_text(uint32_t code)
{
  switch (code) {
    case com::master::RESPONSE_NO_RESPONSE: return "no response";
    case com::master::RESPONSE_SUCCESS: return "success";
    case com::master::RESPONSE_ERROR: return "general error";
    case com::master::RESPONSE_ERROR_REQUEST: return "invalid request";
    case com::master::RESPONSE_ERROR_SECTION: return "invalid section";
    case com::master::RESPONSE_ERROR_ID: return "invalid id";
    case com::master::RESPONSE_ERROR_PROT: return "invalid protection";
    case com::master::RESPONSE_ERROR_MIN: return "value below minimum";
    case com::master::RESPONSE_ERROR_MAX: return "value above maximum";
    case com::master::RESPONSE_ERROR_NAN: return "value is not a number";
    case com::master::RESPONSE_ERROR_TYPE: return "invalid instruction type";
    case com::master::RESPONSE_ERROR_DIM: return "invalid dimension";
    case com::master::RESPONSE_ERROR_ELEMENT: return "invalid element";
    case com::master::RESPONSE_ERROR_SIGNATURE: return "invalid signature";
    case com::master::RESPONSE_ERROR_ACCESS_LVL: return "invalid access level";
    default: return "unknown response type";
  }
}

template<typename T>
nlohmann::json read_instruction_value(
  const std::shared_ptr<com::master::ResponseBatch> & batch,
  const std::string & section, const std::string & name)
{
  std::vector<std::shared_ptr<com::master::Response<T>>> values;
  if (!batch || !batch->GetResponse<T>(section, name, values) ||
    values.size() != 1 || !values.front())
  {
    return {{"response_type", 0}, {"error", "Missing or unexpected sensor response"}};
  }
  const auto code = values.front()->GetResponseType();
  nlohmann::json result = {{"response_type", code}};
  if (code == com::master::RESPONSE_SUCCESS) {
    result["value"] = values.front()->GetValue();
  } else {
    result["error"] = std::string("Sensor rejected the instruction: ") + response_type_text(code);
  }
  return result;
}

// Decodes every item of a reply. "success" is true only if the sensor accepted
// each instruction; values are keyed by instruction name.
inline nlohmann::json decode_instruction_reply(
  const std::shared_ptr<com::master::ResponseBatch> & batch, uint32_t sensor_id,
  const std::string & section, const std::vector<InstructionItem> & items)
{
  nlohmann::json result = {{"sensor_id", sensor_id}, {"section", section},
    {"success", true}, {"values", nlohmann::json::object()}};
  for (const auto & item : items) {
    nlohmann::json value;
    switch (item.type) {
      case InstructionValueType::kF32:
        value = read_instruction_value<float>(batch, item.section, item.name);
        break;
      case InstructionValueType::kU32:
        value = read_instruction_value<uint32_t>(batch, item.section, item.name);
        break;
      case InstructionValueType::kU16:
        value = read_instruction_value<uint16_t>(batch, item.section, item.name);
        break;
      case InstructionValueType::kU8:
        value = read_instruction_value<uint8_t>(batch, item.section, item.name);
        break;
      case InstructionValueType::kI32:
        value = read_instruction_value<int32_t>(batch, item.section, item.name);
        break;
    }
    if (value["response_type"] != com::master::RESPONSE_SUCCESS) {
      result["success"] = false;
    }
    result["values"][item.name] = std::move(value);
  }
  return result;
}
}  // namespace smartmicro::drivers::radar
#endif  // UMRR_ROS2_DRIVER__INSTRUCTION_REPLY_HPP_
