// SPDX-License-Identifier: Apache-2.0
// Intentionally exercises the vendor F32 conversion; ASan should detect the SDK defect.
#include <Instruction.h>
#include <iostream>
int main() {
  com::master::SetParamRequest<float> request("test", "float", 1.5f);
  uint64_t bits = 0;
  const bool ok = request.GetConvertValue(bits);
  std::cout << ok << " " << std::hex << bits << std::endl;
}
