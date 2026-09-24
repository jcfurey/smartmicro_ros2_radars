// SPDX-License-Identifier: Apache-2.0
// ASan detects the original vendor F32 defect; the patched header passes this probe.
#include <Instruction.h>
#include <iostream>
int main() {
  com::master::SetParamRequest<float> request("test", "float", 1.5f);
  uint64_t bits = 0;
  const bool ok = request.GetConvertValue(bits);
  std::cout << ok << " " << std::hex << bits << std::endl;
}
