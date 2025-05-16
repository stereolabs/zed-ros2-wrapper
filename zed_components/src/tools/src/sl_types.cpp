// Copyright 2025 Stereolabs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sl_types.hpp"

namespace stereolabs
{

std::string toString(const PubRes & res)
{
  switch (res) {
    case NATIVE:
      return "NATIVE";
    case CUSTOM:
      return "CUSTOM";
    default:
      return "";
  }
}

std::string toString(const PcRes & res)
{
  switch (res) {
    case PUB:
      return "PUB";
    case FULL:
      return "FULL";
    case COMPACT:
      return "COMPACT";
    case REDUCED:
      return "REDUCED";
    default:
      return "";
  }
}

} // namespace stereolabs
