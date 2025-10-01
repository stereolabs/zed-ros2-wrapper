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

#ifndef VERSION_HPP_
#define VERSION_HPP_

#include <stddef.h>

namespace stereolabs
{
const size_t WRAPPER_MAJOR = 5;
const size_t WRAPPER_MINOR = 1;
const size_t WRAPPER_PATCH = 0;

const size_t SDK_MAJOR_MIN_SUPP = 4;
const size_t SDK_MINOR_MIN_SUPP = 2;

const size_t SDK_MAJOR_MAX_SUPP = 5;
const size_t SDK_MINOR_MAX_SUPP = 1;
}  // namespace stereolabs

#endif  // VERSION_HPP_
