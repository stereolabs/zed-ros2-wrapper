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

#ifndef VISIBILITY_CONTROL_HPP_
#define VISIBILITY_CONTROL_HPP_

/* *INDENT-OFF* */
#ifdef __cplusplus
extern "C" {
#endif
/* *INDENT-ON* */

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ZED_COMPONENTS_EXPORT __attribute__((dllexport))
#define ZED_COMPONENTS_IMPORT __attribute__((dllimport))
#else
#define ZED_COMPONENTS_EXPORT __declspec(dllexport)
#define ZED_COMPONENTS_IMPORT __declspec(dllimport)
#endif
#ifdef ZED_COMPONENTS_BUILDING_DLL
#define ZED_COMPONENTS_PUBLIC ZED_COMPONENTS_EXPORT
#else
#define ZED_COMPONENTS_PUBLIC ZED_COMPONENTS_IMPORT
#endif
#define ZED_COMPONENTS_PUBLIC_TYPE ZED_COMPONENTS_PUBLIC
#define ZED_COMPONENTS_LOCAL
#else
#define ZED_COMPONENTS_EXPORT __attribute__((visibility("default")))
#define ZED_COMPONENTS_IMPORT
#if __GNUC__ >= 4
#define ZED_COMPONENTS_PUBLIC __attribute__((visibility("default")))
#define ZED_COMPONENTS_LOCAL __attribute__((visibility("hidden")))
#else
#define ZED_COMPONENTS_PUBLIC
#define ZED_COMPONENTS_LOCAL
#endif
#define ZED_COMPONENTS_PUBLIC_TYPE
#endif

/* *INDENT-OFF* */
#ifdef __cplusplus
}
#endif
/* *INDENT-ON* */

#endif  // VISIBILITY_CONTROL_HPP_
