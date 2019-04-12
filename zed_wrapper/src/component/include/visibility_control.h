#ifndef ZED__VISIBILITY_CONTROL_H_
#define ZED__VISIBILITY_CONTROL_H_

// /////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2019, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// /////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ZED_EXPORT __attribute__ ((dllexport))
#define ZED_IMPORT __attribute__ ((dllimport))
#else
#define ZED_EXPORT __declspec(dllexport)
#define ZED_IMPORT __declspec(dllimport)
#endif
#ifdef ZED_BUILDING_DLL
#define ZED_PUBLIC ZED_EXPORT
#else
#define ZED_PUBLIC ZED_IMPORT
#endif
#define ZED_PUBLIC_TYPE ZED_PUBLIC
#define ZED_LOCAL
#else
#define ZED_EXPORT __attribute__ ((visibility("default")))
#define ZED_IMPORT
#if __GNUC__ >= 4
#define ZED_PUBLIC __attribute__ ((visibility("default")))
#define ZED_LOCAL  __attribute__ ((visibility("hidden")))
#else
#define ZED_PUBLIC
#define ZED_LOCAL
#endif
#define ZED_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ZED__VISIBILITY_CONTROL_H_
