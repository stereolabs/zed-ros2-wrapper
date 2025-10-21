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

#ifndef SL_WIN_AVG_HPP_
#define SL_WIN_AVG_HPP_

#include <cstddef>  // size_t
#include <deque>    // std::dequeue
#include <mutex>

namespace sl_tools
{

class WinAvg
{
public:
  explicit WinAvg(size_t win_size = 15);
  ~WinAvg();

  double setNewSize(size_t win_size);
  double addValue(double val);

  /// @brief Get the current average of the stored values
  /// @return average of the stored values
  double getAvg();

  inline size_t size() {return mVals.size();}

private:
  size_t mWinSize;

  std::deque<double> mVals;  // The values in the queue used to calculate the windowed average
  double mSumVals = 0.0;     // The updated sum of the values in the queue

  std::mutex mQueueMux;
};

}

#endif  // SL_WIN_AVG_HPP_
