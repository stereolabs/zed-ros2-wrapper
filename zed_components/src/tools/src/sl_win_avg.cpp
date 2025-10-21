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

#include "sl_win_avg.hpp"

namespace sl_tools
{

WinAvg::WinAvg(size_t win_size)
{
  mWinSize = win_size;
  mSumVals = 0.0;
}

WinAvg::~WinAvg() {}

double WinAvg::setNewSize(size_t win_size)
{
  std::lock_guard<std::mutex> guard(mQueueMux);

  mWinSize = win_size;
  while (mVals.size() > mWinSize) {
    double val = mVals.back();
    mVals.pop_back();
    mSumVals -= val;
  }

  return mSumVals / mVals.size();
}

double WinAvg::addValue(double val)
{
  std::lock_guard<std::mutex> guard(mQueueMux);
  if (mVals.size() == mWinSize) {
    double older = mVals.back();
    mVals.pop_back();
    mSumVals -= older;
  }

  mVals.push_front(val);
  mSumVals += val;

  auto avg = mSumVals / mVals.size();

  // std::cout << "New val: " << val << " - Size: " << mVals.size()
  // << " - Sum: " << mSumVals << " - Avg: " << avg << std::endl;

  return avg;
}

double WinAvg::getAvg()
{
  std::lock_guard<std::mutex> guard(mQueueMux);

  double avg = mSumVals / mVals.size();

  return avg;
}

}
