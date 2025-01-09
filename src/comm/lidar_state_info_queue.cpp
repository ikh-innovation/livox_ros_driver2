//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "lidar_state_info_queue.h"

namespace livox_ros {

void LidarStateInfoQueue::Push(StateInfo* state_info) {
  StateInfo data;
  data.lidar_type = state_info->lidar_type;
  data.handle = state_info->handle;
  data.data = state_info->data;
  std::lock_guard<std::mutex> lock(mutex_);
  state_info_queue_.push_back(std::move(data));
}

bool LidarStateInfoQueue::Pop(StateInfo& state_info) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_info_queue_.empty()) {
    return false;
  }
  state_info = state_info_queue_.front();
  state_info_queue_.pop_front();
  return true;
}

bool LidarStateInfoQueue::Empty() {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_info_queue_.empty();
}

void LidarStateInfoQueue::Clear() {
  std::list<StateInfo> tmp_state_info_queue;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_info_queue_.swap(tmp_state_info_queue);
  }
}

} // namespace livox_ros