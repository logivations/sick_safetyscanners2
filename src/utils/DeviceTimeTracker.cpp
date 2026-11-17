// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

/*!
 *  Copyright (C) 2026, Logivations GmbH
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// -- END LICENSE BLOCK ------------------------------------------------

#include <sick_safetyscanners2/utils/DeviceTimeTracker.h>

#include <algorithm>

namespace sick {

DeviceTimeTracker::DeviceTimeTracker(double window_s) : m_window_s(window_s) {}

double DeviceTimeTracker::deviceSeconds(std::uint16_t timestamp_date,
                                        std::uint32_t timestamp_time_ms) {
  return static_cast<double>(timestamp_date) * 86400.0 +
         static_cast<double>(timestamp_time_ms) * 1e-3;
}

void DeviceTimeTracker::reset() {
  m_window.clear();
  m_has_last = false;
}

void DeviceTimeTracker::setWindow(double window_s) { m_window_s = window_s; }

double DeviceTimeTracker::offset() const {
  return m_window.empty() ? 0.0 : m_window.front().offset_s;
}

double DeviceTimeTracker::stamp(double device_s, double host_s) {
  // The device clock only runs forward between two scans of one sensor. Going
  // backwards (or repeating) means the sensor rebooted or its clock was set:
  // the old offsets describe another clock.
  if (m_has_last && device_s <= m_last_device_s) {
    reset();
    ++m_clock_resets;
  }
  m_has_last = true;
  m_last_device_s = device_s;
  ++m_samples;

  const double offset_s = host_s - device_s;
  while (!m_window.empty() && m_window.back().offset_s >= offset_s) {
    m_window.pop_back();
  }
  m_window.push_back({device_s, offset_s});
  // Keep the current sample even with a zero window: the minimum is then the
  // current offset and the stamp equals the receive time.
  while (m_window.size() > 1 && m_window.front().device_s < device_s - m_window_s) {
    m_window.pop_front();
  }

  const double stamp_s = device_s + m_window.front().offset_s;
  m_last_delay_s = host_s - stamp_s;
  m_max_delay_s = std::max(m_max_delay_s, m_last_delay_s);
  return stamp_s;
}

} // namespace sick
