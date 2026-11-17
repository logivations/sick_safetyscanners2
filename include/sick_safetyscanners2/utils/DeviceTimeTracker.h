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

#pragma once

#include <cstdint>
#include <deque>

namespace sick {

/**
 * Maps the scanner's own clock onto host time, so that scans are stamped with
 * when they were measured instead of when their UDP packet was processed.
 *
 * Every data output packet carries a device time stamp (first beam of the
 * scan, millisecond resolution). Without time synchronization configured in
 * Safety Designer it counts from power-on, so it is only useful relative to
 * host time: offset = host receive time - device time. Each receive adds a
 * positive delay (transport, a stalled callback thread), so the smallest
 * offset seen within a sliding window is the delay-free mapping. The stamp
 * of a scan is device time + that minimum: equal to the receive time when the
 * packet was processed without extra delay, earlier by exactly the extra
 * delay otherwise, and never later than the receive time.
 *
 * Measured on nanoScan3 (2026-11-17): scans exactly 176-177 ms apart by
 * device time while the receive time stamps ranged 47-355 ms; device clocks
 * drift up to ~27 ppm against the host, i.e. < 1 ms over a 30 s window.
 *
 * Not thread safe; the owner serializes access.
 */
class DeviceTimeTracker {
public:
  /**
   * @param window_s Length of the sliding window (seconds of device time)
   *                 over which the minimum offset is taken.
   */
  explicit DeviceTimeTracker(double window_s = 30.0);

  /**
   * Convert the device time of one scan into host time.
   *
   * @param device_s Device time of the scan in seconds (date * 86400 + time / 1000).
   * @param host_s   Host time at which the packet is processed, in seconds.
   * @return Host time at which the scan was measured, <= host_s.
   */
  double stamp(double device_s, double host_s);

  /** Forget all samples, e.g. after a reconnect to the sensor. */
  void reset();

  void setWindow(double window_s);
  double window() const { return m_window_s; }

  /** Current offset estimate (host - device) in seconds; valid once samples() > 0. */
  double offset() const;
  /** host_s - returned stamp of the last scan: the delay removed from it. */
  double lastDelay() const { return m_last_delay_s; }
  /** Largest delay removed since construction. */
  double maxDelay() const { return m_max_delay_s; }
  /** Number of times the estimate was dropped because the device clock went backwards. */
  std::uint64_t clockResets() const { return m_clock_resets; }
  std::uint64_t samples() const { return m_samples; }

  /** Device time in seconds from the data header fields. */
  static double deviceSeconds(std::uint16_t timestamp_date, std::uint32_t timestamp_time_ms);

private:
  struct Sample {
    double device_s;
    double offset_s;
  };

  double m_window_s;
  // Monotonic deque: offsets strictly increasing from front to back, so the
  // front holds the minimum of the window.
  std::deque<Sample> m_window;
  bool m_has_last = false;
  double m_last_device_s = 0.0;
  double m_last_delay_s = 0.0;
  double m_max_delay_s = 0.0;
  std::uint64_t m_clock_resets = 0;
  std::uint64_t m_samples = 0;
};

} // namespace sick
