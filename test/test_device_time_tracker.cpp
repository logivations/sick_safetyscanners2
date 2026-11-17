// Copyright (C) 2026, Logivations GmbH
// Licensed under the Apache License, Version 2.0

#include <sick_safetyscanners2/utils/DeviceTimeTracker.h>

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

namespace {

using sick::DeviceTimeTracker;

// nanoScan3 with skip 5: every 6th revolution of a 29.37 ms scan cycle
constexpr double kScanPeriod = 6 * 0.02937;
// host - device at power-on of the simulated sensor (arbitrary)
constexpr double kOffset = 1794896000.0 - 80000.0;
// smallest transport + processing delay of a packet
constexpr double kBaseDelay = 0.002;

// Device time as the sensor reports it: whole milliseconds.
double deviceMs(double t) { return std::floor(t * 1e3) * 1e-3; }

struct Scan {
  double device_s;   // what the header says
  double true_host;  // host time at which the scan was measured
  double rx_host;    // host time at which the callback runs
};

// Regular scans with random positive delays (exponential tail, rare long stalls).
std::vector<Scan> makeScans(int n, unsigned seed, double drift_ppm = 0.0,
                            double t0 = 1000.0) {
  std::mt19937 rng(seed);
  std::exponential_distribution<double> jitter(1.0 / 0.006);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  std::vector<Scan> scans;
  for (int i = 0; i < n; ++i) {
    const double t = t0 + i * kScanPeriod;
    const double dev = deviceMs(t);
    const double true_host = kOffset + t * (1.0 + drift_ppm * 1e-6);
    double delay = kBaseDelay + jitter(rng);
    if (u(rng) < 0.01) {
      delay += 0.1 + 0.2 * u(rng);  // stalled callback
    }
    scans.push_back({dev, true_host, true_host + delay});
  }
  return scans;
}

TEST(DeviceTimeTracker, DeviceSeconds) {
  EXPECT_DOUBLE_EQ(DeviceTimeTracker::deviceSeconds(0, 0), 0.0);
  EXPECT_DOUBLE_EQ(DeviceTimeTracker::deviceSeconds(1, 1000), 86401.0);
  EXPECT_DOUBLE_EQ(DeviceTimeTracker::deviceSeconds(0, 80962013), 80962.013);
}

TEST(DeviceTimeTracker, FirstScanIsStampedWithReceiveTime) {
  DeviceTimeTracker tracker;
  EXPECT_DOUBLE_EQ(tracker.stamp(100.0, 5000.0), 5000.0);
  EXPECT_DOUBLE_EQ(tracker.offset(), 4900.0);
  EXPECT_DOUBLE_EQ(tracker.lastDelay(), 0.0);
}

TEST(DeviceTimeTracker, RemovesReceiveJitter) {
  DeviceTimeTracker tracker(30.0);
  const auto scans = makeScans(2000, 1);  // ~6 min
  double prev_stamp = 0.0, prev_dev = 0.0;
  double worst_spacing_error = 0.0, worst_rx_spacing_error = 0.0;
  for (size_t i = 0; i < scans.size(); ++i) {
    const auto &s = scans[i];
    const double stamp = tracker.stamp(s.device_s, s.rx_host);
    ASSERT_LE(stamp, s.rx_host) << "stamp later than the receive time";
    // after the window has filled once, the stamp sits within ~1 ms of the
    // measurement time plus the constant base delay
    if (s.device_s - scans[0].device_s > 30.0) {
      EXPECT_NEAR(stamp - s.true_host, kBaseDelay, 0.0015) << "scan " << i;
      worst_spacing_error = std::max(
          worst_spacing_error, std::abs((stamp - prev_stamp) - (s.device_s - prev_dev)));
      worst_rx_spacing_error = std::max(
          worst_rx_spacing_error,
          std::abs((s.rx_host - scans[i - 1].rx_host) - (s.device_s - prev_dev)));
    }
    prev_stamp = stamp;
    prev_dev = s.device_s;
  }
  EXPECT_LT(worst_spacing_error, 0.0015);
  EXPECT_GT(worst_rx_spacing_error, 0.05);  // the receive times really were bad
}

// miele-amr1 2026-11-17 07:13:58.99 CET: the callback stalled ~0.4 s, two
// scans measured 176 ms apart were processed 1.2 ms apart.
TEST(DeviceTimeTracker, LateBurstKeepsScanSpacing) {
  DeviceTimeTracker tracker(30.0);
  auto scans = makeScans(400, 2);
  const size_t a = 300, b = 301;
  const double stall_end = scans[b].true_host + 0.07;
  scans[a].rx_host = stall_end;
  scans[b].rx_host = stall_end + 0.0012;
  scans[b + 1].rx_host = std::max(scans[b + 1].rx_host, stall_end + 0.1);

  std::vector<double> stamps;
  for (const auto &s : scans) {
    stamps.push_back(tracker.stamp(s.device_s, s.rx_host));
  }
  const double rx_gap = scans[b].rx_host - scans[a].rx_host;
  const double stamp_gap = stamps[b] - stamps[a];
  // absolute host times ~1.8e9 s: a double resolves ~0.24 us there
  EXPECT_NEAR(rx_gap, 0.0012, 1e-6);
  EXPECT_NEAR(stamp_gap, scans[b].device_s - scans[a].device_s, 0.0015);
  EXPECT_NEAR(stamps[a], scans[a].true_host + kBaseDelay, 0.0015);
  EXPECT_NEAR(stamps[b], scans[b].true_host + kBaseDelay, 0.0015);
}

TEST(DeviceTimeTracker, FollowsClockDrift) {
  // 27 ppm measured on miele-amr2; test both signs and a bit more
  for (double ppm : {-50.0, 27.0, 50.0}) {
    DeviceTimeTracker tracker(30.0);
    const auto scans = makeScans(20000, 3, ppm);  // ~1 h
    double worst = 0.0;
    for (const auto &s : scans) {
      const double stamp = tracker.stamp(s.device_s, s.rx_host);
      if (s.device_s - scans[0].device_s > 30.0) {
        worst = std::max(worst, std::abs(stamp - s.true_host - kBaseDelay));
      }
    }
    // 50 ppm over a 30 s window = 1.5 ms, plus the ms rounding of the header
    EXPECT_LT(worst, 0.003) << ppm << " ppm";
  }
}

TEST(DeviceTimeTracker, SensorRebootResetsTheMapping) {
  DeviceTimeTracker tracker(30.0);
  auto before = makeScans(200, 4);
  for (const auto &s : before) {
    tracker.stamp(s.device_s, s.rx_host);
  }
  // reboot: the device clock starts again near 0, so host - device is much
  // larger than before; keeping the old minimum would stamp scans in the past
  const double rx = before.back().rx_host + 20.0;
  EXPECT_DOUBLE_EQ(tracker.stamp(1.5, rx), rx);
  EXPECT_EQ(tracker.clockResets(), 1u);
  EXPECT_DOUBLE_EQ(tracker.stamp(1.5 + kScanPeriod, rx + kScanPeriod + 0.001),
                   rx + kScanPeriod);
}

TEST(DeviceTimeTracker, RepeatedDeviceTimeResets) {
  DeviceTimeTracker tracker(30.0);
  tracker.stamp(10.0, 110.0);
  tracker.stamp(10.0, 110.2);
  EXPECT_EQ(tracker.clockResets(), 1u);
  EXPECT_DOUBLE_EQ(tracker.offset(), 100.2);
}

TEST(DeviceTimeTracker, OldMinimumExpiresAfterWindow) {
  DeviceTimeTracker tracker(30.0);
  tracker.stamp(0.0, 100.0);  // offset 100.000
  for (int i = 1; i <= 200; ++i) {
    const double dev = i * kScanPeriod;
    tracker.stamp(dev, dev + 100.05);  // offset 100.050
    if (dev <= 30.0) {
      EXPECT_DOUBLE_EQ(tracker.offset(), 100.0) << dev;
    }
  }
  EXPECT_NEAR(tracker.offset(), 100.05, 1e-9);
}

TEST(DeviceTimeTracker, ResetForgetsSamples) {
  DeviceTimeTracker tracker(30.0);
  tracker.stamp(1.0, 101.0);
  tracker.reset();
  EXPECT_DOUBLE_EQ(tracker.stamp(2.0, 103.0), 103.0);
  EXPECT_EQ(tracker.clockResets(), 0u);
}

}  // namespace
