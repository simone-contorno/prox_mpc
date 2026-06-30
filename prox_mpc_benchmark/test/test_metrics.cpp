// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Unit tests for the pure benchmark metric helpers (cross-track distance to a
// reference polyline and the linear-interpolated percentile).

#include <vector>

#include <gtest/gtest.h>

#include <prox_mpc_benchmark/metrics_math.hpp>

using prox_mpc_benchmark::crossTrack;
using prox_mpc_benchmark::percentile;

TEST(CrossTrack, PerpendicularOffsetFromStraightLine)
{
  // Reference along the x axis from (0,0) to (4,0); a point 0.5 m above mid-span.
  const std::vector<double> xs{0.0, 4.0};
  const std::vector<double> ys{0.0, 0.0};
  EXPECT_NEAR(crossTrack(xs, ys, 2.0, 0.5), 0.5, 1e-9);
  EXPECT_NEAR(crossTrack(xs, ys, 2.0, 0.0), 0.0, 1e-9);
}

TEST(CrossTrack, ClampsToSegmentEndpoints)
{
  const std::vector<double> xs{0.0, 4.0};
  const std::vector<double> ys{0.0, 0.0};
  // Beyond the end of the segment: distance is to the (4,0) endpoint.
  EXPECT_NEAR(crossTrack(xs, ys, 6.0, 0.0), 2.0, 1e-9);
}

TEST(CrossTrack, SinglePointReference)
{
  const std::vector<double> xs{1.0};
  const std::vector<double> ys{1.0};
  EXPECT_NEAR(crossTrack(xs, ys, 4.0, 5.0), 5.0, 1e-9);
}

TEST(Percentile, MedianAndExtremes)
{
  const std::vector<double> v{1.0, 2.0, 3.0, 4.0, 5.0};
  EXPECT_NEAR(percentile(v, 0.0), 1.0, 1e-9);
  EXPECT_NEAR(percentile(v, 0.5), 3.0, 1e-9);
  EXPECT_NEAR(percentile(v, 1.0), 5.0, 1e-9);
}

TEST(Percentile, InterpolatesBetweenSamples)
{
  const std::vector<double> v{0.0, 10.0};
  EXPECT_NEAR(percentile(v, 0.5), 5.0, 1e-9);
  EXPECT_NEAR(percentile(v, 0.95), 9.5, 1e-9);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
