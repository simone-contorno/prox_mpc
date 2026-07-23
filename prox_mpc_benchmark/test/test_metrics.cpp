// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Unit tests for the pure benchmark metric helpers (cross-track distance to a
// reference polyline and the linear-interpolated percentile).

#include <cmath>
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

TEST(CrossTrack, EmptyReference)
{
  const std::vector<double> empty;
  EXPECT_NEAR(crossTrack(empty, empty, 3.0, 4.0), 0.0, 1e-9);
}

TEST(CrossTrack, MismatchedReferenceLengthsTruncateToCommonPrefix)
{
  // ref_x / ref_y are independent parameter arrays, so a malformed params file
  // can hand in unequal lengths; only the common prefix may be read.
  const std::vector<double> xs3{0.0, 4.0, 8.0};
  const std::vector<double> ys2{0.0, 0.0};
  EXPECT_NEAR(crossTrack(xs3, ys2, 2.0, 0.5), 0.5, 1e-9);
  // The (4,0)-(8,?) segment does not exist, so a point past x=4 clamps to (4,0).
  EXPECT_NEAR(crossTrack(xs3, ys2, 6.0, 0.0), 2.0, 1e-9);

  // Symmetric when the y array is the longer one.
  const std::vector<double> xs2{0.0, 4.0};
  const std::vector<double> ys3{0.0, 0.0, 0.0};
  EXPECT_NEAR(crossTrack(xs2, ys3, 6.0, 0.0), 2.0, 1e-9);

  // A one-sided empty array leaves no polyline at all.
  const std::vector<double> empty;
  EXPECT_NEAR(crossTrack(xs3, empty, 6.0, 0.0), 0.0, 1e-9);
}

TEST(Percentile, EmptyInputIsNan)
{
  EXPECT_TRUE(std::isnan(percentile(std::vector<double>{}, 0.5)));
}

TEST(Percentile, SingleElement)
{
  const std::vector<double> v{7.5};
  EXPECT_NEAR(percentile(v, 0.0), 7.5, 1e-9);
  EXPECT_NEAR(percentile(v, 0.5), 7.5, 1e-9);
  EXPECT_NEAR(percentile(v, 1.0), 7.5, 1e-9);
}

TEST(Percentile, EvenLengthMedianInterpolates)
{
  // Unsorted on input: the helper sorts its own copy.
  const std::vector<double> v{4.0, 1.0, 3.0, 2.0};
  EXPECT_NEAR(percentile(v, 0.0), 1.0, 1e-9);
  EXPECT_NEAR(percentile(v, 0.5), 2.5, 1e-9);
  EXPECT_NEAR(percentile(v, 1.0), 4.0, 1e-9);
}

TEST(Percentile, OddLengthMedianIsTheMiddleSample)
{
  const std::vector<double> v{5.0, 1.0, 4.0, 2.0, 3.0, 7.0, 6.0};
  EXPECT_NEAR(percentile(v, 0.0), 1.0, 1e-9);
  EXPECT_NEAR(percentile(v, 0.5), 4.0, 1e-9);
  EXPECT_NEAR(percentile(v, 1.0), 7.0, 1e-9);
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
