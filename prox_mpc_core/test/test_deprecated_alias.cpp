// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// The deprecated prox_mpc_core/Bicycle alias's runtime warning: what it says
// and how often it is said.
//
// This is its own binary because the property under test is process-scoped. The
// warning is gated on a std::once_flag, so any other test that constructs a
// Bicycle first consumes it and leaves nothing to observe; test_model_interface
// constructs several.

#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <rcutils/logging.h>

#include <prox_mpc/models/bicycle.hpp>

using prox_mpc::Bicycle;

namespace
{
// Collects WARN and above from the rcutils logging handler, restoring the
// previous handler on destruction.
class LogCapture
{
public:
  LogCapture()
  : previous_(rcutils_logging_get_output_handler())
  {
    messages_.clear();
    rcutils_logging_set_output_handler(&LogCapture::handle);
  }
  ~LogCapture() {rcutils_logging_set_output_handler(previous_);}

  static const std::vector<std::string> & messages() {return messages_;}

private:
  static void handle(
    const rcutils_log_location_t *, int severity, const char *,
    rcutils_time_point_value_t, const char * format, va_list * args)
  {
    if (severity < RCUTILS_LOG_SEVERITY_WARN) {return;}
    char buf[1024];
    va_list copy;
    va_copy(copy, *args);
    vsnprintf(buf, sizeof(buf), format, copy);
    va_end(copy);
    messages_.emplace_back(buf);
  }

  rcutils_logging_output_handler_t previous_;
  static std::vector<std::string> messages_;
};
std::vector<std::string> LogCapture::messages_;

std::size_t countDeprecationWarnings()
{
  std::size_t n = 0;
  for (const auto & m : LogCapture::messages()) {
    if (m.find("'prox_mpc_core/Bicycle' is deprecated") != std::string::npos) {n++;}
  }
  return n;
}
}  // namespace

// One warning per process, however many times the alias is constructed, and it
// names both the replacement plugin and the behaviour that changed with the
// rename: an operator upgrading a 1.0.0 configuration gets the same model under
// a new name, but not the same emitted twist or the same collision-check pose.
TEST(DeprecatedAlias, WarnsOncePerProcessAndNamesTheBehaviourChange)
{
  LogCapture log;
  Bicycle first;
  Bicycle second;
  Bicycle third;

  ASSERT_EQ(countDeprecationWarnings(), 1u);
  const std::string & warning = LogCapture::messages().front();
  EXPECT_NE(warning.find("prox_mpc_core/BicycleFrontAxle"), std::string::npos) << warning;
  EXPECT_NE(warning.find("prox_mpc_core/BicycleRearAxle"), std::string::npos) << warning;
  EXPECT_NE(warning.find("twist"), std::string::npos) << warning;
  EXPECT_NE(warning.find("collision check"), std::string::npos) << warning;
}
