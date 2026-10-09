#pragma once

#include <cstdint>

// Verified scale for LCMT-01SLR23ZB-40M00330B on the LC10E.
// One real axis only. Additional SCARA axes are not declared here.
constexpr int32_t kCountsPerRevolution = 8388608;
constexpr double kMaxMoveDegrees = 1.0;
constexpr double kMaxSpeedRpm = 5.0;
constexpr double kMaxAccelRpmPerS = 20.0;
constexpr double kMaxJerkRpmPerS2 = 2000.0;
constexpr const char *kAxisTitle = "LC10E-100W / LCMT-01SLR23ZB-40M00330B";
