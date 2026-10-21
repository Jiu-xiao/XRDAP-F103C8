// SPDX-License-Identifier: Apache-2.0
// Host tests of led_drive.hpp. Expected values come from:
//   [hw]    the board: PA9 high lights red, low lights blue, half duty looks purple;
//   [rule]  XRDAP-Core status_light.hpp: Frame::level 0 is off, and an on/off-only LED is
//           dark while Frame::pulse is above 500; levels and shares are in permille;
//   [user]  the family colours: red wired, blue wireless, purple no link.

#include <cstdio>

#include "led_drive.hpp"

namespace
{
int failures = 0;

void Check(bool ok, const char* what)
{
  if (!ok)
  {
    std::printf("FAIL: %s\n", what);
    ++failures;
  }
}

XrDap::Light::Frame MakeFrame(XrDap::Light::Rgb color, uint16_t level, uint16_t pulse)
{
  return XrDap::Light::Frame{color, level, pulse};
}
}  // namespace

int main()
{
  using XrDap::RedShare;
  using XrDap::ToLedDrive;
  namespace Light = XrDap::Light;

  // [hw][user] the three family colours map to the three drive points of the light.
  Check(RedShare(Light::kRed) == 1000U, "red (wired) drives PA9 high all the time");
  Check(RedShare(Light::kBlue) == 0U, "blue (wireless) drives PA9 low all the time");
  Check(RedShare(Light::kPurple) == 500U, "purple (no link) is half duty");

  // [rule] shares are permille: every colour stays within 0..1000.
  bool in_range = true;
  for (uint32_t r = 0; r < 256U; ++r)
  {
    for (uint32_t b = 0; b < 256U; ++b)
    {
      const Light::Rgb c{static_cast<uint8_t>(r), 0, static_cast<uint8_t>(b)};
      in_range = in_range && RedShare(c) <= 1000U;
    }
  }
  Check(in_range, "red share within 0..1000 for every red/blue mix");

  // [rule] level 0 is off whatever the pulse.
  Check(!ToLedDrive(MakeFrame(Light::kRed, 0, 0)).lit, "level 0 is dark");
  // [rule] lit at rest and while the pulse is at most 500.
  Check(ToLedDrive(MakeFrame(Light::kRed, 200, 0)).lit, "resting level is lit");
  Check(ToLedDrive(MakeFrame(Light::kRed, 600, 500)).lit, "pulse 500 is still lit");
  // [rule] dark while the activity pulse is above 500.
  Check(!ToLedDrive(MakeFrame(Light::kRed, 700, 501)).lit, "pulse 501 is dark");
  Check(!ToLedDrive(MakeFrame(Light::kRed, 1000, 1000)).lit, "pulse 1000 is dark");

  // The colour share does not depend on lit/dark.
  Check(ToLedDrive(MakeFrame(Light::kBlue, 1000, 1000)).red_permille == 0U,
        "blue stays blue while blinking dark");

  if (failures == 0)
  {
    std::printf("led_drive: all checks passed\n");
  }
  return failures == 0 ? 0 : 1;
}
