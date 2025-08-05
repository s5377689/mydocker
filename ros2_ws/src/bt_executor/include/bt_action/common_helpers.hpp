#pragma once
#include <cmath>

namespace bt_action
{

inline double deg2rad(double deg) { return deg * M_PI / 180.0; }
inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }

}  // namespace bt_action