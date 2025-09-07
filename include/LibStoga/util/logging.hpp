/**
 * @file logging.hpp
 * @brief Lightweight logging helpers and stream operators for debugging.
 */
#ifndef LS_LOGGING_HPP
#define LS_LOGGING_HPP

#include <iostream>
#include "LibStoga/geometry.h"
#include "LibStoga/odom.h"

namespace ls {
inline std::ostream& operator<<(std::ostream& os, const Angle& a) {
    os << a.getAngle() << "deg"; return os;
}
inline std::ostream& operator<<(std::ostream& os, const Position& p) {
    os << "{X=" << p.X << ", Y=" << p.Y << ", theta=" << p.theta.getAngle() << "}"; return os;
}
}

// Basic log macro (enable by defining LS_STDLOG before including)
#ifdef LS_STDLOG
  #define LS_LOG(msg) do { std::cout << "[LS] " << msg << '\n'; } while(0)
#else
  #define LS_LOG(msg) do {} while(0)
#endif

#endif // LS_LOGGING_HPP
