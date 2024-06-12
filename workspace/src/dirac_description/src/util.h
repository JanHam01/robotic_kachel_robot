//
// Created by robotik on 10.06.24.
//
#pragma once

#ifndef UTIL_H
#define UTIL_H

#include <cmath>

class Vector2 {
public:
  Vector2(const double x, const double y) : m_x(x), m_y(y) {
  }

  Vector2() : m_x(0), m_y(0) {
  }

  double getX() const { return m_x; }
  double getY() const { return m_y; }

  Vector2 normalizeNew() const {
    Vector2 t(m_x, m_y);
    const double length = t.length();
    t.m_x /= length;
    t.m_y /= length;
    return t;
  }

  double length() const {
    return std::sqrt(m_x * m_x + m_y * m_y);
  }

  double dot(const Vector2 o) const {
    return m_x * o.m_x + m_y * o.m_y;
  }

  double getAngle() const {
    return std::atan2(m_y, m_x);
  }

  Vector2 operator+(const Vector2 &o) const {
    return Vector2(m_x + o.m_x, m_y + o.m_y);
  }

  Vector2 operator+(const double &o) const {
    return Vector2(m_x + o, m_y + o);
  }

  Vector2 operator-(const Vector2 &o) const {
    return Vector2(m_x - o.m_x, m_y - o.m_y);
  }

  Vector2 operator-(const double &o) const {
    return Vector2(m_x - o, m_y - o);
  }

  Vector2 operator*(const double s) const {
    return Vector2(m_x * s, m_y * s);
  }

  friend bool operator==(const Vector2 &s, const Vector2 &o) {
    return s.m_x == o.m_x && s.m_y == o.m_y;
  }

  friend bool operator!=(const Vector2 &s, const Vector2 &o) {
    return !operator==(s, o);
  }

private:
  double m_x;
  double m_y;
};


inline int square(const int a) {
  return a * a;
}

inline double normalizeAngle(const double angle) {
  // Don't call this a hack! It's numeric!
  return angle - (std::round((angle / (M_PI * 2)) - 1e-6) * M_PI * 2);
}

#endif //UTIL_H
