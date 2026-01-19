#pragma once

#include "compiler_switches.h"

template <class T>
struct Vec3
{
public:
    T values[3];

    Vec3() = default;

    Vec3(T x, T y, T z)
    {
        values[0] = x;
        values[1] = y;
        values[2] = z;
    }

    Vec3(T values[3])
    {
        this->values[0] = values[0];
        this->values[1] = values[1];
        this->values[2] = values[2];
    }

    T &x()
    {
        return values[0];
    }

    T &y()
    {
        return values[1];
    }

    T &z()
    {
        return values[2];
    }

    template <class U>
    operator Vec3<U>() const
    {
        return Vec3<U>(
            (U)x(),
            (U)y(),
            (U)z());
    }

    Vec3 operator+(const Vec3 other) const
    {
        return Vec3(
            x() + other.x(),
            y() + other.y(),
            z() + other.z());
    }

    Vec3 operator-(const Vec3 other) const
    {
        return Vec3(
            x() - other.x(),
            y() - other.y(),
            z() - other.z());
    }

    Vec3 operator*(const Vec3 other) const
    {
        return Vec3(
            x() * other.x(),
            y() * other.y(),
            z() * other.z());
    }

    Vec3 operator/(const Vec3 other) const
    {
        return Vec3(
            x() / other.x(),
            y() / other.y(),
            z() / other.z());
    }

    Vec3 operator+(const T other) const
    {
        return Vec3(
            x() + other,
            y() + other,
            z() + other);
    }

    Vec3 operator-(const T other) const
    {
        return Vec3(
            x() - other,
            y() - other,
            z() - other);
    }

    Vec3 operator*(const T other) const
    {
        return Vec3(
            x() * other,
            y() * other,
            z() * other);
    }

    Vec3 operator/(const T other) const
    {
        return Vec3(
            x() / other,
            y() / other,
            z() / other);
    }

    Vec3 &operator+=(const Vec3 other)
    {
        return *this = *this + other;
    }

    Vec3 &operator-=(const Vec3 other)
    {
        return *this = *this - other;
    }

    Vec3 &operator*=(const Vec3 other)
    {
        return *this = *this * other;
    }

    Vec3 &operator/=(const Vec3 other)
    {
        return *this = *this / other;
    }

    Vec3 &operator+=(const T other)
    {
        return *this = *this + other;
    }

    Vec3 &operator-=(const T other)
    {
        return *this = *this - other;
    }

    Vec3 &operator*=(const T other)
    {
        return *this = *this * other;
    }

    Vec3 &operator/=(const T other)
    {
        return *this = *this / other;
    }

    static T dot(const Vec3 a, const Vec3 b)
    {
        return (a.x() * b.x()) +
               (a.y() * b.y()) +
               (a.z() * b.z());
    }
};
