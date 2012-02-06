/*
 * point.h
 *
 *  Created on: 10-dic-2008
 *      Author: pablo
 */

#ifndef POINT_H_
#define POINT_H_

#include <cmath>
#include <limits>

namespace gns {
    class point;

    class vector {
        public:
            double x, y, z;

            vector();
            vector(double x, double y, double z = 0);
            vector(const gns::point& p);
            static vector polar(double length, double angle, double z = 0);

            vector operator+(const gns::vector& p) const;
            vector operator-() const;
            vector operator-(const gns::vector& p) const;

            vector operator*(const double s) const;

            double length() const;
            double angle() const;
            vector direction() const;
    };

    double dot(const gns::vector& a, const gns::vector& b);
    gns::vector cross(const gns::vector& a, const gns::vector& b);
    double angle(const vector& a, const vector& b);

    class point {
        public:
            double x, y, z;

            point();
            point(double x, double y, double z = 0);
            static point polar(double length, double angle, double z = 0);

            point operator+(const gns::point& p) const;
            point operator-() const;

            point operator*(const double s) const;

            point operator+(const gns::vector& p) const;
            vector operator-(const gns::point& p) const;
    };

    double distance(const point& a, const point& b);

    class frame {
        public:
            double x, y, theta;

            frame();
            frame(double x, double y, double theta);

            frame operator~() const;
            frame operator*(const frame& f) const;
            gns::point operator*(const gns::point& input) const;
            gns::vector operator*(const gns::vector& input) const;

            gns::point zero() const;
    };
}
////////////////////////////////////////////////////////////////////////////////
inline gns::vector::vector() :
    x(0), y(0), z(0) {
}
////////////////////////////////////////////////////////////////////////////////
inline gns::vector::vector(double p_x, double p_y, double p_z) :
    x(p_x), y(p_y), z(p_z) {
}
////////////////////////////////////////////////////////////////////////////////
inline gns::vector::vector(const gns::point& p) :
    x(p.x), y(p.y), z(p.z) {
}
////////////////////////////////////////////////////////////////////////////////
inline gns::vector gns::vector::polar(double length, double angle, double z) {
    return gns::vector(length * cos(angle), length * sin(angle), z);
}
////////////////////////////////////////////////////////////////////////////////
inline gns::vector gns::vector::operator+(const gns::vector& p) const {
    return gns::vector(x + p.x, y + p.y, z + p.z);
}
////////////////////////////////////////////////////////////////////////////////
inline gns::vector gns::vector::operator-() const {
    return gns::vector(-x, -y, -z);
}
////////////////////////////////////////////////////////////////////////////////
inline gns::vector gns::vector::operator-(const gns::vector& p) const {
    return this->operator +(p.operator -());
}
////////////////////////////////////////////////////////////////////////////////
inline gns::vector gns::vector::operator*(const double s) const {
    return gns::vector(s * x, s * y, s * z);
}
////////////////////////////////////////////////////////////////////////////////
inline double gns::vector::length() const {
    return sqrt(dot(*this, *this));
}
////////////////////////////////////////////////////////////////////////////////
inline double gns::vector::angle() const {
    return atan2(y, x);
}
////////////////////////////////////////////////////////////////////////////////
inline gns::vector gns::vector::direction() const {
    double l = length();
    if (l > std::numeric_limits<double>::epsilon())
        return gns::vector(x / l, y / l, z / l);
    else
        return gns::vector();
}
////////////////////////////////////////////////////////////////////////////////
inline double gns::dot(const gns::vector& a, const gns::vector& b) {
    return (a.x * b.x + a.y * b.y + a.z * b.z);
}
////////////////////////////////////////////////////////////////////////////////
inline gns::vector gns::cross(const gns::vector& a, const gns::vector& b) {
    return gns::vector(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y
            - a.y * b.x);
}
////////////////////////////////////////////////////////////////////////////////
inline gns::point::point() :
    x(0), y(0), z(0) {
}
////////////////////////////////////////////////////////////////////////////////
inline gns::point::point(double p_x, double p_y, double p_z) :
    x(p_x), y(p_y), z(p_z) {
}
////////////////////////////////////////////////////////////////////////////////
inline gns::point gns::point::polar(double length, double angle, double z) {
    return gns::point(length * cos(angle), length * sin(angle), z);
}
////////////////////////////////////////////////////////////////////////////////
inline gns::point gns::point::operator+(const gns::point& p) const {
    return gns::point(x + p.x, y + p.y, z + p.z);
}
////////////////////////////////////////////////////////////////////////////////
inline gns::point gns::point::operator-() const {
    return gns::point(-x, -y, -z);
}
////////////////////////////////////////////////////////////////////////////////
inline gns::point gns::point::operator*(const double s) const {
    return gns::point(s * x, s * y, s * z);
}
////////////////////////////////////////////////////////////////////////////////
inline gns::point gns::point::operator+(const gns::vector& p) const {
    return gns::point(x + p.x, y + p.y, z + p.z);
}
////////////////////////////////////////////////////////////////////////////////
inline gns::vector gns::point::operator-(const gns::point& p) const {
    return gns::vector(x - p.x, y - p.y, z - p.z);
}
////////////////////////////////////////////////////////////////////////////////
inline double gns::distance(const gns::point& a, const gns::point& b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}
////////////////////////////////////////////////////////////////////////////////
inline double gns::angle(const gns::vector& a, const gns::vector& b) {
    return acos(gns::dot(a, b) / (a.length() * b.length()));
}
////////////////////////////////////////////////////////////////////////////////
inline gns::frame::frame() :
    x(0), y(0), theta(0) {
}
////////////////////////////////////////////////////////////////////////////////
inline gns::frame::frame(double p_x, double p_y, double p_theta) :
    x(p_x), y(p_y), theta(p_theta) {
}
////////////////////////////////////////////////////////////////////////////////
inline gns::frame gns::frame::operator~() const {
    return gns::frame(-x * cos(theta) - y * sin(theta), x * sin(theta) - y
            * cos(theta), -theta);
}
////////////////////////////////////////////////////////////////////////////////
inline gns::frame gns::frame::operator*(const gns::frame& f) const {
    return gns::frame(x + cos(theta) * f.x - sin(theta) * f.y, y + cos(theta)
            * f.y + sin(theta) * f.x, theta + f.theta);
}
////////////////////////////////////////////////////////////////////////////////
inline gns::point gns::frame::operator*(const gns::point& input) const {
    return gns::point(x + input.x * cos(theta) - input.y * sin(theta), y
            + input.x * sin(theta) + input.y * cos(theta), input.z);
}
////////////////////////////////////////////////////////////////////////////////
inline gns::vector gns::frame::operator*(const gns::vector& input) const {
    return gns::vector(input.x * cos(theta) - input.y * sin(theta), input.x
            * sin(theta) + input.y * cos(theta), input.z);
}
////////////////////////////////////////////////////////////////////////////////
inline gns::point gns::frame::zero() const {
    return gns::point(x, y);
}
////////////////////////////////////////////////////////////////////////////////
#endif /* POINT_H_ */
