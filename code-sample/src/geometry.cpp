#include "geometry.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace std;

double Point::distTo(const Point &p) const
{
    double dx = x - p.x;
    double dy = y - p.y;
    double dz = z - p.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

double Point::distTo(const Line &l) const { return l.distTo(*this); }

double Point::norm() const
{
    Point o;
    return distTo(o);
}

double Point::dot(const Point &p) const { return x * p.x + y * p.y + z * p.z; }

Point Point::cross(const Point &p) const
{
    double xx = y * p.z - p.y * z;
    double yy = p.x * z - x * p.z;
    double zz = x * p.y - p.x * y;
    return Point(xx, yy, zz);
}

Point Point::operator-() const { return Point(-x, -y, -z); }

Point Point::operator-(const Point &p) const
{
    return Point(x - p.x, y - p.y, z - p.z);
}

Point Point::operator+(const Point &p) const
{
    return Point(x + p.x, y + p.y, z + p.z);
}

Point Point::operator*(double scale) const
{
    return Point(x * scale, y * scale, z * scale);
}

Point Point::operator*(const Point &p) const {
    return dot(p);
}

Point Point::operator/(double scale) const
{
    return Point(x / scale, y / scale, z / scale);
}

std::ostream &operator<<(std::ostream &out, const Point &p)
{
    out << "Point(" << p.x << "," << p.y << "," << p.z << ")";
}

std::string Point::operator()() const
{
    stringstream ss;
    ss << *this;
    return ss.str();
}

double dot(const Point &p0, const Point &p1) { return p0.dot(p1); }

Point cross(const Point &p0, const Point &p1) { return p0.cross(p1); }

double Line::distTo(const Point &p, Point *perp) const
{
    Vector ap = p - a;
    Vector ab = b - a;
    double dot = ap.dot(ab);
    double len = norm();
    if (dot <= 0) {
        if (perp) {
            *perp = a;
        }
        return ap.norm();
    }
    if (dot / len >= len) {
        if (perp) {
            *perp = b;
        }
        return (p - b).norm();
    }
    if (perp) {
        double scale = dot / len;
        Point pp = a + (b - a) / len * scale;
        *perp = pp;
    }
    return fabs(ap.cross(ab).norm() / len);
}

double Line::norm() const { return (a - b).norm(); }

std::ostream &operator<<(std::ostream &out, const Line &l)
{
    out << "Line(" << l.a << "," << l.b << ")";
}

std::string Line::operator()() const
{
    stringstream ss;
    ss << *this;
    return ss.str();
}