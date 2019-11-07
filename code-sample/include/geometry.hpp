#pragma once

#include <ostream>

class Line;

struct Point
{
    double x, y, z;
    Point(double _x = 0.0, double _y = 0.0, double _z = 0.0)
        : x(_x), y(_y), z(_z)
    {
    }

    double distTo(const Point &p) const;
    double distTo(const Line &l) const;
    double norm() const;

    double dot(const Point &p) const;
    Point cross(const Point &p) const;

    Point operator-() const;
    Point operator-(const Point &p) const;
    Point operator+(const Point &p) const;
    Point operator*(double scale) const;
    Point operator*(const Point &p) const; // dot product
    Point operator/(double scale) const;

    friend std::ostream &operator<<(std::ostream &out, const Point &p);
    std::string operator()() const;
};

double dot(const Point &p0, const Point &p1);
Point cross(const Point &p0, const Point &p1);

typedef Point Vector;

struct Line
{
    Point a, b;
    Line(Point _a = Point(), Point _b = Point()) : a(_a), b(_b) {}

    double distTo(const Point &p, Point *perp = NULL) const;
    double norm() const;

    friend std::ostream &operator<<(std::ostream &out, const Line &l);
    std::string operator()() const;
};

std::ostream &operator<<(std::ostream &out, const Point &p);
std::ostream &operator<<(std::ostream &out, const Line &l);