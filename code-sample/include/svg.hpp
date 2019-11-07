#pragma once

#include <iostream>
#include <ostream>
#include <string>
#include <vector>

struct SVG
{
    SVG(double _width, double _height) : width(_width), height(_height) {}

    struct Color
    {
        int r, g, b;
        Color(int _r = 0, int _g = 0, int _b = 0) : r(_r), g(_g), b(_b) {}
        const static Color RED, GREEN, BLUE, WHITE, BLACK, YELLOW;
        friend std::ostream &operator<<(std::ostream &out, const Color &c);
        bool invalid() const { return r < 0 || g < 0 || b < 0; }
    };

    struct Point
    {
        double x, y;
        Point(double _x = 0.0, double _y = 0.0) : x(_x), y(_y) {}
    };

    struct Circle
    {
        Point p;
        double r;
        Color stroke, fill;
        double stroke_width;
        Circle(Point _p, double _r, Color _stroke = Color(),
               Color _fill = Color(-1, -1, -1), double _stroke_width = 1.0)
            : p(_p), r(_r), stroke(_stroke), fill(_fill),
              stroke_width(_stroke_width)
        {
            if (fill.invalid()) {
                fill = stroke;
            }
        }
        friend std::ostream &operator<<(std::ostream &out,
                                        const SVG::Circle &c);
    };

    struct Line
    {
        Point p1, p2;
        Color stroke;
        double stroke_width;
        Line(Point _p1, Point _p2, Color _stroke = Color(),
             double _stroke_width = 1.0)
            : p1(_p1), p2(_p2), stroke(_stroke), stroke_width(_stroke_width)
        {
        }

        friend std::ostream &operator<<(std::ostream &out, const SVG::Line &l);
    };

    struct Text
    {
        Point p;
        std::string text;
        Color fill;
        double fontsize;
        Text(Point _p, std::string _text, Color _fill = Color(),
             double _fontsize = 10)
            : p(_p), text(_text), fill(_fill), fontsize(_fontsize)
        {
        }
        friend std::ostream &operator<<(std::ostream &out, const SVG::Text &t);
    };

    struct Polyline
    {
        std::vector<Point> points;
        Color stroke;
        double stroke_width;
        Polyline(std::vector<Point> _points, Color _stroke = Color(),
                 double _stroke_width = 1.0)
            : points(_points), stroke(_stroke), stroke_width(_stroke_width)
        {
        }
        friend std::ostream &operator<<(std::ostream &out,
                                        const SVG::Polyline &p);
    };

    void save(std::string path) const;

    double width, height;
    Color background;
    std::vector<Line> lines;
    std::vector<Circle> circles;
    std::vector<Text> texts;
    std::vector<Polyline> polylines;
    static double GRID_STEP;
    static Color BACKGROUND_COLOR;

    friend std::ostream &operator<<(std::ostream &out, const SVG &s);
};

std::ostream &operator<<(std::ostream &out, const SVG::Color &c);
std::ostream &operator<<(std::ostream &out, const SVG::Circle &c);
std::ostream &operator<<(std::ostream &out, const SVG::Line &l);
std::ostream &operator<<(std::ostream &out, const SVG::Text &t);
std::ostream &operator<<(std::ostream &out, const SVG::Polyline &p);
std::ostream &operator<<(std::ostream &out, const SVG &s);